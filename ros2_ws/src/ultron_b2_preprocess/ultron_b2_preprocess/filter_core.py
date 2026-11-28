"""Point-cloud preprocessing maths (numpy only, no ROS): parse, crop, self-filter, deskew, voxel, pack.

All geometry is in the cloud's own frame (lidar_link). Thresholds come from the node parameters and are
PROPOSED; see config/cloud_filter_<profile>.yaml for provenance.
"""
from dataclasses import dataclass, field

import numpy as np

# PointField datatype -> numpy dtype (sensor_msgs/PointField constants).
POINTFIELD_DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
FLOAT32, UINT16 = 7, 4
# Output layout (same as the ULTRON sim bridge): x,y,z,intensity f32, ring u16, time f32 (s after header stamp).
OUT_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                      'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                      'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24})
OUT_FIELDS = (('x', 0, FLOAT32), ('y', 4, FLOAT32), ('z', 8, FLOAT32), ('intensity', 12, FLOAT32),
              ('ring', 16, UINT16), ('time', 20, FLOAT32))
TIME_FIELD_NAMES = ('time', 't', 'timestamp', 'time_stamp')
# Per-point times larger than this are absolute epoch seconds (rslidar_sdk `timestamp` f64).
ABSOLUTE_TIME_THRESHOLD_S = 1.0e6


@dataclass
class FilterConfig:
    range_min: float = 0.3
    range_max: float = 40.0
    z_min: float = -1.5
    z_max: float = 3.0
    self_box_enabled: bool = True
    self_box_min: tuple = (-0.25, -0.35, -0.60)
    self_box_max: tuple = (0.20, 0.35, 0.05)
    voxel_size: float = 0.05


@dataclass
class FilterStats:
    n_in: int = 0
    n_nonfinite: int = 0
    n_range: int = 0
    n_height: int = 0
    n_self: int = 0
    n_voxel_merged: int = 0
    n_out: int = 0
    deskewed: bool = False
    extra: dict = field(default_factory=dict)


def structured_dtype(fields, point_step, is_bigendian=False):
    """numpy dtype for PointCloud2 fields; fields = iterable of (name, offset, datatype, count)."""
    names, formats, offsets = [], [], []
    for name, offset, datatype, count in fields:
        if count != 1 or datatype not in POINTFIELD_DTYPES:
            continue
        names.append(name)
        formats.append(('>' if is_bigendian else '<') + POINTFIELD_DTYPES[datatype])
        offsets.append(offset)
    return np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': point_step})


def parse(buffer, fields, point_step, n_points, stamp_s, is_bigendian=False):
    """Return xyz (N,3 f64), intensity (N f32), ring (N u16), time (N f32, s after stamp), has_time."""
    if n_points == 0 or point_step == 0:
        return (np.zeros((0, 3)), np.zeros(0, np.float32), np.zeros(0, np.uint16), np.zeros(0, np.float32), False)
    arr = np.frombuffer(buffer, dtype=structured_dtype(fields, point_step, is_bigendian), count=n_points)
    names = arr.dtype.names
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)
    intensity = arr['intensity'].astype(np.float32) if 'intensity' in names else np.zeros(n_points, np.float32)
    ring = arr['ring'].astype(np.uint16) if 'ring' in names else np.zeros(n_points, np.uint16)
    time_name = next((n for n in TIME_FIELD_NAMES if n in names), None)
    if time_name is None:
        return xyz, intensity, ring, np.zeros(n_points, np.float32), False
    t = arr[time_name].astype(np.float64)
    finite = np.isfinite(t)
    if finite.any() and np.nanmax(np.abs(t[finite])) > ABSOLUTE_TIME_THRESHOLD_S:
        t = t - stamp_s
    return xyz, intensity, ring, t.astype(np.float32), True


def crop_masks(xyz, cfg):
    """Boolean keep-masks (finite, range, height, not-self) evaluated on all points."""
    finite = np.isfinite(xyz).all(axis=1)
    safe = np.where(finite[:, None], xyz, 0.0)
    r = np.linalg.norm(safe, axis=1)
    in_range = (r >= cfg.range_min) & (r <= cfg.range_max)
    in_height = (safe[:, 2] >= cfg.z_min) & (safe[:, 2] <= cfg.z_max)
    if cfg.self_box_enabled:
        lo, hi = np.asarray(cfg.self_box_min, float), np.asarray(cfg.self_box_max, float)
        not_self = ~np.all((safe >= lo) & (safe <= hi), axis=1)
    else:
        not_self = np.ones(len(xyz), bool)
    return finite, in_range, in_height, not_self


def inside_box(xyz, lo, hi):
    return np.all((xyz >= np.asarray(lo, float)) & (xyz <= np.asarray(hi, float)), axis=1)


def rotation_vectors_to_matrices(rotvec):
    """Rodrigues: (N,3) rotation vectors -> (N,3,3) rotation matrices."""
    theta = np.linalg.norm(rotvec, axis=1)
    small = theta < 1e-12
    k = np.where(small[:, None], 0.0, rotvec / np.where(small, 1.0, theta)[:, None])
    kx, ky, kz = k[:, 0], k[:, 1], k[:, 2]
    zero = np.zeros_like(kx)
    K = np.stack([np.stack([zero, -kz, ky], 1), np.stack([kz, zero, -kx], 1), np.stack([-ky, kx, zero], 1)], 1)
    s, c = np.sin(theta)[:, None, None], (1.0 - np.cos(theta))[:, None, None]
    return np.eye(3)[None] + s * K + c * (K @ K)


def deskew_rotation(xyz, t, omega, t_ref):
    """Rotation-only deskew with constant angular velocity `omega` (rad/s, cloud frame).

    The sensor frame at point time t_i relates to the frame at t_ref by Exp(omega (t_i - t_ref)), so
    p_ref = Exp(omega (t_i - t_ref)) p_i. Translation during the sweep is NOT compensated (no velocity source).
    """
    if len(xyz) == 0:
        return xyz
    rotvec = np.outer(t.astype(np.float64) - t_ref, np.asarray(omega, float))
    return np.einsum('nij,nj->ni', rotation_vectors_to_matrices(rotvec), xyz)


def voxel_first(xyz, voxel_size):
    """Indices (scan order) of the first point in each occupied voxel; keeps real point attributes."""
    if voxel_size <= 0 or len(xyz) == 0:
        return np.arange(len(xyz))
    keys = np.floor(xyz / voxel_size).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = keys.max(axis=0) + 1
    flat = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
    _, first = np.unique(flat, return_index=True)
    return np.sort(first)


def filter_cloud(xyz, intensity, ring, t, cfg, omega=None, t_ref=None):
    """Full pipeline. Returns (packed OUT_DTYPE array, FilterStats). omega -> rotation deskew to t_ref."""
    stats = FilterStats(n_in=len(xyz))
    finite, in_range, in_height, not_self = crop_masks(xyz, cfg)
    stats.n_nonfinite = int((~finite).sum())
    stats.n_range = int((finite & ~in_range).sum())
    stats.n_height = int((finite & in_range & ~in_height).sum())
    keep = finite & in_range & in_height
    stats.n_self = int((keep & ~not_self).sum())
    keep &= not_self
    xyz, intensity, ring, t = xyz[keep], intensity[keep], ring[keep], t[keep]
    if omega is not None and len(xyz):
        xyz = deskew_rotation(xyz, t, omega, t_ref)
        t = np.full(len(xyz), 0.0, np.float32)
        stats.deskewed = True
    idx = voxel_first(xyz, cfg.voxel_size)
    stats.n_voxel_merged = len(xyz) - len(idx)
    out = np.zeros(len(idx), dtype=OUT_DTYPE)
    out['x'], out['y'], out['z'] = xyz[idx, 0], xyz[idx, 1], xyz[idx, 2]
    out['intensity'], out['ring'], out['time'] = intensity[idx], ring[idx], t[idx]
    stats.n_out = len(out)
    return out, stats


def mean_angular_velocity(samples, t0, t1, margin=0.02):
    """Mean of (stamp_s, wx, wy, wz) samples within [t0 - margin, t1 + margin]; None if none."""
    if len(samples) == 0:
        return None
    a = np.asarray(samples, dtype=np.float64)
    sel = (a[:, 0] >= t0 - margin) & (a[:, 0] <= t1 + margin)
    if not sel.any():
        return None
    return a[sel, 1:4].mean(axis=0)


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])
