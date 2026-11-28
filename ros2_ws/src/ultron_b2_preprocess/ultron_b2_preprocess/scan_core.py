"""Pure numpy PointCloud -> planar scan projection (no ROS imports; unit-tested directly)."""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ScanConfig:
    min_height: float = 0.45
    max_height: float = 1.00
    angle_min: float = -math.pi
    angle_max: float = math.pi
    angle_increment: float = math.radians(0.2)
    range_min: float = 0.35
    range_max: float = 12.0

    @property
    def n_bins(self):
        return int(math.floor((self.angle_max - self.angle_min) / self.angle_increment + 1e-9))


def quaternion_matrix(translation, quaternion):
    """4x4 homogeneous transform from (x, y, z) and quaternion (x, y, z, w)."""
    x, y, z, w = quaternion
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = translation
    return m


def project(xyz, transform, cfg):
    """Nearest return per angle bin of the points inside the height band of the target frame.

    xyz: (N, 3) points in the cloud frame; transform: 4x4 target<-cloud. Returns float32 ranges
    (inf where a bin has no return), one per bin starting at cfg.angle_min.
    """
    ranges = np.full(cfg.n_bins, np.inf, dtype=np.float32)
    if len(xyz) == 0:
        return ranges
    p = np.asarray(xyz, float) @ transform[:3, :3].T + transform[:3, 3]
    p = p[np.isfinite(p).all(axis=1) & (p[:, 2] >= cfg.min_height) & (p[:, 2] <= cfg.max_height)]
    r = np.hypot(p[:, 0], p[:, 1])
    a = np.arctan2(p[:, 1], p[:, 0])
    keep = (r >= cfg.range_min) & (r <= cfg.range_max) & (a >= cfg.angle_min) & (a <= cfg.angle_max)
    idx = np.floor((a[keep] - cfg.angle_min) / cfg.angle_increment).astype(int)
    idx = np.clip(idx, 0, cfg.n_bins - 1)
    np.minimum.at(ranges, idx, r[keep].astype(np.float32))
    return ranges
