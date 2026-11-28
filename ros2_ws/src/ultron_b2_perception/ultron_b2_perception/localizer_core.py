"""Detection -> map-frame ladder position from aligned depth, intrinsics and TF at the image stamp.

Pure numpy; the ROS node supplies the transform looked up *at the image stamp*. Any TF failure
(stale, extrapolation, missing) drops the detection and is counted, never approximated with a
later transform. Thresholds here are PROPOSED simulation values.
"""
import math
from collections import Counter

import numpy as np

from .geometry import apply, backproject, project

DROP_REASONS = ('STALE_TF', 'NO_DEPTH', 'DEPTH_NOT_SYNCED', 'LOCALIZATION_NOT_VALID', 'INSUFFICIENT_DEPTH',
                'OUT_OF_RANGE', 'BAD_BOX', 'NO_CAMERA_INFO', 'IMAGE_TOO_OLD', 'DEPTH_NOT_REGISTERED',
                'NO_ORACLE_MASK', 'ORACLE_FRAME_NOT_SYNCED')


class Rejected(ValueError):
    def __init__(self, reason, detail=''):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        self.reason = reason


class DropCounter(Counter):
    def drop(self, reason):
        self[reason] += 1


def require_registered_depth(registered):
    """Reject color/depth fusion unless registration is explicitly established."""
    if not registered:
        raise Rejected('DEPTH_NOT_REGISTERED')


def depth_to_metres(raw, encoding, scale_16u=0.001):
    """Image data -> float32 metres (16UC1 uses ``scale_16u``, RealSense default 1 mm TO_VERIFY)."""
    if encoding in ('16UC1', 'mono16'):
        out = raw.astype(np.float32) * float(scale_16u)
        out[raw == 0] = np.nan
        return out
    if encoding == '32FC1':
        out = raw.astype(np.float32)
        out[~np.isfinite(out) | (out <= 0)] = np.nan
        return out
    raise Rejected('NO_DEPTH', f'unsupported depth encoding {encoding}')


def color_box_to_depth(box, K_color, K_depth):
    """Map a colour-image box into depth-image pixels via normalized coordinates.

    Requires a depth image registered to the color optical center. This mapping accounts for
    differing image intrinsics, not a physical baseline or optical rotation.
    """
    def conv(u, v):
        xn, yn = (u - K_color[0, 2]) / K_color[0, 0], (v - K_color[1, 2]) / K_color[1, 1]
        return K_depth[0, 0] * xn + K_depth[0, 2], K_depth[1, 1] * yn + K_depth[1, 2]
    u0, v0 = conv(box[0], box[1])
    u1, v1 = conv(box[2], box[3])
    return (u0, v0, u1, v1)


def oracle_depth_mask(labels, box, depth_shape, K_color, K_depth):
    """Map one exact oracle box's instance pixels onto registered depth rays.

    The instance label is used only to select pixels, not as a persistent asset identity.
    Ambiguous or missing boxes are rejected rather than falling back to box-only depth.
    """
    if labels.ndim != 2 or not np.issubdtype(labels.dtype, np.integer):
        raise Rejected('NO_ORACLE_MASK', 'invalid labels')
    matches = []
    for label in np.unique(labels):
        if label == 0:
            continue
        v, u = np.nonzero(labels == label)
        candidate = (float(u.min()), float(v.min()), float(u.max() + 1), float(v.max() + 1))
        if candidate == tuple(box):
            matches.append(label)
    if len(matches) != 1:
        raise Rejected('NO_ORACLE_MASK', 'box has no unique instance')
    h, w = depth_shape
    vd, ud = np.indices((h, w))
    uc = np.rint(K_color[0, 0] * (ud - K_depth[0, 2]) / K_depth[0, 0] + K_color[0, 2]).astype(int)
    vc = np.rint(K_color[1, 1] * (vd - K_depth[1, 2]) / K_depth[1, 1] + K_color[1, 2]).astype(int)
    inside = (uc >= 0) & (uc < labels.shape[1]) & (vc >= 0) & (vc < labels.shape[0])
    mask = np.zeros((h, w), dtype=bool)
    mask[inside] = labels[vc[inside], uc[inside]] == matches[0]
    return mask


def foreground_depth(depth_m, box, *, percentile=35.0, min_valid=10, min_m=0.2, max_m=15.0, mask=None):
    """Foreground depth inside a box (ladders show background through the rungs)."""
    h, w = depth_m.shape
    x0, y0 = max(0, int(box[0])), max(0, int(box[1]))
    x1, y1 = min(w, int(math.ceil(box[2]))), min(h, int(math.ceil(box[3])))
    if x1 <= x0 or y1 <= y0:
        raise Rejected('BAD_BOX', f'{box}')
    patch = depth_m[y0:y1, x0:x1]
    valid = np.isfinite(patch) & (patch > min_m) & (patch < max_m)
    if mask is not None:
        if mask.shape != depth_m.shape:
            raise Rejected('NO_ORACLE_MASK', 'mask and depth dimensions differ')
        valid &= mask[y0:y1, x0:x1]
    v, u = np.nonzero(valid)
    if u.size < min_valid:
        raise Rejected('INSUFFICIENT_DEPTH', f'{u.size} valid pixels')
    values = patch[v, u]
    cut = np.percentile(values, percentile)
    keep = values <= cut
    z = float(np.median(values[keep]))
    return z, float(np.median(u[keep]) + x0), float(np.median(v[keep]) + y0), float(np.std(values[keep]))


def localize(box_depth_px, depth_m, K_depth, T_map_optical, *, max_range_m=8.0,
             depth_sigma_a=0.02, depth_sigma_b=0.005, pixel_sigma=4.0, mask=None):
    """Return (position_map, covariance_map 3x3, range_m)."""
    z, u, v, spread = foreground_depth(depth_m, box_depth_px, mask=mask)
    if z > max_range_m:
        raise Rejected('OUT_OF_RANGE', f'{z:.2f} m')
    p_cam = backproject(K_depth, u, v, z)
    sig_z = depth_sigma_a + depth_sigma_b * z * z + spread
    sig_xy = z * pixel_sigma / K_depth[0, 0] + 0.5 * spread
    C_cam = np.diag([sig_xy ** 2, sig_xy ** 2, sig_z ** 2])
    R = T_map_optical[:3, :3]
    return apply(T_map_optical, p_cam), R @ C_cam @ R.T, float(np.linalg.norm(p_cam))


def visible_candidates(candidates, depth_m, K_depth, T_map_optical, *, max_range_m=6.0, margin_m=0.5,
                       border_px=20):
    """IDs of known candidates whose position is in view and not occluded (for MISSING checks)."""
    T_optical_map = np.linalg.inv(T_map_optical)
    h, w = depth_m.shape
    out = []
    for cand in candidates:
        p = apply(T_optical_map, cand['position'])
        dist = float(np.linalg.norm(p))
        if p[2] <= 0.3 or dist > max_range_m:
            continue
        px = project(K_depth, p)
        if px is None or not (border_px <= px[0] < w - border_px and border_px <= px[1] < h - border_px):
            continue
        u, v = int(px[0]), int(px[1])
        patch = depth_m[max(0, v - 5):v + 6, max(0, u - 5):u + 6]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if not valid.size or float(np.median(valid)) < p[2] - margin_m:
            continue
        out.append(cand['candidate_id'])
    return out
