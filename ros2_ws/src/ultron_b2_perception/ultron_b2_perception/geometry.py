"""Small rigid-transform and pinhole helpers (numpy only, no ROS)."""
import math

import numpy as np


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError('invalid quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def transform(translation, quaternion):
    """4x4 homogeneous matrix from (x, y, z) and quaternion (x, y, z, w)."""
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(*quaternion)
    T[:3, 3] = translation
    return T


def planar(x, y, yaw, z=0.0):
    return transform((x, y, z), (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)))


def inverse(T):
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def apply(T, point):
    return (T @ np.append(np.asarray(point, dtype=float), 1.0))[:3]


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


# Standard ROS optical frame (z forward, x right, y down) relative to a body frame (x forward, y left, z up).
BODY_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0, 0.0],
                              [-1.0, 0.0, 0.0, 0.0],
                              [0.0, -1.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 1.0]])


def intrinsics(k):
    """3x3 K from a CameraInfo.k-like sequence of 9 values."""
    K = np.asarray(k, dtype=float).reshape(3, 3)
    if K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError('camera intrinsics are not calibrated (fx/fy <= 0)')
    return K


def project(K, point_optical):
    x, y, z = point_optical
    if z <= 1e-6:
        return None
    return (K[0, 0] * x / z + K[0, 2], K[1, 1] * y / z + K[1, 2])


def backproject(K, u, v, z):
    return np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z])
