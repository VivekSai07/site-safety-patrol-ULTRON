"""Small pose helpers shared by the odometry nodes (numpy only, no ROS)."""
import math

import numpy as np


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError('zero-norm quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r):
    """3x3 rotation -> (x, y, z, w) with w >= 0."""
    t = np.trace(r)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        w, x, y, z = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w, x, y, z = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    return x, y, z, w


def yaw_of(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def transform_matrix(translation, quat_xyzw):
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(*quat_xyzw)
    m[:3, 3] = translation
    return m


def flatten_pose(m):
    """Project a 4x4 pose onto the ground plane: keep x, y, yaw; z = roll = pitch = 0."""
    yaw = math.atan2(m[1, 0], m[0, 0])
    out = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    out[:2, :2] = [[c, -s], [s, c]]
    out[0, 3], out[1, 3] = m[0, 3], m[1, 3]
    return out


def twist_from_delta(prev, cur, dt):
    """Body-frame (child frame) linear and angular velocity between two 4x4 poses."""
    if dt <= 0:
        return np.zeros(3), np.zeros(3)
    d = np.linalg.inv(prev) @ cur
    angle_axis = rotation_vector(d[:3, :3])
    return d[:3, 3] / dt, angle_axis / dt


def rotation_vector(r):
    cos = max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0))
    angle = math.acos(cos)
    if angle < 1e-9:
        return np.zeros(3)
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return v * angle / (2.0 * math.sin(angle))
