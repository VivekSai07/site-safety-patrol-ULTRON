"""Where is the ladder? Lidar points that fall inside the ladder's image box give its range.

Needs a calibration file measured on the real robot (nothing here is assumed):
    {"intrinsics": [fx, fy, cx, cy], "image_size": [width, height],
     "T_cam_lidar": 4x4, "T_base_lidar": 4x4}
`T_cam_lidar` maps lidar points into the camera's optical frame (x right, y down, z forward).
"""
import json
import math

import numpy as np

MIN_RANGE = 0.3     # metres: closer returns are the robot itself
MIN_POINTS = 5
NEAR_PERCENTILE = 25  # the ladder stands in front of whatever is behind it
BOX_CORE = 0.6      # use the middle of the box, where the ladder is rather than the background


class Calibration:
    def __init__(self, intrinsics, image_size, T_cam_lidar, T_base_lidar=None):
        self.fx, self.fy, self.cx, self.cy = map(float, intrinsics)
        self.width, self.height = image_size
        self.T_cam_lidar = np.asarray(T_cam_lidar, dtype=float).reshape(4, 4)
        self.T_base_lidar = np.eye(4) if T_base_lidar is None else np.asarray(T_base_lidar, dtype=float).reshape(4, 4)

    @classmethod
    def load(cls, path):
        with open(path) as handle:
            return cls(**json.load(handle))


def _transform(matrix, points):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def locate(points, box, calibration, pose=None):
    """Ladder position from lidar points inside `box` (thousandths of the image).

    Returns (x, y) in the map frame if `pose` (x, y, yaw) is given, else in the robot's
    base frame. None if too few points hit the box.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    camera = _transform(calibration.T_cam_lidar, points)
    in_front = camera[:, 2] > MIN_RANGE
    points, camera = points[in_front], camera[in_front]
    u = calibration.fx * camera[:, 0] / camera[:, 2] + calibration.cx
    v = calibration.fy * camera[:, 1] / camera[:, 2] + calibration.cy
    x0, y0, x1, y1 = (box[0] * calibration.width / 1000, box[1] * calibration.height / 1000,
                      box[2] * calibration.width / 1000, box[3] * calibration.height / 1000)
    margin_x, margin_y = (x1 - x0) * (1 - BOX_CORE) / 2, (y1 - y0) * (1 - BOX_CORE) / 2
    inside = (u > x0 + margin_x) & (u < x1 - margin_x) & (v > y0 + margin_y) & (v < y1 - margin_y)
    if inside.sum() < MIN_POINTS:
        return None
    depth = camera[inside, 2]
    near = depth <= np.percentile(depth, NEAR_PERCENTILE)
    x, y, _ = np.median(_transform(calibration.T_base_lidar, points[inside][near]), axis=0)
    if pose is None:
        return float(x), float(y)
    px, py, yaw = pose
    return (float(px + x * math.cos(yaw) - y * math.sin(yaw)),
            float(py + x * math.sin(yaw) + y * math.cos(yaw)))
