"""Place a detection in the map: range from depth image or lidar, bearing from the camera model.

Needs the robot pose on the frame and a `Calibration` measured on the real robot. Without a
range sensor hit the object is placed at an assumed range and marked approximate.
"""
import math

import numpy as np

from .types import Sighting
from ..protocol.lidar import BOX_CORE, MIN_POINTS, MIN_RANGE, NEAR_PERCENTILE, locate

ASSUMED_RANGE = 2.5  # metres, used only when neither depth nor lidar hits the box


def _depth_range(depth, box):
    """Range to the object from an aligned depth image: the near surface in the middle of the box."""
    height, width = depth.shape
    x0, y0, x1, y1 = box[0] * width / 1000, box[1] * height / 1000, box[2] * width / 1000, box[3] * height / 1000
    mx, my = (x1 - x0) * (1 - BOX_CORE) / 2, (y1 - y0) * (1 - BOX_CORE) / 2
    core = depth[int(y0 + my):int(math.ceil(y1 - my)), int(x0 + mx):int(math.ceil(x1 - mx))]
    valid = core[np.isfinite(core) & (core > MIN_RANGE)]
    return float(np.percentile(valid, NEAR_PERCENTILE)) if valid.size >= MIN_POINTS else None


def _along_bearing(frame, box, calibration, distance):
    """Map position `distance` metres along the ray through the box centre."""
    u = (box[0] + box[2]) / 2000 * calibration.width
    angle = -math.atan2(u - calibration.cx, calibration.fx)  # image right is robot's right: negative yaw
    x, y, yaw = frame.pose
    return (x + distance * math.cos(yaw + angle), y + distance * math.sin(yaw + angle))


def lift(frame, detection, calibration=None):
    """A `Sighting` for this detection; without pose or calibration only the detection is kept."""
    if frame.pose is None or calibration is None:
        return Sighting(detection)
    if frame.points is not None:
        position = locate(frame.points, detection.box, calibration, frame.pose)
        if position:
            return Sighting(detection, position, math.dist(position, frame.pose[:2]))
    depth = getattr(frame, "depth", None)
    distance = _depth_range(np.asarray(depth, dtype=float), detection.box) if depth is not None else None
    if distance:
        return Sighting(detection, _along_bearing(frame, detection.box, calibration, distance), distance)
    return Sighting(detection, _along_bearing(frame, detection.box, calibration, ASSUMED_RANGE), ASSUMED_RANGE, True)
