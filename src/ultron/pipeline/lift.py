"""Place a detection in the map: range from depth image or lidar, bearing from the camera model.

Needs the robot pose on the frame and a `Calibration` measured on the real robot. Without a
range sensor hit the object is placed at an assumed range and marked approximate.
"""
import math

import numpy as np
from PIL import Image, ImageDraw

from .types import Sighting
from ..protocol.lidar import BOX_CORE, MIN_POINTS, MIN_RANGE, NEAR_PERCENTILE, locate

ASSUMED_RANGE = 2.5  # metres, used only when neither depth nor lidar hits the box
OUTLINE_PERCENTILE = 35  # of depths inside the outline: thin rails let some background through


def _depth_range(depth, box):
    """Range to the object from an aligned depth image: the near surface in the middle of the box."""
    height, width = depth.shape
    x0, y0, x1, y1 = box[0] * width / 1000, box[1] * height / 1000, box[2] * width / 1000, box[3] * height / 1000
    mx, my = (x1 - x0) * (1 - BOX_CORE) / 2, (y1 - y0) * (1 - BOX_CORE) / 2
    core = depth[int(y0 + my):int(math.ceil(y1 - my)), int(x0 + mx):int(math.ceil(x1 - mx))]
    valid = core[np.isfinite(core) & (core > MIN_RANGE)]
    return float(np.percentile(valid, NEAR_PERCENTILE)) if valid.size >= MIN_POINTS else None


def _outline_range(depth, outline):
    """Range to the object from the depth pixels inside its outline."""
    height, width = depth.shape
    mask = Image.new("1", (width, height))
    ImageDraw.Draw(mask).polygon([(x * width / 1000, y * height / 1000) for x, y in outline], fill=1)
    inside = depth[np.asarray(mask, dtype=bool)]
    valid = inside[np.isfinite(inside) & (inside > MIN_RANGE)]
    return float(np.percentile(valid, OUTLINE_PERCENTILE)) if valid.size >= MIN_POINTS else None


def _along_bearing(frame, box, calibration, distance):
    """Map position `distance` metres along the ray through the box centre."""
    u = (box[0] + box[2]) / 2000 * calibration.width
    angle = -math.atan2(u - calibration.cx, calibration.fx)  # image right is robot's right: negative yaw
    x, y, yaw = frame.pose
    return (x + distance * math.cos(yaw + angle), y + distance * math.sin(yaw + angle))


def _box_height(box, calibration, distance):
    """Height in metres of what the box spans at `distance`."""
    return (box[3] - box[1]) / 1000 * calibration.height / calibration.fy * distance


def lift(frame, detection, calibration=None):
    """A `Sighting` for this detection; without pose or calibration only the detection is kept."""
    if frame.pose is None or calibration is None:
        return Sighting(detection)
    if frame.points is not None:
        position = locate(frame.points, detection.box, calibration, frame.pose)
        if position:
            distance = math.dist(position, frame.pose[:2])
            return Sighting(detection, position, distance, height=_box_height(detection.box, calibration, distance))
    depth = getattr(frame, "depth", None)
    distance = None
    if depth is not None:
        depth = np.asarray(depth, dtype=float)
        distance = (_outline_range(depth, detection.outline) if detection.outline else None) or _depth_range(depth, detection.box)
    if distance:
        return Sighting(detection, _along_bearing(frame, detection.box, calibration, distance), distance,
                        height=_box_height(detection.box, calibration, distance))
    return Sighting(detection, _along_bearing(frame, detection.box, calibration, ASSUMED_RANGE), ASSUMED_RANGE, True)
