"""Place a detection in the map: range from depth image or lidar, bearing from the camera model.

Needs the robot pose on the frame and a `Calibration` measured on the real robot. Without a
range sensor hit the object is placed at an assumed range and marked approximate.
"""
import math

import numpy as np
from PIL import Image, ImageDraw

from .catalog import KINDS
from .types import Sighting
from ..protocol.lidar import BOX_CORE, MIN_POINTS, MIN_RANGE, NEAR_PERCENTILE, locate

ASSUMED_RANGE = 2.5  # metres, used only when neither depth nor lidar hits the box
OUTLINE_PERCENTILE = 35  # of depths inside the outline: thin rails let some background through
CELL = 0.1          # metres: floor grid of object footprints
COLUMN_BAND = 0.25  # metres: per image column, pixels this far behind the nearest one are background seen through gaps


def _depth_range(depth, box):
    """Range to the object from an aligned depth image: the near surface in the middle of the box."""
    height, width = depth.shape
    x0, y0, x1, y1 = box[0] * width / 1000, box[1] * height / 1000, box[2] * width / 1000, box[3] * height / 1000
    mx, my = (x1 - x0) * (1 - BOX_CORE) / 2, (y1 - y0) * (1 - BOX_CORE) / 2
    core = depth[int(y0 + my):int(math.ceil(y1 - my)), int(x0 + mx):int(math.ceil(x1 - mx))]
    valid = core[np.isfinite(core) & (core > MIN_RANGE)]
    return float(np.percentile(valid, NEAR_PERCENTILE)) if valid.size >= MIN_POINTS else None


def _mask(shape, outline):
    height, width = shape
    mask = Image.new("1", (width, height))
    ImageDraw.Draw(mask).polygon([(x * width / 1000, y * height / 1000) for x, y in outline], fill=1)
    return np.asarray(mask, dtype=bool)


def _outline_range(depth, outline):
    """Range to the object from the depth pixels inside its outline."""
    inside = depth[_mask(depth.shape, outline)]
    valid = inside[np.isfinite(inside) & (inside > MIN_RANGE)]
    return float(np.percentile(valid, OUTLINE_PERCENTILE)) if valid.size >= MIN_POINTS else None


def footprint(depth, outline, calibration, pose, kind, distance):
    """Floor cells covered by the object's visible surface, and their centroid, in the map frame.

    Every depth pixel inside the outline is placed on the floor. Pixels more than the kind's extent
    nearer or further than the object's measured `distance` belong to something else (a ladder in
    front, the far wall). Of the rest, pixels further than COLUMN_BAND behind the nearest pixel of
    their own image column are background seen past the object (a ladder is mostly gaps).
    Returns (cells, (x, y)) or (None, None) with too few pixels.
    """
    rows, columns = np.nonzero(_mask(depth.shape, outline) & np.isfinite(depth) & (depth > MIN_RANGE))
    z = depth[rows, columns]
    keep = np.abs(z - distance) <= KINDS[kind].extent
    columns, z = columns[keep], z[keep]
    order = np.lexsort((z, columns))
    columns, z = columns[order], z[order]
    starts = np.unique(columns, return_index=True)[1]
    front = np.repeat(z[starts], np.diff(np.append(starts, len(z))))
    keep = z <= front + COLUMN_BAND
    columns, z = columns[keep], z[keep]
    if z.size < MIN_POINTS:
        return None, None
    # Depth is registered to the colour camera, possibly at a lower resolution than the calibration.
    u = (columns + 0.5) * calibration.width / depth.shape[1]
    left = -(u - calibration.cx) / calibration.fx * z
    x, y, yaw = pose
    xs = x + z * math.cos(yaw) - left * math.sin(yaw)
    ys = y + z * math.sin(yaw) + left * math.cos(yaw)
    cells = np.unique(np.stack([np.floor(xs / CELL), np.floor(ys / CELL)], axis=1).astype(int), axis=0)
    return cells.tolist(), (float(xs.mean()), float(ys.mean()))


def _along_bearing(frame, box, calibration, distance):
    """Map position `distance` metres along the ray through the box centre."""
    u = (box[0] + box[2]) / 2000 * calibration.width
    angle = -math.atan2(u - calibration.cx, calibration.fx)  # image right is robot's right: negative yaw
    x, y, yaw = frame.pose
    return (x + distance * math.cos(yaw + angle), y + distance * math.sin(yaw + angle))


def _zband(box, calibration, distance):
    """Heights relative to the camera that the box spans at `distance`; None for a box cut off
    at the top or bottom of the image, which shows only part of the object."""
    if box[1] <= 10 or box[3] >= 990:
        return None
    return tuple(round((calibration.cy - v / 1000 * calibration.height) / calibration.fy * distance, 3)
                 for v in (box[3], box[1]))


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
        if detection.outline and distance:
            cells, centre = footprint(depth, detection.outline, calibration, frame.pose, detection.kind, distance)
            if cells:
                # The middle of the visible surface, not a point along the box's centre ray.
                return Sighting(detection, centre, distance, height=_box_height(detection.box, calibration, distance),
                                cells=cells, zband=_zband(detection.box, calibration, distance))
    if distance:
        return Sighting(detection, _along_bearing(frame, detection.box, calibration, distance), distance,
                        height=_box_height(detection.box, calibration, distance))
    return Sighting(detection, _along_bearing(frame, detection.box, calibration, ASSUMED_RANGE), ASSUMED_RANGE, True)
