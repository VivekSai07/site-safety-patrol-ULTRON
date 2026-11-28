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
# Inside a ladder's outline most pixels may be background seen between rails and rungs (seen 10 m
# away the rails are a pixel or two wide: on the simulated yard a quarter of the outline is ladder).
# The object is a dense peak of the depths: NEAR_SHARE of them within +-WINDOW of one range. Of
# such peaks the nearest is taken that reaches across the outline (the detector drew the outline
# around the object), as background seen through the object does too; something smaller in front,
# caught by a sloppy outline, does not.
NEAR_SHARE = 0.15   # of the outline's pixels
WINDOW = 0.1        # share of the range: depth spread of an object, and coarser depth further away
SPAN = 0.7          # share of the widest-reaching peak's rows and columns a peak must reach
# A mask that takes in floor (the object's shadow, the ground around its feet) gives depths from
# the camera to the object and beyond. The floor is found as a plane in the lower half of each
# depth image and left out.
FLOOR_BAND = 0.08   # metres above the floor plane still taken as floor
FLOOR_FIT = 0.03    # metres from the plane that a floor pixel lies within
FLOOR_TILT = 0.2    # largest slope of the floor seen from the camera (about 11 degrees of pitch or roll)
FLOOR_SHARE = 0.2   # of the lower half's pixels that must lie on it
CELL = 0.1          # metres: floor grid of object footprints
COLUMN_BAND = 0.25  # metres: per image column, pixels this far behind the nearest one are background seen through gaps
# A ladder lying on the floor is a strip 5-15 cm high. FLOOR_BAND removes most of it, its box height
# is its length foreshortened (the size check rejects it), and per image column its far end lies
# far behind its near end (COLUMN_BAND would keep 25 cm of a ladder pointing away). For kinds that
# may lie (catalog `lying`), a mask below the horizon with (almost) nothing measured higher than
# LYING_TOP above the floor lies flat. Its footprint is where the mask's rays meet a plane LYING_RAY
# above the floor: that needs the floor plane, not depth on thin, shiny rails or blurred by stereo.
LYING_TOP = 0.4     # metres above the floor: a ladder flat on the floor stays far below this ...
LYING_HIGH = 0.1    # ... share of the mask's pixels allowed above it (or above the horizon)
LYING_RAY = 0.05    # metres above the floor the rays are cut at: about half a rail's height
LYING_FAR = 10.0    # metres: rays meeting the floor further away graze it, too coarse to place
# Floor pixels about the fitted plane on the hall recording (71 frames): sd 0.9 cm at 1-2 m, 1.4 cm
# at 2-3 m, 2.6 cm at 4-6 m, mode +0.5 cm out to 8 m.
RAISED = 0.04       # metres over the floor plane: the object, not the floor
NEARER = 0.8        # share of a ray's floor range: a pixel measured nearer is something in front (a wall past its end)


def _depth_range(depth, box):
    """Range to the object from an aligned depth image: the near surface in the middle of the box."""
    height, width = depth.shape
    x0, y0, x1, y1 = box[0] * width / 1000, box[1] * height / 1000, box[2] * width / 1000, box[3] * height / 1000
    mx, my = (x1 - x0) * (1 - BOX_CORE) / 2, (y1 - y0) * (1 - BOX_CORE) / 2
    core = depth[int(y0 + my):int(math.ceil(y1 - my)), int(x0 + mx):int(math.ceil(x1 - mx))]
    valid = core[np.isfinite(core) & (core > MIN_RANGE)]
    return float(np.percentile(valid, NEAR_PERCENTILE)) if valid.size >= MIN_POINTS else None


def _camera_points(depth, calibration, rows, columns):
    """(x, y, z) in the colour camera (x right, y down) of depth pixels at `rows`, `columns`."""
    z = depth[rows, columns]
    u = (columns + 0.5) * calibration.width / depth.shape[1]
    v = (rows + 0.5) * calibration.height / depth.shape[0]
    return (u - calibration.cx) / calibration.fx * z, (v - calibration.cy) / calibration.fy * z, z


def floor_plane(depth, calibration, tries=60):
    """(a, b, c): the floor at y = a*x + b*z + c in the camera (y down, c its height), or None."""
    height, width = depth.shape
    rows, columns = (g.ravel() for g in np.mgrid[height // 2:height:4, 0:width:4])
    valid = np.isfinite(depth[rows, columns]) & (depth[rows, columns] > MIN_RANGE)
    if valid.sum() < 50:
        return None
    x, y, z = _camera_points(depth, calibration, rows[valid], columns[valid])
    design = np.stack([x, z, np.ones_like(z)], axis=1)
    random = np.random.default_rng(0)
    best = None
    for _ in range(tries):
        pick = random.choice(len(z), 3, replace=False)
        try:
            a, b, c = np.linalg.solve(design[pick], y[pick])
        except np.linalg.LinAlgError:
            continue
        if abs(a) > FLOOR_TILT or abs(b) > FLOOR_TILT or c <= 0.1:
            continue
        inliers = np.abs(design @ (a, b, c) - y) < FLOOR_FIT
        if best is None or inliers.sum() > best.sum():
            best = inliers
    if best is None or best.sum() < FLOOR_SHARE * len(z):
        return None
    return tuple(np.linalg.lstsq(design[best], y[best], rcond=None)[0])


def frame_floor(frame, depth, calibration):
    """`floor_plane` of the frame's depth image, computed once per frame."""
    cached = getattr(frame, "_floor_plane", False)
    if cached is not False and getattr(frame, "_floor_shape", None) == depth.shape:
        return cached
    plane = floor_plane(depth, calibration)
    try:
        frame._floor_plane, frame._floor_shape = plane, depth.shape
    except AttributeError:
        pass
    return plane


def off_floor(frame, depth, calibration):
    """Pixels of `depth` that are not floor (all of them when no floor is seen); kept on the frame."""
    cached = getattr(frame, "_off_floor", None)
    if cached is not None and cached.shape == depth.shape:
        return cached
    keep = np.ones(depth.shape, dtype=bool)
    plane = frame_floor(frame, depth, calibration)
    if plane is not None:
        rows, columns = np.nonzero(np.isfinite(depth) & (depth > MIN_RANGE))
        x, y, z = _camera_points(depth, calibration, rows, columns)
        a, b, c = plane
        keep[rows, columns] = a * x + b * z + c - y > FLOOR_BAND
    try:
        frame._off_floor = keep
    except AttributeError:
        pass
    return keep


def _mask(shape, outline):
    height, width = shape
    mask = Image.new("1", (width, height))
    ImageDraw.Draw(mask).polygon([(x * width / 1000, y * height / 1000) for x, y in outline], fill=1)
    return np.asarray(mask, dtype=bool)


def _outline_range(depth, outline, keep=None):
    """Range to the object from the depth pixels inside its outline (and `keep`): the median of
    the depths around the nearest dense range that reaches across the outline."""
    mask = _mask(depth.shape, outline) & np.isfinite(depth) & (depth > MIN_RANGE)
    rows, columns = np.nonzero(mask if keep is None else mask & keep)
    if rows.size < MIN_POINTS:
        return None
    depths = depth[rows, columns]
    peaks = []  # (nearest depth, reach, pixels)
    for low, high in _peaks(depths):
        inside = (depths >= low) & (depths < high)
        reach = min(len(np.unique(rows[inside])) / len(np.unique(rows)),
                    len(np.unique(columns[inside])) / len(np.unique(columns)))
        peaks.append((low, reach, depths[inside]))
    widest = max(reach for _, reach, _ in peaks)
    return float(np.median(min((p for p in peaks if p[1] >= SPAN * widest), key=lambda p: p[0])[2]))


def _peaks(depths, share=NEAR_SHARE):
    """(low, high) depth windows (+-WINDOW, relative) around each dense peak of `depths`; the
    densest one when none holds `share` of them."""
    logs = np.log(depths)
    width = WINDOW / 5
    bins = ((logs - logs.min()) / width).astype(int)
    mass = np.convolve(np.bincount(bins), np.ones(11, dtype=int), "same")  # +-5 bins: +-WINDOW
    top = (mass >= np.append(0, mass[:-1])) & (mass >= np.append(mass[1:], 0))
    found = np.nonzero(top & (mass >= share * depths.size))[0]
    found = found if found.size else [int(np.argmax(mass))]
    return [(math.exp(logs.min() + (i - 5) * width), math.exp(logs.min() + (i + 6) * width)) for i in found]


def footprint(depth, outline, calibration, pose, kind, distance, keep=None):
    """Floor cells covered by the object's visible surface, and their centroid, in the map frame.

    Every depth pixel inside the outline is placed on the floor. Pixels more than the kind's extent
    nearer or further than the object's measured `distance` belong to something else (a ladder in
    front, the far wall). Of the rest, pixels further than COLUMN_BAND behind the nearest pixel of
    their own image column are background seen past the object (a ladder is mostly gaps).
    Only pixels in `keep` (default: all) count: see `off_floor`.
    Returns (cells, (x, y)) or (None, None) with too few pixels.
    """
    mask = _mask(depth.shape, outline) & np.isfinite(depth) & (depth > MIN_RANGE)
    rows, columns = np.nonzero(mask if keep is None else mask & keep)
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


def flat_on_floor(frame, depth, detection, calibration):
    """(cells, centre, distance, zband) of an object lying flat on the floor, or None when its mask
    says otherwise (or its kind never lies, or no floor is seen). zband is floor to measured top,
    relative to the camera."""
    if not KINDS[detection.kind].lying or not detection.outline:
        return None
    plane = frame_floor(frame, depth, calibration)
    if plane is None:
        return None
    a, b, c = plane
    rows, columns = np.nonzero(_mask(depth.shape, detection.outline))
    if rows.size < MIN_POINTS:
        return None
    u = (columns + 0.5) * calibration.width / depth.shape[1]
    v = (rows + 0.5) * calibration.height / depth.shape[0]
    right, below = (u - calibration.cx) / calibration.fx, (v - calibration.cy) / calibration.fy
    descent = below - a * right - b  # > 0: the ray goes down to the floor
    if (descent <= 0).mean() > LYING_HIGH:
        return None  # reaches above the horizon: higher than the camera, so standing
    z = depth[rows, columns]
    measured = np.isfinite(z) & (z > MIN_RANGE)
    if measured.sum() < MIN_POINTS:
        return None
    with np.errstate(divide="ignore", invalid="ignore"):
        above = np.where(measured, (a * right + b - below) * z + c, np.nan)  # height over the floor
        reach = np.where(descent > 0, (c - LYING_RAY) / descent, np.inf)
    if (above[measured] > LYING_TOP).mean() > LYING_HIGH:
        return None
    # Grazing rays make the plane's range sensitive to its tilt (at 6 m, 0.5 degrees of pitch is
    # 0.4 m), so a measured depth that agrees with the plane is used instead; one that disagrees is
    # something else in the mask (a wall past its end, a leg in front). Of the pixels that agree,
    # those raised off the floor are the object: the floor a mask takes in past its far edge lies
    # up to 0.4 m behind it at 4 m. Without such pixels (stereo blurred the rails into the floor)
    # every agreeing pixel counts, and pixels without depth fall back to the plane.
    agrees = measured & (z >= NEARER * reach) & (z <= reach / NEARER)
    raised = agrees & (above >= RAISED)
    usable = (reach > MIN_RANGE) & (reach <= LYING_FAR)
    hit = usable & ((raised if (usable & raised).sum() >= MIN_POINTS else agrees) | ~measured)
    if hit.sum() < MIN_POINTS:
        return None
    # One scale for the whole mask: per pixel, floor depth is too noisy at range (sd 2.6 cm in
    # height at 4-6 m on the hall floor is about 16 cm in depth).
    if agrees.sum() >= MIN_POINTS:
        reach = reach * float(np.median(z[agrees] / reach[agrees]))
    reach = reach[hit]
    left = -right[hit] * reach
    x, y, yaw = frame.pose
    xs = x + reach * math.cos(yaw) - left * math.sin(yaw)
    ys = y + reach * math.sin(yaw) + left * math.cos(yaw)
    cells = np.unique(np.stack([np.floor(xs / CELL), np.floor(ys / CELL)], axis=1).astype(int), axis=0)
    top = max(float(np.percentile(above[measured], 95)), LYING_RAY)
    return (cells.tolist(), (float(xs.mean()), float(ys.mean())), float(np.median(reach)),
            (round(-c, 3), round(top - c, 3)))


def lying(site_object):
    """Whether a stored object lies on the floor: its measured height band is lower than its kind
    stands (an upright one shorter than that never passes the size check)."""
    kind, band = KINDS.get(site_object.kind), site_object.zband
    return bool(kind and kind.lying and band and band[1] - band[0] < kind.size[0])


# A second line behind the operator's fence (docs/EXPLORATION_CONTRACT.md) against detections on
# glass. On the hall recording the depth camera looks through an interior window and reads 12-18 m
# where the wall around it is 2.5-6 m away. A reflection is matched at its mirror depth, also
# behind the glass, or not at all; then the outline range falls back to an assumed 2.5 m, inside
# the room, where the fence cannot catch it. Every real sighting of the hall recording passes both
# tests (226 with depth, at rate 8).
# Share of the outline with depth: hall outlines 0.43 and up (ladders 0.68 and up); a rendered ladder
# in front of glass that gives no depth 0.42-0.50 (rails, rungs and floor); a reflection on such glass
# about none. Far ladders are mostly gaps, hence the margin.
MIN_VALID = 0.15
DEPTH_WORKS = 0.5   # share of the whole image with depth for that test to apply: else the sensor failed
# A window or glass wall starts above a sill (wall, radiator band). Seen from the robot's camera
# (0.75 m up) through glass that starts higher, a reflected or a far object shows only its upper
# part, and right below its outline is the sill: an upright surface clearly nearer than it. Below
# a real object is floor, its own feet, or the wall it hangs on at its own range.
SILL_BAND = 0.3     # metres below the outline's bottom, at the object's range, that are looked at
SILL_SHARE = 0.5    # share of that band's measured pixels that are upright and nearer
SILL_GAP = 0.5      # metres nearer at least (and SILL_GAP_SHARE of the range)
SILL_GAP_SHARE = 0.15


def doubt(frame, detection, calibration):
    """Why this detection is likely not an object in the room ("no depth", "over a sill"), or None.
    Needs a depth image and an outline; says nothing without them."""
    depth = getattr(frame, "depth", None)
    if depth is None or calibration is None or not detection.outline:
        return None
    depth = np.asarray(depth, dtype=float)
    measured = np.isfinite(depth) & (depth > MIN_RANGE)
    if measured.mean() < DEPTH_WORKS:
        return None
    mask = _mask(depth.shape, detection.outline)
    if not mask.any():
        return None
    if (measured & mask).sum() < MIN_VALID * mask.sum():
        return "no depth"
    keep = off_floor(frame, depth, calibration)
    distance = _outline_range(depth, detection.outline, keep)
    if not distance:
        return None
    rows, columns = np.nonzero(mask)
    height = depth.shape[0]
    bottom = rows.max() + 1
    span = max(2, int(math.ceil(SILL_BAND * calibration.fy / distance * height / calibration.height)))
    if bottom >= height:
        return None  # reaches the image's lower edge: nothing below it in view
    band = (slice(bottom, min(bottom + span, height)), slice(columns.min(), columns.max() + 1))
    seen = measured[band]
    if seen.sum() < MIN_POINTS:
        return None
    nearer = seen & keep[band] & (depth[band] < distance - max(SILL_GAP, SILL_GAP_SHARE * distance))
    return "over a sill" if nearer.sum() >= SILL_SHARE * seen.sum() else None


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
    sighting = _lift(frame, detection, calibration)
    sighting.doubt = doubt(frame, detection, calibration)
    return sighting


def _lift(frame, detection, calibration):
    if frame.points is not None:
        position = locate(frame.points, detection.box, calibration, frame.pose)
        if position:
            distance = math.dist(position, frame.pose[:2])
            return Sighting(detection, position, distance, height=_box_height(detection.box, calibration, distance))
    depth = getattr(frame, "depth", None)
    distance = None
    if depth is not None:
        depth = np.asarray(depth, dtype=float)
        flat = flat_on_floor(frame, depth, detection, calibration)
        if flat:
            # No height: a lying ladder's box height is its length foreshortened, not its size.
            cells, centre, distance, zband = flat
            return Sighting(detection, centre, distance, cells=cells, zband=zband, lying=True)
        keep = off_floor(frame, depth, calibration) if detection.outline else None
        distance = (_outline_range(depth, detection.outline, keep) if detection.outline else None) or _depth_range(depth, detection.box)
        if detection.outline and distance:
            cells, centre = footprint(depth, detection.outline, calibration, frame.pose, detection.kind, distance, keep)
            if cells:
                # The middle of the visible surface, not a point along the box's centre ray.
                return Sighting(detection, centre, distance, height=_box_height(detection.box, calibration, distance),
                                cells=cells, zband=_zband(detection.box, calibration, distance))
    if distance:
        return Sighting(detection, _along_bearing(frame, detection.box, calibration, distance), distance,
                        height=_box_height(detection.box, calibration, distance))
    return Sighting(detection, _along_bearing(frame, detection.box, calibration, ASSUMED_RANGE), ASSUMED_RANGE, True)
