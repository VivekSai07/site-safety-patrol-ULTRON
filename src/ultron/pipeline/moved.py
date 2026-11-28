"""Is a stored object still where it was? Look through its footprint with the depth image.

An object that is not detected may simply have been missed. It counts as gone only when the
depth image looks right through the place it stood: in the image columns where its footprint
cells should appear, at the heights the object filled, nothing is at that distance. Something
there means it may still be there; something in front of it hides the place and says nothing.
"""
import numpy as np

from ..protocol.lidar import MIN_RANGE
from .lift import CELL, off_floor
from .objects import footprint

GOOD_RANGE = (1.0, 4.5)  # metres: nearer the object may not fit in the image, further depth is too coarse
BEHIND = 0.5        # metres: depth reaching this far past a cell is looking through it
SOMETHING = 0.03    # share of a column's pixels at the cell's distance that means something is there
HIDDEN = 0.5        # share of a column's pixels in front of the cell that hides it
# Stored positions and poses are known to a few decimetres (0.3-0.6 m between the passes of the
# hall recording): a cell is judged over the image columns it may fall into, all inside the image.
UNCERTAIN = 0.4     # metres
IN_VIEW = 0.5       # share of the footprint that must be judged for a verdict
EMPTY = 0.8         # share of judged cells seen through for "empty"
MIN_PIXELS = 5


def look_through(frame, calibration, site_object):
    """"empty", "occupied", or None when this frame cannot tell (out of view, too far, no depth)."""
    depth = getattr(frame, "depth", None)
    if depth is None or frame.pose is None or calibration is None or not site_object.zband:
        return None
    cells = footprint(site_object)
    if not len(cells):
        return None
    depth = np.asarray(depth, dtype=float)
    height, width = depth.shape
    # Floor is not the place: a stored height band can reach below the floor (a box that took in
    # the object's shadow), and then the rows below the object show floor at its distance.
    standing = off_floor(frame, depth, calibration)
    x, y, yaw = frame.pose
    middle = (cells + 0.5) * CELL - (x, y)
    ahead = middle[:, 0] * np.cos(yaw) + middle[:, 1] * np.sin(yaw)
    left = -middle[:, 0] * np.sin(yaw) + middle[:, 1] * np.cos(yaw)
    low, high = site_object.zband
    judged = empty = 0
    for forward, side in zip(ahead, left):
        if not GOOD_RANGE[0] <= forward <= GOOD_RANGE[1]:
            continue
        centre = calibration.cx - calibration.fx * side / forward
        spread = calibration.fx * UNCERTAIN / forward
        first, last = (int(c * width / calibration.width) for c in (centre - spread, centre + spread))
        # Below the camera, a ray at the object's lowest height meets the floor (taken to be there)
        # at the cell; slightly higher rays meet it within BEHIND: those rows show floor, not the place.
        bottom_height = low * forward / (forward + BEHIND) if low < 0 else low
        if bottom_height >= high:
            continue
        top = int((calibration.cy - high * calibration.fy / forward) * height / calibration.height)
        bottom = int((calibration.cy - bottom_height * calibration.fy / forward) * height / calibration.height)
        if first < 0 or last >= width:
            continue  # may lie beside the image
        rows = slice(max(top, 0), min(bottom, height))
        band = depth[rows, first:last + 1]
        band = band[np.isfinite(band) & (band > MIN_RANGE) & standing[rows, first:last + 1]]
        if band.size < MIN_PIXELS:
            continue
        if float((np.abs(band - forward) <= BEHIND).mean()) >= SOMETHING:
            judged += 1         # something stands there
        elif float((band < forward - BEHIND).mean()) < HIDDEN:
            judged += 1         # seen through
            empty += 1
    if judged < IN_VIEW * len(cells):
        return None
    return "empty" if empty >= EMPTY * judged else "occupied"
