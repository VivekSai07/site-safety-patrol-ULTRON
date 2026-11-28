"""Shared occupancy grid conventions for online SLAM navigation in simulation."""

import math

import numpy as np

from .localization_state import MapMatcher

TOPIC = '/map'
MAX_AGE_S = 2.5  # Mapping publishes at 1 Hz in simulation.
MIN_MAPPED_ENDPOINT_FRACTION = 0.55  # Proposed simulation-only scan support threshold.
NANOSECONDS = 1_000_000_000


def grid_stamp_ns(msg):
    stamp = msg.header.stamp
    return stamp.sec * NANOSECONDS + stamp.nanosec


def grid_is_fresh(stamp_ns, now_s):
    return stamp_ns > 0 and -0.001 <= now_s - stamp_ns / NANOSECONDS <= MAX_AGE_S


def grid_cells(msg):
    """Return lower-left-origin cells, resolution and origin from SLAM Toolbox."""
    info = msg.info
    point, q = info.origin.position, info.origin.orientation
    if (msg.header.frame_id != 'map' or info.width <= 0 or info.height <= 0
            or len(msg.data) != info.width * info.height
            or not all(map(math.isfinite, (info.resolution, point.x, point.y)))
            or info.resolution <= 0 or (q.x, q.y, q.z) != (0.0, 0.0, 0.0) or abs(q.w) != 1.0):
        raise ValueError('Invalid or rotated live occupancy grid')
    cells = np.asarray(msg.data).reshape(info.height, info.width)
    if ((cells < -1) | (cells > 100)).any():
        raise ValueError('Occupancy cells must range from -1 to 100')
    return cells, info.resolution, np.array([point.x, point.y])


class LiveMapMatcher(MapMatcher):
    """Compare endpoints against mapped space without counting unknown walls as mismatches."""

    def __init__(self, msg, tolerance):
        cells, resolution, origin = grid_cells(msg)
        rows, cols = np.nonzero(cells >= 65)
        occupied = np.column_stack((cols + 0.5, rows + 0.5)) * resolution + origin
        super().__init__(occupied, resolution, tolerance)
        self.known = cells >= 0
        self.grid_origin = origin

    def score(self, pose, endpoints_base):
        if not len(endpoints_base):
            return 0.0
        world, hits = self.match_endpoints(pose, endpoints_base)
        cols, rows = np.floor((world - self.grid_origin) / self.res).astype(int).T
        height, width = self.known.shape
        inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        observed = hits.copy()
        observed[inside] |= self.known[rows[inside], cols[inside]]
        if float(observed.mean()) < MIN_MAPPED_ENDPOINT_FRACTION:
            return 0.0
        return float(hits.sum() / observed.sum())
