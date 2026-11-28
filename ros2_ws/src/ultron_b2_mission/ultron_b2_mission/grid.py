"""2D occupancy grid helpers for mission planning (numpy; nav_msgs/OccupancyGrid-compatible layout).

Cell values: -1 unknown, 0..100 occupancy. ``free`` = 0..free_max, ``occupied`` = >= occupied_min;
values in between are neither free nor confirmed occupied and are treated as not traversable.
Thresholds PROPOSED (slam_toolbox writes 0 / 100 / -1).
"""
from collections import deque
import math

import numpy as np


def point_in_polygon(x, y, polygon):
    inside = False
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def polygon_mask(polygon, xs, ys):
    """Vectorised point-in-polygon for cell-centre coordinate arrays."""
    inside = np.zeros(xs.shape, dtype=bool)
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        if ay == by:
            continue
        cond = (ay > ys) != (by > ys)
        xint = ax + (ys - ay) * (bx - ax) / (by - ay)
        inside ^= cond & (xs < xint)
    return inside


def disk_offsets(radius_cells):
    r = int(math.ceil(radius_cells))
    return [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1) if dx * dx + dy * dy <= radius_cells ** 2]


def dilate(mask, radius_cells):
    if radius_cells <= 0:
        return mask.copy()
    h, w = mask.shape
    out = mask.copy()
    for dy, dx in disk_offsets(radius_cells):
        ys0, ys1 = max(0, dy), min(h, h + dy)
        xs0, xs1 = max(0, dx), min(w, w + dx)
        out[ys0:ys1, xs0:xs1] |= mask[ys0 - dy:ys1 - dy, xs0 - dx:xs1 - dx]
    return out


class GridMap:
    def __init__(self, data, resolution, origin_xy, *, free_max=25, occupied_min=65, frame_id='map'):
        self.data = np.asarray(data, dtype=np.int16)
        if self.data.ndim != 2:
            raise ValueError('grid data must be 2D (height, width)')
        self.resolution = float(resolution)
        self.origin = (float(origin_xy[0]), float(origin_xy[1]))
        self.frame_id = frame_id
        self.height, self.width = self.data.shape
        self.unknown = self.data < 0
        self.free = (self.data >= 0) & (self.data <= free_max)
        self.occupied = self.data >= occupied_min
        jj, ii = np.meshgrid(np.arange(self.width), np.arange(self.height))
        self.xs = self.origin[0] + (jj + 0.5) * self.resolution
        self.ys = self.origin[1] + (ii + 0.5) * self.resolution
        self._clear_cache = {}

    @classmethod
    def from_msg(cls, msg, **kw):
        q = msg.info.origin.orientation
        if abs(q.z) > 1e-6 or abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
            raise ValueError('rotated OccupancyGrid origins are not supported')
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        return cls(data, msg.info.resolution, (msg.info.origin.position.x, msg.info.origin.position.y),
                   frame_id=msg.header.frame_id or 'map', **kw)

    def cell(self, x, y):
        j = int(math.floor((x - self.origin[0]) / self.resolution))
        i = int(math.floor((y - self.origin[1]) / self.resolution))
        if 0 <= i < self.height and 0 <= j < self.width:
            return i, j
        return None

    def centre(self, i, j):
        return (self.origin[0] + (j + 0.5) * self.resolution, self.origin[1] + (i + 0.5) * self.resolution)

    def cell_area(self):
        return self.resolution ** 2

    def traversable(self, clearance_m):
        """Free cells whose disk of radius clearance contains only known-free cells."""
        key = round(clearance_m, 3)
        if key not in self._clear_cache:
            blocked = ~self.free
            self._clear_cache[key] = self.free & ~dilate(blocked, clearance_m / self.resolution)
        return self._clear_cache[key]

    def reachable_from(self, xy, clearance_m, *, snap_m=0.5):
        """Boolean mask of traversable cells 8-connected to the cell nearest ``xy`` (snapped)."""
        trav = self.traversable(clearance_m)
        start = self._snap(xy, trav, snap_m)
        reach = np.zeros_like(trav)
        if start is None:
            return reach
        q = deque([start])
        reach[start] = True
        h, w = trav.shape
        while q:
            i, j = q.popleft()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    a, b = i + di, j + dj
                    if 0 <= a < h and 0 <= b < w and trav[a, b] and not reach[a, b]:
                        reach[a, b] = True
                        q.append((a, b))
        return reach

    def _snap(self, xy, mask, snap_m):
        c = self.cell(*xy)
        if c is not None and mask[c]:
            return c
        r = int(math.ceil(snap_m / self.resolution))
        if c is None:
            return None
        best = None
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                a, b = c[0] + di, c[1] + dj
                if 0 <= a < self.height and 0 <= b < self.width and mask[a, b]:
                    d = di * di + dj * dj
                    if best is None or d < best[0]:
                        best = (d, (a, b))
        return None if best is None else best[1]

    def is_reachable(self, reach, xy, snap_m=0.3):
        return self._snap(xy, reach, snap_m) is not None

    def line_of_sight(self, a, b, *, ignore_radius_m=0.0):
        """True if the segment a->b crosses no occupied cell (cells near b within ignore radius skipped)."""
        dist = math.dist(a, b)
        steps = max(2, int(dist / (0.5 * self.resolution)))
        for k in range(steps + 1):
            t = k / steps
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            if math.hypot(x - b[0], y - b[1]) <= ignore_radius_m:
                continue
            c = self.cell(x, y)
            if c is None or self.occupied[c]:
                return False
        return True
