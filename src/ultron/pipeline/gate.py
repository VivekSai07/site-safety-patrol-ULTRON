"""Goal gate: the pipeline sends a goal only where the robot may and can stand.

    inside the fence     operator-drawn room polygon, by at least the robot's clearance
                         (lidar sees through windows: SLAM calls the space beyond them free)
    known free           every map cell within clearance is free in the latest map, none unknown
    reachable            connected to the robot through free space at that clearance
    clear of objects     no known object's footprint within clearance + margin (a ladder on the
                         floor is a footprint like any other; Nav2's costmap may not see it)

A rejected goal is not sent; the caller tries another side. `Gate` is the `is_free(x, y)` of
`coverage.missing_views`, and `check(x, y)` says why not. Also: `Fence.contains` drops sightings
outside the room before they become objects (reflections, things seen through glass).
See docs/EXPLORATION_CONTRACT.md.
"""
import io
import json
import math
from pathlib import Path
import time
import urllib.error
import urllib.request

import numpy as np

from .lift import CELL
from .objects import footprint

FREE_AT_LEAST = 250   # PNG grey from the dashboard: free 254, unknown 205, occupied 0


def point_in_polygon(polygon, x, y):
    inside = False
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def distance_to_edges(polygon, x, y):
    best = math.inf
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = x1 - x0, y1 - y0
        t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(x - x0 - t * dx, y - y0 - t * dy))
    return best


class Fence:
    """The room polygon in the map frame."""

    def __init__(self, polygon, frame_id="map"):
        self.polygon = [(float(x), float(y)) for x, y in polygon]
        if len(self.polygon) < 3:
            raise ValueError("a fence needs at least 3 points")
        self.frame_id = frame_id

    @classmethod
    def from_json(cls, data):
        return cls(data["polygon"], data.get("frame_id", "map"))

    @classmethod
    def load(cls, path):
        return cls.from_json(json.loads(Path(path).read_text()))

    def contains(self, x, y, margin=0.0):
        return point_in_polygon(self.polygon, x, y) and (margin <= 0 or distance_to_edges(self.polygon, x, y) >= margin)


class GridMap:
    """The dashboard's map: a greyscale PNG with row 0 at the highest y (as `ros_link.grid_png`)."""

    def __init__(self, pixels, resolution, origin, seq=None):
        self.free = np.asarray(pixels) >= FREE_AT_LEAST
        self.free = self.free[::-1]          # row 0 = lowest y from here on, as the OccupancyGrid
        self.resolution, self.origin, self.seq = float(resolution), (float(origin[0]), float(origin[1])), seq
        self._reach = {}                      # clearance cells -> (labels of the eroded free space)

    @classmethod
    def from_png(cls, data, resolution, origin, seq=None):
        from PIL import Image
        with Image.open(io.BytesIO(data)) as image:
            return cls(np.asarray(image.convert("L")), resolution, origin, seq)

    def cell(self, x, y):
        return int(math.floor((y - self.origin[1]) / self.resolution)), int(math.floor((x - self.origin[0]) / self.resolution))

    def _labels(self, radius):
        cells = max(0, int(math.ceil(radius / self.resolution)))
        if cells not in self._reach:
            from scipy import ndimage
            disk = np.hypot(*np.mgrid[-cells:cells + 1, -cells:cells + 1]) <= cells
            standable = ndimage.binary_erosion(self.free, structure=disk, border_value=0) if cells else self.free
            self._reach[cells] = ndimage.label(standable, structure=np.ones((3, 3)))[0]
        return self._reach[cells]

    def standable(self, x, y, radius):
        row, col = self.cell(x, y)
        labels = self._labels(radius)
        return 0 <= row < labels.shape[0] and 0 <= col < labels.shape[1] and labels[row, col] > 0

    def reachable(self, start, x, y, radius):
        """Both points standable and connected through standable space. A robot standing where
        the map is not yet free (just started, pose noise) counts from its nearest standable cell."""
        labels = self._labels(radius)
        row, col = self.cell(x, y)
        if not (0 <= row < labels.shape[0] and 0 <= col < labels.shape[1]) or labels[row, col] == 0:
            return False
        r0, c0 = self.cell(*start[:2])
        window = labels[max(0, r0 - 10):r0 + 11, max(0, c0 - 10):c0 + 11]
        here = set(np.unique(window[window > 0]).tolist())
        return int(labels[row, col]) in here


class DashboardMap:
    """The latest map from the dashboard (`/api/map` + `/api/map.png`), fetched again when its
    sequence number changes, at most every `refresh` seconds. None until the dashboard has one."""

    def __init__(self, url, refresh=2.0, clock=time.monotonic):
        self.url, self.refresh, self.clock = url.rstrip("/"), refresh, clock
        self._map, self._checked = None, -math.inf

    def _get(self, path):
        with urllib.request.urlopen(self.url + path, timeout=5) as response:
            return response.read()

    def __call__(self):
        if self.clock() - self._checked < self.refresh:
            return self._map
        self._checked = self.clock()
        try:
            meta = json.loads(self._get("/api/map") or b"null")
            if meta and (self._map is None or self._map.seq != meta["seq"]):
                self._map = GridMap.from_png(self._get("/api/map.png"), meta["resolution"], meta["origin"], meta["seq"])
        except (urllib.error.URLError, OSError, ValueError, KeyError):
            pass  # keep the last map: a dashboard hiccup must not stop the patrol
        return self._map


class Gate:
    def __init__(self, fence=None, grid=None, objects=None, clearance=0.45, object_margin=0.25, require_map=False,
                 also=None):
        self.fence = fence                  # Fence or None
        self.grid = grid                    # callable -> GridMap or None (DashboardMap), or None
        self.objects = objects              # ObjectMap: footprints of known objects are kept clear
        self.clearance = clearance          # robot half-width plus a margin, metres
        self.object_margin = object_margin  # extra distance to keep from an object's footprint
        self.require_map = require_map      # no map yet: refuse (True) or judge without one (False)
        self.also = also                    # another is_free(x, y), e.g. a saved map's footprint check
        self.robot = None                   # robot (x, y, yaw), set by the caller each frame
        self.reason = ""                    # why the last check failed

    def __call__(self, x, y):
        return self.check(x, y)[0]

    def informed(self):
        """Its refusals say something about the place (it has the map it needs)."""
        return self.grid is None or self.grid() is not None

    def check(self, x, y):
        ok, self.reason = self._check(x, y)
        return ok, self.reason

    def _check(self, x, y):
        if self.fence and not self.fence.contains(x, y, self.clearance):
            return False, "outside the fence"
        grid = self.grid() if self.grid else None
        if grid is None and self.require_map:
            return False, "no map yet"
        if grid is not None:
            if not grid.standable(x, y, self.clearance):
                return False, "not known free in the map"
            if self.robot is not None and not grid.reachable(self.robot, x, y, self.clearance):
                return False, "not reachable from the robot"
        if self.also and not self.also(x, y):
            return False, "not free in the saved map"
        near = self.too_close(x, y)
        if near:
            return False, f"too close to {near}"
        return True, ""

    def too_close(self, x, y):
        if self.objects is None:
            return None
        keep = self.clearance + self.object_margin
        for site_object in self.objects.all():
            if site_object.status in ("REJECTED", "MOVED"):
                continue
            cells = footprint(site_object)
            if len(cells):
                centres = (cells + 0.5) * CELL
                if np.hypot(centres[:, 0] - x, centres[:, 1] - y).min() < keep:
                    return site_object.id
            elif site_object.position and math.dist(site_object.position, (x, y)) < keep:
                return site_object.id
        return None


def obstacle_cells(objects):
    """Footprint cell centres of every known object, for the navigation stack's obstacle layer."""
    cells = []
    for site_object in objects.all():
        if site_object.status in ("REJECTED", "MOVED"):
            continue
        found = footprint(site_object)
        if len(found):
            cells.extend(((found + 0.5) * CELL).round(3).tolist())
        elif site_object.position:  # no footprint yet: a disc of 0.3 m around its position
            x, y = site_object.position
            cells.extend([round(x + dx * CELL, 3), round(y + dy * CELL, 3)]
                         for dx in range(-3, 4) for dy in range(-3, 4) if dx * dx + dy * dy <= 9)
    return cells
