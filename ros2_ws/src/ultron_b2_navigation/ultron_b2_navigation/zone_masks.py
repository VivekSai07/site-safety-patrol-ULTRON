"""PatrolZone -> keepout / speed filter masks (no ROS). PROPOSED values; verify on site.

Keepout mask: 100 (lethal) outside the zone boundary and inside every keepout polygon, else 0.
Speed mask (nav2 SpeedFilter, percent mode, base 0, multiplier 1): ``slow_percent`` within
``slow_band_m`` of any boundary/keepout edge inside the allowed area, 0 (= no limit) elsewhere,
100 (no reduction; keepout already blocks) in forbidden cells.
"""
from dataclasses import dataclass
import math

import numpy as np

ZONE_SCHEMA = 'ultron.patrol_zone/v1'
SLOW_BAND_M = 1.0          # PROPOSED
SLOW_PERCENT = 50          # PROPOSED: % of max speed near edges


@dataclass
class Grid:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def centers(self):
        xs = self.origin_x + (np.arange(self.width) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(self.height) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)     # row 0 = lowest y (OccupancyGrid order)


def _as_array(poly):
    a = np.asarray(poly, dtype=float).reshape(-1, 2)
    if len(a) < 3 or not np.all(np.isfinite(a)):
        raise ValueError('polygon needs >= 3 finite vertices')
    return a


def points_in_polygon(x, y, poly):
    """Even-odd rule, vectorized over arrays x, y."""
    p = _as_array(poly)
    inside = np.zeros(np.shape(x), dtype=bool)
    j = len(p) - 1
    for i in range(len(p)):
        xi, yi = p[i]
        xj, yj = p[j]
        cross = (yi > y) != (yj > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            xint = (xj - xi) * (y - yi) / (yj - yi) + xi
        inside ^= cross & (x < xint)
        j = i
    return inside


def distance_to_edges(x, y, polys):
    d = np.full(np.shape(x), np.inf)
    for poly in polys:
        p = _as_array(poly)
        for a, b in zip(p, np.roll(p, -1, axis=0)):
            ab = b - a
            L2 = float(ab @ ab) or 1e-12
            t = np.clip(((x - a[0]) * ab[0] + (y - a[1]) * ab[1]) / L2, 0.0, 1.0)
            d = np.minimum(d, np.hypot(x - (a[0] + t * ab[0]), y - (a[1] + t * ab[1])))
    return d


def grid_for_zone(boundary, resolution=0.05, margin_m=2.0):
    p = _as_array(boundary)
    lo, hi = p.min(axis=0) - margin_m, p.max(axis=0) + margin_m
    w, h = (int(math.ceil((hi - lo)[i] / resolution)) for i in (0, 1))
    return Grid(resolution, float(lo[0]), float(lo[1]), w, h)


def make_masks(grid, boundary, keepouts=(), slow_band_m=SLOW_BAND_M, slow_percent=SLOW_PERCENT):
    """Return (keepout int8[h, w], speed int8[h, w]) in OccupancyGrid row order (row 0 = min y)."""
    x, y = grid.centers()
    allowed = points_in_polygon(x, y, boundary)
    for k in keepouts:
        allowed &= ~points_in_polygon(x, y, k)
    keepout = np.where(allowed, 0, 100).astype(np.int8)
    near = distance_to_edges(x, y, [boundary, *keepouts]) <= slow_band_m
    speed = np.where(allowed, np.where(near, slow_percent, 0), 100).astype(np.int8)
    return keepout, speed


def zone_from_dict(d):
    if d.get('schema', ZONE_SCHEMA) != ZONE_SCHEMA:
        raise ValueError(f'schema mismatch: {d.get("schema")!r}')
    boundary = _as_array(d['boundary'])
    keepouts = [_as_array(k) for k in d.get('keepout') or []]
    return {'zone_id': str(d['zone_id']), 'map_id': str(d['map_id']), 'map_version': int(d['map_version']),
            'boundary': boundary, 'keepout': keepouts, 'base_pose': d.get('base_pose')}


def write_pgm(path, mask):
    """Mask (0..100, row 0 = min y) -> PGM in map_server 'scale' convention (100 => black)."""
    from pathlib import Path
    img = (255 - np.round(np.flipud(mask).astype(float) * 2.55)).clip(0, 255).astype(np.uint8)
    h, w = img.shape
    Path(path).write_bytes(f'P5\n{w} {h}\n255\n'.encode() + img.tobytes())
