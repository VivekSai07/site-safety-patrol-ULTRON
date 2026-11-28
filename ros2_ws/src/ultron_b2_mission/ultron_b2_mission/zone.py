"""PatrolZone (map frame): boundary polygon, keep-out polygons and base pose."""
from dataclasses import dataclass, field
import math

import numpy as np
import yaml

from .grid import point_in_polygon, polygon_mask


@dataclass
class PatrolZone:
    zone_id: str
    boundary: list
    keepout: list = field(default_factory=list)
    base_pose: tuple = None  # (x, y, yaw) or None => recorded at mission start

    def __post_init__(self):
        if len(self.boundary) < 3:
            raise ValueError('patrol zone boundary needs at least 3 vertices')
        for x, y in self.boundary:
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError('non-finite zone vertex')

    def contains(self, x, y):
        return point_in_polygon(x, y, self.boundary) and not any(point_in_polygon(x, y, k) for k in self.keepout)

    def mask(self, grid):
        m = polygon_mask(self.boundary, grid.xs, grid.ys)
        for k in self.keepout:
            m &= ~polygon_mask(k, grid.xs, grid.ys)
        return m

    def area_m2(self):
        def shoelace(poly):
            return 0.5 * abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, poly[1:] + poly[:1])))
        return shoelace(self.boundary) - sum(shoelace(k) for k in self.keepout)


def map_from_world(points, origin_in_world):
    """World-frame points -> map frame, given the map origin pose in the world frame (x, y, yaw)."""
    ox, oy, oyaw = origin_in_world
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    return [(c * (x - ox) - s * (y - oy), s * (x - ox) + c * (y - oy)) for x, y in points]


def load_zone(path):
    with open(path) as fh:
        d = yaml.safe_load(fh)
    boundary = [tuple(map(float, p)) for p in d['boundary']]
    keepout = [[tuple(map(float, p)) for p in k] for k in d.get('keepout', [])]
    base = tuple(map(float, d['base_pose'])) if d.get('base_pose') else None
    if d.get('frame', 'map') == 'world':
        origin = d['map_origin_in_world']  # TO_VERIFY against the map actually loaded (owner C)
        boundary = map_from_world(boundary, origin)
        keepout = [map_from_world(k, origin) for k in keepout]
        if base is not None:
            bx, by = map_from_world([base[:2]], origin)[0]
            base = (bx, by, base[2] - origin[2])
    return PatrolZone(d.get('zone_id', 'zone'), boundary, keepout, base)


def load_route(path, origin_in_world=None):
    """Survey route waypoints (map frame). ``frame: world`` routes need ``origin_in_world``."""
    with open(path) as fh:
        d = yaml.safe_load(fh)
    pts = [tuple(map(float, p[:2])) for p in d['waypoints']]
    frame = d.get('frame', 'world' if d.get('schema') == 'ultron.sim_route/v1' else 'map')
    if frame == 'world':
        if origin_in_world is None:
            raise ValueError('world-frame route needs map_origin_in_world')
        pts = map_from_world(pts, origin_in_world)
    return [np.array(p) for p in pts]
