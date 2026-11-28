"""Bounded frontier exploration (own implementation after the m-explore-ros2 idea, BSD reference).

Frontiers are known-free cells inside the PatrolZone that border unknown cells inside the zone,
so exploration never pulls outside the zone. Clusters smaller than ``min_size_cells`` or with
less than ``min_gain_m2`` of unknown zone area nearby are ignored (PROPOSED). Each frontier gets
a goal on a traversable cell reachable from the robot; frontiers without one are reported as
unreachable (separately from unknown area). Failed goals are blacklisted.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class Frontier:
    frontier_id: str
    size_cells: int
    centroid: tuple
    goal: tuple            # (x, y, yaw) or None when unreachable
    gain_m2: float
    distance_m: float
    reachable: bool
    blacklisted: bool = False


def frontier_mask(grid, zone_mask):
    unk = grid.unknown & zone_mask
    nb = np.zeros_like(unk)
    nb[1:, :] |= unk[:-1, :]
    nb[:-1, :] |= unk[1:, :]
    nb[:, 1:] |= unk[:, :-1]
    nb[:, :-1] |= unk[:, 1:]
    return grid.free & zone_mask & nb


def _clusters(mask):
    seen = np.zeros_like(mask)
    h, w = mask.shape
    for i0, j0 in zip(*np.nonzero(mask)):
        if seen[i0, j0]:
            continue
        cells, q = [], deque([(i0, j0)])
        seen[i0, j0] = True
        while q:
            i, j = q.popleft()
            cells.append((i, j))
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    a, b = i + di, j + dj
                    if 0 <= a < h and 0 <= b < w and mask[a, b] and not seen[a, b]:
                        seen[a, b] = True
                        q.append((a, b))
        yield cells


def find_frontiers(grid, zone, robot_xy, *, clearance_m=0.35, min_size_cells=5, min_gain_m2=0.25,
                   gain_radius_m=1.5, goal_search_m=1.5, blacklist=(), blacklist_radius_m=0.75,
                   reach=None):
    """Return (reachable frontiers ranked best-first, unreachable frontiers)."""
    zmask = zone.mask(grid)
    fmask = frontier_mask(grid, zmask)
    if reach is None:
        reach = grid.reachable_from(robot_xy, clearance_m)
    unknown_zone = grid.unknown & zmask
    res = grid.resolution
    reachable, unreachable = [], []
    for n, cells in enumerate(_clusters(fmask)):
        if len(cells) < min_size_cells:
            continue
        ii = np.array([c[0] for c in cells])
        jj = np.array([c[1] for c in cells])
        cx, cy = float(grid.xs[ii, jj].mean()), float(grid.ys[ii, jj].mean())
        near = (grid.xs - cx) ** 2 + (grid.ys - cy) ** 2 <= gain_radius_m ** 2
        gain = float((unknown_zone & near).sum()) * res * res
        if gain < min_gain_m2:
            continue
        ui, uj = np.nonzero(unknown_zone & near)
        ux, uy = (float(grid.xs[ui, uj].mean()), float(grid.ys[ui, uj].mean())) if ui.size else (cx, cy)
        fid = f'F-{int(round(cx / res))}-{int(round(cy / res))}'
        blk = any(math.hypot(cx - bx, cy - by) <= blacklist_radius_m for bx, by in blacklist)
        cand = reach & zmask & ((grid.xs - cx) ** 2 + (grid.ys - cy) ** 2 <= goal_search_m ** 2)
        if not cand.any():
            unreachable.append(Frontier(fid, len(cells), (cx, cy), None, gain, math.inf, False, blk))
            continue
        gi, gj = np.nonzero(cand)
        k = int(np.argmin((grid.xs[gi, gj] - cx) ** 2 + (grid.ys[gi, gj] - cy) ** 2))
        gx, gy = float(grid.xs[gi[k], gj[k]]), float(grid.ys[gi[k], gj[k]])
        yaw = math.atan2(uy - gy, ux - gx) if math.hypot(uy - gy, ux - gx) > 1e-3 else 0.0
        f = Frontier(fid, len(cells), (cx, cy), (gx, gy, yaw), gain,
                     math.hypot(gx - robot_xy[0], gy - robot_xy[1]), True, blk)
        (unreachable if blk else reachable).append(f)
    reachable.sort(key=lambda f: -f.gain_m2 / (1.0 + f.distance_m))
    return reachable, unreachable
