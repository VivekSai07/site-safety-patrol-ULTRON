"""Zone coverage accounting. Waypoint completion is never coverage; only observed map cells count.

observed     = known (free or occupied) cells inside the zone
unknown      = unknown cells inside the zone that border reachable free space (still explorable)
unreachable  = unknown cells not explorable from reachable space, plus known-free cells the robot
               cannot reach with clearance (reported separately from unknown)
Gap polygons are axis-aligned bounding boxes of 8-connected gap components (map frame).
Ladders that were pending, unreachable or missing are listed as gap ids too (CoverageReport v1
has no ladder fields; see CONTRACT_CHANGES).
"""
from dataclasses import dataclass, field

import numpy as np

from .frontier_explorer import _clusters, frontier_mask
from .grid import dilate


@dataclass
class Coverage:
    zone_area_m2: float
    observed_area_m2: float
    unknown_area_m2: float
    unreachable_area_m2: float
    coverage_fraction: float
    gap_ids: list = field(default_factory=list)
    gap_polygons: list = field(default_factory=list)   # list of [(x, y), ...]
    complete: bool = False
    reason: str = ''


def _components(mask, grid, prefix, min_cells):
    ids, polys = [], []
    for n, cells in enumerate(sorted(_clusters(mask), key=lambda c: (-len(c), c[0]))):
        if len(cells) < min_cells:
            continue
        ii = np.array([c[0] for c in cells])
        jj = np.array([c[1] for c in cells])
        r = grid.resolution
        x0, x1 = grid.origin[0] + jj.min() * r, grid.origin[0] + (jj.max() + 1) * r
        y0, y1 = grid.origin[1] + ii.min() * r, grid.origin[1] + (ii.max() + 1) * r
        ids.append(f'{prefix}-{len(ids) + 1}')
        polys.append([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
    return ids, polys


def compute_coverage(grid, zone, reach, *, ladders_pending=(), ladders_unreachable=(), ladders_missing=(),
                     min_gap_cells=4, complete_fraction=0.95, reach_margin_m=0.6):
    zmask = zone.mask(grid)
    cell = grid.cell_area()
    zone_cells = int(zmask.sum())
    known = (~grid.unknown) & zmask
    unknown = grid.unknown & zmask
    # unknown cells explorable: connected (through unknown) to a frontier adjacent to reachable space
    near_reach = dilate(reach, reach_margin_m / grid.resolution)
    fr = frontier_mask(grid, zmask) & near_reach
    seed = dilate(fr, 1.0) & unknown
    explorable = np.zeros_like(unknown)
    for cells in _clusters(unknown):
        if any(seed[c] for c in cells):
            for c in cells:
                explorable[c] = True
    unknown_unreach = unknown & ~explorable
    # free cells within reach_margin of reachable space (e.g. next to walls) are not gaps (PROPOSED margin)
    free_unreach = grid.free & zmask & ~near_reach
    unreach = unknown_unreach | free_unreach
    gap_ids, gap_polys = _components(explorable, grid, 'GAP-UNK', min_gap_cells)
    ids2, polys2 = _components(unreach, grid, 'GAP-UNR', min_gap_cells)
    gap_ids += ids2
    gap_polys += polys2
    gap_ids += [f'LADDER_PENDING:{a}' for a in ladders_pending]
    gap_ids += [f'LADDER_UNREACHABLE:{a}' for a in ladders_unreachable]
    gap_ids += [f'LADDER_MISSING:{a}' for a in ladders_missing]
    frac = float(known.sum()) / zone_cells if zone_cells else 0.0
    complete = frac >= complete_fraction and not explorable.any() and not (
        ladders_pending or ladders_unreachable or ladders_missing)
    if zone_cells == 0:
        reason = 'Zone does not overlap the map'
    elif complete:
        reason = 'Zone observed to threshold; no pending, unreachable or missing ladders (pre-screening only)'
    else:
        parts = []
        if explorable.any():
            parts.append('unknown explorable area remains')
        if unreach.any():
            parts.append('unreachable area')
        if frac < complete_fraction:
            parts.append(f'observed fraction {frac:.2f} < {complete_fraction:.2f}')
        if ladders_pending or ladders_unreachable or ladders_missing:
            parts.append('ladders pending/unreachable/missing')
        reason = '; '.join(parts) or 'incomplete'
    return Coverage(zone_cells * cell, float(known.sum()) * cell, float(explorable.sum()) * cell,
                    float(unreach.sum()) * cell, frac, gap_ids, gap_polys, complete, reason)

