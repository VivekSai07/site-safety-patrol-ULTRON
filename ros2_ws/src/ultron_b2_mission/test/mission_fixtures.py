"""Shared fixtures for mission core tests (pure Python; no ROS needed)."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'ultron_b2_perception'))

from ultron_b2_mission.grid import GridMap  # noqa: E402
from ultron_b2_mission.mission_core import Inputs  # noqa: E402
from ultron_b2_mission.zone import PatrolZone  # noqa: E402


def room(width=10.0, height=8.0, res=0.1, unknown_rect=None, walls=()):
    """Map with a wall border; optional unknown rect (x0, y0, x1, y1) and wall rects."""
    w, h = int(round(width / res)), int(round(height / res))
    data = np.zeros((h, w), dtype=np.int16)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    g = GridMap(data, res, (0.0, 0.0))

    def rect(r, val):
        x0, y0, x1, y1 = r
        data[(g.xs >= x0) & (g.xs < x1) & (g.ys >= y0) & (g.ys < y1)] = val
    if unknown_rect:
        rect(unknown_rect, -1)
    for r in walls:
        rect(r, 100)
    return GridMap(data, res, (0.0, 0.0))


def zone_for(width=10.0, height=8.0, keepout=(), base=(1.0, 1.0, 0.0)):
    return PatrolZone('Z1', [(0.0, 0.0), (width, 0.0), (width, height), (0.0, height)], list(keepout), base)


def cand(cid, x, y, ladder_type='TRESTLE', state='NEW', asset_id='', oracle=True, yaw=None):
    return {'candidate_id': cid, 'asset_id': asset_id or cid, 'position': [x, y, 0.0], 'yaw': yaw,
            'ladder_type': ladder_type, 'state': state, 'from_oracle': oracle}


def good_inputs(grid, robot=(1.0, 1.0, 0.0), candidates=(), now=0.0):
    return Inputs(grid=grid, robot=robot, candidates=list(candidates), loc_state='VALID', loc_map=('hall', 1),
                  loc_stamp=now, operator_enabled=True, operator_stamp=now, external_hold=False)


def refresh(inp, now):
    inp.loc_stamp = now
    inp.operator_stamp = now
    return inp


def kinds(cmds):
    return [k for k, _ in cmds]
