"""Integration check (item e): the mission's map_origin_in_world equals the sim start pose.

C's slam_toolbox map frame is the odom origin, i.e. the pose where the mapping run started; the sim
(F, src/ultron/autonomy/sim_bridge.py DEFAULT_START) spawns there unless a route/scenario says otherwise.
Valid only for maps built from the default start and localized with initial_pose:=0,0,0.
"""
import math
import pathlib
import re

import yaml

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]


def default_start():
    text = (REPO / 'src' / 'ultron' / 'autonomy' / 'sim_bridge.py').read_text()
    x, y, yaw = re.search(r'^DEFAULT_START = \((.+)\)$', text, re.M).group(1).split(',')
    return float(x), float(y), eval(yaw.strip(), {'math': math})  # noqa: S307 - literal from our own source


def test_zone_and_mission_origin_match_sim_start():
    start = default_start()
    zone = yaml.safe_load((HERE.parent / 'config' / 'hall_zone.yaml').read_text())
    mission = yaml.safe_load((HERE.parent / 'config' / 'mission_sim.yaml').read_text())['/**']['ros__parameters']
    for pose in (zone['map_origin_in_world'], zone['base_pose'], mission['map_origin_in_world']):
        assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(pose, start)), (pose, start)
