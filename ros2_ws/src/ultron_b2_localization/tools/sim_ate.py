#!/usr/bin/env python3
"""ATE of a sim eval run (tools/eval_sim.sh) against the bridge's logged ground truth (evaluation only).

slam.csv (map->base_link at odometry stamps) and odom.csv (state/odom) are compared with ground truth
linearly interpolated at each estimate stamp, anchored at the start pose (as src/ultron/autonomy/eval_slam.py).
Wall recall of the saved map uses eval_slam.map_scores when its dependencies are available.
"""
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / 'src'))
from ultron.autonomy.eval_slam import THRESHOLDS, compose, trajectory_errors  # noqa: E402


def _rows(path, cols=('t', 'x', 'y', 'yaw')):
    return np.array([[float(r[c]) for c in cols] for r in csv.DictReader(open(path))])


def _interp_truth(truth, stamps):
    t = truth[:, 0]
    yaw = np.unwrap(truth[:, 3])
    ok = (stamps >= t[0]) & (stamps <= t[-1])
    s = stamps[ok]
    return s, np.column_stack((np.interp(s, t, truth[:, 1]), np.interp(s, t, truth[:, 2]), np.interp(s, t, yaw)))


def errors(est_rows, truth, start, align=False):
    if len(est_rows) == 0:
        return None
    s, ref = _interp_truth(truth, est_rows[:, 0])
    est = {round(float(a), 6): tuple(b) for a, b in zip(est_rows[:, 0], est_rows[:, 1:4])}
    ref = {round(float(a), 6): tuple(b) for a, b in zip(s, ref)}
    return trajectory_errors(est, ref, start, align=align)


def main(run):
    run = Path(run)
    meta = json.loads((run / 'sim' / 'run_meta.json').read_text())
    start = tuple(meta['start_pose_world'])
    truth = _rows(run / 'sim' / 'ground_truth.csv')
    slam = _rows(run / 'slam.csv')
    odom = _rows(run / 'odom.csv')
    leg = _rows(run / 'sim' / 'odometry.csv')
    out = {'provenance': 'SIMULATION_GROUND_TRUTH_EVALUATION', 'seed': meta.get('noise_seed'),
           'truth_duration_s': round(truth[-1, 0] - truth[0, 0], 2),
           'truth_path_m': round(float(np.sum(np.hypot(np.diff(truth[:, 1]), np.diff(truth[:, 2])))), 2),
           'estimate_coverage': round(len(slam) / max(1, len(odom)), 3),
           'slam_start_anchored': errors(slam, truth, start),
           'slam_best_fit_reference_only': errors(slam, truth, start, align=True),
           'odometry_only_state_odom': errors(odom, truth, start),
           'sim_leg_odometry_reference': errors(leg, truth, start),
           'thresholds_proposed': {k: THRESHOLDS[k] for k in ('ate_rmse_m', 'ate_max_m', 'yaw_rmse_deg')}}
    try:
        from ultron.autonomy.eval_slam import load_map, map_scores, reference_surfaces, run_config
        save = (run / 'save.log').read_text()
        yaml_path = save.split("yaml_path='")[1].split("'")[0]
        config, people = run_config(meta)
        grid = load_map(yaml_path)
        world = np.array([compose(start, (x, y, 0.0))[:2] for x, y in grid['occupied_xy']])
        reference, walls = reference_surfaces(truth_dict(truth), config, people=people)
        out['map'] = {'yaml': yaml_path, **map_scores(world, reference, walls, tolerance=THRESHOLDS['tolerance_m'])}
    except Exception as exc:  # noqa: BLE001 - report, do not fail the trajectory evaluation
        out['map'] = f'UNRUN: {type(exc).__name__}: {exc}'
    s = out['slam_start_anchored']
    out['checks'] = None if s is None else {'ate_rmse': s['rmse_m'] <= THRESHOLDS['ate_rmse_m'],
                                            'ate_max': s['max_m'] <= THRESHOLDS['ate_max_m'],
                                            'yaw_rmse': s['yaw_rmse_deg'] <= THRESHOLDS['yaw_rmse_deg']}
    (run / 'ate.json').write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps(out, indent=2))


def truth_dict(truth):
    return {round(float(r[0]), 3): (float(r[1]), float(r[2]), float(r[3])) for r in truth}


if __name__ == '__main__':
    main(sys.argv[1])
