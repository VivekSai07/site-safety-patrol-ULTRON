"""Evaluate a simulated localization run against ground truth. EVALUATION ONLY.

The estimate (saved-map frame) is placed in the world with the MAPPING run's start pose
(map origin). Thresholds are fixed before evaluation (docs/SAHIL_SLAM_PLAN.md, Phase 4).
The key safety property is ``false_valid``: any sample labelled VALID whose true position
error exceeds ``false_valid_error_m``. It must be zero in every scenario.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from .eval_slam import compose, read_csv

THRESHOLDS = {'converge_within_s': 10.0, 'valid_rmse_m': 0.10, 'valid_max_m': 0.25, 'valid_yaw_rmse_deg': 2.0,
              'false_valid_error_m': 0.25, 'stale_detect_s': 0.75, 'recover_within_s': 5.0,
              'kidnap_detect_s': 2.0}


def load_run(run):
    run = Path(run)
    meta = json.loads((run / 'run_meta.json').read_text())
    truth = read_csv(run / 'sim' / 'ground_truth.csv')
    estimate, states = {}, {}
    for row in csv.DictReader((run / 'estimate.csv').open()):
        t = round(float(row['t']), 3)
        estimate[t] = (float(row['x']), float(row['y']), float(row['yaw']))
        states[t] = row['state']
    transitions, scans = [], []
    for line in (run / 'states.jsonl').read_text().splitlines():
        record = json.loads(line)
        if 'transition' in record:
            transitions.append(record['transition'])
        elif record.get('stamp') is not None:
            scans.append(record)
    events = [json.loads(line) for line in (run / 'sim' / 'events.jsonl').read_text().splitlines()]
    return meta, truth, estimate, states, transitions, scans, events


def first_transition(transitions, after, predicate):
    for tr in transitions:
        if tr['t'] >= after - 1e-6 and predicate(tr['to']):
            return tr
    return None


def evaluate(run):
    meta, truth, estimate, states, transitions, scans, events = load_run(run)
    origin = tuple(meta['map_origin_world'])
    stamps = sorted(set(estimate) & set(truth))
    if not stamps:
        raise ValueError('No common stamps between estimate and truth')
    err, yaw_err, valid = [], [], []
    for t in stamps:
        x, y, yaw = compose(origin, estimate[t])
        gx, gy, gyaw = truth[t]
        err.append(math.hypot(x - gx, y - gy))
        yaw_err.append(abs(math.degrees(math.atan2(math.sin(yaw - gyaw), math.cos(yaw - gyaw)))))
        valid.append(states[t] == 'VALID')
    err, yaw_err, valid = np.array(err), np.array(yaw_err), np.array(valid)
    false_valid = valid & (err > THRESHOLDS['false_valid_error_m'])
    start_t = stamps[0]
    first_valid = first_transition(transitions, start_t, lambda s: s == 'VALID')
    result = {'provenance': 'SIMULATION_GROUND_TRUTH_EVALUATION', 'thresholds': THRESHOLDS,
              'scenario': meta['scenario'], 'initial_guess_offset': meta['initial_guess_offset'],
              'samples': len(stamps), 'valid_fraction': float(valid.mean()),
              'false_valid_samples': int(false_valid.sum()),
              'max_error_while_valid_m': float(err[valid].max()) if valid.any() else None,
              'valid_rmse_m': float(np.sqrt(np.mean(err[valid] ** 2))) if valid.any() else None,
              'valid_yaw_rmse_deg': float(np.sqrt(np.mean(yaw_err[valid] ** 2))) if valid.any() else None,
              'error_all_rmse_m': float(np.sqrt(np.mean(err ** 2))), 'error_all_max_m': float(err.max()),
              'time_to_first_valid_s': round(first_valid['t'] - start_t, 3) if first_valid else None,
              'transitions': transitions,
              'scan_score_median': float(np.median([s['score'] for s in scans if s['score'] is not None])) if scans else None}
    checks = {'no_false_valid': result['false_valid_samples'] == 0,
              'converged': first_valid is not None and result['time_to_first_valid_s'] <= THRESHOLDS['converge_within_s']}
    name = meta['scenario']
    if name == 'loc_reload':
        checks['valid_rmse'] = result['valid_rmse_m'] is not None and result['valid_rmse_m'] <= THRESHOLDS['valid_rmse_m']
        checks['valid_max'] = result['max_error_while_valid_m'] is not None and result['max_error_while_valid_m'] <= THRESHOLDS['valid_max_m']
        checks['valid_yaw'] = result['valid_yaw_rmse_deg'] is not None and result['valid_yaw_rmse_deg'] <= THRESHOLDS['valid_yaw_rmse_deg']
    # Only a blackout (no data) must drive STALE; a dropout (degraded data) must not cause false VALID,
    # which the global no_false_valid check covers.
    fault = next((e for e in events if e['event'] == 'FAULT_START' and e.get('fault', 'lidar_blackout') == 'lidar_blackout'), None)
    dropout = next((e for e in events if e['event'] == 'FAULT_START' and e.get('fault') == 'lidar_dropout'), None)
    if dropout:
        window = [i for i, t in enumerate(stamps) if dropout['t'] <= t <= dropout['t'] + dropout['duration_s']]
        result['dropout'] = {'start': dropout['t'], 'duration_s': dropout['duration_s'], 'probability': dropout.get('probability'),
                             'valid_fraction_during': float(valid[window].mean()) if window else None,
                             'max_error_during_m': float(err[window].max()) if window else None}
    if fault:
        t_f, d = fault['t'], fault['duration_s']
        lost = first_transition(transitions, t_f, lambda s: s != 'VALID')
        back = first_transition(transitions, t_f + d, lambda s: s == 'VALID')
        result['fault'] = {'start': t_f, 'duration_s': d, 'left_valid_at': lost and lost['t'], 'left_valid_to': lost and lost['to'],
                           'detect_delay_s': lost and round(lost['t'] - t_f, 3),
                           'valid_again_at': back and back['t'], 'recover_delay_s': back and round(back['t'] - (t_f + d), 3)}
        checks['fault_detected_as_stale'] = bool(lost) and lost['to'] == 'STALE' and lost['t'] - t_f <= THRESHOLDS['stale_detect_s']
        checks['fault_recovered'] = bool(back) and back['t'] - (t_f + d) <= THRESHOLDS['recover_within_s']
    kidnap = next((e for e in events if e['event'] == 'TELEPORT'), None)
    if kidnap:
        t_k = kidnap['t']
        lost = first_transition(transitions, t_k, lambda s: s != 'VALID')
        back = first_transition(transitions, t_k, lambda s: s == 'VALID')
        result['kidnap'] = {'teleport_at': t_k, 'distance_m': math.dist(kidnap['true_before'][:2], kidnap['true_after'][:2]),
                            'left_valid_at': lost and lost['t'], 'left_valid_to': lost and lost['to'],
                            'detect_delay_s': lost and round(lost['t'] - t_k, 3),
                            'valid_again_at': back and back['t'],
                            'final_state': states[stamps[-1]], 'final_error_m': float(err[-1])}
        checks['kidnap_detected'] = bool(lost) and lost['t'] - t_k <= THRESHOLDS['kidnap_detect_s']
    result['checks'] = checks
    result['verdict'] = 'PASS' if all(checks.values()) else 'FAIL'
    return result


def summary(result):
    keys = ('scenario', 'verdict', 'checks', 'time_to_first_valid_s', 'valid_fraction', 'false_valid_samples',
            'valid_rmse_m', 'max_error_while_valid_m', 'valid_yaw_rmse_deg', 'error_all_max_m', 'scan_score_median',
            'fault', 'dropout', 'kidnap')
    return {k: result[k] for k in keys if k in result}


def main(argv=None):
    parser = argparse.ArgumentParser(description='Evaluate a localization run (evaluation only)')
    parser.add_argument('run', type=Path)
    args = parser.parse_args(argv)
    result = evaluate(args.run)
    (args.run / 'eval.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(summary(result), indent=2))


if __name__ == '__main__':
    main()
