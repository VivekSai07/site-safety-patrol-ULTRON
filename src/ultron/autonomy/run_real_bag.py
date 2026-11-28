"""Map an offline B2 recording and replay it against the serialized map in isolation."""

import argparse
from collections import Counter
import csv
import json
import math
import os
from pathlib import Path
import sys
import uuid

from .config import load_sim_slam_config
from .eval_slam import load_map
from .real_bag import ODOM, SCAN, make_replay
from .run_localization import LOC_PARAMS
from .run_mapping import (SLAM_PARAMS, ensure_no_running_stack, fix_free_threshold, save_outputs,
                          sha256, spawn, stop, terminate_cleanly)
from .sim_bridge import check_isolation


def replay(out, rate, *, localization=False):
    """Start a fresh SLAM process and replay only the derived sensor and odometry topics."""
    folder = out / 'reload' if localization else out
    if localization:
        folder.mkdir()
    config = LOC_PARAMS if localization else SLAM_PARAMS
    executable = 'localization_slam_toolbox_node' if localization else 'async_slam_toolbox_node'
    command = ['ros2', 'run', 'slam_toolbox', executable, '--ros-args', '--params-file', str(config),
               '-r', '__node:=slam_toolbox', '-p', f'scan_topic:={SCAN}']
    if localization:
        command += ['-p', f'map_file_name:={out / "map"}', '-p', 'map_start_pose:=[0.0,0.0,0.0]']
        observer = ['ultron.autonomy.localization_monitor', '--recorded-b2', '--map', str(out), '--out', str(folder)]
    else:
        command += ['-p', 'minimum_time_interval:=0.2', '-p', 'minimum_travel_distance:=0.12',
                    '-p', 'minimum_travel_heading:=0.15']
        observer = ['ultron.autonomy.slam_logger', str(out / 'estimate.csv'), '--odom-topic', ODOM]
    (folder / 'slam_args.json').write_text(json.dumps(command, indent=2) + '\n')
    (folder / 'params_sha256.txt').write_text(sha256(config) + '\n')
    processes = {}
    try:
        processes['slam_toolbox'] = spawn(command, folder / 'slam_toolbox.log')
        processes['observer'] = spawn([sys.executable, '-m', *observer], folder / 'observer.log')

        # Static TF must remain available to late subscribers while the player runs.
        qos = out / 'replay_qos.yaml'
        if not localization:
            qos.write_text('/tf_static:\n  history: keep_last\n  depth: 1\n'
                           '  durability: transient_local\n  reliability: reliable\n')
        processes['player'] = spawn(
            ['ros2', 'bag', 'play', str(out / 'replay'), '--clock', '50', '--delay', '4',
             '--rate', str(rate), '--disable-keyboard-controls', '--qos-profile-overrides-path', str(qos)],
            folder / 'bag_play.log')
        if processes['player'].wait() != 0:
            raise RuntimeError(f'Bag playback failed; see {folder / "bag_play.log"}')
        for name in ('slam_toolbox', 'observer'):
            if processes[name].poll() is not None:
                raise RuntimeError(f'{name} exited unexpectedly; see {folder / (name + ".log")}')
        if not localization and save_outputs(out) != (0, 0):
            raise RuntimeError(f'Map save or serialization failed; see {out / "slam_toolbox.log"}')
    finally:
        for process in reversed(processes.values()):
            stop(process)


def trajectory_metrics(path):
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < 2:
        raise RuntimeError(f'Fewer than two estimated poses in {path}')
    xy = [(float(row['x']), float(row['y'])) for row in rows]
    yaw_delta = float(rows[-1]['yaw']) - float(rows[0]['yaw'])
    return {
        'samples': len(rows), 'first_t': float(rows[0]['t']), 'last_t': float(rows[-1]['t']),
        'path_length_m': sum(math.dist(a, b) for a, b in zip(xy, xy[1:])),
        'endpoint_separation_m': math.dist(xy[0], xy[-1]),
        'endpoint_heading_difference_deg': abs(math.degrees(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='Extracted ROS 2 SQLite bag folder')
    parser.add_argument('out', type=Path, help='New run folder')
    parser.add_argument('--rate', type=float, default=1.0, help='Replay speed multiplier (default 1)')
    args = parser.parse_args(argv)
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('--rate must be positive and finite')
    if not (args.source / 'metadata.yaml').is_file():
        parser.error('source must be an extracted rosbag2 folder with metadata.yaml')
    check_isolation(load_sim_slam_config())
    terminate_cleanly()
    ensure_no_running_stack()

    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('ROS_LOG_DIR', str(out / 'ros_logs'))
    print('Preparing replay from the recorded LiDAR clouds...', flush=True)
    frontend = make_replay(args.source, out)

    print('Mapping the recording...', flush=True)
    replay(out, args.rate)
    fix_free_threshold(out / 'map.yaml')
    files = {name: sha256(out / name) for name in ('map.pgm', 'map.yaml', 'map.posegraph', 'map.data')}
    metadata = {
        'schema': 'ultron.map_meta/v1', 'map_id': f'ultron-b2-replay-{uuid.uuid4().hex[:12]}', 'map_version': 1,
        'frame_id': 'map', 'units': 'metres, radians (REP-103)', 'clock_id': 'ros_b2_bag_time',
        'provenance': 'SLAM_TOOLBOX_ESTIMATE_FROM_RECORDED_B2_LIDAR',
        'estimator_inputs': [SCAN, '/tf (odom->base_link, base_link->ultron_lidar_level)'],
        'ground_truth_used_by_estimator': False, 'frontend': frontend, 'files_sha256': files,
        'backend': {'name': 'slam_toolbox', 'params_sha256': sha256(SLAM_PARAMS),
                    'command_file': 'slam_args.json'},
    }
    (out / 'map_meta.json').write_text(json.dumps(metadata, indent=2) + '\n')
    grid = load_map(out / 'map.yaml')
    if not len(grid['occupied_xy']):
        raise RuntimeError('Saved map has no occupied cells')

    summary = {'kiss_icp': trajectory_metrics(out / 'odometry.csv'),
               'mapping': trajectory_metrics(out / 'estimate.csv'),
               'occupied_cells': len(grid['occupied_xy']),
               'ground_truth_available': False}
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print('Reloading the serialized map in a fresh localization process...', flush=True)
    replay(out, args.rate, localization=True)
    summary['reload'] = trajectory_metrics(out / 'reload' / 'estimate.csv')
    with (out / 'reload' / 'states.jsonl').open() as handle:
        counts = Counter(row['state'] for row in map(json.loads, handle) if row.get('stamp') is not None)
    summary['reload']['scan_states'] = dict(counts)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
