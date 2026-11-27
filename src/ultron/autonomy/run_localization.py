"""One-command localization run on a saved map: slam_toolbox (localization mode) + monitor +
scenario bridge, then evaluation. SIMULATION ONLY; run inside scripts/sim-slam.sh.

Usage: python -m ultron.autonomy.run_localization OUT --map MAPPING_RUN --scenario FILE
       [--offset DX DY DYAW_DEG] [--rate R]
The initial pose guess given to slam_toolbox = true start pose in the map frame + offset.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

from .config import CONFIG_DIR, load_scenario, load_sim_slam_config
from .eval_slam import compose
from .run_mapping import sha256, spawn, stop, terminate_cleanly
from .sim_bridge import check_isolation

LOC_PARAMS = CONFIG_DIR / 'slam_toolbox_localization.yaml'


def inverse(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return -c * x - s * y, s * x - c * y, -yaw


def main(argv=None):
    parser = argparse.ArgumentParser(description='Simulated localization on a saved map')
    parser.add_argument('out', type=Path)
    parser.add_argument('--map', type=Path, required=True, help='Mapping run folder (map.posegraph, map_meta.json)')
    parser.add_argument('--scenario', type=Path, default=CONFIG_DIR / 'scenarios' / 'loc_reload.yaml')
    parser.add_argument('--offset', type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=('DX', 'DY', 'DYAW_DEG'))
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--no-eval', action='store_true')
    args = parser.parse_args(argv)
    config = load_sim_slam_config()
    check_isolation(config)
    terminate_cleanly()
    map_meta = json.loads((args.map / 'map_meta.json').read_text())
    if not (args.map / 'map.posegraph').exists():
        raise SystemExit(f'{args.map} has no serialized pose graph')
    scenario = load_scenario(args.scenario)
    origin = tuple(map_meta['sim_run']['start_pose_world'])        # map origin in the world (mapping start)
    true_start = compose(inverse(origin), scenario.start_pose)       # scenario start in the map frame
    dx, dy, dyaw = args.offset
    guess = (true_start[0] + dx, true_start[1] + dy, true_start[2] + math.radians(dyaw))
    out = args.out
    out.mkdir(parents=True, exist_ok=False)
    (out / 'run_meta.json').write_text(json.dumps({
        'schema': 'ultron.localization_run/v1', 'provenance': 'SLAM_TOOLBOX_LOCALIZATION_ON_SIMULATED_LIDAR',
        'map_id': map_meta['map_id'], 'map_version': map_meta['map_version'], 'map_dir': str(args.map),
        'map_files_sha256': map_meta['files_sha256'], 'map_origin_world': list(origin),
        'scenario': scenario.name, 'scenario_sha256': scenario.sha256,
        'true_start_map_frame': list(true_start), 'initial_guess_map_frame': list(guess),
        'initial_guess_offset': {'dx_m': dx, 'dy_m': dy, 'dyaw_deg': dyaw, 'distance_m': math.hypot(dx, dy)},
        'backend': {'name': 'slam_toolbox', 'version': '2.6.10', 'mode': 'localization', 'params_sha256': sha256(LOC_PARAMS)},
        'ground_truth_used_by_estimator': False}, indent=2))
    py = sys.executable
    procs = {}
    try:
        procs['slam'] = spawn(['ros2', 'run', 'slam_toolbox', 'localization_slam_toolbox_node', '--ros-args',
                               '--params-file', str(LOC_PARAMS), '-r', '__node:=slam_toolbox',
                               '-p', f'map_file_name:={(args.map / "map").resolve()}',
                               '-p', f'map_start_pose:=[{guess[0]:.4f},{guess[1]:.4f},{guess[2]:.4f}]'],
                              out / 'slam_toolbox.log')
        procs['monitor'] = spawn([py, '-m', 'ultron.autonomy.localization_monitor', '--map', str(args.map),
                                  '--out', str(out)], out / 'monitor.log')
        time.sleep(5.0)  # deserialize the pose graph before the first scan
        for name, proc in procs.items():
            if proc.poll() is not None:
                raise RuntimeError(f'{name} exited early; see {out}')
        print(f'localization: scenario {scenario.name}, guess offset {math.hypot(dx, dy):.2f} m / {dyaw:.0f} deg ...', flush=True)
        bridge = subprocess.run([py, '-m', 'ultron.autonomy.sim_bridge', '--scenario', str(args.scenario),
                                 '--rate', str(args.rate), '--out', str(out / 'sim'),
                                 *(['--seed', str(args.seed)] if args.seed is not None else [])],
                                stdout=(out / 'bridge.log').open('w'), stderr=subprocess.STDOUT)
        if bridge.returncode != 0:
            raise RuntimeError(f'bridge failed ({bridge.returncode}); see {out}/bridge.log')
        time.sleep(1.0)
    finally:
        for name in ('monitor', 'slam'):
            if name in procs:
                stop(procs[name])
    if not args.no_eval:
        from .eval_localization import evaluate, summary
        result = evaluate(out)
        (out / 'eval.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(summary(result), indent=2))


if __name__ == '__main__':
    main()
