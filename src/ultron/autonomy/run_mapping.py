"""One-command simulated mapping run: bridge + slam_toolbox + logger (+ bag), then save,
serialize and evaluate. SIMULATION ONLY; run inside scripts/sim-slam.sh.

Output folder (new):
  sim/                bridge run metadata, ground_truth.csv (evaluation only), odometry.csv
  estimate.csv        SLAM map->base_link at odometry stamps
  map.pgm/.yaml       occupancy grid (nav2 map_saver)
  map.posegraph/.data serialized slam_toolbox pose graph (for reload / localization)
  map_meta.json       map ID/version, provenance, config hashes
  eval.json           evaluation against simulator ground truth
  bag/                optional rosbag2 (scan, odom, tf, clock, ground truth; no point clouds)
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from .config import CONFIG_DIR, load_sim_slam_config
from .sim_bridge import check_isolation

SLAM_PARAMS = CONFIG_DIR / 'slam_toolbox_mapping.yaml'
DEFAULT_ROUTE = CONFIG_DIR / 'routes' / 'hall_survey_loop.yaml'
BAG_TOPICS = ['/clock', '/tf', '/tf_static', '/ultron/sim/scan', '/ultron/sim/odom',
              '/ultron/sim/ground_truth/pose', '/ultron/sim/bridge_status']


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def terminate_cleanly():
    """Turn SIGTERM (e.g. from `timeout`) into KeyboardInterrupt so `finally` blocks stop children;
    otherwise slam_toolbox and helper nodes (own process groups) were left running."""
    def handler(*_):
        raise KeyboardInterrupt('SIGTERM')
    signal.signal(signal.SIGTERM, handler)


def spawn(cmd, log):
    return subprocess.Popen(cmd, stdout=log.open('w'), stderr=subprocess.STDOUT, start_new_session=True)


def stop(proc, timeout=15):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    return proc.returncode


def call_service(node, srv_type, name, request, timeout=60.0):
    import rclpy
    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=15.0):
        raise RuntimeError(f'Service {name} unavailable')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError(f'Service {name} timed out')
    return future.result()


def save_outputs(out):
    import rclpy
    from rclpy.node import Node
    from slam_toolbox.srv import SaveMap, SerializePoseGraph
    from std_msgs.msg import String
    rclpy.init()
    node = Node('ultron_map_saver_client')
    try:
        request = SaveMap.Request()
        request.name = String(data=str((out / 'map').resolve()))
        saved = call_service(node, SaveMap, '/slam_toolbox/save_map', request)
        request = SerializePoseGraph.Request()
        request.filename = str((out / 'map').resolve())
        serialized = call_service(node, SerializePoseGraph, '/slam_toolbox/serialize_map', request)
        return int(saved.result), int(serialized.result)
    finally:
        node.destroy_node()
        rclpy.shutdown()


UNKNOWN_PIXEL = 205      # map_saver trinary value for unknown cells: occupancy (255-205)/255 = 0.196
SAFE_FREE_THRESH = 0.196


def fix_free_threshold(yaml_path):
    """map_saver writes free_thresh 0.25, which makes nav2_map_server load unknown (205 -> 0.196)
    as FREE (measured: 3,259 unknown cells became free). 0.196 keeps them unknown (-1)."""
    lines = Path(yaml_path).read_text().splitlines()
    changed = False
    for i, line in enumerate(lines):
        if line.startswith('free_thresh:'):
            if float(line.split(':', 1)[1]) > SAFE_FREE_THRESH:
                lines[i] = f'free_thresh: {SAFE_FREE_THRESH}'
                changed = True
    Path(yaml_path).write_text('\n'.join(lines) + '\n')
    return changed


def main(argv=None):
    parser = argparse.ArgumentParser(description='Simulated 2D lidar mapping run with slam_toolbox')
    parser.add_argument('out', type=Path, help='New output folder')
    parser.add_argument('--route', type=Path, default=DEFAULT_ROUTE)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--record', action='store_true', help='Record a small rosbag for replay')
    parser.add_argument('--seed', type=int, help='Override the simulation noise seed')
    parser.add_argument('--no-eval', action='store_true')
    parser.add_argument('--replay', type=Path, metavar='RUN',
                        help='Instead of simulating, replay RUN/bag (scan, odom, tf without map) into a new SLAM instance')
    args = parser.parse_args(argv)
    config = load_sim_slam_config()
    check_isolation(config)
    terminate_cleanly()
    out = args.out
    out.mkdir(parents=True, exist_ok=False)
    py = sys.executable
    procs = {}
    try:
        procs['slam'] = spawn(['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node', '--ros-args',
                               '--params-file', str(SLAM_PARAMS), '-r', '__node:=slam_toolbox'], out / 'slam_toolbox.log')
        procs['logger'] = spawn([py, '-m', 'ultron.autonomy.slam_logger', str(out / 'estimate.csv')], out / 'logger.log')
        if args.record:
            procs['bag'] = spawn(['ros2', 'bag', 'record', '-o', str(out / 'bag'), '--use-sim-time', *BAG_TOPICS],
                                 out / 'bag.log')
        time.sleep(4.0)  # let subscribers come up before the first scan
        for name, proc in procs.items():
            if proc.poll() is not None:
                raise RuntimeError(f'{name} exited early; see {out}/{name}.log')
        if args.replay:
            import shutil
            shutil.copytree(args.replay / 'sim', out / 'sim')  # same evaluation logs / provenance
            procs['tf_filter'] = spawn([py, '-m', 'ultron.autonomy.tf_filter'], out / 'tf_filter.log')
            time.sleep(2.0)
            print(f'mapping: replaying {args.replay / "bag"} ...', flush=True)
            play = subprocess.run(['ros2', 'bag', 'play', str(args.replay / 'bag'), '--rate', str(args.rate),
                                   '--topics', '/clock', '/tf', '/tf_static', '/ultron/sim/scan', '/ultron/sim/odom',
                                   '--remap', '/tf:=/ultron/sim/replay/tf'],
                                  stdout=(out / 'bag_play.log').open('w'), stderr=subprocess.STDOUT)
            if play.returncode != 0:
                raise RuntimeError(f'bag play failed ({play.returncode}); see {out}/bag_play.log')
        else:
            print('mapping: route running ...', flush=True)
            bridge = subprocess.run([py, '-m', 'ultron.autonomy.sim_bridge', '--route', str(args.route),
                                     '--rate', str(args.rate), '--out', str(out / 'sim'),
                                     *(['--seed', str(args.seed)] if args.seed is not None else [])],
                                    stdout=(out / 'bridge.log').open('w'), stderr=subprocess.STDOUT)
            if bridge.returncode != 0:
                raise RuntimeError(f'bridge failed ({bridge.returncode}); see {out}/bridge.log')
        time.sleep(2.0)  # let slam_toolbox finish queued scans and publish the map
        saved, serialized = save_outputs(out)
        print(f'mapping: save_map result {saved}, serialize result {serialized}', flush=True)
    finally:
        for name in ('bag', 'tf_filter', 'logger', 'slam'):
            if name in procs:
                stop(procs[name])
    free_thresh_fixed = fix_free_threshold(out / 'map.yaml') if (out / 'map.yaml').exists() else False
    files = {name: sha256(out / name) for name in ('map.pgm', 'map.yaml', 'map.posegraph', 'map.data')
             if (out / name).exists()}
    sim_meta = json.loads((out / 'sim' / 'run_meta.json').read_text())
    meta = {
        'schema': 'ultron.map_meta/v1', 'map_id': f'ultron-hall-{uuid.uuid4().hex[:12]}', 'map_version': 1,
        'frame_id': config.frames['map'], 'units': 'metres, radians (REP-103)',
        'provenance': ('SLAM_TOOLBOX_ESTIMATE_FROM_REPLAYED_SIMULATION_BAG' if args.replay
                       else 'SLAM_TOOLBOX_ESTIMATE_FROM_SIMULATED_LIDAR_AND_ODOMETRY'),
        'replay_of': str(args.replay) if args.replay else None,
        'estimator_inputs': [config.topics['scan'], config.topics['odom'], '/tf (odom->base_link, base_link->lidar_link)'],
        'ground_truth_used_by_estimator': False,
        'backend': {'name': 'slam_toolbox', 'version': '2.6.10', 'mode': 'online_async',
                    'params_sha256': sha256(SLAM_PARAMS)},
        'map_origin_note': 'map frame origin = robot start pose (odom origin) for this run',
        'scene': sim_meta['scene'], 'sim_run': sim_meta, 'files_sha256': files,
        'save_map_result': saved, 'serialize_result': serialized,
        'free_thresh_rewritten_to_keep_unknown': free_thresh_fixed,
    }
    (out / 'map_meta.json').write_text(json.dumps(meta, indent=2))
    if not files.get('map.yaml') or not files.get('map.posegraph'):
        raise SystemExit(f'Map files missing in {out}; see slam_toolbox.log')
    if not args.no_eval:
        from .eval_slam import evaluate, overlay_png
        result = evaluate(out)
        (out / 'eval.json').write_text(json.dumps(result, indent=2))
        overlay_png(out, out / 'overlay.png')
        print(json.dumps({'verdict': result['verdict'], 'checks': result['checks'],
                          'slam': result['slam_start_anchored'], 'odometry_only': result['odometry_only'],
                          'map': {k: result['map'][k] for k in ('precision', 'recall_all', 'recall_walls', 'occupied_cells')}},
                         indent=2))


if __name__ == '__main__':
    main()
