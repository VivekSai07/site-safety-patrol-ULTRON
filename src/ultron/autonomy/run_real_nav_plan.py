"""Nav2 PLANNING-ONLY check on the real B2 hall map. No controller, no velocity output.

Part A: map_server + planner_server on the saved real map; compute paths for chosen goals and
score them against the map. Part B: replay the recorded scans with slam_toolbox localization and
build a global costmap from the obstacle layer alone, then compare it with the saved map.

Offline evidence only: there is no ground truth, and nothing here shows that a robot can drive
these paths. Run inside scripts/sim-slam.sh exec.
"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np

from .config import CONFIG_DIR, load_sim_slam_config
from .navigation_contract import MapFootprint
from .real_bag import ODOM, SCAN
from .run_localization import LOC_PARAMS
from .run_mapping import ensure_no_running_stack, sha256, spawn, stop, terminate_cleanly
from .sim_bridge import check_isolation

NAV2_PARAMS = CONFIG_DIR / 'nav2_params.yaml'
MAP_TOPIC = '/ultron/sim/nav/map'
RADIUS = 0.55
# Fixed before the run.
ACCEPT = {'goal_error_m': 0.35, 'min_clearance_m': 0.50, 'obstacle_near_map_m': 0.15,
          'obstacle_agreement_min': 0.80}
VALID_GOALS = {'loop_far_end': (1.8, -4.8), 'east_open': (3.5, -2.0), 'south_west': (-1.8, -3.8),
               'north_east': (3.5, 0.8), 'far_east': (4.0, -5.0)}
INVALID_GOALS = {'inside_obstacle': (-1.45, -1.65), 'in_unknown': (-1.0, -7.5), 'outside_map': (40.0, 0.0)}


class Grid:
    def __init__(self, map_dir):
        self.fp = MapFootprint(map_dir / 'map.yaml')
        rows, cols = np.nonzero(self.fp.occupied)
        self.occ_xy = np.column_stack((self.fp.origin[0] + (cols + 0.5) * self.fp.res,
                                       self.fp.origin[1] + (self.fp.height - 1 - rows + 0.5) * self.fp.res))

    def clearance(self, points):
        """Distance from each (x, y) to the nearest occupied cell centre."""
        out = np.empty(len(points))
        for i in range(0, len(points), 200):
            p = np.asarray(points[i:i + 200], dtype=float)
            near = self.occ_xy
            low, high = p.min(axis=0) - 3.0, p.max(axis=0) + 3.0
            near = near[np.all((near >= low) & (near <= high), axis=1)]
            if not len(near):
                out[i:i + 200] = 3.0
                continue
            out[i:i + 200] = np.sqrt(((p[:, None, :] - near[None, :, :]) ** 2).sum(axis=2)).min(axis=1)
        return out

    def state(self, x, y):
        r, c = self.fp._cell(x, y)
        if not (0 <= r < self.fp.height and 0 <= c < self.fp.width):
            return 'outside'
        return 'occupied' if self.fp.occupied[r, c] else 'free' if self.fp.free[r, c] else 'unknown'


def wait_until(node, rclpy, condition, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if condition():
            return True
    return False


def draw(grid, paths, trajectory, out):
    from PIL import Image, ImageDraw
    fp = grid.fp
    shade = np.where(fp.occupied, 0, np.where(fp.free, 255, 200)).astype(np.uint8)
    xs = [0.0] + [x for x, _ in trajectory] + [g[0] for g in VALID_GOALS.values()]
    ys = [0.0] + [y for _, y in trajectory] + [g[1] for g in VALID_GOALS.values()]
    x0, x1, y0, y1 = min(xs) - 2.5, max(xs) + 2.5, min(ys) - 2.5, max(ys) + 2.5
    r0, c0 = fp._cell(x0, y1)
    r1, c1 = fp._cell(x1, y0)
    crop = Image.fromarray(shade[max(r0, 0):r1, max(c0, 0):c1]).convert('RGB')
    scale = 4
    crop = crop.resize((crop.width * scale, crop.height * scale), Image.NEAREST)
    d = ImageDraw.Draw(crop)

    def px(x, y):
        r = (fp.height - (y - fp.origin[1]) / fp.res) - max(r0, 0)
        c = (x - fp.origin[0]) / fp.res - max(c0, 0)
        return c * scale, r * scale

    d.line([px(x, y) for x, y in trajectory], fill=(150, 150, 150), width=3)
    colours = [(30, 90, 230), (230, 120, 0), (20, 150, 60), (180, 40, 180), (200, 30, 30)]
    for i, (name, path) in enumerate(paths.items()):
        c = colours[i % len(colours)]
        d.line([px(x, y) for x, y in path], fill=c, width=4)
        gx, gy = px(*VALID_GOALS[name])
        d.ellipse((gx - 8, gy - 8, gx + 8, gy + 8), outline=c, width=4)
        d.text((gx + 10, gy - 6), name, fill=c)
    for name, point in INVALID_GOALS.items():
        gx, gy = px(*point)
        if 0 <= gx < crop.width and 0 <= gy < crop.height:
            d.line((gx - 8, gy - 8, gx + 8, gy + 8), fill=(220, 0, 0), width=4)
            d.line((gx - 8, gy + 8, gx + 8, gy - 8), fill=(220, 0, 0), width=4)
            d.text((gx + 10, gy - 6), name, fill=(220, 0, 0))
    sx, sy = px(0, 0)
    d.ellipse((sx - 9, sy - 9, sx + 9, sy + 9), fill=(0, 0, 0))
    d.text((10, 10), 'grey line: recorded walk (estimate)  colours: Nav2 paths  x: invalid goals', fill=(0, 0, 0))
    crop.save(out)


def params_for(out, name, obstacle_only, clearing=False):
    import yaml
    data = yaml.safe_load(NAV2_PARAMS.read_text())
    cm = data['global_costmap']['global_costmap']['ros__parameters']
    cm['obstacle_layer']['scan']['topic'] = SCAN
    if obstacle_only:
        cm['plugins'] = ['obstacle_layer']
        cm['obstacle_layer']['scan']['clearing'] = clearing  # False accumulates every recorded return
        cm.update(rolling_window=False, width=26, height=26, origin_x=-13.0, origin_y=-18.0,
                  track_unknown_space=True, update_frequency=5.0, publish_frequency=2.0, always_send_full_costmap=True)
    path = out / name
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return path


def part_a(map_dir, out, grid):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose
    params = params_for(out, 'planner_params.yaml', False)
    procs = {}
    try:
        procs['static_tf'] = spawn(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map',
                                    '--child-frame-id', 'base_link'], out / 'static_tf.log')
        procs['map_server'] = spawn(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p',
                                     f'yaml_filename:={map_dir / "map.yaml"}', '-p', f'topic_name:={MAP_TOPIC}',
                                     '-r', f'map:={MAP_TOPIC}'], out / 'map_server.log')
        # No clock is published in part A: planner and costmap must use wall time.
        import yaml
        data = yaml.safe_load(params.read_text())
        data['global_costmap']['global_costmap']['ros__parameters']['use_sim_time'] = False
        data['planner_server']['ros__parameters']['use_sim_time'] = False
        params.write_text(yaml.safe_dump(data, sort_keys=False))
        procs['planner'] = spawn(['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file',
                                  str(params)], out / 'planner_server.log')
        procs['lifecycle'] = spawn(['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args', '-r',
                                    '__node:=lifecycle_manager_plan', '-p', 'autostart:=true', '-p',
                                    "node_names:=['map_server','planner_server']"], out / 'lifecycle.log')
        rclpy.init()
        node = Node('ultron_real_plan_probe')
        try:
            client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
            if not client.wait_for_server(timeout_sec=60):
                raise RuntimeError('planner action not available; see planner_server.log')
            wait_until(node, rclpy, lambda: False, 6.0)  # let the static layer load

            def pose(x, y):
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.pose.position.x, p.pose.position.y, p.pose.orientation.w = float(x), float(y), 1.0
                return p

            def plan(goal):
                msg = ComputePathToPose.Goal()
                msg.start, msg.goal, msg.use_start = pose(0.0, 0.0), pose(*goal), True
                msg.planner_id = 'GridBased'
                sent = client.send_goal_async(msg)
                rclpy.spin_until_future_complete(node, sent, timeout_sec=20)
                handle = sent.result()
                if not handle or not handle.accepted:
                    return None, 'NOT_ACCEPTED', None
                res = handle.get_result_async()
                rclpy.spin_until_future_complete(node, res, timeout_sec=30)
                wrapped = res.result()
                if wrapped is None:
                    return None, 'NO_RESULT', None
                pts = [(p.pose.position.x, p.pose.position.y) for p in wrapped.result.path.poses]
                seconds = wrapped.result.planning_time.sec + wrapped.result.planning_time.nanosec * 1e-9
                return pts, {4: 'SUCCEEDED', 6: 'ABORTED', 5: 'CANCELED'}.get(wrapped.status, str(wrapped.status)), seconds

            records, paths = [], {}
            for name, goal in {**VALID_GOALS, **INVALID_GOALS}.items():
                valid = name in VALID_GOALS
                ok, why = grid.fp.check(*goal, RADIUS)
                pts, status, seconds = plan(goal)
                rec = {'goal': name, 'xy': goal, 'expected': 'plan' if valid else 'refuse',
                       'adapter_precheck': 'OK' if ok else why, 'planner_status': status,
                       'planning_time_s': None if seconds is None else round(seconds, 4)}
                if pts:
                    dense = np.array(pts)
                    clearance = grid.clearance(dense)
                    rec.update(path_length_m=round(float(np.linalg.norm(np.diff(dense, axis=0), axis=1).sum()), 2),
                               straight_line_m=round(math.dist((0, 0), goal), 2),
                               end_error_m=round(math.dist(tuple(dense[-1]), goal), 3),
                               min_clearance_m=round(float(clearance.min()), 3),
                               unknown_points=sum(grid.state(x, y) != 'free' for x, y in pts))
                if valid:
                    rec['pass'] = bool(ok and status == 'SUCCEEDED' and pts and rec['end_error_m'] <= ACCEPT['goal_error_m']
                                       and rec['min_clearance_m'] >= ACCEPT['min_clearance_m'] and rec['unknown_points'] == 0)
                    if pts:
                        paths[name] = pts
                else:
                    rec['pass'] = not ok   # the adapter must refuse it before the planner is trusted
                records.append(rec)
                print(json.dumps(rec), flush=True)
            return records, paths
        finally:
            node.destroy_node()
            rclpy.shutdown()
    finally:
        for name in ('lifecycle', 'planner', 'map_server', 'static_tf'):
            if name in procs:
                stop(procs[name])


def part_b(map_dir, out, grid, clearing=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    params = params_for(out, 'obstacle_params.yaml', True, clearing)
    procs = {}
    try:
        procs['slam'] = spawn(['ros2', 'run', 'slam_toolbox', 'localization_slam_toolbox_node', '--ros-args',
                               '--params-file', str(LOC_PARAMS), '-r', '__node:=slam_toolbox', '-p', f'scan_topic:={SCAN}',
                               '-p', f'map_file_name:={map_dir / "map"}', '-p', 'map_start_pose:=[0.0,0.0,0.0]'],
                              out / 'slam_toolbox.log')
        procs['planner'] = spawn(['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file',
                                  str(params)], out / 'obstacle_planner.log')
        procs['lifecycle'] = spawn(['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args', '-r',
                                    '__node:=lifecycle_manager_plan', '-p', 'use_sim_time:=true', '-p',
                                    'autostart:=true', '-p', "node_names:=['planner_server']"], out / 'obstacle_lifecycle.log')
        rclpy.init()
        node = Node('ultron_real_costmap_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
        try:
            grids = []
            node.create_subscription(OccupancyGrid, '/global_costmap/costmap', grids.append,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))
            time.sleep(5.0)
            qos = out / 'replay_qos.yaml'
            qos.write_text('/tf_static:\n  history: keep_last\n  depth: 1\n  durability: transient_local\n'
                           '  reliability: reliable\n')
            procs['player'] = spawn(['ros2', 'bag', 'play', str(map_dir / 'replay'), '--clock', '50', '--delay', '4',
                                     '--disable-keyboard-controls', '--qos-profile-overrides-path', str(qos)],
                                    out / 'bag_play.log')
            while procs['player'].poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            wait_until(node, rclpy, lambda: False, 1.0)
            if not grids:
                raise RuntimeError('no costmap received; see obstacle_planner.log')
            m = grids[-1]
            data = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
            rows, cols = np.nonzero(data == 100)
            xy = np.column_stack((m.info.origin.position.x + (cols + 0.5) * m.info.resolution,
                                  m.info.origin.position.y + (rows + 0.5) * m.info.resolution))
            if not len(xy):
                raise RuntimeError('obstacle layer marked no cells')
            distance = grid.clearance(xy)
            near = float((distance <= ACCEPT['obstacle_near_map_m']).mean())
            result = {'clearing': clearing, 'costmap_updates': len(grids), 'lethal_cells': int(len(xy)),
                      'fraction_within_0.15m_of_saved_map': round(near, 3),
                      'cells_over_0.5m_from_saved_map': int((distance > 0.5).sum()),
                      'pass': near >= ACCEPT['obstacle_agreement_min']}
            np.save(out / 'obstacle_cells.npy', np.column_stack((xy, distance)))
            print(json.dumps(result), flush=True)
            return result
        finally:
            node.destroy_node()
            rclpy.shutdown()
    finally:
        for name in ('player', 'lifecycle', 'planner', 'slam'):
            if name in procs:
                stop(procs[name])


def main(argv=None):
    parser = argparse.ArgumentParser(description='Nav2 planning-only check on the real B2 map')
    parser.add_argument('map', type=Path, help='real-bag run folder (map.yaml, map.posegraph, replay/)')
    parser.add_argument('out', type=Path)
    parser.add_argument('--goals', type=Path, help='JSON {"valid": {name: [x, y]}, "invalid": {...}} for this map')
    parser.add_argument('--clearing', action='store_true', help='Obstacle-layer check with raytrace clearing on')
    args = parser.parse_args(argv)
    if args.goals:
        chosen = json.loads(args.goals.read_text())
        for table, key in ((VALID_GOALS, 'valid'), (INVALID_GOALS, 'invalid')):
            table.clear()
            table.update({k: tuple(map(float, v)) for k, v in chosen[key].items()})
    check_isolation(load_sim_slam_config())
    terminate_cleanly()
    ensure_no_running_stack()
    map_dir, out = args.map.resolve(), args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    grid = Grid(map_dir)
    import csv
    walk = [(float(r['x']), float(r['y'])) for r in csv.DictReader((map_dir / 'estimate.csv').open())]
    walk_clearance = grid.clearance(walk)
    records, paths = part_a(map_dir, out, grid)
    draw(grid, paths, walk, out / 'planned_paths.png')
    obstacles = part_b(map_dir, out, grid, args.clearing)
    meta = json.loads((map_dir / 'map_meta.json').read_text())
    report = {'provenance': 'OFFLINE_PLANNING_ONLY_ON_REAL_RECORDING_NO_GROUND_TRUTH_NO_MOTION',
              'map_id': meta['map_id'], 'map_version': meta['map_version'], 'accept': ACCEPT,
              'footprint_radius_m': RADIUS, 'nav2_params_sha256': sha256(NAV2_PARAMS),
              'recorded_walk_min_clearance_m': round(float(walk_clearance.min()), 3),
              'goals': records, 'obstacle_layer': obstacles,
              'verdict': 'PASS' if all(r['pass'] for r in records) and obstacles['pass'] else 'FAIL'}
    (out / 'report.json').write_text(json.dumps(report, indent=2))
    print('walk min clearance', report['recorded_walk_min_clearance_m'], 'VERDICT', report['verdict'])


if __name__ == '__main__':
    main()
