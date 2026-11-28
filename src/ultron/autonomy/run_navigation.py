"""Nav2 bring-up (N1) and failure cases (N2) on a saved map. SIMULATION ONLY.

Starts slam_toolbox (localization), the LocalizationState monitor, Nav2 (map_server, planner,
controller, behaviors, BT navigator + lifecycle manager), the motion gate and the bridge in
command mode, then runs test cases through the Nav2 adapter. The case runner reads ground
truth only to score cases; it never feeds Nav2, the gate or localization.

Usage: python -m ultron.autonomy.run_navigation OUT --map MAPPING_RUN [--cases c1,c2,...] [--rate R]
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

from .config import CONFIG_DIR, load_sim_slam_config
from .eval_slam import compose
from .navigation_contract import make_goal
from .run_localization import LOC_PARAMS, inverse
from .run_mapping import ensure_no_running_stack, sha256, spawn, stop, terminate_cleanly
from .sim_bridge import check_isolation

NAV2_PARAMS = CONFIG_DIR / 'nav2_params.yaml'
NAV_MAP_TOPIC = '/ultron/sim/nav/map'
NAV2_NODES = ['map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
START_WORLD = (1.6, 1.5, math.pi / 2)
# Thresholds fixed before evaluation (docs/SAHIL_NAV_PLAN.md).
ACCEPT = {'arrive_xy_m': 0.25, 'arrive_yaw_rad': 0.35, 'stop_within_s': 1.2, 'still_m': 0.02,
          'gate_after_operator_s': 0.10, 'gate_after_blackout_s': 0.80, 'unreachable_timeout_s': 60.0}
ALL_CASES = ['reach_near', 'reach_far', 'reject_occupied', 'reject_outside', 'unreachable', 'new_obstacle',
             'cancel', 'operator_inhibit', 'localization_loss']


class Runner:
    def __init__(self, config, map_dir, out):
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool, String
        from .nav2_adapter import Nav2Adapter
        self.rclpy, self.config, self.out = rclpy, config, out
        self.node = Node('ultron_nav_case_runner', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.nav = Nav2Adapter(self.node, config, map_dir)
        meta = json.loads((Path(map_dir) / 'map_meta.json').read_text())
        self.origin = tuple(meta['sim_run']['start_pose_world'])       # map frame origin in the world
        self.Bool, self.String = Bool, String
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.operator_pub = self.node.create_publisher(Bool, config.topics['operator_enable'], qos)
        self.control_pub = self.node.create_publisher(String, config.topics['sim_control'], qos)
        self.operator = True
        self.truth = []   # (t, x, y, yaw) EVALUATION ONLY
        self.node.create_subscription(PoseStamped, config.topics['ground_truth'], self._on_truth, qos)
        self.node.create_timer(0.2, self._publish_operator)
        self.node.create_timer(0.05, self.nav.tick)
        self.results = []

    def _on_truth(self, msg):
        q = msg.pose.orientation
        self.truth.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.pose.position.x,
                           msg.pose.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)))

    def _publish_operator(self):
        self.nav.operator_enabled = self.operator
        self.operator_pub.publish(self.Bool(data=self.operator))

    def now(self):
        return self.nav.now()

    def spin(self, seconds=None, until=None, wall_limit=180.0):
        """Spin until sim time advanced by ``seconds`` or ``until()`` is true (wall-clock bounded)."""
        start, wall = self.now(), time.monotonic()
        while time.monotonic() - wall < wall_limit:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            if until is not None and until():
                return True
            if seconds is not None and self.now() - start >= seconds:
                return True
        return False

    def control(self, **cmd):
        self.control_pub.publish(self.String(data=json.dumps(cmd)))

    def to_map(self, world):
        return compose(inverse(self.origin), world)

    def to_world(self, pose):
        return compose(self.origin, pose)

    def goal(self, world_xy, yaw_world=0.0, timeout=None, reason='test'):
        x, y, yaw = self.to_map((world_xy[0], world_xy[1], yaw_world))
        return make_goal(x, y, yaw, mission_id='NAV-TEST', map_id=self.nav.map_id, map_version=self.nav.map_version,
                         params=self.config.navigation, reason=reason, now=self.now(), timeout_s=timeout)

    def true_pose(self):
        return self.truth[-1][1:] if self.truth else None

    def moved_after(self, t0, t1=None):
        """Largest true displacement between samples in [t0, t1]."""
        window = [s for s in self.truth if s[0] >= t0 and (t1 is None or s[0] <= t1)]
        if len(window) < 2:
            return 0.0
        x0, y0 = window[0][1:3]
        return max(math.hypot(x - x0, y - y0) for _, x, y, _ in window)

    def last_motion_time(self, after):
        """Last true sample after ``after`` that moved OR turned (rotation in place counts as motion)."""
        moving = [b[0] for a, b in zip(self.truth, self.truth[1:])
                  if b[0] >= after and (math.hypot(b[1] - a[1], b[2] - a[2]) > 1e-4
                                        or abs(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))) > 1e-4)]
        return moving[-1] if moving else after

    def run_goal(self, goal, wall_limit=240.0):
        result = self.nav.submit(goal)
        if result is None:
            self.spin(until=lambda: self.nav.done, wall_limit=wall_limit)
        return self.nav.tracker.result

    def wait_for_valid(self):
        ok = self.spin(until=lambda: self.nav._pose() is not None, wall_limit=60)
        if not ok:
            raise RuntimeError('Localization never became VALID')

    # --- cases -------------------------------------------------------------------------
    def case(self, name, fn):
        print(f'nav case {name} ...', flush=True)
        t0 = self.now()
        try:
            record = fn()
        except Exception as exc:  # a failed case is recorded, the run continues
            record = {'checks': {'ran': False}, 'error': repr(exc)}
        record.update(case=name, started=round(t0, 3), ended=round(self.now(), 3),
                      events=[e for e in self.nav.tracker.events if e['t'] >= t0 - 1e-6])
        record['verdict'] = 'PASS' if record['checks'] and all(record['checks'].values()) else 'FAIL'
        self.results.append(record)
        print(f'  -> {record["verdict"]} {record["checks"]}', flush=True)
        self.spin(1.0)

    def arrive(self, world_xy, yaw_world):
        goal = self.goal(world_xy, yaw_world)
        result = self.run_goal(goal)
        tx, ty, tyaw = self.true_pose()
        err_xy = math.hypot(tx - world_xy[0], ty - world_xy[1])
        err_yaw = abs(math.atan2(math.sin(tyaw - yaw_world), math.cos(tyaw - yaw_world)))
        return {'status': result.status, 'reason': result.reason, 'capture_ready': result.capture_ready,
                'true_error_xy_m': round(err_xy, 3), 'true_error_yaw_rad': round(err_yaw, 3),
                'checks': {'reached': result.status == 'REACHED', 'capture_ready': result.capture_ready,
                           'xy_within': err_xy <= ACCEPT['arrive_xy_m'], 'yaw_within': err_yaw <= ACCEPT['arrive_yaw_rad']}}

    def rejected(self, world_xy, expect):
        t0 = self.now()
        result = self.run_goal(self.goal(world_xy))
        self.spin(2.0)
        moved = self.moved_after(t0)
        return {'status': result.status, 'reason': result.reason, 'moved_m': round(moved, 4),
                'checks': {'rejected': result.status == 'REJECTED', 'reason_matches': expect in result.reason,
                           'no_motion': moved <= ACCEPT['still_m']}}

    def unreachable(self):
        target = (7.5, 4.5)
        self.control(obstacle={'id': 0, 'x': target[0], 'y': target[1]})
        self.spin(1.0)
        t0 = self.now()
        result = self.run_goal(self.goal(target, timeout=ACCEPT['unreachable_timeout_s']))
        t_end = self.now()
        elapsed = t_end - t0
        self.spin(3.0)
        stop_after = self.last_motion_time(t_end) - t_end   # same stop budget as the other cases
        self.control(obstacle={'id': 0, 'x': None, 'y': None})
        return {'status': result.status, 'reason': result.reason, 'elapsed_s': round(elapsed, 2),
                'stop_after_s': round(stop_after, 3),
                'checks': {'failed_or_timeout': result.status in ('FAILED', 'TIMEOUT'),
                           'bounded': elapsed <= ACCEPT['unreachable_timeout_s'] + 5.0,
                           'stopped_in_time': stop_after <= ACCEPT['stop_within_s']}}

    def new_obstacle(self):
        collisions_before = self.collisions()
        goal = self.goal((2.5, 5.5), math.pi)
        self.nav.submit(goal)
        start = self.true_pose()
        self.spin(until=lambda: self.true_pose() and math.dist(self.true_pose()[:2], start[:2]) > 1.0, wall_limit=60)
        x, y, yaw = self.true_pose()
        spot = (x + 1.6 * math.cos(yaw), y + 1.6 * math.sin(yaw))
        self.control(obstacle={'id': 1, 'x': spot[0], 'y': spot[1]})
        self.spin(until=lambda: self.nav.done, wall_limit=240)
        result = self.nav.tracker.result
        self.control(obstacle={'id': 1, 'x': None, 'y': None})
        collisions = self.collisions() - collisions_before
        return {'status': result.status, 'reason': result.reason, 'obstacle_at_world': [round(v, 2) for v in spot],
                'collision_stops': collisions,
                'checks': {'replanned_or_stopped': result.status in ('REACHED', 'FAILED', 'TIMEOUT'),
                           'no_collision_stop': collisions == 0,
                           'no_false_arrival': result.status != 'REACHED' or
                           math.dist(self.true_pose()[:2], (2.5, 5.5)) <= ACCEPT['arrive_xy_m']}}

    def stop_case(self, trigger):
        goal = self.goal((9.6, 7.8), 0.0)
        self.nav.submit(goal)
        self.spin(until=lambda: self.nav.tracker.status == 'ACTIVE', wall_limit=30)
        self.spin(3.0)
        t_trigger = self.now()
        gate_before = len(self.gate_log)
        trigger()
        self.spin(until=lambda: self.nav.tracker.status in ('CANCELLED', 'TIMEOUT', 'FAILED', 'REACHED'), wall_limit=30)
        self.spin(3.0)
        result = self.nav.tracker.result
        stopped_at = self.last_motion_time(t_trigger)
        inhibit = next((g for g in self.gate_log[gate_before:] if g.get('inhibited')), None)
        return result, t_trigger, stopped_at, inhibit

    def collisions(self):
        return self.nav.bridge_status.get('collisions', 0)

    def cancel(self):
        result, t, stopped, _ = self.stop_case(lambda: self.nav.cancel('USER_CANCEL'))
        return {'status': result.status, 'stop_after_s': round(stopped - t, 3),
                'checks': {'cancelled': result.status == 'CANCELLED', 'stopped_in_time': stopped - t <= ACCEPT['stop_within_s']}}

    def operator_inhibit(self):
        def disable():
            self.operator = False
            self._publish_operator()
        result, t, stopped, inhibit = self.stop_case(disable)
        self.operator = True
        self.spin(1.0)
        delay = inhibit['t'] - t if inhibit else None
        return {'status': result.status, 'gate_reason': inhibit and inhibit['reason'], 'gate_delay_s': delay and round(delay, 3),
                'stop_after_s': round(stopped - t, 3),
                'checks': {'cancelled': result.status == 'CANCELLED',
                           'gate_inhibited_fast': delay is not None and delay <= ACCEPT['gate_after_operator_s'],
                           'stopped_in_time': stopped - t <= ACCEPT['stop_within_s']}}

    def localization_loss(self):
        result, t, stopped, inhibit = self.stop_case(lambda: self.control(fault='lidar_blackout', duration_s=3.0))
        delay = inhibit['t'] - t if inhibit else None
        moved_while_invalid = self.moved_after(t + ACCEPT['gate_after_blackout_s'] + ACCEPT['stop_within_s'], t + 3.0)
        return {'status': result.status, 'gate_reason': inhibit and inhibit['reason'], 'gate_delay_s': delay and round(delay, 3),
                'stop_after_s': round(stopped - t, 3), 'moved_while_invalid_m': round(moved_while_invalid, 4),
                'checks': {'cancelled': result.status == 'CANCELLED',
                           'gate_inhibited_in_time': delay is not None and delay <= ACCEPT['gate_after_blackout_s'],
                           'no_motion_while_invalid': moved_while_invalid <= ACCEPT['still_m']}}


def main(argv=None):
    parser = argparse.ArgumentParser(description='Nav2 bring-up and failure cases on a saved map (simulation)')
    parser.add_argument('out', type=Path)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--cases', default=','.join(ALL_CASES))
    parser.add_argument('--rate', type=float, default=1.0)
    args = parser.parse_args(argv)
    cases = [c for c in args.cases.split(',') if c]
    unknown = set(cases) - set(ALL_CASES)
    if unknown:
        raise SystemExit(f'Unknown cases {sorted(unknown)}; choose from {ALL_CASES}')
    config = load_sim_slam_config()
    check_isolation(config)
    terminate_cleanly()
    ensure_no_running_stack()
    out = args.out
    out.mkdir(parents=True, exist_ok=False)
    map_dir = args.map.resolve()
    py = sys.executable
    procs = {}
    ros = lambda *a: ['ros2', 'run', *a]
    remap_cmd = ['-r', f'cmd_vel:={config.topics["nav_cmd_raw"]}']
    remap_odom = ['-r', f'odom:={config.topics["odom"]}']
    try:
        procs['slam'] = spawn(ros('slam_toolbox', 'localization_slam_toolbox_node') + [
            '--ros-args', '--params-file', str(LOC_PARAMS), '-r', '__node:=slam_toolbox',
            '-p', f'map_file_name:={map_dir / "map"}', '-p', 'map_start_pose:=[0.0,0.0,0.0]'], out / 'slam_toolbox.log')
        procs['monitor'] = spawn([py, '-m', 'ultron.autonomy.localization_monitor', '--map', str(map_dir), '--out', str(out)],
                                 out / 'monitor.log')
        procs['gate'] = spawn([py, '-m', 'ultron.autonomy.motion_gate_node', '--map', str(map_dir),
                               '--log', str(out / 'gate.json')], out / 'gate.log')
        procs['map_server'] = spawn(ros('nav2_map_server', 'map_server') + [
            '--ros-args', '-p', 'use_sim_time:=true', '-p', f'yaml_filename:={map_dir / "map.yaml"}',
            '-p', 'topic_name:=' + NAV_MAP_TOPIC, '-r', 'map:=' + NAV_MAP_TOPIC], out / 'map_server.log')
        for name in ('planner_server', 'controller_server', 'behavior_server', 'bt_navigator'):
            package = {'planner_server': 'nav2_planner', 'controller_server': 'nav2_controller',
                       'behavior_server': 'nav2_behaviors', 'bt_navigator': 'nav2_bt_navigator'}[name]
            extra = (remap_cmd if name in ('controller_server', 'behavior_server') else []) + remap_odom
            procs[name] = spawn(ros(package, name) + ['--ros-args', '--params-file', str(NAV2_PARAMS), *extra],
                                out / f'{name}.log')
        procs['lifecycle'] = spawn(ros('nav2_lifecycle_manager', 'lifecycle_manager') + [
            '--ros-args', '-r', '__node:=lifecycle_manager_navigation', '-p', 'use_sim_time:=true', '-p', 'autostart:=true',
            '-p', 'node_names:=' + json.dumps(NAV2_NODES).replace('"', "'")], out / 'lifecycle.log')
        time.sleep(5.0)
        procs['bridge'] = spawn([py, '-m', 'ultron.autonomy.sim_bridge', '--rate', str(args.rate), '--duration', '3000',
                                 '--out', str(out / 'sim')], out / 'bridge.log')
        for name, proc in procs.items():
            if proc.poll() is not None:
                raise RuntimeError(f'{name} exited early; see {out}')
        import rclpy
        rclpy.init()
        try:
            runner = Runner(config, map_dir, out)
            _attach_status(runner)
            print('navigation: waiting for VALID localization and Nav2 ...', flush=True)
            runner.wait_for_valid()
            if not runner.nav.client.wait_for_server(timeout_sec=60):
                raise RuntimeError('Nav2 NavigateToPose server not available')
            runner.spin(2.0)
            plan = {'reach_near': lambda: runner.arrive((2.5, 5.5), 0.0),
                    'reach_far': lambda: runner.arrive((9.6, 4.5), -math.pi / 2),
                    'reject_occupied': lambda: runner.rejected((4.0, 4.0), 'OCCUPIED'),
                    'reject_outside': lambda: runner.rejected((14.0, 5.0), 'OUTSIDE_MAP'),
                    'unreachable': runner.unreachable, 'new_obstacle': runner.new_obstacle,
                    'cancel': runner.cancel, 'operator_inhibit': runner.operator_inhibit,
                    'localization_loss': runner.localization_loss}
            for name in cases:
                runner.case(name, plan[name])
            summary = {'schema': 'ultron.nav_cases/v1', 'map_id': runner.nav.map_id, 'map_version': runner.nav.map_version,
                       'accept': ACCEPT, 'nav2_params_sha256': sha256(NAV2_PARAMS),
                       'cases': runner.results, 'gate_log': runner.gate_log,
                       'verdict': 'PASS' if all(r['verdict'] == 'PASS' for r in runner.results) else 'FAIL'}
            (out / 'nav_cases.json').write_text(json.dumps(summary, indent=2, default=str))
            print(json.dumps({r['case']: r['verdict'] for r in runner.results}, indent=1))
            print('navigation verdict:', summary['verdict'])
        finally:
            rclpy.shutdown()
    finally:
        for name in ('bridge', 'lifecycle', 'bt_navigator', 'behavior_server', 'controller_server', 'planner_server',
                     'map_server', 'gate', 'monitor', 'slam'):
            if name in procs:
                stop(procs[name])


def _attach_status(runner):
    """Gate status log and bridge status (collisions) for scoring."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    runner.gate_log = []
    runner.nav.bridge_status = {}

    def on_gate(msg):
        runner.gate_log.append(json.loads(msg.data))

    def on_bridge(msg):
        runner.nav.bridge_status = json.loads(msg.data)

    runner.node.create_subscription(runner.String, runner.config.topics['gate_status'], on_gate, latched)
    runner.node.create_subscription(runner.String, runner.config.topics['status'], on_bridge, latched)


if __name__ == '__main__':
    main()
