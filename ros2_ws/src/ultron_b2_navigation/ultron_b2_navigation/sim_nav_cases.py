"""Nav N1/N2 cases through the FULL B2 command chain. SIMULATION ONLY (domain != 0, localhost).

Chain under test: NavigateToView (nav/navigate_to_view) -> nav_contract_server -> Nav2 -> nav/cmd_vel_raw
-> velocity_smoother -> control/in/autonomy -> twist_mux -> collision_monitor -> motion_gate -> cmd_vel
-> sim_driver -> sim/cmd_vel -> ultron_b2_sim. Teleop cases use control/in/teleop on the same chain.

The whole integrated stack is started here: ``ros2 launch ultron_b2_bringup sim.launch.py mode:=nav
map:=MAP_DIR`` (F's ultron_b2_sim, B's preprocess, C's odometry/slam_toolbox localization/monitor, D's
nav + control). MAP_DIR is a saved map from C's map store, built by mapping from the default sim start
pose, so the map frame is the world pose ``--origin`` (default sim start 1.6, 1.5, pi/2).
Ground truth (``sim/ground_truth/base_pose``) is read ONLY to score cases; it never feeds navigation,
the gate or localization. Thresholds are the pre-registered ones of
``ultron.autonomy.run_navigation.ACCEPT`` (SAHIL_NAV_PLAN).

  sim_nav_cases OUT --map MAP_DIR [--cases a,b] [--rate 1.0] [--odom-source mola]
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from .ultron_path import ensure_ultron

ensure_ultron()
from ultron.autonomy.config import load_sim_slam_config  # noqa: E402
from ultron.autonomy.eval_slam import compose  # noqa: E402
from ultron.autonomy.navigation_contract import make_goal  # noqa: E402
from ultron.autonomy.run_localization import inverse  # noqa: E402
from ultron.autonomy.sim_bridge import DEFAULT_START  # noqa: E402
from ultron.autonomy.run_navigation import ACCEPT  # noqa: E402

NS = '/ultron/sim'
ALL_CASES = ['reach_near', 'reach_far', 'reject_occupied', 'reject_outside', 'unreachable', 'new_obstacle',
             'cancel', 'operator_inhibit', 'localization_loss', 'teleop_hold', 'teleop_collision',
             'teleop_no_localization']
GATE_IGNORED = ('', 'NO_COMMAND', 'COMMAND_STALE')


def spawn(cmd, log, env):
    return subprocess.Popen(cmd, stdout=log.open('w'), stderr=subprocess.STDOUT, start_new_session=True, env=env)


def stop(proc, timeout=20):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)            # launch / nodes forward SIGINT themselves
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    return proc.returncode


class Runner:
    def __init__(self, map_dir, origin):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import PoseStamped, Twist
        from std_msgs.msg import Bool, String
        from ultron_b2_interfaces.action import NavigateToView
        from ultron_b2_interfaces.msg import LocalizationState, MotionGateStatus, OperatorEnable
        self.rclpy, self.Twist, self.Bool, self.String = rclpy, Twist, Bool, String
        self.OperatorEnable, self.NavigateToView = OperatorEnable, NavigateToView
        self.node = rclpy.create_node('d_nav_case_runner', namespace=NS,
                                      parameter_overrides=[Parameter('use_sim_time', value=True)])
        n = self.node
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(n)
        meta = json.loads((Path(map_dir) / 'map_meta.json').read_text())
        self.map_id, self.map_version = meta['map_id'], int(meta['map_version'])
        self.origin = tuple(meta.get('sim_run', {}).get('start_pose_world', origin))
        self.params = load_sim_slam_config().navigation
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.client = ActionClient(n, NavigateToView, 'nav/navigate_to_view')
        self.op_pub = n.create_publisher(OperatorEnable, 'ops/operator_enable', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.hold_pub = n.create_publisher(Bool, 'ops/hold', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.teleop_pub = n.create_publisher(Twist, 'control/in/teleop', 10)
        self.control_pub = n.create_publisher(String, 'sim/sim_control', rel)
        self.truth, self.gate_log, self.feedback = [], [], []
        self.chain = {'nav/cmd_vel_raw': [], 'control/in/autonomy': [], 'control/cmd_vel_muxed': [],
                      'control/cmd_vel_safe': [], 'cmd_vel': []}
        self.bridge_status, self.loc = {}, None
        n.create_subscription(PoseStamped, 'sim/ground_truth/base_pose', self._on_truth, rel)
        n.create_subscription(MotionGateStatus, 'safety/motion_gate', self._on_gate, 10)
        n.create_subscription(String, 'sim/bridge_status', lambda m: setattr(self, 'bridge_status',
                                                                            json.loads(m.data)), latched)
        n.create_subscription(LocalizationState, 'state/localization', lambda m: setattr(self, 'loc', m), rel)
        for topic in self.chain:
            n.create_subscription(Twist, topic, lambda m, t=topic: self.chain[t].append(
                (self.now(), m.linear.x, m.angular.z)), 10)
        self.operator = True
        self.seq = 0
        self.teleop = None                         # (vx, wz) streamed at 20 Hz when set
        n.create_timer(0.2, self._publish_operator)
        n.create_timer(0.05, self._publish_teleop)
        self.results = []

    # ------------------------------------------------------------------ io
    def _on_truth(self, msg):
        q = msg.pose.orientation
        self.truth.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.pose.position.x,
                           msg.pose.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)))

    def _on_gate(self, m):
        self.gate_log.append({'t': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 'open': m.open,
                              'reason': m.inhibited_reason, 'source': m.active_source})

    def _publish_operator(self):
        self.seq += 1
        m = self.OperatorEnable(schema_version=self.OperatorEnable.SCHEMA_VERSION, enabled=self.operator,
                                operator_id='d_nav_case_runner', reason='sim case runner', sequence=self.seq)
        m.header.stamp = self.node.get_clock().now().to_msg()
        self.op_pub.publish(m)

    def _publish_teleop(self):
        if self.teleop is not None:
            t = self.Twist()
            t.linear.x, t.angular.z = self.teleop
            self.teleop_pub.publish(t)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def spin(self, seconds=None, until=None, wall_limit=180.0):
        start, wall = self.now(), time.monotonic()
        while time.monotonic() - wall < wall_limit:
            self.ex.spin_once(timeout_sec=0.02)
            if until is not None and until():
                return True
            if seconds is not None and self.now() - start >= seconds:
                return True
        return False

    def control(self, **cmd):
        self.control_pub.publish(self.String(data=json.dumps(cmd)))

    def to_map(self, world):
        return compose(inverse(self.origin), world)

    def true_pose(self):
        return self.truth[-1][1:] if self.truth else None

    def moved_after(self, t0, t1=None):
        w = [s for s in self.truth if s[0] >= t0 and (t1 is None or s[0] <= t1)]
        if len(w) < 2:
            return 0.0
        x0, y0 = w[0][1:3]
        return max(math.hypot(x - x0, y - y0) for _, x, y, _ in w)

    def turned_after(self, t0, t1=None):
        w = [s for s in self.truth if s[0] >= t0 and (t1 is None or s[0] <= t1)]
        return abs(math.atan2(math.sin(w[-1][3] - w[0][3]), math.cos(w[-1][3] - w[0][3]))) if len(w) > 1 else 0.0

    def last_motion_time(self, after):
        moving = [b[0] for a, b in zip(self.truth, self.truth[1:])
                  if b[0] >= after and (math.hypot(b[1] - a[1], b[2] - a[2]) > 1e-4
                                        or abs(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))) > 1e-4)]
        return moving[-1] if moving else after

    def collisions(self):
        return int(self.bridge_status.get('collisions', 0))

    def gate_inhibit_after(self, t, pred=lambda r: r not in GATE_IGNORED):
        return next((g for g in self.gate_log if g['t'] >= t and not g['open'] and pred(g['reason'])), None)

    # ------------------------------------------------------------------ NavigateToView
    def goal_msg(self, world_xy, yaw_world=0.0, timeout=None):
        from ultron_b2_contracts import navigation_goal_to_msg
        x, y, yaw = self.to_map((world_xy[0], world_xy[1], yaw_world))
        g = make_goal(x, y, yaw, mission_id='NAV-TEST-D', map_id=self.map_id, map_version=self.map_version,
                      params=self.params, reason='agent D sim case', now=self.now(), timeout_s=timeout)
        return navigation_goal_to_msg(g.to_dict())

    def send(self, goal):
        self.feedback = []
        fut = self.client.send_goal_async(goal, feedback_callback=lambda f: self.feedback.append(
            (self.now(), f.feedback.status)))
        if not self.spin(until=fut.done, wall_limit=30):
            raise RuntimeError('NavigateToView goal not acknowledged')
        handle = fut.result()
        if not handle.accepted:
            raise RuntimeError('NavigateToView goal not accepted at the action level')
        return handle, handle.get_result_async()

    def run_goal(self, goal, wall_limit=240.0):
        handle, res = self.send(goal)
        if not self.spin(until=res.done, wall_limit=wall_limit):
            raise RuntimeError('NavigateToView result not received')
        return res.result().result

    @staticmethod
    def summary(r):
        return {'status': r.status, 'reason': r.reason, 'settled': r.settled, 'stop_acknowledged': r.stop_acknowledged}

    def wait_ready(self):
        if not self.spin(until=lambda: self.loc is not None and self.loc.state == 'VALID', wall_limit=90):
            raise RuntimeError(f'Localization never became VALID (last {self.loc and self.loc.state})')
        if not self.client.wait_for_server(timeout_sec=60):
            raise RuntimeError('nav/navigate_to_view not available')
        # Nav2 STARTUP is requested by nav_contract_server once map->base_link exists.
        ok = self.spin(until=lambda: False, seconds=8.0, wall_limit=30)
        return ok

    # ------------------------------------------------------------------ cases
    def case(self, name, fn):
        print(f'nav case {name} ...', flush=True)
        t0 = self.now()
        try:
            record = fn()
        except Exception as exc:                   # noqa: BLE001 - a failed case is recorded, the run continues
            record = {'checks': {'ran': False}, 'error': repr(exc)}
        finally:
            self.teleop, self.operator = None, True
            self.hold_pub.publish(self.Bool(data=False))
        record.update(case=name, started=round(t0, 3), ended=round(self.now(), 3))
        record['verdict'] = 'PASS' if record['checks'] and all(record['checks'].values()) else 'FAIL'
        self.results.append(record)
        print(f'  -> {record["verdict"]} {json.dumps({k: v for k, v in record.items() if k != "checks"}, default=str)}'
              f' {record["checks"]}', flush=True)
        self.spin(1.0)

    def arrive(self, world_xy, yaw_world):
        r = self.run_goal(self.goal_msg(world_xy, yaw_world))
        tx, ty, tyaw = self.true_pose()
        err_xy = math.hypot(tx - world_xy[0], ty - world_xy[1])
        err_yaw = abs(math.atan2(math.sin(tyaw - yaw_world), math.cos(tyaw - yaw_world)))
        return {**self.summary(r), 'true_error_xy_m': round(err_xy, 3), 'true_error_yaw_rad': round(err_yaw, 3),
                'checks': {'reached': r.status == 'REACHED', 'settled': r.settled,
                           'xy_within': err_xy <= ACCEPT['arrive_xy_m'], 'yaw_within': err_yaw <= ACCEPT['arrive_yaw_rad']}}

    def rejected(self, world_xy, expect):
        t0 = self.now()
        r = self.run_goal(self.goal_msg(world_xy))
        self.spin(2.0)
        moved = self.moved_after(t0)
        return {**self.summary(r), 'moved_m': round(moved, 4),
                'checks': {'rejected': r.status == 'REJECTED', 'reason_matches': expect in r.reason,
                           'no_motion': moved <= ACCEPT['still_m']}}

    def unreachable(self):
        target = (7.5, 4.5)
        self.control(obstacle={'id': 0, 'x': target[0], 'y': target[1]})
        self.spin(1.0)
        t0 = self.now()
        r = self.run_goal(self.goal_msg(target, timeout=ACCEPT['unreachable_timeout_s']))
        t_end = self.now()
        self.spin(3.0)
        stop_after = self.last_motion_time(t_end) - t_end
        self.control(obstacle={'id': 0, 'x': None, 'y': None})
        return {**self.summary(r), 'elapsed_s': round(t_end - t0, 2), 'stop_after_s': round(stop_after, 3),
                'checks': {'failed_or_timeout': r.status in ('FAILED', 'TIMEOUT'),
                           'bounded': t_end - t0 <= ACCEPT['unreachable_timeout_s'] + 5.0,
                           'stopped_in_time': stop_after <= ACCEPT['stop_within_s']}}

    def new_obstacle(self):
        before = self.collisions()
        handle, res = self.send(self.goal_msg((2.5, 5.5), math.pi))
        start = self.true_pose()
        self.spin(until=lambda: math.dist(self.true_pose()[:2], start[:2]) > 1.0 or res.done(), wall_limit=60)
        x, y, yaw = self.true_pose()
        spot = (x + 1.6 * math.cos(yaw), y + 1.6 * math.sin(yaw))
        self.control(obstacle={'id': 1, 'x': spot[0], 'y': spot[1]})
        self.spin(until=res.done, wall_limit=240)
        r = res.result().result
        self.control(obstacle={'id': 1, 'x': None, 'y': None})
        collisions = self.collisions() - before
        return {**self.summary(r), 'obstacle_at_world': [round(v, 2) for v in spot], 'collisions': collisions,
                'checks': {'replanned_or_stopped': r.status in ('REACHED', 'FAILED', 'TIMEOUT'),
                           'no_collision': collisions == 0,
                           'no_false_arrival': r.status != 'REACHED'
                           or math.dist(self.true_pose()[:2], (2.5, 5.5)) <= ACCEPT['arrive_xy_m']}}

    def stop_case(self, trigger):
        handle, res = self.send(self.goal_msg((9.6, 7.8), 0.0))
        self.spin(until=lambda: any(s == 'ACTIVE' for _, s in self.feedback) or res.done(), wall_limit=30)
        self.spin(3.0)
        t = self.now()
        trigger(handle)
        self.spin(until=res.done, wall_limit=30)
        self.spin(3.0)
        r = res.result().result if res.done() else None
        return r, t, self.last_motion_time(t)

    def cancel(self):
        r, t, stopped = self.stop_case(lambda h: h.cancel_goal_async())
        return {**self.summary(r), 'stop_after_s': round(stopped - t, 3),
                'checks': {'cancelled': r.status == 'CANCELLED', 'stopped_in_time': stopped - t <= ACCEPT['stop_within_s'],
                           'stop_acknowledged': r.stop_acknowledged}}

    def operator_inhibit(self):
        def disable(_h):
            self.operator = False
            self._publish_operator()
        r, t, stopped = self.stop_case(disable)
        inhibit = self.gate_inhibit_after(t)
        self.operator = True
        delay = inhibit['t'] - t if inhibit else None
        return {**self.summary(r), 'gate_reason': inhibit and inhibit['reason'], 'gate_delay_s': delay and round(delay, 3),
                'stop_after_s': round(stopped - t, 3),
                'checks': {'cancelled': r.status == 'CANCELLED',
                           'gate_inhibited_fast': delay is not None and delay <= ACCEPT['gate_after_operator_s'],
                           'gate_reason_operator': bool(inhibit) and inhibit['reason'].startswith('OPERATOR'),
                           'stopped_in_time': stopped - t <= ACCEPT['stop_within_s']}}

    def localization_loss(self):
        r, t, stopped = self.stop_case(lambda _h: self.control(fault='lidar_blackout', duration_s=3.0))
        inhibit = self.gate_inhibit_after(t, lambda reason: reason.startswith('LOCALIZATION'))
        delay = inhibit['t'] - t if inhibit else None
        moved = self.moved_after(t + ACCEPT['gate_after_blackout_s'] + ACCEPT['stop_within_s'], t + 3.0)
        self.spin(until=lambda: self.loc is not None and self.loc.state == 'VALID', wall_limit=60)
        return {**self.summary(r), 'gate_reason': inhibit and inhibit['reason'], 'gate_delay_s': delay and round(delay, 3),
                'stop_after_s': round(stopped - t, 3), 'moved_while_invalid_m': round(moved, 4),
                'checks': {'cancelled': r.status == 'CANCELLED',
                           'gate_inhibited_in_time': delay is not None and delay <= ACCEPT['gate_after_blackout_s'],
                           'no_motion_while_invalid': moved <= ACCEPT['still_m']}}

    def _to_open_spot(self):
        r = self.run_goal(self.goal_msg((2.5, 5.5), 0.0))
        if r.status != 'REACHED':
            raise RuntimeError(f'setup goal failed: {r.status} {r.reason}')
        self.spin(1.0)

    def teleop_hold(self):
        """Teleop (rotate in place) through the full chain, then HOLD wins over teleop."""
        self._to_open_spot()
        t0 = self.now()
        self.teleop = (0.0, 0.4)
        self.spin(3.0)
        turned = self.turned_after(t0)
        t_hold = self.now()
        self.hold_pub.publish(self.Bool(data=True))
        self.spin(2.5)
        stop_after = self.last_motion_time(t_hold) - t_hold
        inhibit = self.gate_inhibit_after(t_hold, lambda reason: reason == 'HOLD')
        self.teleop = None
        self.hold_pub.publish(self.Bool(data=False))
        return {'turned_rad': round(turned, 3), 'stop_after_s': round(stop_after, 3),
                'gate_delay_s': inhibit and round(inhibit['t'] - t_hold, 3),
                'checks': {'teleop_moved': turned > 0.5, 'hold_inhibits': inhibit is not None,
                           'stopped_in_time': stop_after <= ACCEPT['stop_within_s']}}

    def teleop_collision(self):
        """Teleop forward toward a test obstacle: collision_monitor (not exempt for teleop) must limit it so the
        robot halts clear of the obstacle. The deterministic hard zero of a stop-zone hit is covered by
        ultron_b2_control/test/test_chain_graph.py; here the obstacle also lowers the scan/map match score, so the
        final hold may come from the motion gate (teleop_requires_localization) - recorded, not hidden."""
        self._to_open_spot()
        before = self.collisions()
        x, y, yaw = self.true_pose()
        spot = (x + 1.8 * math.cos(yaw), y + 1.8 * math.sin(yaw))
        self.control(obstacle={'id': 2, 'x': spot[0], 'y': spot[1]})
        self.spin(1.5)
        t0 = self.now()
        self.teleop = (0.3, 0.0)
        self.spin(8.0)
        late_gate = [g for g in self.gate_log if t0 + 6.0 <= g['t'] <= t0 + 8.0]
        self.teleop = None
        self.spin(1.0)
        px, py, _ = self.true_pose()
        gap = math.hypot(spot[0] - px, spot[1] - py)
        muxed = [v for t, v, _ in self.chain['control/cmd_vel_muxed'] if t0 + 6.0 <= t <= t0 + 8.0]
        safe = [v for t, v, _ in self.chain['control/cmd_vel_safe'] if t0 + 6.0 <= t <= t0 + 8.0]
        safe_last = [v for t, v, _ in self.chain['control/cmd_vel_safe'] if t0 <= t <= t0 + 8.0][-1:]
        moved_late = self.moved_after(t0 + 6.0, t0 + 8.0)
        self.control(obstacle={'id': 2, 'x': None, 'y': None})
        collisions = self.collisions() - before
        # Humble collision_monitor may go silent after a stop-zone hit; the gate then zeroes on COMMAND_STALE.
        cm_limit = max((abs(v) for v in safe), default=abs(safe_last[0]) if safe_last else None)
        return {'travelled_m': round(self.moved_after(t0), 3), 'final_center_gap_m': round(gap, 3),
                'collisions': collisions, 'muxed_vx_late': muxed[-1:] and round(muxed[-1], 3),
                'safe_vx_late_max': cm_limit if cm_limit is None else round(cm_limit, 3),
                'moved_late_m': round(moved_late, 4),
                'gate_late_reasons': sorted({g['reason'] or 'OPEN' for g in late_gate}),
                'checks': {'moved_toward': self.moved_after(t0) > 0.1, 'no_collision': collisions == 0,
                           'stopped_before_obstacle': gap > 0.3 + 0.55,
                           'collision_monitor_limited_teleop': bool(muxed) and muxed[-1] > 0.25
                           and cm_limit is not None and cm_limit <= 0.05,
                           'halted': moved_late <= ACCEPT['still_m']}}

    def teleop_no_localization(self):
        """teleop_requires_localization defaults true: during a lidar blackout teleop must not move the robot."""
        self.spin(until=lambda: self.loc is not None and self.loc.state == 'VALID', wall_limit=60)
        self.control(fault='lidar_blackout', duration_s=4.0)
        self.spin(until=lambda: self.loc is not None and self.loc.state != 'VALID', wall_limit=5)
        self.spin(0.3)
        t0 = self.now()
        self.teleop = (0.0, 0.4)
        self.spin(2.0)
        turned = self.turned_after(t0 + 0.3)
        inhibit = self.gate_inhibit_after(t0, lambda reason: reason.startswith('LOCALIZATION'))
        self.teleop = None
        self.spin(until=lambda: self.loc is not None and self.loc.state == 'VALID', wall_limit=60)
        return {'turned_rad': round(turned, 4), 'gate_reason': inhibit and inhibit['reason'],
                'checks': {'gate_inhibited': inhibit is not None, 'no_motion': turned < 0.02}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('out', type=Path)
    parser.add_argument('--map', type=Path, required=True, help='saved map directory (C map store v<N>)')
    parser.add_argument('--cases', default=','.join(ALL_CASES))
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--odom-source', default='mola', choices=['sim', 'mola', 'kiss'])
    parser.add_argument('--origin', default=','.join(str(v) for v in DEFAULT_START),
                        help='world pose x,y,yaw of the map frame (mapping start pose)')
    parser.add_argument('--launch-arg', action='append', default=[], help='extra sim.launch.py key:=value')
    args = parser.parse_args(argv)
    cases = [c for c in args.cases.split(',') if c]
    if set(cases) - set(ALL_CASES):
        raise SystemExit(f'unknown cases {sorted(set(cases) - set(ALL_CASES))}; choose from {ALL_CASES}')
    if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('REFUSED: simulation needs a non-zero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    map_dir = args.map.resolve()
    origin = tuple(float(v) for v in args.origin.split(','))
    env = dict(os.environ)
    repo_src = str(Path(sys.modules['ultron'].__file__).resolve().parents[1])
    env['PYTHONPATH'] = repo_src + os.pathsep + env.get('PYTHONPATH', '')
    launch = ['ros2', 'launch', 'ultron_b2_bringup', 'sim.launch.py', 'mode:=nav', f'map:={map_dir}',
              f'odom_source:={args.odom_source}', 'initial_pose:=0,0,0', f'sim_rate:={args.rate}',
              *args.launch_arg]
    (out / 'launch_cmd.txt').write_text(' '.join(launch) + '\n')
    procs = {}
    try:
        procs['stack'] = spawn(launch, out / 'stack.log', env)
        time.sleep(8.0)
        if procs['stack'].poll() is not None:
            raise RuntimeError(f'stack launch exited early (code {procs["stack"].returncode}); see {out}')
        import rclpy
        rclpy.init()
        try:
            runner = Runner(map_dir, origin)
            print('waiting for VALID localization, nav/navigate_to_view and Nav2 startup ...', flush=True)
            runner.wait_ready()
            plan = {'reach_near': lambda: runner.arrive((2.5, 5.5), 0.0),
                    'reach_far': lambda: runner.arrive((9.6, 4.5), -math.pi / 2),
                    'reject_occupied': lambda: runner.rejected((4.0, 4.0), 'OCCUPIED'),
                    'reject_outside': lambda: runner.rejected((14.0, 5.0), 'OUTSIDE_MAP'),
                    'unreachable': runner.unreachable, 'new_obstacle': runner.new_obstacle,
                    'cancel': runner.cancel, 'operator_inhibit': runner.operator_inhibit,
                    'localization_loss': runner.localization_loss, 'teleop_hold': runner.teleop_hold,
                    'teleop_collision': runner.teleop_collision,
                    'teleop_no_localization': runner.teleop_no_localization}
            for name in cases:
                runner.case(name, plan[name])
            summary = {'schema': 'ultron.b2_nav_chain_cases/v1', 'map_id': runner.map_id,
                       'map_version': runner.map_version, 'accept': ACCEPT, 'chain': __doc__.split('\n\n')[1],
                       'stack': ' '.join(launch), 'odom_source': args.odom_source, 'origin_world': origin,
                       'cases': runner.results, 'gate_log_len': len(runner.gate_log),
                       'verdict': 'PASS' if all(r['verdict'] == 'PASS' for r in runner.results) else 'FAIL'}
            (out / 'nav_cases.json').write_text(json.dumps(summary, indent=2, default=str))
            (out / 'chain_trace.json').write_text(json.dumps({'chain': runner.chain, 'gate': runner.gate_log,
                                                               'truth': runner.truth[::5]}))
            print(json.dumps({r['case']: r['verdict'] for r in runner.results}, indent=1))
            print('verdict:', summary['verdict'], flush=True)
        finally:
            rclpy.shutdown()
    finally:
        for proc in procs.values():
            stop(proc, timeout=40)


if __name__ == '__main__':
    main()
