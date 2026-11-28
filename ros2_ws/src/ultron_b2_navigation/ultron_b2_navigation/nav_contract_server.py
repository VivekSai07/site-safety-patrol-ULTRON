"""nav_contract_server: ``nav/navigate_to_view`` (ultron_b2_interfaces/NavigateToView) wrapping Nav2 ``navigate_to_pose``.

Port of ``ultron.autonomy.nav2_adapter``. One goal at a time. Every goal is accepted at the action
level and then answered with a contract status in the result:

  REACHED   -> succeed()   (``settled`` = CAPTURE_READY: still for settle_time_s with VALID localization
                            inside the goal tolerances; false if not settled within settle_timeout_s)
  FAILED    <- a Nav2 success not confirmed by a VALID localized pose within goal tolerance + 0.3 m after
               settle_timeout_s (BACKEND_REACHED_OUT_OF_TOLERANCE; guards Humble's false "Reached the goal")
  CANCELLED -> canceled() when the client requested it, else abort()
  REJECTED / TIMEOUT / FAILED -> abort()

Pre-checks (REJECTED, never reach Nav2): another goal active, schema, frame, map id/version, finite
values, operator enable heartbeat, VALID fresh localization, target footprint occupied / unknown /
outside map, target inside a keepout, Nav2 not ready. While active: timeout -> cancel -> TIMEOUT;
motion gate inhibited (except NO_COMMAND/COMMAND_STALE) or teleop/HOLD active -> cancel; late
results from older goals are ignored (GoalTracker).
"""
import json
import math
from pathlib import Path
import threading
import time
import uuid

import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry

from ultron_b2_interfaces.action import NavigateToView
from ultron_b2_interfaces.msg import LocalizationState as LocalizationStateMsg
from ultron_b2_interfaces.msg import MapInfo, MotionGateStatus, OperatorEnable
from ultron_b2_contracts import (msg_to_localization_state, msg_to_navigation_goal, navigation_result_to_msg,
                                 ContractError)

from .gate_logic import localization_from_dict
from .ultron_path import ensure_ultron

ensure_ultron()
from ultron.autonomy.localization_state import accept_pose, wrap  # noqa: E402
from ultron.autonomy.navigation_contract import (GoalTracker, MapFootprint, NavigationGoal,  # noqa: E402
                                                 NavigationResult, TERMINAL)

NAV_DEFAULTS = {
    'goal_xy_tolerance_m': 0.25, 'goal_yaw_tolerance_rad': 0.35, 'default_timeout_s': 90.0,
    'settle_time_s': 0.8, 'settle_speed_m_s': 0.02, 'settle_yaw_rate_rad_s': 0.03,
    'target_clearance_m': 0.55, 'operator_enable_max_age_s': 1.0,
    'settle_timeout_s': 10.0,          # PROPOSED
    'cancel_confirm_timeout_s': 5.0,   # PROPOSED
    'stop_ack_timeout_s': 2.0,         # PROPOSED
    'gate_status_max_age_s': 0.5,      # PROPOSED
}
GATE_IGNORED = ('', 'NO_COMMAND', 'COMMAND_STALE')


class GridFootprint(MapFootprint):
    """MapFootprint over a live OccupancyGrid (0 free, 100 occupied, -1 unknown)."""

    def __init__(self, msg, occupied_thresh=65, free_thresh=20):   # noqa: D107 - no super().__init__ (file based)
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        data = np.flipud(data)                     # image row order (row 0 = max y), as MapFootprint
        self.occupied = data >= occupied_thresh
        self.free = (data >= 0) & (data < free_thresh)
        self.res = float(msg.info.resolution)
        self.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.height, self.width = data.shape


def in_keepout(mask, x, y, radius):
    if mask is None:
        return False
    info = mask.info
    res = info.resolution
    data = np.asarray(mask.data, dtype=np.int16).reshape(info.height, info.width)
    n = int(math.ceil(radius / res))
    c0 = int(math.floor((x - info.origin.position.x) / res))
    r0 = int(math.floor((y - info.origin.position.y) / res))
    for dr in range(-n, n + 1):
        for dc in range(-n, n + 1):
            if (dr * dr + dc * dc) * res * res > radius * radius:
                continue
            r, c = r0 + dr, c0 + dc
            if 0 <= r < info.height and 0 <= c < info.width and data[r, c] >= 100:
                return True
    return False


class NavContractServer(Node):
    def __init__(self):
        super().__init__('nav_contract_server')
        for k, v in NAV_DEFAULTS.items():
            self.declare_parameter(k, v)
        self.declare_parameter('map_meta_path', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_id', '')
        self.declare_parameter('map_version', 0)
        self.declare_parameter('use_map_topic_for_checks', True)
        # Nav2 startup gating: 'on_tf' => STARTUP lifecycle_manager_navigation only once the clock runs and
        # map->base_link exists (Nav2 is never activated without localization TF); 'none' => external/autostart.
        self.declare_parameter('nav2_startup', 'on_tf')
        self.declare_parameter('nav2_lifecycle_manager', 'lifecycle_manager_navigation')
        self.p = {k: self.get_parameter(k).value for k in NAV_DEFAULTS}
        self.map_id, self.map_version = self._initial_map()
        map_yaml = str(self.get_parameter('map_yaml').value)
        self.footprint = MapFootprint(map_yaml) if map_yaml else None
        self.lock = threading.RLock()
        self.busy = False
        self.localization = None
        self.operator = (None, None)
        self.gate = None
        self.gate_stamp = None
        self.speed = self.yaw_rate = 0.0
        self.keepout = None
        cb = ReentrantCallbackGroup()
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LocalizationStateMsg, 'state/localization', self._on_loc, reliable,
                                 callback_group=cb)
        self.create_subscription(OperatorEnable, 'ops/operator_enable', self._on_operator, reliable,
                                 callback_group=cb)
        self.create_subscription(MotionGateStatus, 'safety/motion_gate', self._on_gate, 10, callback_group=cb)
        self.create_subscription(Odometry, 'state/odom', self._on_odom, 10, callback_group=cb)
        self.create_subscription(OccupancyGrid, 'nav/keepout_mask', lambda m: setattr(self, 'keepout', m),
                                 latched, callback_group=cb)
        self.create_subscription(MapInfo, 'map/info', self._on_map_info, latched, callback_group=cb)
        if self.footprint is None and self.get_parameter('use_map_topic_for_checks').value:
            self.create_subscription(OccupancyGrid, 'map', self._on_map, latched, callback_group=cb)
        self.nav2 = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self.server = ActionServer(self, NavigateToView, 'nav/navigate_to_view', self._execute,
                                   goal_callback=lambda _g: GoalResponse.ACCEPT,
                                   cancel_callback=lambda _h: CancelResponse.ACCEPT, callback_group=cb)
        self.nav2_started = False
        self._startup_future = None
        if self.get_parameter('nav2_startup').value == 'on_tf':
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.lifecycle_cli = self.create_client(
                ManageLifecycleNodes, f'{self.get_parameter("nav2_lifecycle_manager").value}/manage_nodes',
                callback_group=cb)
            self.create_timer(1.0, self._maybe_start_nav2, callback_group=cb)
        self.get_logger().info(f'nav/navigate_to_view ready (map {self.map_id or "<none>"} v{self.map_version})')

    def _maybe_start_nav2(self):
        if self.nav2_started or (self._startup_future is not None and not self._startup_future.done()):
            return
        if self._startup_future is not None:
            res = self._startup_future.result()
            self._startup_future = None
            if res is not None and res.success:
                self.nav2_started = True
                self.get_logger().info('Nav2 lifecycle STARTUP complete')
                return
            self.get_logger().warn('Nav2 lifecycle STARTUP failed; retrying')
        if self.now() <= 0 or not self.lifecycle_cli.service_is_ready():
            return
        if not self.tf_buffer.can_transform('map', 'base_link', Time()):
            return
        self.get_logger().info('map->base_link available: requesting Nav2 lifecycle STARTUP')
        self._startup_future = self.lifecycle_cli.call_async(
            ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP))

    # ------------------------------------------------------------------ inputs
    def _initial_map(self):
        meta = str(self.get_parameter('map_meta_path').value)
        if meta:
            p = Path(meta)
            data = json.loads((p / 'map_meta.json' if p.is_dir() else p).read_text())
            return str(data['map_id']), int(data['map_version'])
        return str(self.get_parameter('map_id').value), int(self.get_parameter('map_version').value)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_loc(self, msg):
        try:
            self.localization = localization_from_dict(msg_to_localization_state(msg))
        except Exception:                          # noqa: BLE001 - malformed counts as missing
            self.localization = None

    def _on_operator(self, msg):
        self.operator = (bool(msg.enabled), self.now())

    def _on_gate(self, msg):
        self.gate, self.gate_stamp = msg, self.now()

    def _on_odom(self, msg):
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.yaw_rate = msg.twist.twist.angular.z

    def _on_map(self, msg):
        self.footprint = GridFootprint(msg)

    def _on_map_info(self, msg):
        if not self.map_id:
            self.map_id, self.map_version = msg.map_id, int(msg.map_version)

    def _pose(self):
        s = self.localization
        if s is None or not accept_pose(s, self.map_id, self.map_version, self.now())[0]:
            return None
        return [s.position[0], s.position[1], s.yaw]

    def _operator_ok(self):
        enabled, stamp = self.operator
        return bool(enabled) and stamp is not None and self.now() - stamp <= self.p['operator_enable_max_age_s']

    # ------------------------------------------------------------------ checks
    def _to_goal(self, g):
        d = msg_to_navigation_goal(g)
        values = (d['x'], d['y'], d['yaw'], d['timeout_s'], d['xy_tolerance_m'], d['yaw_tolerance_rad'])
        if not all(math.isfinite(v) for v in values):
            raise ContractError('NONFINITE_GOAL')
        d['goal_id'] = d['goal_id'] or f'GOAL-{uuid.uuid4().hex[:10]}'
        d['timeout_s'] = d['timeout_s'] if d['timeout_s'] > 0 else self.p['default_timeout_s']
        d['xy_tolerance_m'] = d['xy_tolerance_m'] if d['xy_tolerance_m'] > 0 else self.p['goal_xy_tolerance_m']
        d['yaw_tolerance_rad'] = d['yaw_tolerance_rad'] if d['yaw_tolerance_rad'] > 0 \
            else self.p['goal_yaw_tolerance_rad']
        d['requested_at'] = self.now()             # timeout counts from receipt (server clock)
        fields = set(NavigationGoal.__dataclass_fields__)
        return NavigationGoal(**{k: v for k, v in d.items() if k in fields})

    def precheck(self, goal):
        if goal.frame_id != 'map':
            return False, f'UNKNOWN_FRAME: {goal.frame_id}'
        if not self.map_id:
            return False, 'NO_MAP_LOADED'
        if goal.map_id != self.map_id or goal.map_version != self.map_version:
            return False, f'MAP_MISMATCH: goal {goal.map_id} v{goal.map_version}, loaded {self.map_id} v{self.map_version}'
        if not self._operator_ok():
            return False, 'OPERATOR_MOTION_NOT_ENABLED'
        ok, why = accept_pose(self.localization, self.map_id, self.map_version, self.now())
        if not ok:
            return False, f'LOCALIZATION: {why}'
        if self.footprint is None:
            return False, 'NO_MAP_FOR_TARGET_CHECK'
        ok, why = self.footprint.check(goal.x, goal.y, self.p['target_clearance_m'])
        if not ok:
            return False, why
        if in_keepout(self.keepout, goal.x, goal.y, self.p['target_clearance_m']):
            return False, 'TARGET_IN_KEEPOUT'
        if not self.nav2.wait_for_server(timeout_sec=2.0):
            return False, 'NAV2_ACTION_SERVER_NOT_READY'
        return True, 'OK'

    # ------------------------------------------------------------------ execution
    def _execute(self, handle):
        with self.lock:
            if self.busy:
                return self._finish(handle, NavigationResult(handle.request.goal_id or '?', 'REJECTED',
                                                             'GOAL_ALREADY_ACTIVE', self.now()))
            self.busy = True
        try:
            return self._run(handle)
        finally:
            with self.lock:
                self.busy = False

    def _run(self, handle):
        try:
            goal = self._to_goal(handle.request)
        except (ContractError, KeyError, ValueError) as exc:
            return self._finish(handle, NavigationResult(handle.request.goal_id or '?', 'REJECTED',
                                                         f'INVALID_GOAL: {exc}', self.now()))
        tracker = GoalTracker(self.p)
        ok, reason = self.precheck(goal)
        rejected = tracker.submit(goal, ok, reason, self.now())
        if rejected:
            self.get_logger().warn(f'{goal.goal_id} REJECTED: {reason}')
            return self._finish(handle, rejected)
        self.get_logger().info(f'{goal.goal_id} -> Nav2 ({goal.x:.2f}, {goal.y:.2f}, {goal.yaw:.2f})')
        state = {'nav2': None, 'cancel_at': None}
        self._send_nav2(goal, tracker, state)
        reached_at = None
        while rclpy.ok():
            now = self.now()
            with self.lock:
                if tracker.tick(now) == 'CANCEL':
                    self._cancel_nav2(state, now)
                if tracker.status in ('ACCEPTED', 'ACTIVE'):
                    why = self._interrupt_reason(handle)
                    if why and tracker.request_cancel(why, now):
                        self.get_logger().warn(f'{goal.goal_id} cancelling: {why}')
                        self._cancel_nav2(state, now)
                    if state['cancel_at'] is not None and now - state['cancel_at'] > self.p['cancel_confirm_timeout_s']:
                        tracker._finish('FAILED', f'{tracker.cancel_requested}: Nav2 did not confirm cancel '
                                        f'within {self.p["cancel_confirm_timeout_s"]} s', now)
                status = tracker.status
            if status == 'REACHED':
                reached_at = reached_at if reached_at is not None else now
                pose = self._pose()
                within = pose is not None and math.hypot(pose[0] - goal.x, pose[1] - goal.y) <= goal.xy_tolerance_m \
                    and abs(wrap(pose[2] - goal.yaw)) <= goal.yaw_tolerance_rad
                if tracker.settle(now, self.speed, self.yaw_rate, within, pose or [math.nan] * 3):
                    return self._finish(handle, tracker.result)
                if now - reached_at > self.p['settle_timeout_s']:
                    with self.lock:
                        r = tracker.settle_expired(now, pose, self.p['settle_timeout_s'])
                    if r.status == 'FAILED':
                        r.stop_acknowledged = self._wait_still()
                    return self._finish(handle, r)
            elif status in TERMINAL:
                r = tracker.result
                r.last_valid_pose = self._pose()
                r.stop_acknowledged = self._wait_still()
                return self._finish(handle, r)
            self._feedback(handle, goal, status)
            time.sleep(0.05)
        return self._finish(handle, NavigationResult(goal.goal_id, 'FAILED', 'SHUTDOWN', self.now()))

    def _interrupt_reason(self, handle):
        if handle.is_cancel_requested:
            return 'CLIENT_CANCEL'
        g = self.gate
        if g is None or self.gate_stamp is None or self.now() - self.gate_stamp > self.p['gate_status_max_age_s']:
            return 'MOTION_GATE: status missing or stale'
        if g.active_source in ('hold', 'teleop'):
            return f'PREEMPTED: active source {g.active_source}'
        if not g.open and g.inhibited_reason not in GATE_IGNORED:
            return f'MOTION_GATE: {g.inhibited_reason}'
        return None

    def _send_nav2(self, goal, tracker, state):
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = goal.x, goal.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(goal.yaw / 2), math.cos(goal.yaw / 2)
        gid = goal.goal_id

        def on_result(f):
            with self.lock:
                tracker.on_backend_status(gid, f.result().status, self.now(), 'Nav2 result')

        def on_response(f):
            h = f.result()
            with self.lock:
                if not h.accepted:
                    tracker.on_backend_status(gid, 6, self.now(), 'Nav2 rejected the goal')
                    return
                state['nav2'] = h
                tracker.on_backend_status(gid, 2, self.now(), 'Nav2 executing')
                if tracker.cancel_requested:           # cancel raced the acceptance
                    h.cancel_goal_async()
            h.get_result_async().add_done_callback(on_result)

        self.nav2.send_goal_async(msg).add_done_callback(on_response)

    def _cancel_nav2(self, state, now):
        state['cancel_at'] = now
        if state['nav2'] is not None:
            state['nav2'].cancel_goal_async()

    def _wait_still(self):
        deadline = time.monotonic() + self.p['stop_ack_timeout_s']
        while time.monotonic() < deadline:
            if abs(self.speed) <= self.p['settle_speed_m_s'] and abs(self.yaw_rate) <= self.p['settle_yaw_rate_rad_s']:
                return True
            time.sleep(0.05)
        return False

    def _feedback(self, handle, goal, status):
        fb = NavigateToView.Feedback()
        fb.status = status or ''
        pose = self._pose()
        fb.pose.header.frame_id = 'map'
        fb.pose.header.stamp = self.get_clock().now().to_msg()
        if pose is not None:
            fb.pose.pose.position.x, fb.pose.pose.position.y = pose[0], pose[1]
            fb.pose.pose.orientation.z, fb.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
            fb.distance_remaining_m = math.hypot(goal.x - pose[0], goal.y - pose[1])
        else:
            fb.distance_remaining_m = math.nan
        handle.publish_feedback(fb)

    def _finish(self, handle, result):
        msg = navigation_result_to_msg(result.to_dict())
        if result.status == 'REACHED':
            handle.succeed()
        elif result.status == 'CANCELLED' and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        self.get_logger().info(f'{result.goal_id} {result.status}: {result.reason} '
                               f'(settled={msg.settled}, stop_ack={msg.stop_acknowledged})')
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = NavContractServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
