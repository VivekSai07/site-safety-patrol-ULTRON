"""Nav2 NavigateToPose adapter implementing ULTRON's navigation contract. SIMULATION ONLY.

Wraps the Nav2 action with pre-checks (map, localization, operator, target footprint), one
active goal, timeout cancellation, cancellation when the motion gate inhibits, late-result
rejection and a settle criterion for capture readiness. It never publishes velocity itself.
"""
from collections import deque
import json
import math
from pathlib import Path

from .localization_state import accept_pose
from .live_grid import TOPIC, grid_is_fresh, grid_stamp_ns
from .motion_gate_node import state_from_json
from .navigation_contract import GoalTracker, MapFootprint, check_goal
from .rtabmap_sim import ODOM as RTABMAP_ODOM

ACTION = '/navigate_to_pose'


class Nav2Adapter:
    def __init__(self, node, config, map_dir):
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid, Odometry
        from std_msgs.msg import Bool, String
        self.node, self.config = node, config
        meta = json.loads((Path(map_dir) / 'map_meta.json').read_text())
        self.map_id, self.map_version = meta['map_id'], int(meta['map_version'])
        self.live_map = meta.get('live_map', False)
        self.grid_stamp_ns = 0
        self.grid_history = deque(maxlen=8)  # More than the fresh window at the mapping publish rate.
        self.footprint = None if self.live_map else MapFootprint(Path(map_dir) / 'map.yaml')
        self.NavigateToPose = NavigateToPose
        self.client = ActionClient(node, NavigateToPose, ACTION)
        self.tracker = GoalTracker(config.navigation)
        self.localization = None
        self.gate = {'inhibited': True, 'reason': 'NO_GATE_STATUS'}
        self.speed = self.yaw_rate = 0.0
        self.operator_enabled = False
        self.operator_stamp = None
        self.handle = None
        self.results = []
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        t = config.topics
        node.create_subscription(String, t['localization_state'], self._on_state, latched)
        node.create_subscription(String, t['gate_status'], self._on_gate, latched)
        node.create_subscription(Bool, t['operator_enable'], self._on_operator, qos)
        odom_topic = RTABMAP_ODOM if meta.get('slam_backend') == 'rtabmap_3d' else t['odom']
        node.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        if self.live_map:
            node.create_subscription(OccupancyGrid, TOPIC, self._on_grid, latched)

    def _on_grid(self, msg):
        stamp = grid_stamp_ns(msg)
        if stamp <= self.grid_stamp_ns:
            return
        try:
            self.footprint = MapFootprint.from_grid(msg)
            self.grid_history.append(stamp)
        except ValueError:
            self.footprint = None
        self.grid_stamp_ns = stamp

    def _live_target(self, goal, now, displayed_grid):
        if not grid_is_fresh(self.grid_stamp_ns, now) or self.footprint is None:
            return False, 'LIVE_MAP_UNAVAILABLE_OR_STALE'
        if displayed_grid <= 0 or displayed_grid > self.grid_stamp_ns:
            return False, 'LIVE_MAP_REVISION_UNAVAILABLE'
        if not grid_is_fresh(displayed_grid, now):
            return False, 'LIVE_MAP_REVISION_STALE'
        if displayed_grid not in self.grid_history:
            return False, 'LIVE_MAP_REVISION_UNAVAILABLE'
        return self.footprint.check(goal.x, goal.y, self.config.navigation['target_clearance_m'])

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg):
        try:
            self.localization = state_from_json(msg.data)
        except (TypeError, ValueError):
            self.localization = None

    def _on_gate(self, msg):
        try:
            status = json.loads(msg.data)
            if (status['map_id'], status['map_version']) != (self.map_id, self.map_version):
                raise ValueError('Motion gate has a different map')
            if type(status['inhibited']) is not bool:
                raise ValueError('Invalid gate status')
            self.gate = status
        except (KeyError, TypeError, ValueError):
            self.gate = {'inhibited': True, 'reason': 'INVALID_GATE_STATUS'}

    def _on_operator(self, msg):
        self.operator_enabled, self.operator_stamp = msg.data, self.now()

    def operator_allowed(self, now):
        return (self.operator_enabled and self.operator_stamp is not None
                and 0 <= now - self.operator_stamp <= self.config.navigation['operator_enable_max_age_s'])

    def _on_odom(self, msg):
        self.speed, self.yaw_rate = msg.twist.twist.linear.x, msg.twist.twist.angular.z

    # --- goal lifecycle ----------------------------------------------------------------
    def submit(self, goal, *, displayed_grid=0):
        """Pre-check and send. Returns a terminal NavigationResult if rejected, else None."""
        now = self.now()
        target_check = (lambda: self._live_target(goal, now, displayed_grid)) if self.live_map else None
        ok, reason = check_goal(goal, map_id=self.map_id, map_version=self.map_version, localization=self.localization,
                                now=now, operator_enabled=self.operator_allowed(now), footprint=self.footprint,
                                clearance_m=self.config.navigation['target_clearance_m'], target_check=target_check)
        if ok and not self.client.server_is_ready():
            ok, reason = False, 'NAV2_ACTION_SERVER_NOT_READY'
        rejected = self.tracker.submit(goal, ok, reason, now)
        if rejected:
            self.results.append(rejected)
            return rejected
        self.handle = None
        if self.live_map:
            self.node.get_logger().info(
                f'Accepted {goal.goal_id} using live grid {self.grid_stamp_ns} (displayed {displayed_grid})')
        msg = self.NavigateToPose.Goal()
        msg.pose.header.frame_id = goal.frame_id
        msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = goal.x, goal.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(goal.yaw / 2), math.cos(goal.yaw / 2)
        goal_id = goal.goal_id
        try:
            future = self.client.send_goal_async(msg)
            future.add_done_callback(lambda f: self._on_goal_response(goal_id, f))
        except Exception as error:
            self._apply(goal_id, 6, f'Nav2 request failed: {error}')
            return self.tracker.result
        return None

    def _on_goal_response(self, goal_id, future):
        try:
            handle = future.result()
        except Exception as error:
            self._apply(goal_id, 6, f'Nav2 goal response failed: {error}')
            return
        if not handle.accepted:
            self._apply(goal_id, 6, 'Nav2 rejected the goal')
            return
        if self.tracker.goal is None or self.tracker.goal.goal_id != goal_id:
            handle.cancel_goal_async()
            return
        self.handle = handle
        if self.tracker.cancel_requested:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda f: self._on_result(goal_id, f))
        self._apply(goal_id, 2, 'Nav2 executing')

    def _on_result(self, goal_id, future):
        try:
            response = future.result()
        except Exception as error:
            self._apply(goal_id, 6, f'Nav2 result failed: {error}')
            return
        self._apply(goal_id, response.status, 'Nav2 result')

    def _apply(self, goal_id, code, reason):
        result = self.tracker.on_backend_status(goal_id, code, self.now(), reason)
        if result is not None:
            pose = self._pose()
            result.last_valid_pose = pose
            self.results.append(result)

    def _pose(self):
        s = self.localization
        if s is None or not accept_pose(s, self.map_id, self.map_version, self.now())[0]:
            return None
        return [s.position[0], s.position[1], s.yaw]

    def cancel(self, why):
        if self.tracker.request_cancel(why, self.now()) and self.handle is not None:
            self.handle.cancel_goal_async()

    def tick(self):
        """Call periodically: timeout, gate-inhibit cancellation, settle."""
        now = self.now()
        if self.tracker.tick(now) == 'CANCEL' and self.handle is not None:
            self.handle.cancel_goal_async()
        if self.tracker.status in ('ACCEPTED', 'ACTIVE') and not self.operator_allowed(now):
            self.cancel('OPERATOR_MOTION_NOT_ENABLED')
        if self.live_map and self.tracker.status in ('ACCEPTED', 'ACTIVE'):
            ok, reason = self._live_target(self.tracker.goal, now, self.grid_stamp_ns)
            if not ok:
                self.cancel(reason)
        if (self.tracker.status in ('ACCEPTED', 'ACTIVE') and self.gate.get('inhibited')
                and self.gate.get('reason') not in (None, 'NO_COMMAND', 'COMMAND_STALE')):
            self.cancel(f'MOTION_GATE: {self.gate.get("reason")}')
        valid = self._pose() is not None
        self.tracker.settle(now, self.speed, self.yaw_rate, valid, self._pose() or [math.nan] * 3)

    @property
    def done(self):
        return self.tracker.status in ('REJECTED', 'CANCELLED', 'TIMEOUT', 'FAILED') or (
            self.tracker.status == 'REACHED' and self.tracker.result is not None and self.tracker.result.capture_ready)
