#!/usr/bin/env python3
"""Bridge the localhost dashboard to ULTRON's map-scoped simulation action."""
import argparse
import json
from pathlib import Path
import queue
import signal
import sys
import threading
import time
from urllib.parse import urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from ultron.autonomy.config import load_sim_slam_config
from ultron.autonomy.dashboard_goal_node import DASHBOARD_ACTION
from ultron.autonomy.localization_state import accept_pose
from ultron.autonomy.motion_gate_node import state_from_json
from ultron.autonomy.run_navigation import NAV_MAP_TOPIC
from ultron.autonomy.sim_bridge import check_isolation
from ultron.dashboard.ros_link import Dashboard, grid_png, quaternion_from_yaw, yaw_from_quaternion

RESULT_STATUS = {'REACHED': 'SUCCEEDED', 'REJECTED': 'REJECTED', 'CANCELLED': 'CANCELED'}


class Outbox(threading.Thread):
    """Send ordered HTTP updates without blocking ROS callbacks."""

    def __init__(self, logger):
        super().__init__(daemon=True)
        self.jobs, self.logger = queue.Queue(maxsize=50), logger

    def put(self, name, fn, *args):
        try:
            self.jobs.put_nowait((name, fn, args))
        except queue.Full:
            self.logger.warning(f'dashboard outbox full: {name}', throttle_duration_sec=5.0)

    def run(self):
        while True:
            name, fn, args = self.jobs.get()
            try:
                fn(*args)
            except Exception as error:
                self.logger.warning(f'dashboard {name} failed: {error}', throttle_duration_sec=5.0)
            finally:
                self.jobs.task_done()


class DashboardBridge(Node):
    def __init__(self, config, map_meta, url):
        super().__init__('ultron_dashboard_bridge', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.live_map = map_meta is None
        self.map_id = None if self.live_map else map_meta['map_id']
        self.map_version = None if self.live_map else int(map_meta['map_version'])
        self.config = config
        self.dash = Dashboard(url)
        self.outbox = Outbox(self.get_logger())
        self.outbox.start()
        self.closed = threading.Event()
        self.lock = threading.Lock()
        self.connected = False
        self.mission = self.pose_mission = self.mission_t0 = None
        self.pending_goal = self.inflight = self.goal_handle = None
        self.cancel_requested = False
        self.localization = None
        self.latest_map = None
        self.map_pending = False
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if self.live_map:
            from tf2_ros import Buffer, TransformListener
            self.tf = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf, self)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        else:
            from ultron_nav_interfaces.action import NavigateDashboard
            self.goal_type = NavigateDashboard
            self.nav = ActionClient(self, NavigateDashboard, DASHBOARD_ACTION)
            self.create_subscription(OccupancyGrid, NAV_MAP_TOPIC, self.on_map, latched)
            self.create_subscription(String, config.topics['localization_state'], self.on_localization, latched)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.5, self.flush_map, clock=steady)
        self.create_timer(0.2, self.send_pose, clock=steady)
        self.create_timer(0.1, self.take_goal, clock=steady)
        threading.Thread(target=self.listen, daemon=True).start()

    def listen(self):
        """Follow mission and goal events. Ignore goals already present on connection."""
        while not self.closed.is_set():
            existing_request = None

            def connected():
                nonlocal existing_request
                live = self.dash.live()
                existing_request = (live.get('goal') or {}).get('request_id')
                with self.lock:
                    self.set_mission(live.get('mission'))
                    self.connected = True
                    self.map_pending = True

            try:
                for kind, data in self.dash.events(on_connect=connected):
                    if self.closed.is_set():
                        break
                    with self.lock:
                        if kind == 'goal' and data['status'] == 'SENT' and data['request_id'] != existing_request:
                            self.pending_goal = data
                        elif kind == 'mission':
                            self.set_mission(data)
            except Exception as error:
                self.get_logger().warning(f'dashboard stream: {error}', throttle_duration_sec=10.0)
            finally:
                with self.lock:
                    self.connected = False
                    self.pending_goal = None
            self.closed.wait(1.0)

    def set_mission(self, mission):
        self.mission = mission['id'] if mission and mission['status'] == 'RUNNING' else None

    def cancel(self):
        if self.goal_handle is not None and not self.cancel_requested:
            self.cancel_requested = True
            self.goal_handle.cancel_goal_async()

    def take_goal(self):
        with self.lock:
            goal, connected, mission = self.pending_goal, self.connected, self.mission
            if self.inflight is None:
                self.pending_goal = None
        if self.inflight is not None:
            if goal is not None or not connected or self.closed.is_set():
                self.cancel()
            return
        if goal is None or not connected or self.closed.is_set():
            return
        if self.live_map:
            self.report(goal, 'REJECTED', 'LIVE_MAP_PREVIEW')
            return
        if not self.nav.server_is_ready():
            self.report(goal, 'FAILED', 'Checked simulation action server is unavailable')
            return
        msg = self.goal_type.Goal()
        msg.mission_id = mission or 'DASHBOARD'
        msg.map_id = goal.get('map_id') or ''
        msg.map_version = goal.get('map_version') or 0
        msg.pose.header.frame_id = goal['frame_id']
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        pose = msg.pose.pose
        pose.position.x, pose.position.y = float(goal['x']), float(goal['y'])
        q = pose.orientation
        q.x, q.y, q.z, q.w = quaternion_from_yaw(float(goal['yaw']))
        self.inflight, self.cancel_requested = goal, False
        try:
            future = self.nav.send_goal_async(msg, feedback_callback=lambda f: self.on_feedback(goal, f))
            future.add_done_callback(lambda f: self.on_accepted(goal, f))
        except Exception as error:
            self.clear_goal()
            self.report(goal, 'FAILED', str(error))

    def on_feedback(self, goal, feedback):
        if self.inflight is goal and feedback.feedback.status == 'ACCEPTED':
            self.report(goal, 'ACCEPTED')

    def on_accepted(self, goal, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.clear_goal()
                self.report(goal, 'REJECTED', 'Checked simulation action declined the request')
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.on_result(goal, f))
        except Exception as error:
            self.clear_goal()
            self.report(goal, 'FAILED', str(error))

    def on_result(self, goal, future):
        self.clear_goal()
        try:
            result = future.result().result
            self.report(goal, RESULT_STATUS.get(result.status, 'FAILED'), result.reason)
        except Exception as error:
            self.report(goal, 'FAILED', str(error))

    def clear_goal(self):
        self.inflight = self.goal_handle = None

    def report(self, goal, status, detail=''):
        self.get_logger().info(f"goal {goal['seq']}: {status} {detail}".rstrip())
        self.outbox.put('goal status', self.dash.goal_status, goal['seq'], status, detail, goal['request_id'])

    def on_map(self, msg):
        if msg.header.frame_id == 'map':
            self.latest_map = msg
            with self.lock:
                self.map_pending = True

    def flush_map(self):
        with self.lock:
            msg = self.latest_map
            if msg is None or not self.connected or not self.map_pending:
                return
            self.map_pending = False
        info = msg.info
        q = info.origin.orientation
        if abs(yaw_from_quaternion(q.x, q.y, q.z, q.w)) > 1e-3:
            self.get_logger().warning('Dashboard cannot display a rotated map origin')
            return
        png = grid_png(info.width, info.height, msg.data)
        self.outbox.put('map', self.dash.post_map, png, info.resolution, info.origin.position.x,
                        info.origin.position.y, self.map_id, self.map_version,
                        'live_slam' if self.live_map else None)

    def on_localization(self, msg):
        try:
            self.localization = state_from_json(msg.data)
        except (ValueError, TypeError):
            self.localization = None

    def send_pose(self):
        mission, state = self.mission, self.localization
        if mission is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.live_map:
            pose = self.mapping_pose(now)
            if pose is None:
                return
            x, y, yaw = pose
            status = 'MAPPING_ESTIMATE'
        else:
            if not accept_pose(state, self.map_id, self.map_version, now)[0]:
                return
            x, y, yaw = state.position[0], state.position[1], state.yaw
            status = 'NAVIGATING' if self.inflight else 'VALID'
        wall = time.monotonic()
        if self.pose_mission != mission:
            self.pose_mission, self.mission_t0 = mission, wall
        self.outbox.put('pose', self.dash.post_pose, mission, round(wall - self.mission_t0, 2),
                        x, y, yaw, status)

    def mapping_pose(self, now):
        from rclpy.time import Time
        from tf2_ros import TransformException

        frames = self.config.frames
        try:
            tf = self.tf.lookup_transform(frames['map'], frames['base'], Time())
        except TransformException:
            return None
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if not 0 < stamp <= now or now - stamp > self.config.localization['pose_age_limit_s']:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--map', type=Path, help='Saved map for checked navigation')
    source.add_argument('--live-map', action='store_true', help='Preview the current slam_toolbox map and estimate')
    parser.add_argument('--url', default='http://127.0.0.1:8000')
    args = parser.parse_args()
    url = urlsplit(args.url)
    if url.scheme != 'http' or url.hostname not in ('127.0.0.1', '::1', 'localhost') or url.username is not None:
        parser.error('The simulation dashboard URL must be HTTP on loopback')
    meta = None
    if args.map is not None:
        meta = json.loads((args.map / 'map_meta.json').read_text())
        if 'sim_run' not in meta:
            parser.error('Load a map produced by the simulation mapping workflow')
    config = load_sim_slam_config()
    check_isolation(config)

    stopping = threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.set())
    node = DashboardBridge(config, meta, args.url)
    try:
        while rclpy.ok() and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.closed.set()
        deadline = time.monotonic() + 2
        while node.inflight and time.monotonic() < deadline:
            node.cancel()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
