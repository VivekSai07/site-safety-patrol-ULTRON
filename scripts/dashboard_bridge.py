#!/usr/bin/env python3
"""ROS 2 bridge between Nav2 and the ULTRON dashboard.

Use only after independently verifying simulation isolation. This bridge has
no hardware-isolation guard and may send goals to any reachable action server.
The dashboard has no user authentication, including on localhost.

  ROS -> dashboard: /map (OccupancyGrid) as a PNG; the robot pose from TF map -> base frame.
  dashboard -> ROS: each goal set on the map becomes a nav2 NavigateToPose action goal; progress
                    (ACCEPTED / SUCCEEDED / FAILED / CANCELED) is reported back to the map.

A new dashboard goal cancels the running one, like RViz's 2D Goal Pose. Goals that existed before the
bridge started are ignored, so a restart never replays an old goal. The dashboard only accepts goals
when started with --goals.

See docs/dashboard_nav_goal.md for the simulation-only procedure and limits.
"""
import queue
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from ultron.dashboard.ros_link import ACTION_STATUS, Dashboard, grid_png, quaternion_from_yaw, yaw_from_quaternion


class Outbox(threading.Thread):
    """Sends HTTP calls in order on its own thread; drops work rather than blocking ROS callbacks."""

    def __init__(self, logger):
        super().__init__(daemon=True)
        self.jobs, self.logger = queue.Queue(maxsize=50), logger

    def put(self, name, fn, *args):
        try:
            self.jobs.put_nowait((name, fn, args))
        except queue.Full:
            pass  # dashboard down or slow: poses/maps are superseded anyway

    def run(self):
        while True:
            name, fn, args = self.jobs.get()
            try:
                fn(*args)
            except Exception as error:  # keep bridging; the dashboard may be restarting
                self.logger.warning(f'dashboard {name} failed: {error}', throttle_duration_sec=5.0)


class DashboardBridge(Node):
    def __init__(self):
        super().__init__('ultron_dashboard_bridge')
        p = {name: self.declare_parameter(name, default).value for name, default in [
            ('dashboard_url', 'http://127.0.0.1:8000'), ('map_topic', '/map'), ('map_frame', 'map'),
            ('base_frame', 'base_link'), ('action_name', 'navigate_to_pose'), ('pose_rate_hz', 5.0),
            ('map_min_period_s', 2.0)]}
        self.p = p
        self.dash = Dashboard(p['dashboard_url'])
        self.outbox = Outbox(self.get_logger())
        self.outbox.start()

        self.mission = None    # running mission id; poses are only sent while one runs
        self.mission_t0 = None
        self.pending_goal = None  # newest goal from the stream, picked up on the ROS thread
        self.lock = threading.Lock()
        self.active = None     # (seq, ClientGoalHandle) of the goal nav2 is executing
        self.latest_seq = None  # newest goal sent to nav2; anything older is cancelled on acceptance

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, p['map_topic'], self.on_map, qos)
        self.latest_map, self.map_sent_at = None, 0.0
        self.create_timer(0.5, self.flush_map)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_timer(1.0 / p['pose_rate_hz'], self.send_pose)

        self.nav = ActionClient(self, NavigateToPose, p['action_name'])
        self.create_timer(0.1, self.take_goal)
        threading.Thread(target=self.listen, daemon=True).start()
        self.get_logger().info(f"bridging {p['dashboard_url']} <-> {p['map_topic']}, TF {p['map_frame']}->{p['base_frame']}, "
                               f"action {p['action_name']}")

    # dashboard -> ROS ------------------------------------------------------------------------
    def listen(self):
        """Stream thread: follow the dashboard's live events, reconnecting when it restarts."""
        while rclpy.ok():
            try:
                live = self.dash.live()
                mission = live.get('mission')
                self.set_mission(mission['id'] if mission and mission['status'] == 'RUNNING' else None)
                for kind, data in self.dash.events():
                    if kind == 'goal' and data['status'] == 'SENT':
                        with self.lock:
                            self.pending_goal = data
                    elif kind == 'mission':
                        self.set_mission(data['id'] if data['status'] == 'RUNNING' else None)
            except Exception as error:
                self.get_logger().warning(f'dashboard stream: {error}; retrying', throttle_duration_sec=10.0)
            time.sleep(2.0)

    def set_mission(self, mission_id):
        if mission_id != self.mission:
            self.mission, self.mission_t0 = mission_id, None

    def take_goal(self):
        with self.lock:
            goal, self.pending_goal = self.pending_goal, None
        if goal is None:
            return
        seq = self.latest_seq = goal['seq']
        if self.active:
            self.active[1].cancel_goal_async()  # its CANCELED report hits the old seq and is ignored
            self.active = None
        if not self.nav.server_is_ready():
            self.report(seq, 'FAILED', f"action server {self.p['action_name']} not available")
            return
        msg = NavigateToPose.Goal()
        pose = msg.pose
        pose.header.frame_id = goal['frame_id']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(goal['x']), float(goal['y'])
        q = pose.pose.orientation
        q.x, q.y, q.z, q.w = quaternion_from_yaw(float(goal['yaw']))
        self.get_logger().info(f"goal {seq}: x={goal['x']:.2f} y={goal['y']:.2f} yaw={goal['yaw']:.2f} in {goal['frame_id']}")
        self.nav.send_goal_async(msg).add_done_callback(lambda future: self.on_accepted(seq, future))

    def on_accepted(self, seq, future):
        handle = future.result()
        if not handle.accepted:
            self.report(seq, 'FAILED', 'rejected by nav2')
            return
        if seq != self.latest_seq:  # replaced while nav2 was still deciding
            handle.cancel_goal_async()
            return
        self.active = (seq, handle)
        self.report(seq, 'ACCEPTED')
        handle.get_result_async().add_done_callback(lambda result: self.on_result(seq, result))

    def on_result(self, seq, future):
        status = future.result().status
        if self.active and self.active[0] == seq:
            self.active = None
        self.report(seq, ACTION_STATUS.get(status, 'FAILED'),
                    '' if status == GoalStatus.STATUS_SUCCEEDED else f'nav2 status {status}')

    def report(self, seq, status, detail=''):
        self.get_logger().info(f'goal {seq}: {status} {detail}'.rstrip())
        self.outbox.put('goal status', self.dash.goal_status, seq, status, detail)

    # ROS -> dashboard ------------------------------------------------------------------------
    def on_map(self, msg):
        self.latest_map = msg

    def flush_map(self):
        msg = self.latest_map
        if msg is None or time.monotonic() - self.map_sent_at < self.p['map_min_period_s']:
            return
        self.latest_map, self.map_sent_at = None, time.monotonic()
        info = msg.info
        if abs(yaw_from_quaternion(*(getattr(info.origin.orientation, k) for k in 'xyzw'))) > 1e-3:
            self.get_logger().warning('rotated map origins are not supported; map not sent', throttle_duration_sec=30.0)
            return
        png = grid_png(info.width, info.height, msg.data)
        self.outbox.put('map', self.dash.post_map, png, info.resolution, info.origin.position.x, info.origin.position.y)

    def send_pose(self):
        mission = self.mission
        if mission is None:
            return
        try:
            tf = self.tf.lookup_transform(self.p['map_frame'], self.p['base_frame'], rclpy.time.Time())
        except TransformException as error:
            self.get_logger().warning(f'no TF yet: {error}', throttle_duration_sec=10.0)
            return
        now = time.monotonic()
        if self.mission_t0 is None:
            self.mission_t0 = now
        t, r = tf.transform.translation, tf.transform.rotation
        state = 'NAVIGATING' if self.active else ''
        self.outbox.put('pose', self.dash.post_pose, mission, round(now - self.mission_t0, 2), t.x, t.y,
                        yaw_from_quaternion(r.x, r.y, r.z, r.w), state)


def main():
    rclpy.init()
    node = DashboardBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C, or SIGTERM from scripts/dashboard.sh
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
