#!/usr/bin/env python3
"""ROS 2 bridge between Nav2 and the ULTRON dashboard.

Use only after independently verifying simulation isolation. This bridge has
no hardware-isolation guard and may send goals to any reachable action server.
The dashboard has no user authentication, including on localhost.

  ROS -> dashboard: /map (OccupancyGrid) as a PNG; the robot pose from TF map -> base frame.
  dashboard -> ROS: each goal set on the map becomes a nav2 NavigateToPose action goal; progress
                    (ACCEPTED / SUCCEEDED / FAILED / CANCELED) is reported back to the map.

  exploration:      Start launches the explore node (`explore_command`) as a child process, Pause / Resume
                    publish explore/resume false / true, Finish stops the node; each step is confirmed to
                    the dashboard (EXPLORING, PAUSED, MAPPED, or FAILED with why).
  fence:            /map_fenced (the map with everything outside the dashboard's fence occupied, for the
                    explorer's costmap_topic) and /keepout_mask (for Nav2's KeepoutFilter).
  obstacles:        the dashboard's object footprints as /ultron/object_obstacles (PointCloud2).

A new dashboard goal cancels the running one, like RViz's 2D Goal Pose. Goals that existed before the
bridge started are ignored, so a restart never replays an old goal, nor an old exploration request. The
dashboard only accepts goals when started with --goals. Contract: docs/EXPLORATION_CONTRACT.md.

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
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from ultron.dashboard.ros_link import (ACTION_STATUS, Dashboard, Explorer, explore_on_connect, explore_request, fence_mask,
                                       fenced_grid, goal_tolerance, grid_png, keepout_grid, obstacle_cloud,
                                       quaternion_from_yaw, yaw_from_quaternion)

try:  # explore/status; not installed everywhere, then the explorer's state is inferred from its process
    from explore_lite_msgs.msg import ExploreStatus
except ImportError:
    ExploreStatus = None


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
            ('map_min_period_s', 2.0),
            # a missing Nav2 is a FAILED goal after this long, never a goal that hangs in SENT
            ('nav_wait_s', 3.0), ('nav_accept_timeout_s', 5.0),
            # explore_lite's own launch file reads its package params (costmap_topic: map, i.e. unfenced);
            # point it at /map_fenced with a params file or `ros2 run` (docs/dashboard_nav_goal.md)
            # Reads the fenced map: the raw SLAM map is free beyond the windows, and its frontiers lie there.
            ('explore_command', 'ros2 run explore_lite explore --ros-args -p use_sim_time:=false -p costmap_topic:=/map_fenced -p costmap_updates_topic:=/map_fenced_updates -p return_to_init:=false'),
            ('explore_resume_topic', 'explore/resume'), ('explore_status_topic', 'explore/status'),
            ('explore_ready_wait_s', 30.0),
            ('fenced_map_topic', '/map_fenced'), ('keepout_mask_topic', '/keepout_mask'),
            # explore_lite subscribes to its costmap without transient-local, so a late start needs a repeat
            ('fenced_period_s', 2.0),
            ('obstacles_topic', '/ultron/object_obstacles'), ('obstacles_period_s', 1.0),
            # Coarse to fine (ros_link.goal_tolerance): set on Nav2's goal checker before each goal.
            # goal_checker '' = the first of the controller's goal_checker_plugins.
            ('controller_server', 'controller_server'), ('goal_checker', ''),
            ('coarse_xy', 0.4), ('coarse_yaw', 0.4), ('fine_xy', 0.1), ('fine_yaw', 0.1)]}
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
        self.waiting = None    # (goal, deadline): Nav2 not discovered yet; FAILED after nav_wait_s
        self.sent_at = None    # (seq, time) a goal went to Nav2 and has no answer yet
        self.cancelled = set()  # goal seqs the dashboard cancelled (a pause ended); stop them whenever seen

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, p['map_topic'], self.on_map, qos)
        self.latest_map, self.map_sent_at = None, 0.0
        self.create_timer(0.5, self.flush_map)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_timer(1.0 / p['pose_rate_hz'], self.send_pose)

        self.nav = ActionClient(self, NavigateToPose, p['action_name'])
        server = '/' + p['controller_server'].lstrip('/')
        self.set_params = self.create_client(SetParameters, server + '/set_parameters')
        self.get_params = self.create_client(GetParameters, server + '/get_parameters')
        self.checker = p['goal_checker'] or None   # found from the controller when not given
        self.checker_asked = 0.0
        self.tolerance = None                      # (xy, yaw) last set on the goal checker
        self.tolerance_wait = None                 # (seq, msg, mode, since) while a tolerance change is in flight
        self.goal_modes = {}                       # seq -> 'coarse 0.40 m / 0.40 rad' for the ACCEPTED detail
        self.create_timer(1.0, self.find_checker)  # known before the first goal, not looked up with it
        self.create_timer(0.1, self.take_goal)

        # exploration: requests run one at a time on their own thread, they wait for the explorer
        self.resume_pub = self.create_publisher(Bool, p['explore_resume_topic'], 10)
        self.explore_statuses = []  # every explore/status text, oldest first (a few per pause)
        if ExploreStatus is not None:
            self.create_subscription(ExploreStatus, p['explore_status_topic'], self.on_explore_status, QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.explorer = Explorer(p['explore_command'], self.publish_resume, self.resume_pub.get_subscription_count,
                                 self.report_exploration, self.get_explore_status if ExploreStatus else None,
                                 ready_wait_s=p['explore_ready_wait_s'],
                                 interrupted=self.newer_request, nav_ready=self.nav.server_is_ready)
        self.explore_jobs = queue.Queue()
        self.exploration = {'seq': 0, 'state': 'IDLE'}  # newest dashboard state seen
        self.explore_seq = None  # newest request handled; None until the first connect (nothing is replayed)
        threading.Thread(target=self.run_explorer, daemon=True).start()

        # fence and obstacles, latched for late subscribers
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.fenced_pub = self.create_publisher(OccupancyGrid, p['fenced_map_topic'], latched)
        self.keepout_pub = self.create_publisher(OccupancyGrid, p['keepout_mask_topic'], latched)
        self.obstacles_pub = self.create_publisher(PointCloud2, p['obstacles_topic'], latched)
        self.slam_map, self.fence, self.obstacles = None, None, None
        self.fence_dirty, self.fenced, self.mask_key, self.mask = False, None, None, None
        self.create_timer(0.5, self.publish_fenced)
        self.create_timer(p['fenced_period_s'], self.repeat_fenced)
        self.create_timer(p['obstacles_period_s'], self.publish_obstacles)
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
                with self.lock:  # unlike goals, the fence and obstacles are state: take them as they are now
                    self.fence, self.fence_dirty = live.get('fence'), True
                    self.cancelled.clear()  # a restarted dashboard numbers its goals from 1 again
                self.obstacles = self.dash.obstacles()
                if live.get('exploration'):
                    self.explore_jobs.put(('connect', live['exploration']))
                for kind, data in self.dash.events():
                    if kind == 'goal' and data['status'] == 'SENT':
                        with self.lock:
                            self.pending_goal = data
                    elif kind == 'goal' and data['status'] == 'CANCELED':
                        with self.lock:
                            self.cancelled.add(data['seq'])
                    elif kind == 'mission':
                        self.set_mission(data['id'] if data['status'] == 'RUNNING' else None)
                    elif kind == 'exploration':
                        self.explore_jobs.put(('event', data))
                    elif kind == 'fence':
                        with self.lock:
                            self.fence, self.fence_dirty = data, True
                    elif kind == 'obstacles':
                        self.obstacles = data
            except Exception as error:
                self.get_logger().warning(f'dashboard stream: {error}; retrying', throttle_duration_sec=10.0)
            time.sleep(2.0)

    def set_mission(self, mission_id):
        if mission_id != self.mission:
            self.mission, self.mission_t0 = mission_id, None

    def take_goal(self):
        with self.lock:
            goal, self.pending_goal = self.pending_goal, None
            cancelled = set(self.cancelled)
        if self.active and self.active[0] in cancelled:  # the dashboard ended it (a pause ended): stop it
            self.get_logger().info(f'goal {self.active[0]}: cancelled by the dashboard')
            self.active[1].cancel_goal_async()  # its CANCELED result is reported as usual
            with self.lock:
                self.cancelled.discard(self.active[0])  # once is enough
        if self.tolerance_wait and time.monotonic() - self.tolerance_wait[2] > 1.0:
            self.goal_modes[self.tolerance_wait[0]] += ' (not set: controller did not answer)'
            self.release(self.tolerance_wait[0])  # never hold a goal back for its tolerance
        if self.sent_at and self.sent_at[1] < time.monotonic() - self.p['nav_accept_timeout_s']:
            seq = self.sent_at[0]
            self.sent_at = None
            with self.lock:
                self.cancelled.add(seq)  # if Nav2 accepts it after all, on_accepted cancels it
            self.report(seq, 'FAILED', 'navigation not available')  # discovered, but gone: same to the pipeline
        retry, self.waiting = self.waiting, None
        if goal is None:  # nothing new: try the goal still waiting for Nav2, if any
            if retry is None:
                return
            goal = retry[0]
        if goal['seq'] in cancelled:
            return
        seq = self.latest_seq = goal['seq']
        if self.active:
            self.active[1].cancel_goal_async()  # its CANCELED report hits the old seq and is ignored
            self.active = None
        if not self.nav.server_is_ready():
            deadline = retry[1] if retry and retry[0]['seq'] == goal['seq'] else time.monotonic() + self.p['nav_wait_s']
            if time.monotonic() < deadline:
                self.waiting = (goal, deadline)  # try again next tick: discovery takes a moment after start
            else:
                self.report(seq, 'FAILED', 'navigation not available')
            return
        msg = NavigateToPose.Goal()
        pose = msg.pose
        pose.header.frame_id = goal['frame_id']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(goal['x']), float(goal['y'])
        q = pose.pose.orientation
        q.x, q.y, q.z, q.w = quaternion_from_yaw(float(goal['yaw']))
        self.get_logger().info(f"goal {seq}: x={goal['x']:.2f} y={goal['y']:.2f} yaw={goal['yaw']:.2f} in {goal['frame_id']}")
        self.send_with_tolerance(seq, msg, goal)

    # coarse to fine ----------------------------------------------------------------------------
    def robot_pose(self):
        try:
            tf = self.tf.lookup_transform(self.p['map_frame'], self.p['base_frame'], rclpy.time.Time())
        except TransformException:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w)

    def find_checker(self):
        """Ask the controller once in a while for its goal checker's name, until it answers."""
        if self.checker or not self.get_params.service_is_ready() or time.monotonic() - self.checker_asked < 3.0:
            return
        self.checker_asked = time.monotonic()
        request = GetParameters.Request(names=['goal_checker_plugins'])

        def answered(future):
            values = future.result().values if future.result() else []
            names = list(values[0].string_array_value) if values else []
            if names:
                self.checker = names[0]
                self.get_logger().info(f'goal checker: {self.checker}')
        self.get_params.call_async(request).add_done_callback(answered)

    def send_with_tolerance(self, seq, msg, goal):
        mode, xy, yaw = goal_tolerance(goal, self.robot_pose(), (self.p['coarse_xy'], self.p['coarse_yaw']),
                                       (self.p['fine_xy'], self.p['fine_yaw']))
        self.goal_modes[seq] = f'{mode} {xy:.2f} m / {yaw:.2f} rad'
        self.find_checker()
        if (xy, yaw) == self.tolerance or not self.checker or not self.set_params.service_is_ready():
            if (xy, yaw) != self.tolerance:
                self.goal_modes[seq] += ' (not set: no goal checker parameters)'
            return self.send_goal(seq, msg)
        request = SetParameters.Request(parameters=[
            Parameter(name=f'{self.checker}.{name}', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                                           double_value=float(value)))
            for name, value in (('xy_goal_tolerance', xy), ('yaw_goal_tolerance', yaw))])
        self.tolerance_wait = (seq, msg, time.monotonic())

        def done(future):
            results = future.result().results if future.result() else []
            if results and all(r.successful for r in results):
                self.tolerance = (xy, yaw)
            else:
                self.goal_modes[seq] += ' (not set: controller refused)'
                self.get_logger().warning(f'goal tolerance {xy}/{yaw} not set: {[r.reason for r in results]}')
            self.release(seq)
        self.set_params.call_async(request).add_done_callback(done)

    def release(self, seq):
        """Send the goal that waited for its tolerance, unless a newer one replaced it meanwhile."""
        if not self.tolerance_wait or self.tolerance_wait[0] != seq:
            return  # an older goal's answer: the newer goal waits for its own
        waiting, self.tolerance_wait = self.tolerance_wait, None
        if seq == self.latest_seq:
            self.send_goal(seq, waiting[1])

    def send_goal(self, seq, msg):
        self.sent_at = (seq, time.monotonic())
        self.nav.send_goal_async(msg).add_done_callback(lambda future: self.on_accepted(seq, future))

    def on_accepted(self, seq, future):
        if self.sent_at and self.sent_at[0] == seq:
            self.sent_at = None
        handle = future.result()
        if not handle.accepted:
            self.report(seq, 'FAILED', 'rejected by nav2')
            return
        with self.lock:
            cancelled = seq in self.cancelled
        if seq != self.latest_seq or cancelled:  # replaced or cancelled while nav2 was still deciding
            handle.cancel_goal_async()
            return
        self.active = (seq, handle)
        self.report(seq, 'ACCEPTED', self.goal_modes.get(seq, ''))
        handle.get_result_async().add_done_callback(lambda result: self.on_result(seq, result))

    def on_result(self, seq, future):
        status = future.result().status
        if self.active and self.active[0] == seq:
            self.active = None
        mode = self.goal_modes.pop(seq, '')  # with what tolerance it was reached
        self.report(seq, ACTION_STATUS.get(status, 'FAILED'),
                    mode if status == GoalStatus.STATUS_SUCCEEDED else f'nav2 status {status}')

    def report(self, seq, status, detail=''):
        self.get_logger().info(f'goal {seq}: {status} {detail}'.rstrip())
        self.outbox.put('goal status', self.dash.goal_status, seq, status, detail)

    # exploration -----------------------------------------------------------------------------
    def run_explorer(self):
        """Explorer thread: carry out requests in order; in between, watch the explorer process."""
        while True:
            try:
                kind, data = self.explore_jobs.get(timeout=0.5)
            except queue.Empty:
                self.explorer.watch(self.exploration['seq'], self.exploration['state'])
                continue
            try:
                self.explore_job(kind, data)
            except Exception as error:  # keep the thread alive whatever happens
                self.get_logger().error(f'exploration {kind}: {error}')

    def explore_job(self, kind, data):
        self.exploration = data
        if kind == 'connect':
            request = explore_on_connect(data, self.explore_seq, self.explorer.running())
        else:
            request = explore_request(data, self.explore_seq)
        self.explore_seq = data['seq']
        if request == 'stop':
            self.get_logger().warning('dashboard shows no exploration: stopping the explorer')
            self.explorer.stop()
        elif request:
            self.get_logger().info(f"exploration {data['seq']}: {request} (by {data.get('by') or '?'})")
            self.explorer.handle(request, data['seq'], data.get('by', ''))

    def newer_request(self):
        """A newer exploration request is queued (a stream reconnect or a renewal is not one)."""
        return any(kind == 'event' and explore_request(data, self.explore_seq) for kind, data in list(self.explore_jobs.queue))

    def publish_resume(self, flag):
        self.resume_pub.publish(Bool(data=flag))

    def on_explore_status(self, msg):
        self.explore_statuses.append(msg.status)
        self.get_logger().info(f'explore/status: {msg.status}')

    def get_explore_status(self):
        return self.explore_statuses

    def report_exploration(self, seq, state, detail='', by=''):
        self.get_logger().info(f'exploration {seq}: {state} {detail}'.rstrip())
        self.exploration = {**self.exploration, 'state': state}
        try:  # straight from the explorer thread, in order; the outbox may drop work
            self.dash.exploration_status(seq, state, detail, by)
        except Exception as error:
            self.get_logger().warning(f'dashboard exploration status failed: {error}')

    # fence and obstacles ---------------------------------------------------------------------
    def publish_fenced(self):
        """/map_fenced and /keepout_mask, whenever the map or the fence changed."""
        with self.lock:
            msg, fence, dirty = self.slam_map, self.fence, self.fence_dirty
            self.fence_dirty = False
        if not dirty or msg is None:
            return
        info = msg.info
        if fence and fence.get('frame_id', 'map') != msg.header.frame_id:
            self.get_logger().warning(f"fence is in {fence['frame_id']}, the map in {msg.header.frame_id}: "
                                      'fence ignored', throttle_duration_sec=30.0)
            fence = None
        if fence:
            key = (info.width, info.height, info.resolution, info.origin.position.x, info.origin.position.y,
                   tuple(map(tuple, fence['polygon'])))
            if key != self.mask_key:
                self.mask_key = key
                self.mask = fence_mask(info.width, info.height, info.resolution, info.origin.position.x,
                                       info.origin.position.y, fence['polygon'])
            data, keepout = fenced_grid(msg.data, self.mask), keepout_grid(self.mask)
        else:  # no fence: the explorer still reads /map_fenced, so pass the map through
            data, keepout = list(msg.data), [0] * (info.width * info.height)
        fenced, mask = OccupancyGrid(header=msg.header, info=info), OccupancyGrid(header=msg.header, info=info)
        fenced.data, mask.data = data, keepout
        self.fenced = (fenced, mask)
        self.repeat_fenced()

    def repeat_fenced(self):
        if self.fenced:
            self.fenced_pub.publish(self.fenced[0])
            self.keepout_pub.publish(self.fenced[1])

    def publish_obstacles(self):
        """Repeated with a fresh stamp: an obstacle layer drops old observations and looks up TF at the stamp."""
        obstacles = self.obstacles
        if not obstacles:
            return
        count, data = obstacle_cloud(obstacles['cells'])
        cloud = PointCloud2(height=1, width=count, is_bigendian=False, point_step=12, row_step=12 * count,
                            is_dense=True, data=data,
                            fields=[PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                                    for i, n in enumerate('xyz')])
        cloud.header.frame_id = obstacles['frame_id']
        cloud.header.stamp = self.get_clock().now().to_msg()
        self.obstacles_pub.publish(cloud)

    # ROS -> dashboard ------------------------------------------------------------------------
    def on_map(self, msg):
        self.latest_map = msg
        with self.lock:
            self.slam_map, self.fence_dirty = msg, True

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
        node.explorer.stop()  # an orphaned explorer would keep driving the robot
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
