#!/usr/bin/env python3
"""Stand-in for Nav2 + SLAM to check the dashboard bridge: a NavigateToPose action server that "drives" each
goal for `drive_s` seconds and succeeds (a newer goal preempts the running one, as in Nav2; cancel works),
and a latched /map (all free, 6 x 4 m) republished every few seconds like slam_toolbox. TF map -> base_link
puts the robot where its last goal succeeded (at the origin before). A `controller_server` node holds
the goal checker's tolerances like Nav2's (goal_checker_plugins, general_goal_checker.xy/yaw_goal_tolerance,
settable at run time) and publishes them on /fake_nav2/tolerance. It logs what happens to every goal.
"""
import threading
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class FakeController(Node):
    """Only the goal checker's parameters of Nav2's controller_server."""

    def __init__(self):
        super().__init__('controller_server')
        self.declare_parameter('goal_checker_plugins', ['general_goal_checker'])
        self.declare_parameter('general_goal_checker.xy_goal_tolerance', 0.25)
        self.declare_parameter('general_goal_checker.yaw_goal_tolerance', 0.25)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, '/fake_nav2/tolerance', latched)
        self.add_on_set_parameters_callback(self.changed)
        self.create_timer(0.5, self.publish)

    def changed(self, parameters):
        for p in parameters:
            self.get_logger().info(f'{p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def publish(self):
        xy = self.get_parameter('general_goal_checker.xy_goal_tolerance').value
        yaw = self.get_parameter('general_goal_checker.yaw_goal_tolerance').value
        self.pub.publish(String(data=f'{xy:.2f} {yaw:.2f}'))


class FakeNav2(Node):
    def __init__(self):
        super().__init__('fake_nav2')
        self.drive_s = self.declare_parameter('drive_s', 2.0).value
        self.lock, self.newest = threading.Lock(), None
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute, goal_callback=lambda _g: GoalResponse.ACCEPT,
                     cancel_callback=lambda _h: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.create_timer(3.0, self.publish_map)
        self.publish_map()
        self.robot = (0.0, 0.0)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.1, self.publish_tf)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.robot
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution, msg.info.width, msg.info.height = 0.1, 60, 40
        msg.info.origin.position.x, msg.info.origin.position.y = -1.0, -1.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (60 * 40)
        self.map_pub.publish(msg)

    def execute(self, handle):
        p = handle.request.pose.pose.position
        with self.lock:
            self.newest = handle.goal_id.uuid.tobytes()
        self.get_logger().info(f'goal x={p.x:.2f} y={p.y:.2f}: driving')
        end = time.monotonic() + self.drive_s
        while time.monotonic() < end:
            if handle.is_cancel_requested:
                handle.canceled()
                self.get_logger().info(f'goal x={p.x:.2f} y={p.y:.2f}: canceled')
                return NavigateToPose.Result()
            with self.lock:
                if self.newest != handle.goal_id.uuid.tobytes():
                    handle.abort()
                    self.get_logger().info(f'goal x={p.x:.2f} y={p.y:.2f}: preempted')
                    return NavigateToPose.Result()
            time.sleep(0.05)
        handle.succeed()
        self.robot = (p.x, p.y)
        self.get_logger().info(f'goal x={p.x:.2f} y={p.y:.2f}: succeeded')
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = FakeNav2()
    controller = FakeController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(controller)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
