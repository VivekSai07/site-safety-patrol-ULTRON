#!/usr/bin/env python3
"""Stand-in for explore_lite (m-explore-ros2) to check the dashboard bridge without the real explorer.

Acts like explore_lite where the bridge can tell: it waits for its first costmap (costmap_topic, default
/map_fenced) before it subscribes to explore/resume; while exploring it sends a NavigateToPose goal to a
free cell inside the costmap every few seconds; explore/resume false cancels its goal and stops, true
resumes. It has no explore_lite_msgs, so it publishes no explore/status; it says what it does on
/fake_explore/state (std_msgs/String) for scripts/b2_standin/explore_check.py.
"""
import random

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class FakeExplore(Node):
    def __init__(self):
        super().__init__('explore_node')
        self.costmap_topic = self.declare_parameter('costmap_topic', '/map_fenced').value
        self.period = self.declare_parameter('goal_period_s', 4.0).value
        self.state_pub = self.create_publisher(String, '/fake_explore/state', QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.costmap, self.resume_sub, self.exploring, self.goal_handle = None, None, False, None
        # explore_lite's costmap subscription: default QoS (volatile), so latched maps from before it started are missed
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.on_costmap, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(self.period, self.plan)
        self.say(f'waiting for costmap on {self.costmap_topic}')

    def say(self, text):
        self.get_logger().info(text)
        self.state_pub.publish(String(data=text))

    def on_costmap(self, msg):
        first = self.costmap is None
        self.costmap = msg
        if first:
            outside = sum(1 for v in msg.data if v == 100)
            self.say(f'costmap {msg.info.width}x{msg.info.height}, {outside} occupied cells')
            self.resume_sub = self.create_subscription(Bool, 'explore/resume', self.on_resume, 10)
            self.exploring = True
            self.say('exploring')

    def on_resume(self, msg):
        if msg.data:
            self.exploring = True
            self.say('resumed')
            self.plan()
        else:  # explore_lite's stop(): cancel all goals on the action server, stop planning
            self.exploring = False
            self.nav.wait_for_server(timeout_sec=0.5)
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            self.say('stopped')

    def plan(self):
        if not self.exploring or self.costmap is None or not self.nav.server_is_ready():
            return
        info, data = self.costmap.info, self.costmap.data
        free = [i for i in range(0, len(data), 7) if data[i] == 0]
        if not free:
            self.say('no frontiers left')
            self.exploring = False
            return
        i = random.choice(free)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.costmap.header.frame_id
        goal.pose.pose.position.x = info.origin.position.x + (i % info.width + 0.5) * info.resolution
        goal.pose.pose.position.y = info.origin.position.y + (i // info.width + 0.5) * info.resolution
        goal.pose.pose.orientation.w = 1.0
        self.nav.send_goal_async(goal).add_done_callback(self.on_accepted)

    def on_accepted(self, future):
        handle = future.result()
        if handle.accepted and self.exploring:
            self.goal_handle = handle
            self.say('frontier goal sent')
        elif handle.accepted:
            handle.cancel_goal_async()


def main():
    rclpy.init()
    node = FakeExplore()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
