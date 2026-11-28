"""Read-only, bounded ROS LowState probe, executed inside the isolated kit image."""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_go.msg import LowCmd, LowState


class Probe(Node):
    def __init__(self):
        super().__init__("ultron_b2_smoke_probe")
        self.count = 0
        self.first = None
        self.last = None
        self.invalid = False
        self.command_count = 0
        self.invalid_command = False
        self.create_subscription(LowState, "/lowstate", self.receive, 10)
        self.create_subscription(LowCmd, "/lowcmd", self.receive_command, 10)

    def receive_command(self, message):
        self.command_count += 1
        if len(message.motor_cmd) < 12 or not all(math.isfinite(m.q) for m in message.motor_cmd[:12]):
            self.invalid_command = True

    def receive(self, message):
        values = [motor.q for motor in message.motor_state[:12]]
        values += list(message.imu_state.quaternion) + list(message.imu_state.accelerometer)
        if (len(message.motor_state) < 12 or len(message.imu_state.quaternion) != 4
                or len(message.imu_state.accelerometer) != 3
                or not all(math.isfinite(value) for value in values)):
            self.invalid = True
        now = time.monotonic()
        if self.first is None:
            self.first = now
        self.last = now
        self.count += 1


def main():
    rclpy.init(args=["--ros-args", "--disable-external-lib-logs"])
    node = Probe()
    deadline = time.monotonic() + 15
    try:
        while time.monotonic() < deadline and (node.first is None or time.monotonic() - node.first < 3):
            rclpy.spin_once(node, timeout_sec=0.25)
        if node.count < 2 or node.invalid:
            raise RuntimeError(f"invalid or insufficient LowState samples: count={node.count}, invalid={node.invalid}")
        if "--require-lowcmd" in sys.argv and (node.command_count < 2 or node.invalid_command):
            raise RuntimeError(f"invalid or insufficient stand commands: count={node.command_count}, invalid={node.invalid_command}")
        print(f"observed ROS topics: {node.get_topic_names_and_types()}")
        rate = (node.count - 1) / (node.last - node.first)
        print(f"observed /lowstate type=unitree_go/msg/LowState count={node.count} receive_rate_hz={rate:.2f} finite_12_joint_q_and_imu=true")
        print(f"observed /lowcmd type=unitree_go/msg/LowCmd count={node.command_count} finite_12_motor_q={not node.invalid_command}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
