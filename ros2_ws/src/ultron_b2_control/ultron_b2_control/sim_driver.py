"""sim_driver: gated ``cmd_vel`` -> simulator command input ``sim/cmd_vel`` (SIMULATION ONLY).

Subscribes to ``cmd_vel`` only (the motion_gate output). Forwards each message; non-finite values
become zero; if no ``cmd_vel`` arrives for ``watchdog_s`` it publishes one zero command. The sim
wrapper (Agent F, ``ultron_b2_sim``) remaps the bridge's command input to ``sim/cmd_vel``.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist


class SimDriver(Node):
    def __init__(self):
        super().__init__('sim_driver')
        self.declare_parameter('watchdog_s', 0.3)
        self.declare_parameter('output_topic', 'sim/cmd_vel')
        self.watchdog = float(self.get_parameter('watchdog_s').value)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, str(self.get_parameter('output_topic').value), qos)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, qos)
        self.last = None
        self.zero_sent = True
        self.create_timer(0.05, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        out = Twist()
        vals = (msg.linear.x, msg.linear.y, msg.angular.z)
        if all(math.isfinite(v) for v in vals):
            out.linear.x, out.linear.y, out.angular.z = vals
        self.pub.publish(out)
        self.last, self.zero_sent = self._now(), False

    def _tick(self):
        if not self.zero_sent and self.last is not None and self._now() - self.last > self.watchdog:
            self.pub.publish(Twist())
            self.zero_sent = True
            self.get_logger().warn(f'no cmd_vel for {self.watchdog} s: sent zero to the simulator')


def main(args=None):
    rclpy.init(args=args)
    node = SimDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
