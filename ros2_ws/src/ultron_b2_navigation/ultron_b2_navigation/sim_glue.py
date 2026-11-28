"""TEMPORARY glue between the legacy simulator (ultron.autonomy.sim_bridge + localization_monitor,
absolute /ultron/sim/* topics) and the B2 stack contract topics. SIMULATION ONLY.

Replaced by Agent F (ultron_b2_sim: sensors/scan, state/odom, consumes sim/cmd_vel) and Agent C
(state/localization LocalizationState msg). Relays:
  /ultron/sim/scan                (LaserScan)        -> /ultron/sim/sensors/scan
  /ultron/sim/odom                (Odometry)         -> /ultron/sim/state/odom
  /ultron/sim/localization_state  (String JSON v1)   -> /ultron/sim/state/localization (LocalizationState)
The legacy sim's cmd_vel input is configured to /ultron/sim/sim/cmd_vel (sim_driver output).
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ultron_b2_contracts import localization_state_to_msg
from ultron_b2_interfaces.msg import LocalizationState

NS = '/ultron/sim'


class SimGlue(Node):
    def __init__(self):
        super().__init__('legacy_sim_glue', namespace=NS)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan = self.create_publisher(LaserScan, 'sensors/scan', qos)
        odom = self.create_publisher(Odometry, 'state/odom', qos)
        self.loc = self.create_publisher(LocalizationState, 'state/localization', qos)
        self.create_subscription(LaserScan, f'{NS}/scan', scan.publish, qos)
        self.create_subscription(Odometry, f'{NS}/odom', odom.publish, qos)
        self.create_subscription(String, f'{NS}/localization_state', self._on_loc, latched)
        self.errors = 0

    def _on_loc(self, msg):
        try:
            self.loc.publish(localization_state_to_msg(json.loads(msg.data)))
        except Exception as exc:                   # noqa: BLE001 - never forward a malformed state
            self.errors += 1
            self.get_logger().error(f'dropped malformed localization_state: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = SimGlue()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
