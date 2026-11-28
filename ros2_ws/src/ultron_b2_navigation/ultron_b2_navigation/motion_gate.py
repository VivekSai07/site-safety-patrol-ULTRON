"""motion_gate: common final gate, the ONLY publisher of ``cmd_vel``.

Input ``control/cmd_vel_safe`` (after twist_mux and collision_monitor), output ``cmd_vel`` at
``rate_hz`` (zero whenever inhibited), status ``safety/motion_gate``. Decision logic in
``gate_logic.FinalGate``. SIMULATION-validated only; real limits are PROPOSED / TO_VERIFY.
"""
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from ultron_b2_interfaces.msg import LocalizationState as LocalizationStateMsg
from ultron_b2_interfaces.msg import MapInfo, MotionGateStatus, OperatorEnable
from ultron_b2_contracts import msg_to_localization_state

from .gate_logic import DEFAULTS, FinalGate, localization_from_dict


class MotionGateNode(Node):
    def __init__(self):
        super().__init__('motion_gate')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('map_id', '')
        self.declare_parameter('map_version', 0)
        self.declare_parameter('map_meta_path', '')
        self.declare_parameter('adopt_map_info', True)
        params = {k: self.get_parameter(k).value for k in DEFAULTS}
        map_id, map_version = self._initial_map()
        self.gate = FinalGate(params, map_id=map_id, map_version=map_version)
        self.adopt_map_info = bool(self.get_parameter('adopt_map_info').value)

        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.out = self.create_publisher(Twist, 'cmd_vel', reliable)
        self.status_pub = self.create_publisher(MotionGateStatus, 'safety/motion_gate', 10)
        self.create_subscription(Twist, 'control/cmd_vel_safe', self._on_cmd, reliable)
        self.create_subscription(Bool, 'ops/hold', self._on_hold, reliable)
        self.create_subscription(OperatorEnable, 'ops/operator_enable', self._on_operator, reliable)
        self.create_subscription(LocalizationStateMsg, 'state/localization', self._on_loc, reliable)
        self.create_subscription(String, 'control/active_source', self._on_source, reliable)
        self.create_subscription(MapInfo, 'state/map_info', self._on_map_info, latched)
        self.cmd_in = Twist()
        self.last_reason = None
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        self.get_logger().info(f'motion_gate up: map {map_id or "<none>"} v{map_version}, '
                               f'teleop_requires_localization={params["teleop_requires_localization"]}')

    def _initial_map(self):
        meta = str(self.get_parameter('map_meta_path').value)
        if meta:
            p = Path(meta)
            data = json.loads((p / 'map_meta.json' if p.is_dir() else p).read_text())
            return str(data['map_id']), int(data['map_version'])
        return str(self.get_parameter('map_id').value), int(self.get_parameter('map_version').value)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        self.cmd_in = msg
        self.gate.set_command(msg.linear.x, msg.linear.y, msg.angular.z, self._now())

    def _on_hold(self, msg):
        self.gate.hold = bool(msg.data)
        if self.gate.hold:
            self._tick()                           # zero immediately, not on the next period

    def _on_operator(self, msg):
        self.gate.set_operator(msg.enabled, self._now())

    def _on_loc(self, msg):
        try:
            self.gate.localization = localization_from_dict(msg_to_localization_state(msg))
        except Exception:                          # noqa: BLE001 - malformed state counts as missing
            self.gate.localization = None

    def _on_source(self, msg):
        self.gate.set_source(msg.data, self._now())

    def _on_map_info(self, msg):
        if self.adopt_map_info and (msg.map_id, msg.map_version) != (self.gate.map_id, self.gate.map_version):
            self.get_logger().info(f'adopting loaded map {msg.map_id} v{msg.map_version} from state/map_info')
            self.gate.map_id, self.gate.map_version = msg.map_id, int(msg.map_version)

    def _tick(self):
        now = self._now()
        if now <= 0:
            return                                 # sim clock not running yet: publish nothing
        d = self.gate.decide(now)
        out = Twist()
        if d.open:
            out.linear.x, out.linear.y, out.angular.z = d.cmd_out
        self.out.publish(out)
        if d.reason != self.last_reason:
            # rclpy forbids changing severity at one call site: separate calls per level.
            if d.open:
                self.get_logger().info(f'gate OPEN (source {d.source})')
            else:
                self.get_logger().warn(f'gate INHIBITED {d.reason} (source {d.source})')
            self.last_reason = d.reason
        s = MotionGateStatus()
        s.schema_version = MotionGateStatus.SCHEMA_VERSION
        s.header.stamp = self.get_clock().now().to_msg()
        s.open, s.inhibited_reason, s.active_source = d.open, d.reason, d.source
        s.map_id, s.map_version = self.gate.map_id, self.gate.map_version
        loc = self.gate.localization
        s.localization_state = loc.state if loc else 'NONE'
        s.operator_enabled = bool(self.gate.operator_enabled)
        s.operator_enable_age_s = now - self.gate.operator_stamp if self.gate.operator_stamp else -1.0
        s.command_age_s = now - self.gate.command_stamp if self.gate.command_stamp else -1.0
        s.hold = self.gate.hold
        s.teleop_requires_localization = bool(self.gate.params['teleop_requires_localization'])
        s.cmd_in, s.cmd_out = self.cmd_in, out
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = MotionGateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
