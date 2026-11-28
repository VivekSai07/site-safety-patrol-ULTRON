"""ROS 2 motion gate between Nav2 and the simulated robot. SIMULATION ONLY.

Nav2 publishes on ``nav_cmd_raw``; only this node publishes ``cmd_vel`` (simulation command
topic). Commands pass only while LocalizationState is VALID and fresh for the expected map,
operator enable is true and fresh, and the Nav2 command is fresh. Otherwise zero is published.
"""
import argparse
import json
from pathlib import Path

from .localization_state import LocalizationState
from .navigation_contract import MotionGate

GATE_RATE_HZ = 20.0


def state_from_json(text):
    data = json.loads(text)
    fields = set(LocalizationState.__dataclass_fields__)
    return LocalizationState(**{k: v for k, v in data.items() if k in fields})


class GateNode:
    def __init__(self, config, map_meta):
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool, String
        self.Twist, self.String = Twist, String
        self.gate = MotionGate(config.navigation, map_meta['map_id'], int(map_meta['map_version']))
        self.node = Node('ultron_motion_gate', parameter_overrides=[Parameter('use_sim_time', value=True)])
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        t = config.topics
        self.out = self.node.create_publisher(Twist, t['cmd_vel'], latest)
        self.status_pub = self.node.create_publisher(String, t['gate_status'], latched)
        self.node.create_subscription(String, t['localization_state'], self._on_state, latched)
        self.node.create_subscription(Bool, t['operator_enable'], self._on_operator, latest)
        self.node.create_subscription(Twist, t['nav_cmd_raw'], self._on_cmd, latest)
        self.command = self.command_stamp = None
        self.last_reason = 'STARTED'
        self.forwarded = self.inhibited = 0
        self.transitions = []
        self.node.create_timer(1.0 / GATE_RATE_HZ, self._tick)

    def _now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg):
        try:
            self.gate.inputs.localization = state_from_json(msg.data)
        except (ValueError, TypeError):
            self.gate.inputs.localization = None   # malformed state counts as missing

    def _on_operator(self, msg):
        self.gate.inputs.operator_enabled, self.gate.inputs.operator_stamp = bool(msg.data), self._now()

    def _on_cmd(self, msg):
        self.command, self.command_stamp = (msg.linear.x, msg.angular.z), self._now()

    def _tick(self):
        now = self._now()
        if now <= 0:
            return
        (vx, wz), reason = self.gate.decide(self.command, self.command_stamp, now)
        msg = self.Twist()
        msg.linear.x, msg.angular.z = float(vx), float(wz)
        self.out.publish(msg)
        if reason:
            self.inhibited += 1
        else:
            self.forwarded += 1
        if reason != self.last_reason:
            self.transitions.append({'t': round(now, 3), 'from': self.last_reason, 'to': reason})
            self.last_reason = reason
            self.status_pub.publish(self.String(data=json.dumps(
                {'t': round(now, 3), 'inhibited': reason is not None, 'reason': reason,
                 'map_id': self.gate.map_id, 'map_version': self.gate.map_version})))


def main(argv=None):
    parser = argparse.ArgumentParser(description='Motion gate between Nav2 and the simulated robot')
    parser.add_argument('--map', type=Path, required=True, help='Mapping run folder (map_meta.json)')
    parser.add_argument('--log', type=Path)
    args = parser.parse_args(argv)
    import signal
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from .config import load_sim_slam_config
    from .sim_bridge import check_isolation
    config = load_sim_slam_config()
    check_isolation(config)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.append(True))
    gate = GateNode(config, json.loads((args.map / 'map_meta.json').read_text()))
    try:
        while not stopping:
            rclpy.spin_once(gate.node, timeout_sec=0.05)
    finally:
        summary = {'forwarded_ticks': gate.forwarded, 'inhibited_ticks': gate.inhibited, 'transitions': gate.transitions}
        if args.log:
            args.log.write_text(json.dumps(summary, indent=2))
        print('motion_gate: ' + json.dumps({k: summary[k] for k in ('forwarded_ticks', 'inhibited_ticks')}), flush=True)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
