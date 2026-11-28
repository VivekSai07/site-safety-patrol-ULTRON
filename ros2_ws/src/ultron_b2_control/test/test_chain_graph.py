"""Graph test of the common command chain (control.launch.py, use_sim_time false, domain 80):
mux priority, HOLD within one control period, collision_monitor stops autonomy AND teleop,
teleop requires localization by default, motion_gate is the only cmd_vel publisher."""
import math
import time

import pytest

from _rosgraph import executable, spin_for, spin_until, start, stop, use_domain

ENV = use_domain(80)

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from geometry_msgs.msg import Twist  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

from ultron_b2_contracts import localization_state_to_msg  # noqa: E402
from ultron_b2_interfaces.msg import MotionGateStatus, OperatorEnable  # noqa: E402

NS = '/ultron/sim'
PERIOD = 0.05


class Probe:
    def __init__(self):
        self.node = rclpy.create_node('chain_probe', namespace=NS)
        rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        n = self.node
        self.pub = {k: n.create_publisher(Twist, f'control/in/{k}', 10) for k in ('teleop', 'autonomy')}
        self.hold_pub = n.create_publisher(Bool, 'ops/hold', rel)
        self.op_pub = n.create_publisher(OperatorEnable, 'ops/operator_enable', rel)
        self.loc_pub = n.create_publisher(type(localization_state_to_msg(self._loc_dict(0.0))), 'state/localization',
                                          rel)
        self.scan_pub = n.create_publisher(LaserScan, 'sensors/scan', 10)
        self.last = {}
        self.events = []                         # (monotonic, topic, vx)
        for topic, typ in (('cmd_vel', Twist), ('control/cmd_vel_muxed', Twist), ('control/cmd_vel_safe', Twist),
                           ('control/active_source', String), ('safety/motion_gate', MotionGateStatus)):
            n.create_subscription(typ, topic, lambda m, t=topic: self._rx(t, m), 10)
        self.streams = {'teleop': None, 'autonomy': None}
        self.localization = True
        self.operator = True                     # True / False: heartbeat with enabled; None: no heartbeat
        self.obstacle = False
        self._next = 0.0
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(n)

    @staticmethod
    def _loc_dict(now):
        return {'schema': 'ultron.localization_state/v1', 'stamp': now, 'evaluated_at': now, 'map_id': 'test_map',
                'map_version': 1, 'state': 'VALID', 'reason': 'test', 'position': [0.0, 0.0, 0.0],
                'orientation_xyzw': [0.0, 0.0, 0.0, 1.0], 'match_score': 0.9, 'covariance_xy_yaw': None,
                'covariance_source': 'test', 'age_s': 0.0, 'age_limit_s': 0.5, 'clock_id': 'ros_time',
                'frame_id': 'map', 'child_frame_id': 'base_link', 'source': 'TEST', 'ground_truth_used': False}

    def _rx(self, topic, msg):
        self.last[topic] = msg
        if isinstance(msg, Twist):
            self.events.append((time.monotonic(), topic, msg.linear.x))

    def step(self, now):
        if now < self._next:
            return
        self._next = now + PERIOD
        ros_now = self.node.get_clock().now()
        t = ros_now.nanoseconds * 1e-9
        if self.operator is not None:
            op = OperatorEnable(schema_version=OperatorEnable.SCHEMA_VERSION, enabled=self.operator,
                                operator_id='test')
            op.header.stamp = ros_now.to_msg()
            self.op_pub.publish(op)
        if self.localization:
            self.loc_pub.publish(localization_state_to_msg(self._loc_dict(t)))
        for k, vx in self.streams.items():
            if vx is not None:
                m = Twist()
                m.linear.x = vx
                self.pub[k].publish(m)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = ros_now.to_msg(), 'base_link'
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi / 180
        scan.range_min, scan.range_max = 0.05, 20.0
        scan.ranges = [0.4 if self.obstacle and abs(-math.pi + i * math.pi / 180) < 0.2 else 15.0
                       for i in range(361)]
        self.scan_pub.publish(scan)

    def spin(self, seconds):
        spin_for(self.ex, seconds, self.step)

    def until(self, pred, timeout=3.0):
        return spin_until(self.ex, pred, timeout, self.step)

    def vx(self, topic):
        m = self.last.get(topic)
        return None if m is None else m.linear.x

    def source(self):
        m = self.last.get('control/active_source')
        return m.data if m else None


@pytest.fixture(scope='module')
def chain():
    launch = start(['ros2', 'launch', 'ultron_b2_control', 'control.launch.py', 'use_sim_time:=false',
                    'map_id:=test_map', 'map_version:=1'])
    tf = start([executable('tf2_ros', 'static_transform_publisher'), '--frame-id', 'odom',
                '--child-frame-id', 'base_link'])
    rclpy.init()
    probe = Probe()
    try:
        assert probe.until(lambda: probe.last.get('safety/motion_gate') is not None, 30), 'motion_gate silent'
        probe.spin(3.0)                          # collision_monitor activation + TF
        yield probe
    finally:
        probe.node.destroy_node()
        rclpy.shutdown()
        out = stop(tf) + stop(launch)
        print(out[-4000:])


def _close(v, target, tol=1e-3):
    return v is not None and abs(v - target) < tol


def test_single_cmd_vel_publisher(chain):
    pubs = chain.node.get_publishers_info_by_topic(f'{NS}/cmd_vel')
    assert [(p.node_namespace, p.node_name) for p in pubs] == [(NS, 'motion_gate')]
    assert chain.node.get_publishers_info_by_topic('/api/sport/request') == []
    subs = chain.node.get_subscriptions_info_by_topic(f'{NS}/cmd_vel')
    assert {p.node_name for p in subs} - {'chain_probe'} == {'sim_driver'}


def test_autonomy_passes_gate(chain):
    chain.streams.update(autonomy=0.2, teleop=None)
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), 0.2), 5), chain.last.get('safety/motion_gate')
    assert chain.source() == 'autonomy'


def test_mux_teleop_over_autonomy(chain):
    chain.streams.update(autonomy=0.2, teleop=0.1)
    assert chain.until(lambda: _close(chain.vx('control/cmd_vel_muxed'), 0.1) and chain.source() == 'teleop'
                       and _close(chain.vx('cmd_vel'), 0.1), 3)
    chain.streams['teleop'] = None
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), 0.2) and chain.source() == 'autonomy', 3)


def test_hold_zeroes_within_one_control_period(chain):
    chain.streams.update(autonomy=0.2, teleop=0.1)
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), 0.1), 3)
    t0 = time.monotonic()
    chain.hold_pub.publish(Bool(data=True))
    assert chain.until(lambda: any(t >= t0 and tp == 'cmd_vel' and v == 0.0 for t, tp, v in chain.events), 1)
    latency = min(t for t, tp, v in chain.events if t >= t0 and tp == 'cmd_vel' and v == 0.0) - t0
    print(f'HOLD -> zero cmd_vel latency {latency * 1e3:.1f} ms')
    assert latency <= PERIOD
    chain.spin(0.5)
    assert all(v == 0.0 for t, tp, v in chain.events if tp == 'cmd_vel' and t >= t0 + latency)
    assert chain.until(lambda: chain.source() == 'hold'
                       and chain.last['safety/motion_gate'].inhibited_reason == 'HOLD', 1)
    chain.hold_pub.publish(Bool(data=False))
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), 0.1), 3)


@pytest.mark.parametrize('source,vx', [('autonomy', 0.2), ('teleop', 0.1)])
def test_collision_monitor_stops_every_source(chain, source, vx):
    chain.streams.update(autonomy=0.2, teleop=0.1 if source == 'teleop' else None)
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), vx) and chain.source() == source, 3)
    chain.obstacle = True
    assert chain.until(lambda: _close(chain.vx('control/cmd_vel_safe'), 0.0) and _close(chain.vx('cmd_vel'), 0.0),
                       2), 'collision_monitor did not stop ' + source
    assert _close(chain.vx('control/cmd_vel_muxed'), vx)
    chain.obstacle = False
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), vx), 3)


def test_teleop_requires_localization_by_default(chain):
    chain.streams.update(autonomy=None, teleop=0.1)
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), 0.1) and chain.source() == 'teleop', 3)
    assert chain.last['safety/motion_gate'].teleop_requires_localization is True
    chain.localization = False
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), 0.0), 2)
    assert chain.until(lambda: chain.last['safety/motion_gate'].inhibited_reason.startswith('LOCALIZATION'), 1)
    chain.localization = True
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), 0.1), 3)


@pytest.mark.parametrize('operator', [False, None])
def test_teleop_zero_without_operator_enable(chain, operator):
    chain.streams.update(autonomy=None, teleop=0.1)
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), 0.1) and chain.source() == 'teleop', 3)
    chain.operator = operator                    # disabled heartbeat, or heartbeat stops (limit 1.0 s)
    try:
        assert chain.until(lambda: _close(chain.vx('cmd_vel'), 0.0)
                           and chain.last['safety/motion_gate'].inhibited_reason.startswith('OPERATOR'), 2)
    finally:
        chain.operator = True
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), 0.1), 3)


def test_stale_command_and_no_source_stop(chain):
    chain.streams.update(autonomy=None, teleop=None)
    assert chain.until(lambda: _close(chain.vx('cmd_vel'), 0.0), 2)
    assert chain.until(lambda: chain.source() == 'none', 2)
    assert chain.until(lambda: chain.last['safety/motion_gate'].active_source == 'autonomy', 1)  # none => strictest
