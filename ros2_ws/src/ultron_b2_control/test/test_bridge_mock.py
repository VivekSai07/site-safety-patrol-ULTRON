"""b2_sport_bridge against mock_sport_server ONLY (domain 81, localhost). Never a robot."""
import json
import math
import os
import subprocess
import time
from pathlib import Path

import pytest

from _rosgraph import executable, spin_for, spin_until, start, stop, use_domain

ENV = use_domain(81)

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

from geometry_msgs.msg import Twist  # noqa: E402

from ultron_b2_interfaces.srv import Arm  # noqa: E402

NS = '/ultron/sim'
WATCHDOG = 0.3
RATE = 20.0
LOG_DIR = Path(os.environ.get('ULTRON_TEST_LOG_DIR', Path(__file__).resolve().parents[3] / 'log' / 'agent_d_tests'))


def bridge_cmd(**params):
    p = {'target': 'mock', 'hardware_enable': 'true', 'watchdog_s': str(WATCHDOG), 'move_rate_hz': str(RATE),
         **{k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in params.items()}}
    cmd = [executable('ultron_b2_control', 'b2_sport_bridge'), '--ros-args', '-r', f'__ns:={NS}']
    for k, v in p.items():
        cmd += ['-p', f'{k}:={v}']
    return cmd


class Harness:
    def __init__(self, tag, **params):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.log = LOG_DIR / f'mock_{tag}_{os.getpid()}.jsonl'
        self.log.unlink(missing_ok=True)
        self.mock = start([executable('ultron_b2_control', 'mock_sport_server'), '--ros-args',
                           '-p', f'log_path:={self.log}'])
        self.bridge = start(bridge_cmd(**params))
        self.node = rclpy.create_node(f'bridge_probe_{tag}', namespace=NS)
        self.cmd_pub = self.node.create_publisher(Twist, 'cmd_vel', 1)
        self.arm_cli = self.node.create_client(Arm, 'control/arm')
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.sent = []                       # monotonic stamps of published cmd_vel

    def close(self):
        self.node.destroy_node()
        return stop(self.bridge) + stop(self.mock)

    def records(self):
        if not self.log.exists():
            return []
        return [json.loads(line) for line in self.log.read_text().splitlines() if line.strip()]

    def wait_ready(self):
        assert spin_until(self.ex, lambda: self.arm_cli.service_is_ready()
                          and self.node.count_subscribers(f'{NS}/cmd_vel') > 0, 20), 'bridge not up'
        spin_for(self.ex, 1.0)

    def arm(self, arm=True, operator='tester'):
        fut = self.arm_cli.call_async(Arm.Request(schema_version=Arm.Request.SCHEMA_VERSION, arm=arm,
                                                  operator_id=operator, reason='test'))
        assert spin_until(self.ex, fut.done, 5)
        return fut.result()

    def stream(self, seconds, vx=0.3, vy=0.0, wz=0.0):
        nxt = [0.0]

        def step(now):
            if now >= nxt[0]:
                nxt[0] = now + 0.05
                m = Twist()
                m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
                self.cmd_pub.publish(m)
                self.sent.append(time.monotonic())
        spin_for(self.ex, seconds, step)


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_hardware_enable_false_is_silent():
    h = Harness('hwoff', hardware_enable=False)
    try:
        h.wait_ready()
        res = h.arm()
        assert not res.success and 'hardware_enable' in res.message
        h.stream(1.0)
        spin_for(h.ex, 0.5)
        assert h.node.get_publishers_info_by_topic('/api/sport/request') == []
        assert h.records() == []
    finally:
        print(h.close()[-2000:])


def test_disarmed_silent_then_move_at_rate_then_one_stopmove():
    h = Harness('main')
    try:
        h.wait_ready()
        h.stream(1.0)
        spin_for(h.ex, 0.5)
        assert h.records() == [], 'disarmed bridge sent requests'
        res = h.arm(operator='')
        assert not res.success                                   # operator_id required
        assert h.arm().success
        t_start = time.monotonic()
        h.stream(2.0, vx=2.0, vy=-1.0, wz=5.0)                   # over-limit: clamped
        t_last = h.sent[-1]
        spin_for(h.ex, 1.5)
        recs = h.records()
        moves = [r for r in recs if r['api_id'] == 1008]
        stops = [r for r in recs if r['api_id'] == 1003]
        rate = len([r for r in moves if t_start + 0.5 <= r['t'] <= t_last]) / (t_last - t_start - 0.5)
        print(f'Move count {len(moves)}, rate {rate:.1f} Hz, StopMove {len(stops)}')
        assert abs(rate - RATE) <= 2.0
        params = [json.loads(r['parameter']) for r in moves]
        assert max(p['x'] for p in params) <= 0.6 + 1e-9 and min(p['y'] for p in params) >= -0.3 - 1e-9
        assert max(p['z'] for p in params) <= 0.8 + 1e-9
        assert params[-1] == pytest.approx({'x': 0.6, 'y': -0.3, 'z': 0.8})
        assert params[0]['x'] <= 1.0 / RATE + 1e-9                # accel-limited start
        assert len(stops) == 1, stops
        stop_latency = stops[0]['t'] - t_last
        print(f'StopMove {stop_latency * 1e3:.0f} ms after last cmd_vel (limit {(WATCHDOG + 0.05) * 1e3:.0f} ms)')
        assert stop_latency <= WATCHDOG + 0.05
        assert all(r['t'] < stops[0]['t'] for r in moves)
    finally:
        print(h.close()[-2000:])


def test_nonfinite_latches_fault_single_stop():
    h = Harness('nan')
    try:
        h.wait_ready()
        assert h.arm().success
        h.stream(0.5)
        t_bad = time.monotonic()
        h.stream(0.2, vx=math.nan)
        h.stream(0.5)                                            # finite again: must stay stopped (fault latched)
        spin_for(h.ex, 0.5)
        recs = h.records()
        stops = [r for r in recs if r['api_id'] == 1003]
        assert len(stops) == 1 and stops[0]['t'] - t_bad < 0.1
        assert not [r for r in recs if r['api_id'] == 1008 and r['t'] > stops[0]['t']]
    finally:
        print(h.close()[-2000:])


def test_wrong_domain_refused():
    out = subprocess.run(bridge_cmd(expected_domain_id=99), env=dict(os.environ), capture_output=True, text=True,
                         timeout=30)
    assert out.returncode == 2
    assert 'REFUSED' in out.stdout + out.stderr


def test_robot_target_refused_off_robot_domain():
    out = subprocess.run(bridge_cmd(target='robot'), env=dict(os.environ), capture_output=True, text=True,
                         timeout=30)
    assert out.returncode == 2 and 'REFUSED' in out.stdout + out.stderr
