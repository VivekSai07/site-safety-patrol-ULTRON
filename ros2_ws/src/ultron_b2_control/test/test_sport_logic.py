"""Pure bridge logic: preflight refusals, clamps, accel limit, arming, watchdog, non-finite."""
import json
import math

import pytest

from ultron_b2_control.active_source_monitor import select_source
from ultron_b2_control.sport_logic import (API_MOVE, API_STOPMOVE, BridgeCore, Limits, clamp_velocity,
                                           limit_accel, move_parameter, preflight)

MOCK_ENV = {'ROS_DOMAIN_ID': '73', 'ROS_LOCALHOST_ONLY': '1'}


def test_api_ids_pinned():
    assert (API_STOPMOVE, API_MOVE) == (1003, 1008)
    assert json.loads(move_parameter(0.1, -0.2, 0.3)) == {'x': 0.1, 'y': -0.2, 'z': 0.3}


def test_source_monitor_only_suppresses_known_shutdown_race(monkeypatch):
    from ultron_b2_control import active_source_monitor as monitor

    class Node:
        def destroy_node(self):
            pass

    monkeypatch.setattr(monitor.rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(monitor.rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(monitor, 'ActiveSourceMonitor', Node)
    message = 'Unable to convert call argument to Python object (compile in debug mode for details)'

    def fail():
        raise RuntimeError(message)

    monkeypatch.setattr(monitor.rclpy, 'spin', lambda _: fail())
    monkeypatch.setattr(monitor.rclpy, 'ok', lambda: False)
    monitor.main()
    monkeypatch.setattr(monitor.rclpy, 'ok', lambda: True)
    with pytest.raises(RuntimeError, match='Unable to convert'):
        monitor.main()
    monkeypatch.setattr(monitor.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(monitor.rclpy, 'spin', lambda _: (_ for _ in ()).throw(RuntimeError('other error')))
    with pytest.raises(RuntimeError, match='other error'):
        monitor.main()


@pytest.mark.parametrize('kw,ok', [
    (dict(target='mock', env=MOCK_ENV, namespace='/ultron/sim'), True),
    (dict(target='mock', env={'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'}, namespace='/ultron/sim'), False),
    (dict(target='mock', env={'ROS_LOCALHOST_ONLY': '1'}, namespace='/ultron/sim'), False),
    (dict(target='mock', env={'ROS_DOMAIN_ID': '73'}, namespace='/ultron/sim'), False),
    (dict(target='mock', env=MOCK_ENV, namespace='/ultron/live'), False),
    (dict(target='mock', env=MOCK_ENV, namespace='/ultron/sim', expected_domain_id=74), False),
    (dict(target='robot', env={'ROS_DOMAIN_ID': '0'}, namespace='/ultron/live'), False),
    (dict(target='robot', env={'ROS_DOMAIN_ID': '0'}, namespace='/ultron/live', hardware_enable=True), False),
    (dict(target='robot', env=MOCK_ENV, namespace='/ultron/live', hardware_enable=True, hw_token_ok=(True, 'x')),
     False),
    (dict(target='other', env=MOCK_ENV, namespace='/ultron/sim'), False),
])
def test_preflight(kw, ok):
    kw.setdefault('hardware_enable', False)
    assert preflight(**kw)[0] is ok


def test_clamp_and_accel():
    lim = Limits()
    assert clamp_velocity((2.0, -1.0, 5.0), lim) == (0.6, -0.3, 0.8)
    assert clamp_velocity((-2.0, 1.0, -5.0), lim) == (-0.3, 0.3, -0.8)
    out = limit_accel((0.6, 0.0, 0.8), (0.0, 0.0, 0.0), lim, 0.05)
    assert out == pytest.approx((0.05, 0.0, 0.1))
    assert limit_accel((0.0, 0.0, 0.0), (0.5, 0.2, 0.4), lim, 0.05) == (0.0, 0.0, 0.0)   # braking immediate


def test_disarmed_is_silent():
    b = BridgeCore()
    b.on_cmd(0.3, 0, 0, 0.0)
    assert b.tick(0.0) is None and b.check_watchdog(1.0) is None


def test_arm_refused_while_moving_command_streams():
    b = BridgeCore()
    b.on_cmd(0.3, 0, 0, 0.0)
    assert not b.arm(0.1)[0]
    assert b.arm(0.5)[0]                           # stale command -> at rest


def test_move_then_exactly_one_watchdog_stop():
    b = BridgeCore(watchdog_s=0.3)
    assert b.arm(0.0)[0]
    b.on_cmd(0.3, 0, 0.2, 0.0)
    kind, v = b.tick(0.0)
    assert kind == 'MOVE' and v[0] > 0
    assert b.check_watchdog(0.29) is None
    assert b.check_watchdog(0.31)[0] == 'STOP'
    assert b.check_watchdog(0.4) is None and b.tick(0.4) is None


def test_zero_command_single_stop():
    b = BridgeCore()
    b.arm(0.0)
    b.on_cmd(0.2, 0, 0, 0.0)
    b.tick(0.0)
    b.on_cmd(0, 0, 0, 0.05)
    assert b.tick(0.05) == ('STOP', 'ZERO_COMMAND')
    assert b.tick(0.1) is None


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_nonfinite_latches_fault_and_stops(bad):
    b = BridgeCore()
    b.arm(0.0)
    b.on_cmd(0.2, 0, 0, 0.0)
    b.tick(0.0)
    b.on_cmd(bad, 0, 0, 0.05)
    assert b.tick(0.05)[0] == 'STOP' and not b.armed and b.fault == 'NONFINITE_CMD_VEL'
    b.on_cmd(0.2, 0, 0, 0.1)
    assert b.tick(0.1) is None


def test_disarm_sends_stop_once():
    b = BridgeCore()
    b.arm(0.0)
    b.on_cmd(0.2, 0, 0, 0.0)
    b.tick(0.0)
    b.disarm('operator')
    assert b.tick(0.01) == ('STOP', 'operator') and b.tick(0.02) is None


INPUTS = lambda t_tel, t_aut: {'teleop': (100, 0.5, t_tel), 'autonomy': (10, 0.5, t_aut)}  # noqa: E731


def test_select_source_priority_and_hold():
    assert select_source(1.0, INPUTS(0.9, 0.9), {'hold': (255, 0, False, None)}) == 'teleop'
    assert select_source(1.0, INPUTS(0.2, 0.9), {'hold': (255, 0, False, None)}) == 'autonomy'
    assert select_source(1.0, INPUTS(None, None), {'hold': (255, 0, False, None)}) == 'none'
    assert select_source(1.0, INPUTS(0.9, 0.9), {'hold': (255, 0, True, 0.5)}) == 'hold'
    assert select_source(1.0, INPUTS(0.9, 0.9), {'hold': (255, 0.2, False, 0.5)}) == 'hold'   # silent lock
