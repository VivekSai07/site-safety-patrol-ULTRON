import threading
from types import SimpleNamespace

import pytest

from ultron_b2_navigation import nav_contract_server as server_module
from ultron_b2_navigation.nav_contract_server import NAV_DEFAULTS, NavContractServer


@pytest.mark.parametrize('stop_ack', [True, False])
@pytest.mark.parametrize('pending_reason', [None, 'CLIENT_CANCEL', 'TIMEOUT'])
def test_cancel_after_backend_success_during_settle(
        monkeypatch, stop_ack, pending_reason):
    goal = server_module.NavigationGoal(
        'goal', 'mission', 'map', 1, 1.0, 0.0, 0.0, 0.2, 0.2, 90.0, 'test')
    handle = SimpleNamespace(request=SimpleNamespace(goal_id='goal'))

    def send_nav2(goal, tracker, state):
        if pending_reason:
            tracker.request_cancel(pending_reason, 1.0)
        tracker.on_backend_status(goal.goal_id, 4, 1.0, 'backend succeeded')

    server = SimpleNamespace(
        lock=threading.RLock(), p=NAV_DEFAULTS, now=lambda: 1.0,
        _to_goal=lambda request: goal, precheck=lambda goal: (True, 'OK'),
        _send_nav2=send_nav2, _interrupt_reason=lambda handle: 'CLIENT_CANCEL',
        _pose=lambda: [1.0, 0.0, 0.0], _wait_still=lambda: stop_ack,
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        _finish=lambda handle, result: result,
    )
    monkeypatch.setattr(server_module.rclpy, 'ok', lambda: True)
    result = NavContractServer._run(server, handle)
    assert result.status == ('TIMEOUT' if pending_reason == 'TIMEOUT' else 'CANCELLED')
    assert result.stop_acknowledged is stop_ack
    assert not result.capture_ready


@pytest.mark.parametrize('stop_ack', [True, False])
def test_unsettled_backend_success_reports_independent_stop_ack(monkeypatch, stop_ack):
    goal = server_module.NavigationGoal(
        'goal', 'mission', 'map', 1, 1.0, 0.0, 0.0, 0.2, 0.2, 90.0, 'test')
    handle = SimpleNamespace(request=SimpleNamespace(goal_id='goal'))

    def send_nav2(goal, tracker, state):
        tracker.on_backend_status(goal.goal_id, 4, 1.0, 'backend succeeded')

    server = SimpleNamespace(
        lock=threading.RLock(), p=dict(NAV_DEFAULTS, settle_timeout_s=-1), now=lambda: 1.0,
        _to_goal=lambda request: goal, precheck=lambda goal: (True, 'OK'),
        _send_nav2=send_nav2, _interrupt_reason=lambda handle: None,
        _pose=lambda: [1.0, 0.0, 0.0], _fresh_odom=lambda: None,
        _wait_still=lambda: stop_ack,
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        _finish=lambda handle, result: result,
    )
    monkeypatch.setattr(server_module.rclpy, 'ok', lambda: True)
    result = NavContractServer._run(server, handle)
    assert result.status == 'REACHED'
    assert result.stop_acknowledged is stop_ack
    assert not result.capture_ready


@pytest.mark.parametrize('stamp,received,speed,expected', [
    (None, None, 0.0, False),
    (9.0, 20.0, 0.0, False),
    (10.0, 19.0, 0.0, False),
    (10.0, 20.0, float('nan'), False),
    (10.0, 20.0, 0.0, True),
])
def test_stop_requires_fresh_finite_odometry(monkeypatch, stamp, received, speed, expected):
    monkeypatch.setattr(server_module.time, 'monotonic', lambda: 20.0)
    server = SimpleNamespace(
        odom_sample=None if stamp is None else (speed, 0.0, stamp, received),
        now=lambda: 10.0, p=NAV_DEFAULTS,
    )
    assert (NavContractServer._fresh_odom(server) is not None) is expected


def test_stop_ack_rejects_odometry_received_before_wait(monkeypatch):
    monkeypatch.setattr(server_module.time, 'monotonic', lambda: 20.0)
    server = SimpleNamespace(odom_sample=(0.0, 0.0, 10.0, 19.9), now=lambda: 10.0, p=NAV_DEFAULTS)
    assert NavContractServer._fresh_odom(server, received_after=19.9) is None
    server.odom_sample = (0.0, 0.0, 10.0, 19.95)
    assert NavContractServer._fresh_odom(server, received_after=19.9) == (0.0, 0.0)
    assert NavContractServer._fresh_odom(server, received_after=19.9, stamped_after=10.0) is None
    server.odom_sample = (0.0, 0.0, 10.1, 19.95)
    assert NavContractServer._fresh_odom(server, received_after=19.9, stamped_after=10.0) == (0.0, 0.0)


def test_stop_ack_checks_post_wait_receipt_and_sensor_stamp(monkeypatch):
    clock = iter([20.0, 20.1, 20.2, 22.1])
    monkeypatch.setattr(server_module.time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(server_module.time, 'sleep', lambda duration: None)
    samples = []

    def fresh_odom(**kwargs):
        samples.append(kwargs)
        return None

    warnings = []
    server = SimpleNamespace(p=NAV_DEFAULTS, now=lambda: 10.0, _fresh_odom=fresh_odom,
                             get_logger=lambda: SimpleNamespace(warning=warnings.append))
    assert not NavContractServer._wait_still(server)
    assert samples == [{'received_after': 20.0, 'stamped_after': 10.0}] * 2
    assert 'fresh odometry checks=0' in warnings[0]


@pytest.mark.parametrize('sample,expected', [
    (None, False), ((0.1, 0.0), False), ((0.0, 0.0), True)])
def test_stop_ack_requires_still_fresh_sample(monkeypatch, sample, expected):
    clock = iter([0.0, 0.0, 3.0])
    monkeypatch.setattr(server_module.time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(server_module.time, 'sleep', lambda duration: None)
    server = SimpleNamespace(p=NAV_DEFAULTS, now=lambda: 10.0, _fresh_odom=lambda **kwargs: sample,
                             get_logger=lambda: SimpleNamespace(warning=lambda msg: None))
    assert NavContractServer._wait_still(server) is expected
