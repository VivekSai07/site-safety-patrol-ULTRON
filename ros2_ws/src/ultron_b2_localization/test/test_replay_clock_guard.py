from types import SimpleNamespace

import numpy as np

from ultron_b2_localization.localization_monitor_node import LocalizationMonitorNode
from ultron_b2_localization.monitor_core import DEFAULT_PARAMS, LocalizationMonitor
from ultron_b2_localization.replay_clock_guard import ReplayClockGuard


def test_guard_start_stall_resume_and_jumps():
    g = ReplayClockGuard(0.5, 0.5)
    assert g.stalled(0)
    assert g.observe(1_000_000_000, 1.0) == 'waiting' and g.stalled(1.0)
    assert g.observe(1_000_000_000, 1.1) == 'unchanged' and g.stalled(1.1)
    assert g.observe(1_010_000_000, 1.2) == 'progress' and not g.stalled(1.2)
    assert g.observe(1_010_000_000, 1.4) == 'unchanged'
    assert g.stalled(1.71)
    assert g.observe(1_020_000_000, 1.8) == 'progress' and not g.stalled(1.8)
    assert g.observe(900_000_000, 1.9) == 'jump' and g.stalled(1.9)
    assert g.observe(910_000_000, 2.0) == 'progress'
    assert g.observe(2_000_000_000, 2.1) == 'jump' and g.stalled(2.1)
    assert g.observe(0, 2.2) == 'jump' and g.stalled(2.2)


def test_monitor_clock_stall_and_loop_fail_closed(monkeypatch):
    wall = [1.0]
    monkeypatch.setattr('ultron_b2_localization.localization_monitor_node.time.monotonic', lambda: wall[0])
    node = LocalizationMonitorNode.__new__(LocalizationMonitorNode)
    node.clock_guard = ReplayClockGuard(0.5, 0.5)
    node.clock_invalidated = True
    node.clock_watchdog_last_publish = 0.0
    node.heartbeat_s = 0.5
    node.pending_scans = []
    node.scored = []
    node.last_grid_t = 3.0
    node.static_transforms = {'lidar': 'mount'}
    calls = []
    node.buffer = SimpleNamespace(clear=lambda: calls.append('clear'),
                                  set_transform_static=lambda tf, _: calls.append(tf))
    node._now = lambda: 10.0
    node.monitor = LocalizationMonitor(DEFAULT_PARAMS, 'map', 1)
    node._publish = lambda _: calls.append(node.monitor.state)
    clock = lambda n: SimpleNamespace(clock=SimpleNamespace(sec=n // 10**9, nanosec=n % 10**9))
    node._on_clock(clock(10_000_000_000))
    node._on_clock(clock(10_010_000_000))
    matcher = SimpleNamespace(score=lambda *_: 1.0)
    points = np.zeros((1, 2))
    for i in range(5):
        node.monitor.update_scan(10.01 + i * 0.01, (0, 0, 0), points, matcher)
    assert node.monitor.state == 'VALID'
    wall[0] = 1.6
    node._watchdog_tick()
    assert node.monitor.state == 'STALE' and calls[-1] == 'STALE'
    assert node.monitor.last_pose_stamp is None
    node._on_clock(clock(10_020_000_000))
    for i in range(DEFAULT_PARAMS['recover_consecutive'] - 1):
        node.monitor.update_scan(10.02 + i * 0.01, (0, 0, 0), points, matcher)
    assert node.monitor.state == 'DEGRADED'
    node.monitor.update_scan(10.12, (0, 0, 0), points, matcher)
    assert node.monitor.state == 'VALID'
    # A resumed clock must invalidate even if its callback wins the race with the wall timer.
    wall[0] = 2.2
    node._on_clock(clock(10_030_000_000))
    assert node.monitor.state == 'STALE'
    for i in range(DEFAULT_PARAMS['recover_consecutive']):
        node.monitor.update_scan(10.03 + i * 0.01, (0, 0, 0), points, matcher)
    assert node.monitor.state == 'VALID'
    node.pending_scans.append('old scan')
    node._on_clock(clock(9_000_000_000))
    assert node.monitor.state == 'STALE' and not node.pending_scans
    assert calls[-3:] == ['clear', 'mount', 'STALE']
