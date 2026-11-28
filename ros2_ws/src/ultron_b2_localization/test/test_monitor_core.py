"""LocalizationState failure cases (blackout/stale, kidnap, data without match) and parity with the
validated simulation monitor (src/ultron/autonomy/localization_state.py) when the repo is present."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

from ultron_b2_localization.monitor_core import DEFAULT_PARAMS, LocalizationMonitor, MapMatcher, confirm_label

# Square room 10 x 8 m, occupied cells every 5 cm along the walls.
_W = np.concatenate([np.column_stack((np.arange(-5, 5, 0.05), np.full(200, y))) for y in (-4.0, 4.0)] +
                    [np.column_stack((np.full(160, x), np.arange(-4, 4, 0.05))) for x in (-5.0, 5.0)])


def _scan_from(pose):
    """Endpoints in base_link of an ideal 360 deg scan taken at ``pose`` (ray to the room walls)."""
    x, y, yaw = pose
    pts = []
    for a in np.linspace(-math.pi, math.pi, 180, endpoint=False):
        d = (math.cos(a + yaw), math.sin(a + yaw))
        ts = [t for t in ((5 - x) / d[0] if d[0] > 1e-9 else None, (-5 - x) / d[0] if d[0] < -1e-9 else None,
                          (4 - y) / d[1] if d[1] > 1e-9 else None, (-4 - y) / d[1] if d[1] < -1e-9 else None) if t]
        r = min(ts)
        pts.append((r * math.cos(a), r * math.sin(a)))
    return np.array(pts)


@pytest.fixture
def matcher():
    return MapMatcher(_W, 0.05, DEFAULT_PARAMS['match_tolerance_m'])


def _drive(monitor, matcher, t0, n, pose_fn, est_fn=None, dt=0.1):
    """Scans at 10 Hz: true pose pose_fn(t) generates the scan; estimator reports est_fn(t)."""
    est_fn = est_fn or pose_fn
    t = t0
    for _ in range(n):
        monitor.update_scan(t, est_fn(t), _scan_from(pose_fn(t)), matcher)
        monitor.tick(t + 0.01)
        t += dt
    return t


def test_valid_only_after_consecutive_matching_scans(matcher):
    m = LocalizationMonitor(DEFAULT_PARAMS, 'm', 1)
    pose = lambda t: (0.5 * t, 0.2, 0.1)
    t = _drive(m, matcher, 0.0, DEFAULT_PARAMS['valid_consecutive'] - 1, pose)
    assert m.state == 'INITIALIZING'
    _drive(m, matcher, t, 1, pose)
    assert m.state == 'VALID' and m.last_score > 0.95


def test_data_arrival_with_wrong_pose_never_validates(matcher):
    m = LocalizationMonitor(DEFAULT_PARAMS, 'm', 1)
    _drive(m, matcher, 0.0, 30, lambda t: (0.0, 0.0, 0.0), est_fn=lambda t: (1.3, -0.9, 0.6))
    assert m.state in ('INITIALIZING', 'LOST')
    assert all(tr['to'] != 'VALID' for tr in m.transitions)


def test_blackout_goes_stale_then_recovers(matcher):
    m = LocalizationMonitor(DEFAULT_PARAMS, 'm', 1)
    pose = lambda t: (0.0, 0.0, 0.0)
    t = _drive(m, matcher, 0.0, 10, pose)
    assert m.state == 'VALID'
    last_scan = t - 0.1
    m.tick(last_scan + 0.4)
    assert m.state == 'VALID'
    m.tick(last_scan + DEFAULT_PARAMS['scan_age_limit_s'] + 0.05)       # blackout
    assert m.state == 'STALE'
    stale_at = next(tr['t'] for tr in m.transitions if tr['to'] == 'STALE')
    assert stale_at - last_scan <= 0.75                                  # Phase 4 threshold
    t = _drive(m, matcher, last_scan + 3.0, DEFAULT_PARAMS['recover_consecutive'] - 1, pose)
    assert m.state == 'DEGRADED'                                         # needs recover_consecutive scans
    _drive(m, matcher, t, 1, pose)
    assert m.state == 'VALID'


def test_stale_recovery_ignores_buffered_pre_stale_scans(matcher):
    m = LocalizationMonitor(DEFAULT_PARAMS, 'm', 1)
    pose = (0.0, 0.0, 0.0)
    _drive(m, matcher, 0.0, 10, lambda _: pose)
    m.tick(1.5)
    assert m.state == 'STALE'
    for i in range(DEFAULT_PARAMS['recover_consecutive'] + 1):
        m.update_scan(1.0 + i * 0.04, pose, _scan_from(pose), matcher)
    assert m.state == 'STALE' and m.good_run == 0
    m.update_scan(1.6, pose, _scan_from(pose), matcher)
    m.update_scan(1.4, pose, _scan_from(pose), matcher)
    m.update_scan(1.55, pose, _scan_from(pose), matcher)
    m.update_scan(1.6, pose, _scan_from(pose), matcher)
    assert m.good_run == 1
    _drive(m, matcher, 1.7, DEFAULT_PARAMS['recover_consecutive'] - 2, lambda _: pose)
    assert m.state == 'DEGRADED'
    _drive(m, matcher, 2.5, 1, lambda _: pose)
    assert m.state == 'VALID'


def test_missing_transform_degrades_then_stale(matcher):
    m = LocalizationMonitor(DEFAULT_PARAMS, 'm', 1)
    t = _drive(m, matcher, 0.0, 10, lambda t: (0.0, 0.0, 0.0))
    m.update_scan(t, None, _scan_from((0.0, 0.0, 0.0)), matcher)
    assert m.state == 'DEGRADED'
    m.tick(t + 0.2)
    m.update_scan(t + 0.2, None, _scan_from((0.0, 0.0, 0.0)), matcher)
    m.tick(t + 0.2 + 0.01)
    m.tick(t + 0.7)
    assert m.state == 'STALE'


def test_received_unscored_scans_keep_scan_fresh_but_not_pose(matcher):
    m = LocalizationMonitor(DEFAULT_PARAMS, 'm', 1)
    t = _drive(m, matcher, 0.0, 10, lambda t: (0.0, 0.0, 0.0))
    assert m.state == 'VALID'
    last_scored = t - 0.1
    for k in range(1, 10):  # scans keep arriving while their transform is still pending
        m.note_scan(last_scored + 0.1 * k)
        m.tick(last_scored + 0.1 * k + 0.05)
        if 0.1 * k + 0.05 <= DEFAULT_PARAMS['pose_age_limit_s']:
            assert m.state == 'VALID'
    assert m.state == 'STALE' and 'transform' in m.reason


def test_kidnap_leaves_valid_on_next_scan_and_goes_lost(matcher):
    m = LocalizationMonitor(DEFAULT_PARAMS, 'm', 1)
    true_pose = lambda t: (0.0, 0.0, 0.0) if t < 0.95 else (3.0, 2.5, 1.2)    # teleported at t = 1 s
    estimate = lambda t: (0.0, 0.0, 0.0)                                   # estimator not told
    t = _drive(m, matcher, 0.0, 10, true_pose, estimate)
    assert m.state == 'VALID'
    _drive(m, matcher, t, 1, true_pose, estimate)
    assert m.state == 'DEGRADED' and m.last_score < DEFAULT_PARAMS['score_valid_min']
    _drive(m, matcher, t + 0.1, DEFAULT_PARAMS['lost_consecutive'], true_pose, estimate)
    assert m.state == 'LOST'
    # Poses between the last good scan and the teleport are only vouched for by a later scan.
    scored = [(0.9, 'VALID'), (1.0, 'DEGRADED')]
    assert confirm_label(0.95, scored, now=1.2, max_wait=0.65) == 'DEGRADED'
    assert confirm_label(1.05, scored, now=1.1, max_wait=0.65) is None
    assert confirm_label(1.05, scored, now=2.0, max_wait=0.65) == 'UNCONFIRMED'


def test_pose_jump_revalidates(matcher):
    m = LocalizationMonitor(DEFAULT_PARAMS, 'm', 1)
    t = _drive(m, matcher, 0.0, 10, lambda t: (0.0, 0.0, 0.0))
    _drive(m, matcher, t, 1, lambda t: (1.0, 0.0, 0.0))                 # consistent but jumped 1 m
    assert m.state == 'DEGRADED' and 'jump' in m.reason


def _repo_src():
    for parent in Path(__file__).resolve().parents:
        if (parent / 'src' / 'ultron' / 'autonomy' / 'localization_state.py').is_file():
            return parent / 'src'
    return None


@pytest.mark.skipif(_repo_src() is None, reason='repository source tree not available')
def test_parity_with_validated_simulation_monitor(matcher):
    sys.path.insert(0, str(_repo_src()))
    try:
        from ultron.autonomy.localization_state import LocalizationMonitor as Original, MapMatcher as OriginalMatcher
    finally:
        sys.path.pop(0)
    original_matcher = OriginalMatcher(_W, 0.05, DEFAULT_PARAMS['match_tolerance_m'])
    assert np.array_equal(original_matcher.mask, matcher.mask)
    rng = np.random.default_rng(3)
    ported, original = LocalizationMonitor(DEFAULT_PARAMS, 'm', 1), Original(DEFAULT_PARAMS, 'm', 1)
    t = 0.0
    for i in range(300):
        true = (0.01 * i, 0.0, 0.0) if i < 150 else (2.0, 1.5, 0.8)
        est = None if 60 <= i < 63 else (0.01 * i, 0.0, 0.0)
        scan = _scan_from(true) + rng.normal(0, 0.02, (180, 2))
        if 100 <= i < 110:
            t += 0.1                                                      # blackout ticks only
        else:
            ported.update_scan(t, est, scan, matcher)
            original.update_scan(t, est, scan, original_matcher)
        ported.tick(t + 0.01)
        original.tick(t + 0.01)
        assert (ported.state, ported.reason) == (original.state, original.reason), i
        t += 0.1
    assert ported.transitions == original.transitions
    assert {tr['to'] for tr in ported.transitions} >= {'VALID', 'DEGRADED', 'STALE', 'LOST'}
