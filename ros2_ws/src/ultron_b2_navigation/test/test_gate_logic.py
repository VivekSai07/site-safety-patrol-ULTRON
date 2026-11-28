"""Unit tests for the common final motion gate (Agent D acceptance: all conditions)."""
import math

import pytest

from ultron_b2_navigation.gate_logic import DEFAULTS, FinalGate, LocalizationState

MAP = ('hall', 3)


def loc(now, state='VALID', map_id=MAP[0], version=MAP[1], age=0.0):
    return LocalizationState(stamp=now - age, evaluated_at=now, map_id=map_id, map_version=version,
                             state=state, reason='test')


def ready_gate(now=10.0, source='autonomy', **params):
    g = FinalGate(params=params, map_id=MAP[0], map_version=MAP[1])
    g.set_operator(True, now)
    g.localization = loc(now)
    g.set_source(source, now)
    g.set_command(0.2, 0.0, 0.1, now)
    return g


def test_open_when_everything_valid():
    d = ready_gate().decide(10.0)
    assert d.open and d.cmd_out == (0.2, 0.0, 0.1) and d.reason == ''


def test_default_is_closed():
    d = FinalGate().decide(0.0)
    assert not d.open and d.cmd_out == (0.0, 0.0, 0.0)


def test_teleop_requires_localization_by_default():
    assert DEFAULTS['teleop_requires_localization'] is True
    g = ready_gate(source='teleop')
    g.localization = None
    assert g.decide(10.0).reason.startswith('LOCALIZATION')


def test_teleop_without_localization_only_when_flag_false():
    g = ready_gate(source='teleop', teleop_requires_localization=False)
    g.localization, g.map_id = None, ''
    assert g.decide(10.0).open
    g.set_source('autonomy', 10.0)
    assert g.decide(10.0).reason == 'NO_MAP_LOADED'


@pytest.mark.parametrize('src', ['bogus', 'none', '', 'TELEOP'])
def test_unknown_source_is_strictest(src):
    g = ready_gate(source=src, teleop_requires_localization=False)
    g.localization = None
    d = g.decide(10.0)
    assert d.source == 'autonomy' and not d.open


def test_stale_source_is_strictest():
    g = ready_gate(source='teleop', teleop_requires_localization=False)
    g.localization = None
    g.set_command(0.1, 0, 0, 10.6)
    g.set_operator(True, 10.6)
    d = g.decide(10.6)                     # source 0.6 s old > 0.5
    assert d.source == 'autonomy' and not d.open


def test_hysteresis_toward_less_strict_source():
    g = ready_gate(source='autonomy', teleop_requires_localization=False)
    g.decide(10.0)
    g.set_source('teleop', 10.05)
    assert g.decide(10.05).source == 'autonomy'
    g.set_source('teleop', 10.2)
    assert g.decide(10.2).source == 'teleop'


def test_hold_dominates():
    g = ready_gate()
    g.hold = True
    assert g.decide(10.0).reason == 'HOLD'
    g.hold = False
    g.set_source('hold', 10.0)
    assert g.decide(10.0).reason == 'HOLD'


@pytest.mark.parametrize('enabled,stamp,reason', [
    (None, None, 'OPERATOR_ENABLE_MISSING'), (False, 10.0, 'OPERATOR_DISABLED'),
    (True, 8.9, 'OPERATOR_ENABLE_STALE')])
def test_operator_enable(enabled, stamp, reason):
    g = ready_gate()
    g.operator_enabled, g.operator_stamp = enabled, stamp
    assert g.decide(10.0).reason == reason


@pytest.mark.parametrize('kw,prefix', [
    (dict(state='LOST'), 'LOCALIZATION: NOT_VALID'), (dict(state='DEGRADED'), 'LOCALIZATION: NOT_VALID'),
    (dict(map_id='other'), 'LOCALIZATION: MAP_MISMATCH'), (dict(version=2), 'LOCALIZATION: MAP_MISMATCH'),
    (dict(age=0.8), 'LOCALIZATION: STALE')])
def test_localization_conditions(kw, prefix):
    g = ready_gate()
    g.localization = loc(10.0, **kw)
    assert g.decide(10.0).reason.startswith(prefix)


def test_no_map_loaded():
    g = ready_gate()
    g.map_id = ''
    assert g.decide(10.0).reason == 'NO_MAP_LOADED'


def test_command_conditions():
    g = ready_gate()
    g.command = None
    assert g.decide(10.0).reason == 'NO_COMMAND'
    g.set_command(0.1, 0, 0, 9.6)
    assert g.decide(10.0).reason == 'COMMAND_STALE'
    for bad in (math.nan, math.inf):
        g.set_command(bad, 0, 0, 10.0)
        assert g.decide(10.0).reason == 'COMMAND_NONFINITE'


@pytest.mark.parametrize('cmd', [(0.7, 0, 0), (-0.4, 0, 0), (0, 0.4, 0), (0, 0, -0.9)])
def test_over_limit_rejected_not_clamped(cmd):
    g = ready_gate()
    g.set_command(*cmd, 10.0)
    d = g.decide(10.0)
    assert d.reason.startswith('COMMAND_OVER_LIMIT') and d.cmd_out == (0.0, 0.0, 0.0)
