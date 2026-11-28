"""Acceptance runner must propagate failed or missing cases to its caller."""
import json
import sys
import types

import pytest

from ultron_b2_navigation import sim_nav_cases


@pytest.mark.parametrize(('verdict', 'exit_code'), [('PASS', 0), ('FAIL', 1)])
def test_runner_exit_matches_recorded_verdict(monkeypatch, tmp_path, verdict, exit_code):
    class FakeRunner:
        map_id = 'test'
        map_version = 1
        gate_log = []
        chain = []
        truth = []

        def __init__(self, *_args):
            self.results = []

        def __getattr__(self, _name):
            return lambda *_args: None

        def case(self, name, _fn):
            self.results.append({'case': name, 'verdict': verdict})

    monkeypatch.setenv('ROS_DOMAIN_ID', '71')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setitem(sys.modules, 'rclpy', types.SimpleNamespace(init=lambda: None, shutdown=lambda: None))
    monkeypatch.setattr(sim_nav_cases, 'Runner', FakeRunner)
    monkeypatch.setattr(sim_nav_cases, 'spawn', lambda *_args: types.SimpleNamespace(poll=lambda: None))
    monkeypatch.setattr(sim_nav_cases, 'stop', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(sim_nav_cases.time, 'sleep', lambda _seconds: None)
    out = tmp_path / 'case'

    assert sim_nav_cases.main([str(out), '--map', str(tmp_path), '--cases', 'reach_near']) == exit_code
    assert json.loads((out / 'nav_cases.json').read_text())['verdict'] == verdict


def test_empty_case_selection_is_rejected(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '71')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    with pytest.raises(SystemExit, match='at least one navigation case'):
        sim_nav_cases.main([str(tmp_path / 'case'), '--map', str(tmp_path), '--cases', ','])


@pytest.mark.parametrize(('responses', 'message'), [
    ([False], 'initially VALID'),
    ([True, False], 'did not invalidate'),
])
def test_teleop_blackout_requires_valid_baseline_and_loss(responses, message):
    class Case:
        def spin(self, *_args, **_kwargs):
            return responses.pop(0)

        def control(self, **_kwargs):
            pass

    with pytest.raises(RuntimeError, match=message):
        sim_nav_cases.Runner.teleop_no_localization(Case())


def test_teleop_blackout_needs_truth_and_a_closed_gate():
    class Case:
        truth = []
        gate_log = [{'t': 11.0, 'open': False}]
        loc = types.SimpleNamespace(state='STALE')
        teleop = None
        times = iter((10.0, 12.0))

        def spin(self, *_args, **_kwargs):
            return True

        def control(self, **_kwargs):
            pass

        def now(self):
            return next(self.times)

        def turned_after(self, *_args):
            return 0.0

        def gate_inhibit_after(self, *_args):
            return {'reason': 'LOCALIZATION: STALE'}

    case = Case()
    result = sim_nav_cases.Runner.teleop_no_localization(case)
    assert not result['checks']['truth_observed']
    assert case.teleop is None
    case.truth = [(10.5, 0, 0, 0), (11.5, 0, 0, 0)]
    case.gate_log.append({'t': 11.5, 'open': True})
    case.times = iter((10.0, 12.0))
    result = sim_nav_cases.Runner.teleop_no_localization(case)
    assert result['checks']['truth_observed']
    assert not result['checks']['gate_closed']
