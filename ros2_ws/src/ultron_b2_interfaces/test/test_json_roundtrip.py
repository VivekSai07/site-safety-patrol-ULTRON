"""JSON <-> msg round-trip tests for every ultron_b2_interfaces message, service and action."""
import json
import math
from pathlib import Path
import sys

import pytest

from ultron_b2_contracts import (
    ContractError, dict_to_msg, float_to_time, localization_state_to_msg, msg_to_dict,
    msg_to_localization_state, msg_to_navigation_goal, msg_to_navigation_result, msg_to_record,
    navigation_goal_to_msg, navigation_result_to_msg, record_to_msg, time_to_float,
)
from ultron_b2_interfaces import action, msg, srv

REPO_SRC = Path(__file__).resolve().parents[4] / 'src'


def _repo_module(name):
    if not (REPO_SRC / 'ultron').is_dir():
        pytest.skip(f'ULTRON repo sources not found at {REPO_SRC}')
    if str(REPO_SRC) not in sys.path:
        sys.path.insert(0, str(REPO_SRC))
    return __import__(name, fromlist=['*'])


def _close(a, b, tol=1e-9):
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_close(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, float) or isinstance(b, float):
        return isinstance(a, (int, float)) and isinstance(b, (int, float)) and abs(a - b) <= tol * max(1.0, abs(b))
    return a == b


def _json_cycle(d):
    return json.loads(json.dumps(d))


# ---------------------------------------------------------------- exact JSON contracts

def test_localization_state_roundtrip_matches_repo_dataclass():
    ls = _repo_module('ultron.autonomy.localization_state')
    state = ls.LocalizationState(stamp=12.25, evaluated_at=12.3, map_id='hall', map_version=3, state='VALID',
                                 reason='5 consecutive scans match the map', position=[1.5, -2.0, 0.0],
                                 orientation_xyzw=[0.0, 0.0, 0.3826834, 0.9238795], match_score=0.91,
                                 age_s=0.05, age_limit_s=0.5)
    record = _json_cycle(state.to_dict())
    back = msg_to_localization_state(localization_state_to_msg(record))
    assert back.keys() == record.keys()
    for key, value in record.items():
        assert _close(back[key], value), key


def test_localization_state_nulls_and_covariance_preserved():
    ls = _repo_module('ultron.autonomy.localization_state')
    for score, cov in ((None, None), (0.4, [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.002])):
        record = ls.LocalizationState(stamp=1.0, evaluated_at=1.0, map_id='m', map_version=1, state='DEGRADED',
                                      reason='r', match_score=score, covariance_xy_yaw=cov).to_dict()
        back = msg_to_localization_state(localization_state_to_msg(record))
        assert back['match_score'] == score
        assert _close(back['covariance_xy_yaw'], cov)


def test_localization_state_rejects_bad_records():
    base = _repo_module('ultron.autonomy.localization_state').LocalizationState(
        stamp=1.0, evaluated_at=1.0, map_id='m', map_version=1, state='VALID', reason='r').to_dict()
    for bad in ({'state': 'SORT_OF_OK'}, {'schema': 'ultron.localization_state/v0'}, {'ground_truth_used': True}):
        with pytest.raises(ContractError):
            localization_state_to_msg(base | bad)


def test_navigation_goal_roundtrip_matches_repo_make_goal():
    nc = _repo_module('ultron.autonomy.navigation_contract')
    params = {'default_timeout_s': 60.0, 'goal_xy_tolerance_m': 0.25, 'goal_yaw_tolerance_rad': 0.2}
    for asset in ('L-03', None):
        goal = nc.make_goal(2.0, -1.0, 2.5, mission_id='M1', map_id='hall', map_version=2, params=params,
                            reason='INSPECT: ladder', now=42.5, asset_id=asset)
        record = _json_cycle(goal.to_dict())
        back = msg_to_navigation_goal(navigation_goal_to_msg(record))
        assert back.keys() == record.keys()
        for key, value in record.items():
            assert _close(back[key], value), key


def test_navigation_goal_rejects_non_map_frame_and_nonfinite():
    nc = _repo_module('ultron.autonomy.navigation_contract')
    params = {'default_timeout_s': 60.0, 'goal_xy_tolerance_m': 0.25, 'goal_yaw_tolerance_rad': 0.2}
    record = nc.make_goal(0, 0, 0, mission_id='M', map_id='m', map_version=1, params=params, reason='r', now=0).to_dict()
    for bad in ({'frame_id': 'odom'}, {'x': math.nan}, {'timeout_s': 0.0}, {'schema': 'other'}):
        with pytest.raises(ContractError):
            navigation_goal_to_msg(record | bad)


def test_navigation_result_roundtrip_all_statuses():
    nc = _repo_module('ultron.autonomy.navigation_contract')
    assert set(nc.STATUSES) == {action.NavigateToView.Goal.STATUS_ACCEPTED, 'ACTIVE', 'REACHED', 'REJECTED',
                                'CANCELLED', 'TIMEOUT', 'FAILED'}
    for status in nc.STATUSES:
        result = nc.NavigationResult('GOAL-1', status, 'why', 7.5)
        if status == 'REACHED':
            result.last_valid_pose, result.capture_ready, result.stop_acknowledged = [1.0, 2.0, -0.5], True, True
        record = _json_cycle(result.to_dict())
        back = msg_to_navigation_result(navigation_result_to_msg(record))
        assert back.keys() == record.keys()
        for key, value in record.items():
            assert _close(back[key], value), key
    with pytest.raises(ContractError):
        navigation_result_to_msg(nc.NavigationResult('G', 'DONE', 'x', 1.0).to_dict())


def test_supervisor_actions_match_mission_constants():
    sup = _repo_module('ultron.supervisor')
    src = Path(sup.__file__).read_text()
    for name in ('INSPECT', 'SURVEY', 'RETURN_BASE', 'RETURN_BASE_WITH_GAPS', 'AWAIT_OPERATOR'):
        assert f"'{name}'" in src
        assert getattr(msg.MissionStatus, f'ACTION_{name}') == name


# ---------------------------------------------------------------- generic round trips

def _sample(cls, depth=0):
    """Populate every field of ``cls`` with deterministic non-default values."""
    m = cls()
    for i, (name, type_str) in enumerate(m.get_fields_and_field_types().items()):
        current = getattr(m, name)
        if name == 'schema_version':
            setattr(m, name, getattr(cls, 'SCHEMA_VERSION', getattr(cls, 'RESULT_SCHEMA_VERSION', 'v-test')))
        elif hasattr(current, 'get_fields_and_field_types'):
            if type(current).__name__ == 'Time':
                setattr(m, name, float_to_time(100.0 + i + 0.25))
            else:
                setattr(m, name, _sample(type(current), depth + 1))
        elif type_str.startswith('sequence<') and '/' in type_str:
            inner = type_str[len('sequence<'):-1].split(',')[0]
            pkg, nm = inner.split('/')
            sub = getattr(__import__(f'{pkg}.msg', fromlist=[nm]), nm)
            setattr(m, name, [_sample(sub, depth + 1)] if depth < 3 else [])
        elif type_str == 'sequence<string>':
            setattr(m, name, [f'{name}-a', f'{name}-b'])
        elif type_str.startswith('double[') or type_str.startswith('float['):
            n = int(type_str.split('[')[1].rstrip(']'))
            setattr(m, name, [0.5 * k for k in range(n)])
        elif type_str in ('sequence<double>', 'sequence<float>'):
            setattr(m, name, [0.5, 1.5])
        elif type_str == 'string':
            setattr(m, name, f'{name}-{i}')
        elif type_str == 'boolean':
            setattr(m, name, True)
        elif type_str in ('double', 'float'):
            setattr(m, name, 0.25 + i)
        elif 'int' in type_str:
            setattr(m, name, i + 1)
    return m


ALL_TYPES = [getattr(msg, n) for n in ('LocalizationState', 'MotionGateStatus', 'OperatorEnable', 'LadderDetection',
                                       'LadderDetectionArray', 'LadderCandidate', 'LadderCandidateArray',
                                       'MissionStatus', 'MissionEvent', 'CoverageReport', 'MapInfo')]
for _srv in (srv.Arm, srv.SaveMap, srv.LoadMap, srv.SetPatrolZone, srv.SimScenario):
    ALL_TYPES += [_srv.Request, _srv.Response]
for _act in (action.NavigateToView, action.CaptureViews, action.RunMission):
    ALL_TYPES += [_act.Goal, _act.Result, _act.Feedback]


@pytest.mark.parametrize('cls', ALL_TYPES, ids=lambda c: c.__name__)
def test_generic_json_roundtrip(cls):
    original = _sample(cls)
    record = _json_cycle(msg_to_record(original))
    assert record_to_msg(cls, record) == original
    assert msg_to_dict(dict_to_msg(cls, record)) == record


def test_every_message_has_schema_version_field_and_constant():
    for cls in [getattr(msg, n) for n in dir(msg) if not n.startswith('_')]:
        fields = cls.get_fields_and_field_types()
        assert 'schema_version' in fields, cls.__name__
        assert getattr(cls, 'SCHEMA_VERSION').startswith('ultron.'), cls.__name__


def test_generic_rejects_unknown_fields_and_schema_mismatch():
    with pytest.raises(ContractError):
        dict_to_msg(msg.OperatorEnable, {'enabled': True, 'surprise': 1})
    with pytest.raises(ContractError):
        record_to_msg(msg.OperatorEnable, {'schema_version': 'ultron.operator_enable/v0', 'enabled': True})


def test_time_conversion_exact_to_nanoseconds():
    for t in (0.0, 1.5, 1732780000.123456789, 0.999999999):
        assert abs(time_to_float(float_to_time(t)) - t) < 1e-6
    with pytest.raises(ContractError):
        float_to_time(-1.0)
