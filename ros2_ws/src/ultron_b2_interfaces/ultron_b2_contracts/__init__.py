"""JSON contract <-> ROS 2 message converters for ``ultron_b2_interfaces``.

Two families:

* Exact mirrors of existing ULTRON JSON records (no ROS dependency upstream):
  ``ultron.localization_state/v1`` (``src/ultron/autonomy/localization_state.py``),
  ``ultron.navigation_goal/v1`` and ``ultron.navigation_result/v1``
  (``src/ultron/autonomy/navigation_contract.py``). The dict layout is the one the
  existing dataclasses produce with ``to_dict()``.
* Generic converters for the new messages: every field maps to a JSON value,
  ``builtin_interfaces/Time`` maps to float seconds, nested messages to objects.

Converters reject unknown schema versions/statuses instead of guessing.
"""
import math

from builtin_interfaces.msg import Duration, Time

LOCALIZATION_SCHEMA = 'ultron.localization_state/v1'
LOCALIZATION_STATES = ('INITIALIZING', 'VALID', 'DEGRADED', 'LOST', 'STALE')
GOAL_SCHEMA = 'ultron.navigation_goal/v1'
RESULT_SCHEMA = 'ultron.navigation_result/v1'
NAV_STATUSES = ('ACCEPTED', 'ACTIVE', 'REACHED', 'REJECTED', 'CANCELLED', 'TIMEOUT', 'FAILED')
SUPERVISOR_ACTIONS = ('INSPECT', 'SURVEY', 'RETURN_BASE', 'RETURN_BASE_WITH_GAPS', 'AWAIT_OPERATOR')


class ContractError(ValueError):
    """A record does not satisfy its contract."""


# ---------------------------------------------------------------- time / pose helpers

def float_to_time(seconds):
    if not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or seconds < 0:
        raise ContractError(f'time must be a finite non-negative number, got {seconds!r}')
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec, nanosec = sec + 1, nanosec - 1_000_000_000
    return Time(sec=sec, nanosec=nanosec)


def time_to_float(t):
    return t.sec + t.nanosec * 1e-9


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _require_schema(d, expected, key='schema'):
    got = d.get(key)
    if got != expected:
        raise ContractError(f'schema mismatch: expected {expected!r}, got {got!r}')


# ---------------------------------------------------------------- generic converters

def msg_to_dict(msg):
    """Any rosidl message -> JSON-compatible dict (Time/Duration -> float seconds)."""
    if isinstance(msg, (Time, Duration)):
        return msg.sec + msg.nanosec * 1e-9
    out = {}
    for name in msg.get_fields_and_field_types():
        out[name] = _value_to_json(getattr(msg, name))
    return out


def _value_to_json(value):
    if hasattr(value, 'get_fields_and_field_types'):
        return msg_to_dict(value)
    if isinstance(value, (list, tuple)) or type(value).__name__ in ('ndarray', 'array'):
        return [_value_to_json(v) for v in value]
    if hasattr(value, 'item') and not isinstance(value, (str, bytes)):   # numpy scalar
        return value.item()
    return value


def dict_to_msg(msg_type, data):
    """JSON dict -> message of ``msg_type``. Unknown keys are rejected."""
    if msg_type in (Time, Duration):
        sec = int(math.floor(data))
        return msg_type(sec=sec, nanosec=int(round((data - sec) * 1e9)))
    msg = msg_type()
    fields = msg.get_fields_and_field_types()
    unknown = set(data) - set(fields)
    if unknown:
        raise ContractError(f'{msg_type.__name__}: unknown fields {sorted(unknown)}')
    for name, value in data.items():
        current = getattr(msg, name)
        setattr(msg, name, _json_to_value(current, fields[name], value))
    return msg


def _json_to_value(current, type_str, value):
    if hasattr(current, 'get_fields_and_field_types'):
        return dict_to_msg(type(current), value)
    if type_str.startswith('sequence<') or '[' in type_str:
        inner = type_str[len('sequence<'):-1] if type_str.startswith('sequence<') else type_str.split('[')[0]
        inner = inner.split(',')[0]
        if '/' in inner:
            pkg, name = inner.split('/')
            module = __import__(f'{pkg}.msg', fromlist=[name])
            cls = getattr(module, name)
            return [dict_to_msg(cls, v) for v in value]
        return list(value)
    return value


# ---------------------------------------------------------------- LocalizationState

def localization_state_to_msg(d):
    from ultron_b2_interfaces.msg import LocalizationState
    _require_schema(d, LOCALIZATION_SCHEMA)
    if d['state'] not in LOCALIZATION_STATES:
        raise ContractError(f'unknown localization state {d["state"]!r}')
    if d.get('ground_truth_used'):
        raise ContractError('ground_truth_used=True: ground truth must never feed localization')
    m = LocalizationState()
    m.schema_version = LOCALIZATION_SCHEMA
    m.header.stamp = float_to_time(d['stamp'])
    m.header.frame_id = d['frame_id']
    m.evaluated_at = float_to_time(d['evaluated_at'])
    m.map_id = d['map_id']
    m.map_version = int(d['map_version'])
    m.state = d['state']
    m.reason = d['reason']
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, d['position'])
    q = m.pose.orientation
    q.x, q.y, q.z, q.w = map(float, d['orientation_xyzw'])
    m.has_match_score = d['match_score'] is not None
    m.match_score = float(d['match_score']) if m.has_match_score else 0.0
    m.has_covariance = d['covariance_xy_yaw'] is not None
    m.covariance_xy_yaw = [float(v) for v in d['covariance_xy_yaw']] if m.has_covariance else []
    m.covariance_source = d['covariance_source']
    m.age_s = float(d['age_s'])
    m.age_limit_s = float(d['age_limit_s'])
    m.clock_id = d['clock_id']
    m.child_frame_id = d['child_frame_id']
    m.source = d['source']
    m.ground_truth_used = False
    return m


def msg_to_localization_state(m):
    if m.schema_version != LOCALIZATION_SCHEMA:
        raise ContractError(f'schema mismatch: {m.schema_version!r}')
    p, q = m.pose.position, m.pose.orientation
    return {
        'stamp': time_to_float(m.header.stamp),
        'evaluated_at': time_to_float(m.evaluated_at),
        'map_id': m.map_id,
        'map_version': int(m.map_version),
        'state': m.state,
        'reason': m.reason,
        'position': [p.x, p.y, p.z],
        'orientation_xyzw': [q.x, q.y, q.z, q.w],
        'match_score': m.match_score if m.has_match_score else None,
        'covariance_xy_yaw': list(m.covariance_xy_yaw) if m.has_covariance else None,
        'covariance_source': m.covariance_source,
        'age_s': m.age_s,
        'age_limit_s': m.age_limit_s,
        'schema': LOCALIZATION_SCHEMA,
        'clock_id': m.clock_id,
        'frame_id': m.header.frame_id,
        'child_frame_id': m.child_frame_id,
        'source': m.source,
        'ground_truth_used': bool(m.ground_truth_used),
    }


# ---------------------------------------------------------------- NavigateToView goal / result

def navigation_goal_to_msg(d):
    from ultron_b2_interfaces.action import NavigateToView
    _require_schema(d, GOAL_SCHEMA)
    if d['frame_id'] != 'map':
        raise ContractError(f'navigation goals must be in the map frame, got {d["frame_id"]!r}')
    values = (d['x'], d['y'], d['yaw'], d['timeout_s'], d['requested_at'])
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ContractError('goal values must be finite numbers')
    if d['timeout_s'] <= 0:
        raise ContractError('goal timeout must be positive')
    g = NavigateToView.Goal()
    g.schema_version = GOAL_SCHEMA
    g.goal_id, g.mission_id, g.map_id = d['goal_id'], d['mission_id'], d['map_id']
    g.map_version = int(d['map_version'])
    g.target.header.frame_id = d['frame_id']
    g.target.header.stamp = float_to_time(d['requested_at'])
    g.target.pose.position.x, g.target.pose.position.y = float(d['x']), float(d['y'])
    q = g.target.pose.orientation
    q.x, q.y, q.z, q.w = yaw_to_quaternion(float(d['yaw']))
    g.xy_tolerance_m = float(d['xy_tolerance_m'])
    g.yaw_tolerance_rad = float(d['yaw_tolerance_rad'])
    g.timeout_s = float(d['timeout_s'])
    g.reason = d['reason']
    g.asset_id = d['asset_id'] or ''
    return g


def msg_to_navigation_goal(g):
    if g.schema_version != GOAL_SCHEMA:
        raise ContractError(f'schema mismatch: {g.schema_version!r}')
    q = g.target.pose.orientation
    return {
        'goal_id': g.goal_id,
        'mission_id': g.mission_id,
        'map_id': g.map_id,
        'map_version': int(g.map_version),
        'x': g.target.pose.position.x,
        'y': g.target.pose.position.y,
        'yaw': quaternion_to_yaw(q.x, q.y, q.z, q.w),
        'xy_tolerance_m': g.xy_tolerance_m,
        'yaw_tolerance_rad': g.yaw_tolerance_rad,
        'timeout_s': g.timeout_s,
        'reason': g.reason,
        'asset_id': g.asset_id or None,
        'frame_id': g.target.header.frame_id,
        'requested_at': time_to_float(g.target.header.stamp),
        'schema': GOAL_SCHEMA,
    }


def navigation_result_to_msg(d, frame_id='map'):
    from ultron_b2_interfaces.action import NavigateToView
    _require_schema(d, RESULT_SCHEMA)
    if d['status'] not in NAV_STATUSES:
        raise ContractError(f'unknown navigation status {d["status"]!r}')
    r = NavigateToView.Result()
    r.schema_version = RESULT_SCHEMA
    r.goal_id, r.status, r.reason = d['goal_id'], d['status'], d['reason']
    r.stamp = float_to_time(d['stamp'])
    pose = d['last_valid_pose']
    r.has_last_valid_pose = pose is not None
    if pose is not None:
        if len(pose) != 3:
            raise ContractError('last_valid_pose must be [x, y, yaw]')
        r.last_valid_pose.header.frame_id = frame_id
        r.last_valid_pose.header.stamp = r.stamp
        r.last_valid_pose.pose.position.x, r.last_valid_pose.pose.position.y = float(pose[0]), float(pose[1])
        q = r.last_valid_pose.pose.orientation
        q.x, q.y, q.z, q.w = yaw_to_quaternion(float(pose[2]))
    r.stop_acknowledged = bool(d['stop_acknowledged'])
    r.settled = bool(d['capture_ready'])
    return r


def msg_to_navigation_result(r):
    if r.schema_version != RESULT_SCHEMA:
        raise ContractError(f'schema mismatch: {r.schema_version!r}')
    pose = None
    if r.has_last_valid_pose:
        p, q = r.last_valid_pose.pose.position, r.last_valid_pose.pose.orientation
        pose = [p.x, p.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)]
    return {
        'goal_id': r.goal_id,
        'status': r.status,
        'reason': r.reason,
        'stamp': time_to_float(r.stamp),
        'last_valid_pose': pose,
        'stop_acknowledged': bool(r.stop_acknowledged),
        'capture_ready': bool(r.settled),
        'schema': RESULT_SCHEMA,
    }


# ---------------------------------------------------------------- generic schema-checked records

def record_to_msg(msg_type, d):
    """Generic contract record (dict with ``schema_version``) -> message, schema checked."""
    expected = getattr(msg_type, 'SCHEMA_VERSION', None)
    if expected is not None and d.get('schema_version') != expected:
        raise ContractError(f'schema mismatch: expected {expected!r}, got {d.get("schema_version")!r}')
    return dict_to_msg(msg_type, d)


def msg_to_record(msg):
    expected = getattr(type(msg), 'SCHEMA_VERSION', None)
    if expected is not None and msg.schema_version != expected:
        raise ContractError(f'schema mismatch: expected {expected!r}, got {msg.schema_version!r}')
    return msg_to_dict(msg)
