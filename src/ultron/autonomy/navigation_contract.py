"""Navigation contracts, goal checks, goal tracking and the motion gate. No ROS dependency.

These are ULTRON's proposed logical records (team contract), not validated hardware APIs.
Everything defaults to "no motion": a missing, stale or invalid input inhibits commands, and a
goal that fails any pre-check is REJECTED before it reaches the navigation backend.
"""
from dataclasses import asdict, dataclass, field
import math
import uuid

import numpy as np

from .localization_state import LocalizationState, accept_pose, wrap

GOAL_SCHEMA = 'ultron.navigation_goal/v1'
RESULT_SCHEMA = 'ultron.navigation_result/v1'
STATUSES = ('ACCEPTED', 'ACTIVE', 'REACHED', 'REJECTED', 'CANCELLED', 'TIMEOUT', 'FAILED')
TERMINAL = {'REACHED', 'REJECTED', 'CANCELLED', 'TIMEOUT', 'FAILED'}
# action_msgs/GoalStatus codes reported by Nav2's NavigateToPose action server.
NAV2_STATUS = {0: 'FAILED',       # STATUS_UNKNOWN: never treated as progress
               1: 'ACCEPTED',     # STATUS_ACCEPTED
               2: 'ACTIVE',       # STATUS_EXECUTING
               3: 'ACTIVE',       # STATUS_CANCELING: still moving until the backend confirms
               4: 'REACHED',      # STATUS_SUCCEEDED
               5: 'CANCELLED',    # STATUS_CANCELED
               6: 'FAILED'}       # STATUS_ABORTED (planner/controller failure, unreachable, ...)
MOTION_ACTIONS = ('INSPECT', 'RETURN_BASE', 'RETURN_BASE_WITH_GAPS')


@dataclass
class NavigationGoal:
    goal_id: str
    mission_id: str
    map_id: str
    map_version: int
    x: float                      # metres, frame_id
    y: float
    yaw: float                    # radians
    xy_tolerance_m: float
    yaw_tolerance_rad: float
    timeout_s: float
    reason: str
    asset_id: str | None = None
    frame_id: str = 'map'
    requested_at: float = 0.0
    schema: str = GOAL_SCHEMA

    def to_dict(self):
        return asdict(self)


@dataclass
class NavigationResult:
    goal_id: str
    status: str
    reason: str
    stamp: float
    last_valid_pose: list | None = None
    stop_acknowledged: bool = False
    capture_ready: bool = False
    schema: str = RESULT_SCHEMA

    def to_dict(self):
        return asdict(self)


def make_goal(x, y, yaw, *, mission_id, map_id, map_version, params, reason, now, asset_id=None,
              timeout_s=None, frame_id='map'):
    values = (x, y, yaw, now, params['default_timeout_s'] if timeout_s is None else timeout_s)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError('Goal values must be finite numbers')
    timeout = float(values[-1])
    if timeout <= 0:
        raise ValueError('Goal timeout must be positive')
    return NavigationGoal(goal_id=f'GOAL-{uuid.uuid4().hex[:10]}', mission_id=mission_id, map_id=map_id,
                          map_version=int(map_version), x=float(x), y=float(y), yaw=float(wrap(yaw)),
                          xy_tolerance_m=params['goal_xy_tolerance_m'], yaw_tolerance_rad=params['goal_yaw_tolerance_rad'],
                          timeout_s=timeout, reason=reason, asset_id=asset_id, frame_id=frame_id, requested_at=float(now))


class MapFootprint:
    """Saved occupancy grid: is a circular footprint at (x, y) fully free and known?"""

    def __init__(self, yaml_path):
        import yaml
        from PIL import Image
        from pathlib import Path
        meta = yaml.safe_load(Path(yaml_path).read_text())
        image = np.array(Image.open(Path(yaml_path).parent / meta['image']))
        if meta.get('negate', 0):
            image = 255 - image
        occ = (255 - image.astype(float)) / 255.0
        self.free = occ < meta['free_thresh']
        self.occupied = occ > meta['occupied_thresh']
        self.res = float(meta['resolution'])
        self.origin = np.array(meta['origin'][:2], dtype=float)
        self.height, self.width = image.shape

    def _cell(self, x, y):
        col = int(math.floor((x - self.origin[0]) / self.res))
        row = self.height - 1 - int(math.floor((y - self.origin[1]) / self.res))
        return row, col

    def check(self, x, y, radius):
        """Return (ok, reason)."""
        cells = int(math.ceil(radius / self.res))
        row0, col0 = self._cell(x, y)
        unknown = occupied = 0
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                if (dr * dr + dc * dc) * self.res * self.res > radius * radius:
                    continue
                r, c = row0 + dr, col0 + dc
                if not (0 <= r < self.height and 0 <= c < self.width):
                    return False, 'TARGET_OUTSIDE_MAP'
                occupied += bool(self.occupied[r, c])
                unknown += bool(not self.free[r, c] and not self.occupied[r, c])
        if occupied:
            return False, f'TARGET_FOOTPRINT_OCCUPIED: {occupied} occupied cells within {radius} m'
        if unknown:
            return False, f'TARGET_FOOTPRINT_UNKNOWN: {unknown} unknown cells within {radius} m'
        return True, 'OK'


def check_goal(goal, *, map_id, map_version, localization, now, operator_enabled, footprint, clearance_m):
    """Pre-checks before a goal may reach the backend. Returns (ok, reason)."""
    if goal.frame_id != 'map':
        return False, f'UNKNOWN_FRAME: {goal.frame_id}'
    if goal.map_id != map_id or goal.map_version != map_version:
        return False, f'MAP_MISMATCH: goal {goal.map_id} v{goal.map_version}, loaded {map_id} v{map_version}'
    if not operator_enabled:
        return False, 'OPERATOR_MOTION_NOT_ENABLED'
    ok, reason = accept_pose(localization, map_id, map_version, now)
    if not ok:
        return False, f'LOCALIZATION: {reason}'
    if footprint is None:
        return False, 'NO_MAP_FOR_TARGET_CHECK'
    return footprint.check(goal.x, goal.y, clearance_m)


class GoalTracker:
    """One active goal at a time; maps backend status; timeout; arrival settle; late results ignored."""

    def __init__(self, params):
        self.p = params
        self.goal = None
        self.status = None
        self.result = None
        self.still_since = None
        self.cancel_requested = None
        self.ignored_late = 0
        self.events = []

    def _event(self, now, status, reason):
        self.events.append({'t': round(now, 3), 'goal_id': self.goal.goal_id, 'status': status, 'reason': reason})

    def submit(self, goal, ok, reason, now):
        if self.goal is not None and self.status not in TERMINAL:
            raise RuntimeError(f'Goal {self.goal.goal_id} still {self.status}; cancel it first')
        self.goal, self.result, self.still_since, self.cancel_requested = goal, None, None, None
        if not ok:
            return self._finish('REJECTED', reason, now)
        self.status = 'ACCEPTED'
        self._event(now, 'ACCEPTED', 'Pre-checks passed; sent to backend')
        return None

    def _finish(self, status, reason, now, pose=None, stopped=False):
        self.status = status
        self.result = NavigationResult(self.goal.goal_id, status, reason, now, pose, stopped)
        self._event(now, status, reason)
        return self.result

    def on_backend_status(self, goal_id, code, now, reason=''):
        """Apply a backend status. Returns the terminal result, or None."""
        if self.goal is None or goal_id != self.goal.goal_id:
            self.ignored_late += 1          # never let an old goal's result complete the current one
            return None
        if self.status in TERMINAL:
            return None
        mapped = NAV2_STATUS.get(code, 'FAILED')
        if mapped == 'CANCELLED' and self.cancel_requested:
            return self._finish('TIMEOUT' if self.cancel_requested == 'TIMEOUT' else 'CANCELLED',
                                f'{self.cancel_requested}: backend confirmed cancellation', now)
        if mapped in TERMINAL:
            return self._finish(mapped, reason or f'backend status {code}', now)
        if mapped != self.status:
            self.status = mapped
            self._event(now, mapped, reason or f'backend status {code}')
        return None

    def request_cancel(self, why, now):
        """Returns True if the backend must be asked to cancel."""
        if self.goal is None or self.status in TERMINAL or self.cancel_requested:
            return False
        self.cancel_requested = why
        self._event(now, self.status, f'cancel requested: {why}')
        return True

    def tick(self, now):
        """Returns 'CANCEL' when the goal timed out and the backend must be cancelled."""
        if self.goal is None or self.status in TERMINAL:
            return None
        if now - self.goal.requested_at > self.goal.timeout_s and not self.cancel_requested:
            self.request_cancel('TIMEOUT', now)
            return 'CANCEL'
        return None

    def settle(self, now, speed, yaw_rate, localization_valid, pose):
        """After REACHED: capture readiness once still for settle_time_s with VALID localization."""
        if self.status != 'REACHED' or self.result is None:
            return False
        still = abs(speed) <= self.p['settle_speed_m_s'] and abs(yaw_rate) <= self.p['settle_yaw_rate_rad_s']
        if not (still and localization_valid):
            self.still_since = None
            return False
        self.still_since = self.still_since if self.still_since is not None else now
        if now - self.still_since >= self.p['settle_time_s'] and not self.result.capture_ready:
            self.result.capture_ready = True
            self.result.stop_acknowledged = True
            self.result.last_valid_pose = list(pose)
            self._event(now, 'REACHED', 'CAPTURE_READY: settled and localization VALID')
        return self.result.capture_ready


@dataclass
class GateInputs:
    localization: LocalizationState | None = None
    operator_enabled: bool | None = None
    operator_stamp: float | None = None


@dataclass
class MotionGate:
    """Forward backend velocity only when every gate is satisfied; otherwise command zero."""
    params: dict
    map_id: str
    map_version: int
    inputs: GateInputs = field(default_factory=GateInputs)
    inhibited_reason: str | None = 'NO_INPUTS_YET'

    def decide(self, command, command_stamp, now):
        """command = (vx, wz) from the backend. Returns ((vx, wz), reason or None)."""
        reason = self._inhibit_reason(command, command_stamp, now)
        self.inhibited_reason = reason
        return ((0.0, 0.0), reason) if reason else (command, None)

    def _inhibit_reason(self, command, command_stamp, now):
        i = self.inputs
        if i.operator_enabled is None or i.operator_stamp is None:
            return 'OPERATOR_ENABLE_MISSING'
        if not i.operator_enabled:
            return 'OPERATOR_DISABLED'
        if not 0 <= now - i.operator_stamp <= self.params['operator_enable_max_age_s']:
            return 'OPERATOR_ENABLE_STALE'
        ok, why = accept_pose(i.localization, self.map_id, self.map_version, now)
        if not ok:
            return f'LOCALIZATION: {why}'
        if command is None or command_stamp is None:
            return 'NO_COMMAND'
        if not 0 <= now - command_stamp <= self.params['command_max_age_s']:
            return 'COMMAND_STALE'
        if not all(math.isfinite(v) for v in command):
            return 'COMMAND_NONFINITE'
        return None


def decision_to_goal(decision, *, world_to_map, mission_id, map_id, map_version, params, now):
    """Supervisor decision -> (NavigationGoal or None, reason). SURVEY/AWAIT_OPERATOR give no goal here.

    The current supervisor works in simulator world coordinates; ``world_to_map`` converts its
    (x, y, yaw) into the saved map frame. On hardware the supervisor should work in the map frame.
    """
    action = decision.get('action')
    if action not in MOTION_ACTIONS:
        return None, f'NO_MOTION_FOR_ACTION: {action}'
    gx, gy = decision['goal'][:2]
    # INSPECT carries the view yaw; return-to-base decisions carry none (0 = map +x).
    x, y, yaw = world_to_map((gx, gy, decision.get('yaw', 0.0)))
    goal = make_goal(x, y, yaw, mission_id=mission_id, map_id=map_id, map_version=map_version, params=params,
                     reason=f'{action}: {decision.get("reason", "")}', now=now, asset_id=decision.get('asset_id'))
    return goal, 'OK'
