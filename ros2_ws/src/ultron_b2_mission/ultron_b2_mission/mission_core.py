"""Mission state machine (pure Python, deterministic, ROS-free; driven by the mission_manager node).

PATROL -> EXPLORE -> DETECT -> APPROACH -> SETTLE -> CAPTURE -> ASSESS -> COMMIT -> next | RETURN_BASE,
with HOLD and AWAIT_OPERATOR reachable from every active state.

Motion authority: the core only *requests* NavigateToView goals (commands ``NAV``) and cancels them.
It never produces velocities. While any inhibit cause is active (hold, operator enable lost or stale,
localization not VALID / stale / wrong map, map unavailable) the core emits no NAV or CAPTURE
commands, cancels the active goal and asserts the ops/hold lock (``HOLD`` command).

The PATROL hub ports ``src/ultron/supervisor.py::decide`` (INSPECT nearest reachable pending ladder,
else SURVEY while survey remains, else RETURN_BASE / RETURN_BASE_WITH_GAPS, AWAIT_OPERATOR when base
is unreachable). Captures are evidence only: ASSESS is an asynchronous hand-off and COMMIT stores
``EVIDENCE_CAPTURED`` with the verdict pending. No verdict, no safe-to-use claim is ever produced here.
"""
from dataclasses import dataclass, field
import math

from .frontier_explorer import find_frontiers
from .view_pose_planner import ViewPlannerConfig, plan_views

IDLE, PATROL, EXPLORE, DETECT, APPROACH, SETTLE, CAPTURE, ASSESS, COMMIT = (
    'IDLE', 'PATROL', 'EXPLORE', 'DETECT', 'APPROACH', 'SETTLE', 'CAPTURE', 'ASSESS', 'COMMIT')
RETURN_BASE, HOLD, AWAIT_OPERATOR, DONE, ABORTED = 'RETURN_BASE', 'HOLD', 'AWAIT_OPERATOR', 'DONE', 'ABORTED'
ACTIVE_STATES = (PATROL, EXPLORE, DETECT, APPROACH, SETTLE, CAPTURE, ASSESS, COMMIT, RETURN_BASE)
TERMINAL = (DONE, ABORTED)
RESUME_TARGET = {APPROACH: APPROACH, SETTLE: APPROACH, CAPTURE: APPROACH, EXPLORE: PATROL, DETECT: DETECT,
                 PATROL: PATROL, ASSESS: ASSESS, COMMIT: COMMIT, RETURN_BASE: RETURN_BASE}
NAV_SUCCESS = 'REACHED'
NAV_FAILURES = ('REJECTED', 'CANCELLED', 'TIMEOUT', 'FAILED')

MODE_PATROL, MODE_EXPLORE, MODE_SURVEY_ROUTE = 'PATROL', 'EXPLORE', 'SURVEY_ROUTE'


@dataclass
class MissionConfig:
    mode: str = MODE_EXPLORE
    max_duration_s: float = 900.0
    detect_dwell_s: float = 2.0             # PROPOSED: let detector/tracker update after a move
    settle_dwell_s: float = 0.5             # PROPOSED: in addition to NavigateToView settled=true
    views_per_ladder: int = 2               # PROPOSED
    hold_timeout_s: float = 120.0           # PROPOSED: HOLD longer than this => AWAIT_OPERATOR
    loc_loss_timeout_s: float = 30.0        # PROPOSED: localization lost longer => AWAIT_OPERATOR
    operator_enable_timeout_s: float = 1.0  # PROPOSED heartbeat freshness
    localization_timeout_s: float = 1.0     # PROPOSED
    nav_xy_tolerance_m: float = 0.25
    nav_yaw_tolerance_rad: float = 0.2
    nav_timeout_s: float = 90.0
    nav_result_grace_s: float = 10.0        # own watchdog beyond the goal's timeout_s
    capture_timeout_s: float = 20.0
    clearance_m: float = 0.45               # robot radius incl. margin, PROPOSED/TO_VERIFY
    require_operator_enable: bool = True
    max_explore_failures: int = 5
    max_frontier_visits: int = 2            # PROPOSED: frontier reached this often but unresolved => blacklisted
    modalities: tuple = ('color', 'depth')
    view: ViewPlannerConfig = field(default_factory=ViewPlannerConfig)


@dataclass
class Inputs:
    """Latest world snapshot provided by the node (all poses in the map frame)."""
    grid: object = None
    robot: tuple = None                      # (x, y, yaw)
    candidates: list = field(default_factory=list)  # dicts: candidate_id, asset_id, position, yaw,
    #                                                 ladder_type, state, from_oracle
    loc_state: str = 'UNKNOWN'
    loc_map: tuple = None                    # (map_id, map_version)
    loc_stamp: float = None
    operator_enabled: bool = False
    operator_stamp: float = None
    external_hold: bool = False


class MissionCore:
    def __init__(self, cfg, zone, *, map_id, map_version, route=(), base_pose=None, memory=None,
                 mission_id=''):
        self.cfg = cfg
        self.zone = zone
        self.map_id, self.map_version = map_id, int(map_version)
        self.route = list(route)
        self.route_index = 0
        self.route_skipped = []
        self.base_pose = base_pose or zone.base_pose
        self.memory = memory
        self.mission_id = mission_id
        self.state = IDLE
        self.prev_state = IDLE
        self.reason = ''
        self.action = ''
        self.inputs = Inputs()
        self.commands = []
        self.events = []
        self.goal_seq = 0
        self.capture_seq = 0
        self.active_goal = None              # dict(goal_id, kind, sent_at, timeout_s, target, asset_id)
        self.active_capture = None
        self.state_since = 0.0
        self.started_at = None
        self.hold_since = None
        self.hold_causes = ()
        self.manual_hold = False
        self.resume_state = None
        self.asserting_hold = False
        self.inspected = {}                  # asset/candidate key -> capture payload
        self.unreachable = {}                # key -> reason
        self.capture_failed = {}
        self.target = None                   # dict(key, cand, views, view_index, captured, refs, settled)
        self.frontier_blacklist = []
        self.frontier_visits = []            # frontier centroids reached by their associated goals
        self.explore_failures = 0
        self.survey_exhausted = False
        self.last_unreachable_frontiers = []
        self.result = None                   # (status, reason) once terminal
        self.now = 0.0

    # ------------------------------------------------------------------ helpers
    def _emit(self, kind, **data):
        self.commands.append((kind, data))

    def _event(self, event_type, reason='', **payload):
        ev = {'event_type': event_type, 'from_state': self.prev_state, 'to_state': self.state,
              'reason': reason, 'asset_id': payload.pop('asset_id', ''), 'goal_id': payload.pop('goal_id', ''),
              'payload': payload, 'time': self.now}
        self.events.append(ev)
        if self.memory is not None:
            self.memory.event(event_type, asset_id=ev['asset_id'] or None, from_state=ev['from_state'],
                              to_state=ev['to_state'], reason=reason, goal_id=ev['goal_id'], **payload)

    def _goto(self, state, reason=''):
        if state != self.state:
            self.prev_state, self.state = self.state, state
            self.state_since = self.now
            self.reason = reason
            self._event('STATE_CHANGED', reason)

    def drain(self):
        out, self.commands = self.commands, []
        return out

    def drain_events(self):
        out, self.events = self.events, []
        return out

    @staticmethod
    def key(cand):
        return cand.get('asset_id') or cand['candidate_id']

    # ------------------------------------------------------------------ external API
    def start(self, now, inputs):
        self.now = now
        self.inputs = inputs
        self.started_at = now
        if self.base_pose is None and inputs.robot is not None and inputs.loc_state == 'VALID':
            self.base_pose = tuple(inputs.robot)
            self._event('BASE_RECORDED', 'Base pose recorded at mission start', base=list(self.base_pose))
        self._goto(PATROL, 'Mission started')
        self.tick(now, inputs)

    def request_hold(self, reason='OPERATOR_MISSION_HOLD'):
        self.manual_hold = True
        self._event('HOLD_REQUESTED', reason)

    def resume(self):
        """Operator resume. Clears the mission-service hold and leaves AWAIT_OPERATOR if causes are clear."""
        self.manual_hold = False
        causes = self._inhibit_causes()
        if causes:
            self._event('RESUME_REFUSED', 'Inhibit causes still active: ' + ','.join(causes))
            return False, 'Inhibit causes still active: ' + ','.join(causes)
        if self.state in (HOLD, AWAIT_OPERATOR):
            self._release_hold('Operator resume')
        return True, 'resumed'

    def abort(self, reason='OPERATOR_ABORT'):
        if self.state in TERMINAL:
            return
        self._cancel_active('ABORT')
        self._finish(ABORTED, 'ABORTED', reason)

    # ------------------------------------------------------------------ inhibit / hold
    def _inhibit_causes(self):
        i, c, causes = self.inputs, self.cfg, []
        if self.manual_hold:
            causes.append('MISSION_HOLD')
        if i.external_hold:
            causes.append('OPS_HOLD')
        if c.require_operator_enable and (not i.operator_enabled or i.operator_stamp is None
                                          or self.now - i.operator_stamp > c.operator_enable_timeout_s):
            causes.append('OPERATOR_ENABLE_LOST')
        if i.loc_state != 'VALID':
            causes.append('LOCALIZATION_' + (i.loc_state or 'UNKNOWN'))
        elif i.loc_stamp is None or self.now - i.loc_stamp > c.localization_timeout_s:
            causes.append('LOCALIZATION_STALE')
        elif i.loc_map is not None and (i.loc_map[0] != self.map_id or int(i.loc_map[1]) != self.map_version):
            causes.append('MAP_MISMATCH')
        if i.grid is None:
            causes.append('MAP_UNAVAILABLE')
        if i.robot is None:
            causes.append('POSE_UNAVAILABLE')
        return causes

    def _cancel_active(self, why):
        if self.active_goal is not None:
            self._emit('CANCEL_NAV', goal_id=self.active_goal['goal_id'], reason=why)
            self._event('GOAL_CANCELLED', why, goal_id=self.active_goal['goal_id'])
            self.active_goal = None
        if self.active_capture is not None:
            self._emit('CANCEL_CAPTURE', capture_id=self.active_capture['capture_id'], reason=why)
            self.active_capture = None

    def _set_hold(self, on):
        if on != self.asserting_hold:
            self.asserting_hold = on
            self._emit('HOLD', on=on)

    def _enter_hold(self, causes):
        self.resume_state = RESUME_TARGET.get(self.state, PATROL)
        self._cancel_active('HOLD')
        self._set_hold(True)
        self.hold_since = self.now
        self.hold_causes = tuple(causes)
        self._goto(HOLD, 'HOLD: ' + ','.join(causes))

    def _release_hold(self, reason):
        self._set_hold(False)
        target = self.resume_state or PATROL
        self.hold_since = None
        self.hold_causes = ()
        self._goto(target, reason)
        self._event('RESUMED', reason, resume_state=target)
        if target == APPROACH and self.target is not None:
            self._send_view_goal()

    # ------------------------------------------------------------------ main tick
    def tick(self, now, inputs):
        self.now = now
        self.inputs = inputs
        if self.state in TERMINAL or self.state == IDLE:
            return
        if self.base_pose is None and inputs.robot is not None and inputs.loc_state == 'VALID':
            self.base_pose = tuple(inputs.robot)
            self._event('BASE_RECORDED', 'Base pose recorded (first valid pose)', base=list(self.base_pose))
        causes = self._inhibit_causes()
        if self.state == AWAIT_OPERATOR:
            self._set_hold(True)
            return
        if self.state == HOLD:
            if causes:
                self.hold_causes = tuple(causes)
                loc = [c for c in causes if c.startswith('LOCALIZATION') or c in ('MAP_MISMATCH', 'MAP_UNAVAILABLE')]
                if loc and now - self.hold_since > self.cfg.loc_loss_timeout_s:
                    self._await_operator('Localization/map lost > %.0f s: %s' % (self.cfg.loc_loss_timeout_s,
                                                                                 ','.join(loc)))
                elif now - self.hold_since > self.cfg.hold_timeout_s:
                    self._await_operator('HOLD exceeded %.0f s: %s' % (self.cfg.hold_timeout_s, ','.join(causes)))
                return
            self._release_hold('Inhibit causes cleared')
            if self.state in TERMINAL or self.state in (HOLD, AWAIT_OPERATOR):
                return
        elif causes:
            self._enter_hold(causes)
            return
        if self.started_at is not None and now - self.started_at > self.cfg.max_duration_s \
                and self.state not in (RETURN_BASE,):
            self._cancel_active('TIME_BUDGET')
            self._return_base('Mission time budget exceeded', gaps=True)
            return
        self._watchdogs()
        handler = getattr(self, '_tick_' + self.state.lower(), None)
        if handler is not None:
            handler()

    def _await_operator(self, reason):
        self._cancel_active('AWAIT_OPERATOR')
        self._set_hold(True)
        if self.state != HOLD:
            self.resume_state = RESUME_TARGET.get(self.state, PATROL)
        self._goto(AWAIT_OPERATOR, reason)
        self.action = 'AWAIT_OPERATOR'

    def _watchdogs(self):
        g = self.active_goal
        if g is not None and self.now - g['sent_at'] > g['timeout_s'] + self.cfg.nav_result_grace_s:
            gid = g['goal_id']
            self._emit('CANCEL_NAV', goal_id=gid, reason='RESULT_WATCHDOG')
            self.on_nav_result(gid, 'TIMEOUT', 'No NavigateToView result within timeout + grace')
        cap = self.active_capture
        if cap is not None and self.now - cap['sent_at'] > self.cfg.capture_timeout_s:
            self._emit('CANCEL_CAPTURE', capture_id=cap['capture_id'], reason='CAPTURE_WATCHDOG')
            self.on_capture_result(cap['capture_id'], 'FAILED', 'No CaptureViews result within timeout', 0, [])

    # ------------------------------------------------------------------ goals
    def _send_nav(self, kind, x, y, yaw, reason, asset_id=''):
        self.goal_seq += 1
        gid = f'{self.mission_id or "M"}-G{self.goal_seq:04}'
        goal = {'goal_id': gid, 'mission_id': self.mission_id, 'map_id': self.map_id,
                'map_version': self.map_version, 'x': float(x), 'y': float(y), 'yaw': float(yaw),
                'xy_tolerance_m': self.cfg.nav_xy_tolerance_m, 'yaw_tolerance_rad': self.cfg.nav_yaw_tolerance_rad,
                'timeout_s': self.cfg.nav_timeout_s, 'reason': reason, 'asset_id': asset_id}
        self.active_goal = dict(goal, kind=kind, sent_at=self.now)
        self._emit('NAV', **goal)
        self._event('GOAL_SENT', reason, goal_id=gid, asset_id=asset_id, kind=kind, target=[x, y, yaw])
        return gid

    def on_nav_result(self, goal_id, status, reason='', settled=False):
        g = self.active_goal
        if g is None or g['goal_id'] != goal_id:
            self._event('LATE_RESULT_IGNORED', f'{status}: {reason}', goal_id=goal_id)
            return
        self.active_goal = None
        self._event('GOAL_RESULT', f'{status}: {reason}', goal_id=goal_id, status=status, settled=bool(settled),
                    asset_id=g.get('asset_id', ''))
        ok = status == NAV_SUCCESS
        kind = g['kind']
        if kind == 'EXPLORE':
            if ok:
                self.explore_failures = 0
                if g.get('route'):
                    self.route_index += 1
                else:
                    centroid = g['frontier_centroid']
                    self.frontier_visits.append(centroid)
                    if sum(math.hypot(centroid[0] - vx, centroid[1] - vy) < 0.5
                           for vx, vy in self.frontier_visits) >= self.cfg.max_frontier_visits:
                        self.frontier_blacklist.append(centroid)
                        self._event('FRONTIER_UNRESOLVED', 'Frontier reached repeatedly without being observed',
                                    target=[g['x'], g['y']])
                self._goto(DETECT, 'Survey goal reached')
            else:
                self.explore_failures += 1
                if g.get('route'):
                    self.route_skipped.append(self.route_index)
                    self.route_index += 1
                else:
                    self.frontier_blacklist.append(g['frontier_centroid'])
                if self.explore_failures >= self.cfg.max_explore_failures:
                    self.survey_exhausted = True
                    self._event('SURVEY_EXHAUSTED', 'Too many consecutive survey goal failures')
                self._goto(PATROL, f'Survey goal {status}')
        elif kind == 'VIEW':
            t = self.target
            if t is None:
                self._goto(PATROL, 'No target')
            elif ok and not settled:
                self._await_operator('View reached without confirmed settling')
            elif ok:
                t['settled'] = bool(settled)
                self._goto(SETTLE, 'View pose reached (settled=%s)' % bool(settled))
            else:
                view = t['views'][t['view_index']]
                t['failed'].append((view.x, view.y))
                t['view_index'] += 1
                self._goto(APPROACH, f'View pose {status}: {reason}')
                self._send_view_goal()
        elif kind == 'BASE':
            if ok:
                self._finish(DONE, 'DONE_WITH_GAPS' if self._gaps() else 'DONE', self.reason)
            else:
                self._await_operator(f'Return to base failed ({status}: {reason}); preserve evidence and stop')

    # ------------------------------------------------------------------ PATROL hub (supervisor.decide port)
    def _pending(self):
        out = []
        for c in self.inputs.candidates:
            k = self.key(c)
            if k in self.inspected or k in self.unreachable or k in self.capture_failed:
                continue
            if c.get('state') == 'MISSING':
                continue
            out.append(c)
        return out

    def _reach(self):
        return self.inputs.grid.reachable_from(self.inputs.robot[:2], self.cfg.clearance_m)

    def _survey_goal(self, reach):
        """Next survey goal, with its frontier centroid (None for route waypoints), or None."""
        if self.survey_exhausted:
            return None
        use_route = self.cfg.mode == MODE_SURVEY_ROUTE or (self.cfg.mode == MODE_PATROL and self.route)
        if not use_route:
            reachable, unreachable = find_frontiers(self.inputs.grid, self.zone, self.inputs.robot[:2],
                                                    clearance_m=self.cfg.clearance_m,
                                                    blacklist=self.frontier_blacklist, reach=reach)
            self.last_unreachable_frontiers = unreachable
            if reachable:
                f = reachable[0]
                return f.goal[0], f.goal[1], f.goal[2], False, f.centroid
            if self.cfg.mode == MODE_PATROL or not self.route:
                return None
        while self.route_index < len(self.route):
            wp = self.route[self.route_index]
            if self.zone.contains(*wp):
                nxt = self.route[self.route_index + 1] if self.route_index + 1 < len(self.route) else None
                yaw = math.atan2(nxt[1] - wp[1], nxt[0] - wp[0]) if nxt is not None else self.inputs.robot[2]
                return float(wp[0]), float(wp[1]), yaw, True, None
            self.route_skipped.append(self.route_index)
            self._event('ROUTE_WAYPOINT_SKIPPED', 'Waypoint outside patrol zone', index=self.route_index)
            self.route_index += 1
        return None

    def decide(self):
        grid, robot = self.inputs.grid, self.inputs.robot
        reach = self._reach()
        survey = self._survey_goal(reach)
        options = []
        for c in self._pending():
            plan = plan_views(self.key(c), c['position'][:2], c.get('ladder_type', 'UNKNOWN'), grid, self.zone,
                              robot[:2], yaw=c.get('yaw'), reach=reach, cfg=self.cfg.view)
            if plan.feasible:
                v = plan.feasible[0]
                options.append((math.hypot(v.x - robot[0], v.y - robot[1]), c, plan))
            elif survey is None:
                reasons = sorted({r for _, r in plan.rejected})
                self._mark_unreachable(c, 'No feasible view pose: ' + ','.join(reasons))
        if options:
            _, c, plan = min(options, key=lambda o: o[0])
            return {'action': 'INSPECT', 'cand': c, 'plan': plan,
                    'reason': 'Uninspected known ladder with reachable observation pose'}
        if survey is not None:
            return {'action': 'SURVEY', 'goal': survey,
                    'reason': 'Continue bounded search; unobserved space may contain more ladders'}
        bx, by = self.base_pose[:2]
        if not grid.is_reachable(reach, (bx, by), snap_m=self.cfg.view.robot_radius_m + 0.3):
            return {'action': 'AWAIT_OPERATOR', 'reason': 'Base is unreachable; preserve evidence and stop'}
        gaps = self._gaps()
        return {'action': 'RETURN_BASE_WITH_GAPS' if gaps else 'RETURN_BASE',
                'reason': ('Unreachable/pending ladders, capture failures or incomplete search coverage'
                           if gaps else 'All known ladders captured and search coverage complete')}

    def _gaps(self):
        return bool(self._pending() or self.unreachable or self.capture_failed or self.route_skipped
                    or self.last_unreachable_frontiers or self.survey_exhausted
                    or not self.coverage_complete())

    def coverage_complete(self):
        """Hook replaced by the node with the coverage reporter result (default: unverified)."""
        return False

    def _mark_unreachable(self, cand, reason):
        k = self.key(cand)
        if k not in self.unreachable:
            self.unreachable[k] = reason
            self._event('LADDER_UNREACHABLE', reason, asset_id=k, inspected=False)

    def _tick_patrol(self):
        d = self.decide()
        self.action = d['action']
        if d['action'] == 'INSPECT':
            c, plan = d['cand'], d['plan']
            self.target = {'key': self.key(c), 'cand': dict(c), 'views': plan.feasible, 'view_index': 0,
                           'captured': 0, 'refs': [], 'settled': False, 'failed': [], 'capture_status': [],
                           'replans': 0}
            self._goto(APPROACH, d['reason'])
            self._send_view_goal()
        elif d['action'] == 'SURVEY':
            x, y, yaw, is_route, centroid = d['goal']
            self._goto(EXPLORE, d['reason'])
            self._send_nav('EXPLORE', x, y, yaw, 'SURVEY_ROUTE' if is_route else 'FRONTIER')
            self.active_goal['route'] = is_route
            self.active_goal['frontier_centroid'] = centroid
        elif d['action'] == 'AWAIT_OPERATOR':
            self._await_operator(d['reason'])
        else:
            self._return_base(d['reason'], gaps=d['action'] == 'RETURN_BASE_WITH_GAPS')

    def _return_base(self, reason, gaps):
        self.action = 'RETURN_BASE_WITH_GAPS' if gaps else 'RETURN_BASE'
        self._goto(RETURN_BASE, reason)
        bx, by = self.base_pose[:2]
        byaw = self.base_pose[2] if len(self.base_pose) > 2 else 0.0
        self._send_nav('BASE', bx, by, byaw, self.action)

    def _tick_return_base(self):
        if self.active_goal is None:   # resumed after HOLD
            self._send_nav('BASE', self.base_pose[0], self.base_pose[1],
                           self.base_pose[2] if len(self.base_pose) > 2 else 0.0, self.action or 'RETURN_BASE')

    def _tick_explore(self):
        if self.active_goal is None:
            self._goto(PATROL, 'No active survey goal')
            return
        # Pre-empt the survey goal when a new pending ladder appears (re-plan at DETECT).
        if self._pending() and not self.active_goal.get('preempt_checked'):
            self.active_goal['preempt_checked'] = True
            gid = self.active_goal['goal_id']
            self._emit('CANCEL_NAV', goal_id=gid, reason='LADDER_DETECTED')
            self._event('GOAL_CANCELLED', 'Pending ladder detected during survey', goal_id=gid)
            self.active_goal = None
            self._goto(DETECT, 'Pending ladder detected during survey')

    def _tick_detect(self):
        if self.now - self.state_since >= self.cfg.detect_dwell_s:
            self._goto(PATROL, 'Detection dwell complete')

    # ------------------------------------------------------------------ approach / capture
    def _current_candidate(self):
        t = self.target
        for c in self.inputs.candidates:
            if self.key(c) == t['key'] or c['candidate_id'] == t['cand']['candidate_id']:
                return c
        return t['cand']

    def _send_view_goal(self):
        t = self.target
        if t['view_index'] >= len(t['views']):
            c = self._current_candidate()
            if t['replans'] < 1 and self.inputs.grid is not None and self.inputs.robot is not None:
                t['replans'] += 1
                plan = plan_views(t['key'], c['position'][:2], c.get('ladder_type', 'UNKNOWN'), self.inputs.grid,
                                  self.zone, self.inputs.robot[:2], yaw=c.get('yaw'), cfg=self.cfg.view,
                                  blacklist=t['failed'])
                if plan.feasible:
                    t['views'], t['view_index'] = plan.feasible, 0
                    return self._send_view_goal()
            if t['captured'] > 0:
                self._goto(ASSESS, 'No further view poses; assess captured views')
                return
            self._mark_unreachable(c, 'All view poses failed or infeasible (%d tried)' % len(t['failed']))
            self.target = None
            self._goto(PATROL, 'Ladder unreachable; not inspected')
            return
        v = t['views'][t['view_index']]
        self._send_nav('VIEW', v.x, v.y, v.yaw, 'INSPECT_VIEW', asset_id=t['cand'].get('asset_id', ''))

    def _tick_approach(self):
        if self.active_goal is None and self.target is not None:
            self._send_view_goal()
        elif self.target is None:
            self._goto(PATROL, 'No target')

    def _tick_settle(self):
        if self.now - self.state_since >= self.cfg.settle_dwell_s:
            t = self.target
            v = t['views'][t['view_index']]
            self.capture_seq += 1
            cap_id = f'{self.mission_id or "M"}-C{self.capture_seq:04}'
            self.active_capture = {'capture_id': cap_id, 'sent_at': self.now}
            self._goto(CAPTURE, 'Settled; capturing views')
            self._emit('CAPTURE', capture_id=cap_id, mission_id=self.mission_id, asset_id=t['cand'].get('asset_id', ''),
                       map_id=self.map_id, map_version=self.map_version, view=(v.x, v.y, v.yaw),
                       modalities=list(self.cfg.modalities), settled=t['settled'])

    def on_capture_result(self, capture_id, status, reason, views_captured, evidence_refs):
        cap = self.active_capture
        if cap is None or cap['capture_id'] != capture_id:
            self._event('LATE_RESULT_IGNORED', f'capture {status}: {reason}', goal_id=capture_id)
            return
        self.active_capture = None
        t = self.target
        self._event('CAPTURE_RESULT', f'{status}: {reason}', goal_id=capture_id, asset_id=t['key'], status=status,
                    views_captured=int(views_captured), evidence_refs=list(evidence_refs))
        t['capture_status'].append(status)
        view = t['views'][t['view_index']]
        t['view_index'] += 1
        if status in ('COMPLETE', 'PARTIAL'):
            t['captured'] += 1
            t['refs'].extend(evidence_refs)
            # Prefer a different side of the ladder for the next view (same angle, other standoff last).
            rest = t['views'][t['view_index']:]
            other = [v for v in rest if v.angle_deg != view.angle_deg]
            t['views'] = t['views'][:t['view_index']] + other + [v for v in rest if v.angle_deg == view.angle_deg]
        if t['captured'] >= self.cfg.views_per_ladder:
            self._goto(ASSESS, 'Requested views captured')
        else:
            self._goto(APPROACH, 'Next view pose')
            if self.state == APPROACH:
                self._send_view_goal()

    def _tick_capture(self):
        if self.active_capture is None:
            self._goto(APPROACH, 'Capture not active; retry view')

    def _tick_assess(self):
        t = self.target
        if t['captured'] > 0:
            self._event('ASSESS_REQUESTED', 'Asynchronous assessment hand-off (no verdict produced here)',
                        asset_id=t['key'], evidence_refs=list(t['refs']), human_review_required=True,
                        from_oracle=bool(t['cand'].get('from_oracle')))
        self._goto(COMMIT, 'Assessment requested asynchronously')

    def _tick_commit(self):
        t = self.target
        c = self._current_candidate()
        status = 'COMPLETE' if t['captured'] >= self.cfg.views_per_ladder else ('PARTIAL' if t['captured'] else 'FAILED')
        payload = {'status': 'EVIDENCE_CAPTURED' if t['captured'] else 'CAPTURE_FAILED', 'capture_status': status,
                   'evidence_refs': list(t['refs']), 'verdict': 'PENDING_ASSESSMENT', 'human_review_required': True}
        if self.memory is not None:
            payload = self.memory.commit_capture(
                c.get('asset_id', ''), ladder_type=c.get('ladder_type', 'UNKNOWN'), position=c['position'],
                capture_status=status, evidence_refs=t['refs'],
                reason=f'{t["captured"]} view(s) captured; settled={t["settled"]}',
                from_oracle=bool(c.get('from_oracle')))
        if t['captured']:
            self.inspected[t['key']] = payload
        else:
            self.capture_failed[t['key']] = payload
        self.target = None
        self._goto(PATROL, 'Committed evidence (verdict pending)')

    # ------------------------------------------------------------------ end
    def _finish(self, state, status, reason):
        self._cancel_active('FINISH')
        self._set_hold(False)
        self.result = (status, reason)
        self._goto(state, reason)
        self._event('MISSION_END', reason, status=status)

    def summary(self):
        cands = {self.key(c): c for c in self.inputs.candidates}
        return {
            'mission_id': self.mission_id, 'state': self.state, 'result': self.result,
            'ladders_known': len(cands), 'ladders_captured': sorted(self.inspected),
            'ladders_unreachable': dict(self.unreachable), 'ladders_capture_failed': sorted(self.capture_failed),
            'ladders_pending': sorted(self.key(c) for c in self._pending()),
            'ladders_missing': sorted(k for k, c in cands.items() if c.get('state') == 'MISSING'),
            'route_waypoints_skipped': list(self.route_skipped),
            'unreachable_frontiers': [f.frontier_id for f in self.last_unreachable_frontiers],
            'oracle_detections_used': any(c.get('from_oracle') for c in cands.values()),
            'verdicts': 'NONE_PRODUCED_BY_MISSION_PENDING_ASSESSMENT_AND_HUMAN_REVIEW',
        }
