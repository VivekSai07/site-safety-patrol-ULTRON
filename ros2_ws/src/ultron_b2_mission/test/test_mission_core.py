"""Mission state machine: HOLD/inhibit/localization loss from every state, resume, unreachable ladders."""
import math

import pytest

from mission_fixtures import cand, good_inputs, kinds, refresh, room, zone_for
from ultron_b2_mission import mission_core as mc
from ultron_b2_mission.mission_core import MissionConfig, MissionCore


class Harness:
    """Drives MissionCore with a fake NavigateToView server (teleports on success) and fake capture."""

    def __init__(self, grid, candidates=(), mode=mc.MODE_EXPLORE, route=(), nav_policy=None, cfg=None,
                 robot=(1.0, 1.0, 0.0)):
        self.cfg = cfg or MissionConfig(mode=mode)
        self.core = MissionCore(self.cfg, zone_for(), map_id='hall', map_version=1, route=route, mission_id='T')
        self.inp = good_inputs(grid, robot, candidates)
        self.now = 0.0
        self.nav_policy = nav_policy or (lambda goal: 'REACHED')
        self.sent, self.cancelled, self.holds, self.captures = [], [], [], []
        self.pending_nav, self.pending_cap = None, None
        self.auto = True

    def process(self, cmds):
        for kind, d in cmds:
            if kind == 'NAV':
                self.sent.append(d)
                self.pending_nav = d
            elif kind == 'CANCEL_NAV':
                self.cancelled.append(d['goal_id'])
                if self.pending_nav and self.pending_nav['goal_id'] == d['goal_id']:
                    self.pending_nav = None
            elif kind == 'HOLD':
                self.holds.append(d['on'])
            elif kind == 'CAPTURE':
                self.captures.append(d)
                self.pending_cap = d
            elif kind == 'CANCEL_CAPTURE':
                self.pending_cap = None

    def start(self):
        self.core.start(self.now, self.inp)
        self.process(self.core.drain())

    def step(self, dt=0.1):
        self.now += dt
        refresh(self.inp, self.now)
        self.core.tick(self.now, self.inp)
        self.process(self.core.drain())
        if self.auto and self.pending_nav is not None:
            g, self.pending_nav = self.pending_nav, None
            status = self.nav_policy(g)
            if status == 'REACHED':
                self.inp.robot = (g['x'], g['y'], g['yaw'])
            self.core.on_nav_result(g['goal_id'], status, 'fake', settled=status == 'REACHED')
            self.process(self.core.drain())
        if self.auto and self.pending_cap is not None:
            c, self.pending_cap = self.pending_cap, None
            self.core.on_capture_result(c['capture_id'], 'COMPLETE', 'fake', 2, [f'sha256:{c["capture_id"]}'])
            self.process(self.core.drain())

    def run_until(self, pred, max_steps=3000):
        for _ in range(max_steps):
            if pred():
                return True
            self.step()
        return pred()


def reach_state(h, state, max_steps=400):
    """Advance (resolving nav/capture as success) until the core is in ``state``; leave it pending there."""
    for _ in range(max_steps):
        if h.core.state == state:
            return True
        if h.pending_nav is not None:
            g, h.pending_nav = h.pending_nav, None
            h.inp.robot = (g['x'], g['y'], g['yaw'])
            h.core.on_nav_result(g['goal_id'], 'REACHED', 'fake', settled=True)
            h.process(h.core.drain())
        elif h.pending_cap is not None:
            c, h.pending_cap = h.pending_cap, None
            h.core.on_capture_result(c['capture_id'], 'COMPLETE', 'fake', 2, ['sha256:x'])
            h.process(h.core.drain())
        else:
            h.now += 0.1
            refresh(h.inp, h.now)
            h.core.tick(h.now, h.inp)
            h.process(h.core.drain())
    return h.core.state == state


def test_full_mission_captures_ladders_and_returns_to_base():
    grid = room()
    h = Harness(grid, [cand('L1', 5.0, 4.0), cand('L2', 8.0, 6.0, 'LEANING_COMBINATION_LADDER', yaw=math.pi / 2)])
    h.start()
    assert h.run_until(lambda: h.core.state in mc.TERMINAL)
    assert h.core.state == mc.DONE
    assert h.core.result[0] in ('DONE', 'DONE_WITH_GAPS')
    assert sorted(h.core.inspected) == ['L1', 'L2']
    assert len(h.captures) == 4  # views_per_ladder=2
    assert h.sent[-1]['reason'] in ('RETURN_BASE', 'RETURN_BASE_WITH_GAPS')
    assert (h.sent[-1]['x'], h.sent[-1]['y']) == (1.0, 1.0)
    for g in h.sent:
        assert g['map_id'] == 'hall' and g['map_version'] == 1
    # Captured evidence never carries a verdict.
    for payload in h.core.inspected.values():
        assert payload['verdict'] == 'PENDING_ASSESSMENT' and payload['human_review_required'] is True
    states = [e['to_state'] for e in h.core.drain_events() if e['event_type'] == 'STATE_CHANGED']
    for s in (mc.APPROACH, mc.SETTLE, mc.CAPTURE, mc.ASSESS, mc.COMMIT, mc.RETURN_BASE):
        assert s in states


def test_unsettled_view_does_not_start_capture():
    h = _harness_in(mc.APPROACH)
    goal = h.core.active_goal
    h.core.on_nav_result(goal['goal_id'], 'REACHED', 'not settled', settled=False)
    h.process(h.core.drain())
    assert h.core.state == mc.APPROACH
    assert h.core.active_goal['goal_id'] != goal['goal_id']
    assert len(h.core.target['failed']) == 1
    assert not h.captures


@pytest.mark.parametrize('status', ['FAILED', 'REACHED'])
def test_frontier_blacklisting_uses_centroid_not_distant_goal(status):
    grid = room(unknown_rect=(6.0, 0.1, 9.9, 7.9))
    cfg = MissionConfig(mode=mc.MODE_EXPLORE, clearance_m=1.2, max_frontier_visits=1)
    h = Harness(grid, cfg=cfg, robot=(2.0, 2.0, 0.0))
    h.auto = False
    h.start()
    h.step()
    goal = h.core.active_goal
    assert goal is not None and goal['kind'] == 'EXPLORE'
    assert math.dist((goal['x'], goal['y']), goal['frontier_centroid']) > 0.75
    h.core.on_nav_result(goal['goal_id'], status, 'test', settled=status == 'REACHED')
    reachable, unreachable = mc.find_frontiers(grid, h.core.zone, h.inp.robot[:2],
                                                clearance_m=cfg.clearance_m,
                                                blacklist=h.core.frontier_blacklist)
    assert not reachable
    assert unreachable and all(f.blacklisted for f in unreachable)


INHIBITS = {
    'ops_hold': lambda inp: setattr(inp, 'external_hold', True),
    'operator_enable_false': lambda inp: setattr(inp, 'operator_enabled', False),
    'localization_lost': lambda inp: setattr(inp, 'loc_state', 'LOST'),
    'map_mismatch': lambda inp: setattr(inp, 'loc_map', ('hall', 2)),
}
CLEAR = {
    'ops_hold': lambda inp: setattr(inp, 'external_hold', False),
    'operator_enable_false': lambda inp: setattr(inp, 'operator_enabled', True),
    'localization_lost': lambda inp: setattr(inp, 'loc_state', 'VALID'),
    'map_mismatch': lambda inp: setattr(inp, 'loc_map', ('hall', 1)),
}
STATES = (mc.PATROL, mc.EXPLORE, mc.DETECT, mc.APPROACH, mc.SETTLE, mc.CAPTURE, mc.RETURN_BASE)


def _harness_in(state):
    grid = room(unknown_rect=(6.0, 0.1, 9.9, 7.9))
    cands = [cand('L1', 3.0, 4.0)]
    cfg = MissionConfig(mode=mc.MODE_EXPLORE, detect_dwell_s=1.0, settle_dwell_s=1.0)
    h = Harness(grid, cands if state in (mc.APPROACH, mc.SETTLE, mc.CAPTURE) else [], cfg=cfg)
    if state in (mc.RETURN_BASE,):
        h.inp.grid = room()
    h.auto = False
    h.start()
    if state == mc.PATROL:
        h.core.drain()
        return h
    assert reach_state(h, state), f'could not reach {state}: at {h.core.state}'
    return h


@pytest.mark.parametrize('cause', sorted(INHIBITS))
@pytest.mark.parametrize('state', STATES)
def test_inhibit_in_every_state_cancels_and_sends_no_goals(state, cause):
    h = _harness_in(state)
    active = h.core.active_goal['goal_id'] if h.core.active_goal else None
    active_cap = h.core.active_capture
    INHIBITS[cause](h.inp)
    n_sent, n_caps = len(h.sent), len(h.captures)
    h.step()
    assert h.core.state == mc.HOLD, (state, cause, h.core.state)
    assert h.holds[-1] is True
    if active:
        assert active in h.cancelled
    if active_cap:
        assert h.core.active_capture is None
    for _ in range(50):           # 5 s held: nothing sent
        h.step()
    assert len(h.sent) == n_sent and len(h.captures) == n_caps
    assert h.core.state == mc.HOLD
    # Late result for the cancelled goal is ignored.
    if active:
        h.core.on_nav_result(active, 'REACHED', 'late', True)
        assert h.core.state == mc.HOLD
    CLEAR[cause](h.inp)
    h.step()
    assert h.core.state not in (mc.HOLD, mc.AWAIT_OPERATOR)
    assert h.holds[-1] is False
    ev = [e for e in h.core.drain_events() if e['event_type'] == 'RESUMED']
    assert ev and ev[-1]['reason'] == 'Inhibit causes cleared'


def test_stale_operator_heartbeat_and_stale_localization_inhibit():
    h = _harness_in(mc.PATROL)
    h.now += 2.0
    h.inp.operator_stamp = h.now - 1.5
    h.inp.loc_stamp = h.now
    h.core.tick(h.now, h.inp)
    assert h.core.state == mc.HOLD and 'OPERATOR_ENABLE_LOST' in h.core.hold_causes
    h2 = _harness_in(mc.PATROL)
    h2.now += 2.0
    h2.inp.operator_stamp = h2.now
    h2.inp.loc_stamp = h2.now - 2.0
    h2.core.tick(h2.now, h2.inp)
    assert h2.core.state == mc.HOLD and 'LOCALIZATION_STALE' in h2.core.hold_causes


@pytest.mark.parametrize('state', STATES)
def test_localization_loss_timeout_ends_in_await_operator(state):
    h = _harness_in(state)
    h.inp.loc_state = 'LOST'
    n_sent = len(h.sent)
    for _ in range(int(h.cfg.loc_loss_timeout_s / 0.1) + 20):
        h.step()
    assert h.core.state == mc.AWAIT_OPERATOR
    assert 'Localization' in h.core.reason
    assert len(h.sent) == n_sent and h.holds[-1] is True
    # Localization back: still awaiting the operator (no automatic resume).
    h.inp.loc_state = 'VALID'
    for _ in range(10):
        h.step()
    assert h.core.state == mc.AWAIT_OPERATOR and len(h.sent) == n_sent
    ok, _ = h.core.resume()
    assert ok and h.core.state not in (mc.AWAIT_OPERATOR, mc.HOLD)


def test_mission_hold_service_requires_explicit_resume_and_abort_ends_with_reason():
    h = _harness_in(mc.APPROACH)
    gid = h.core.active_goal['goal_id']
    h.core.request_hold()
    h.step()
    assert h.core.state == mc.HOLD and gid in h.cancelled
    for _ in range(20):
        h.step()
    assert h.core.state == mc.HOLD
    ok, _ = h.core.resume()
    h.process(h.core.drain())
    assert ok and h.core.state == mc.APPROACH
    assert h.sent[-1]['goal_id'] != gid
    h.core.abort('test abort')
    h.process(h.core.drain())
    assert h.core.state == mc.ABORTED and h.core.result == ('ABORTED', 'test abort')
    assert h.holds[-1] is False


def test_resume_refused_while_cause_active():
    h = _harness_in(mc.EXPLORE)
    h.inp.external_hold = True
    h.step()
    ok, why = h.core.resume()
    assert not ok and 'OPS_HOLD' in why and h.core.state == mc.HOLD


def test_hold_timeout_goes_to_await_operator():
    cfg_h = _harness_in(mc.DETECT)
    cfg_h.cfg.hold_timeout_s = 3.0
    cfg_h.inp.external_hold = True
    for _ in range(40):
        cfg_h.step()
    assert cfg_h.core.state == mc.AWAIT_OPERATOR and 'HOLD exceeded' in cfg_h.core.reason


def test_unreachable_view_poses_mark_ladder_unreachable_not_inspected():
    grid = room()
    h = Harness(grid, [cand('L1', 5.0, 4.0)], nav_policy=lambda g: 'FAILED' if g['reason'] == 'INSPECT_VIEW' else 'REACHED')
    h.start()
    assert h.run_until(lambda: h.core.state in mc.TERMINAL)
    assert 'L1' in h.core.unreachable and 'L1' not in h.core.inspected
    assert h.core.result[0] == 'DONE_WITH_GAPS'
    assert not h.captures
    tried = [g for g in h.sent if g['reason'] == 'INSPECT_VIEW']
    assert len(tried) >= 4


def test_ladder_enclosed_by_walls_has_no_feasible_view_and_is_unreachable():
    grid = room(walls=[(3.5, 2.5, 6.5, 2.7), (3.5, 5.3, 6.5, 5.5), (3.5, 2.5, 3.7, 5.5), (6.3, 2.5, 6.5, 5.5)])
    h = Harness(grid, [cand('L1', 5.0, 4.0)])
    h.start()
    assert h.run_until(lambda: h.core.state in mc.TERMINAL)
    assert 'No feasible view pose' in h.core.unreachable['L1']
    assert not [g for g in h.sent if g['reason'] == 'INSPECT_VIEW']


def test_return_base_failure_awaits_operator():
    h = Harness(room(), [], nav_policy=lambda g: 'FAILED')
    h.start()
    assert h.run_until(lambda: h.core.state in (mc.AWAIT_OPERATOR,) + mc.TERMINAL)
    assert h.core.state == mc.AWAIT_OPERATOR and 'Return to base failed' in h.core.reason
    assert h.holds[-1] is True


def test_time_budget_returns_with_gaps():
    cfg = MissionConfig(mode=mc.MODE_EXPLORE, max_duration_s=1.0)
    grid = room(unknown_rect=(6.0, 0.1, 9.9, 7.9))
    h = Harness(grid, [], cfg=cfg)
    h.auto = False
    h.start()
    for _ in range(15):
        h.step()
    assert h.core.state == mc.RETURN_BASE and h.core.action == 'RETURN_BASE_WITH_GAPS'


def test_late_and_foreign_results_are_ignored():
    h = _harness_in(mc.EXPLORE)
    h.core.on_nav_result('someone-else', 'REACHED', 'x', True)
    assert h.core.state == mc.EXPLORE
    assert any(e['event_type'] == 'LATE_RESULT_IGNORED' for e in h.core.drain_events())


def test_survey_route_mode_follows_route_and_skips_out_of_zone_waypoints():
    import numpy as np
    route = [np.array((2.0, 2.0)), np.array((20.0, 2.0)), np.array((8.0, 6.0))]
    h = Harness(room(), [], mode=mc.MODE_SURVEY_ROUTE, route=route)
    h.start()
    assert h.run_until(lambda: h.core.state in mc.TERMINAL)
    targets = [(g['x'], g['y']) for g in h.sent if g['reason'] == 'SURVEY_ROUTE']
    assert targets == [(2.0, 2.0), (8.0, 6.0)]
    assert h.core.route_skipped == [1]
    assert h.core.result[0] == 'DONE_WITH_GAPS'


def test_missing_candidates_are_not_approached():
    h = Harness(room(), [cand('L1', 5.0, 4.0, state='MISSING')])
    h.start()
    assert h.run_until(lambda: h.core.state in mc.TERMINAL)
    assert not [g for g in h.sent if g['reason'] == 'INSPECT_VIEW']
    assert h.core.summary()['ladders_missing'] == ['L1']


def test_decide_matches_supervisor_action_order():
    """PATROL hub ordering equals src/ultron/supervisor.decide: INSPECT > SURVEY > AWAIT_OPERATOR > RETURN_BASE*."""
    grid = room(unknown_rect=(6.0, 0.1, 9.9, 7.9))
    h = Harness(grid, [cand('L1', 3.0, 4.0)])
    h.core.started_at = 0.0
    h.core.base_pose = (1.0, 1.0, 0.0)
    h.core.inputs = h.inp
    assert h.core.decide()['action'] == 'INSPECT'
    h.inp.candidates = []
    assert h.core.decide()['action'] == 'SURVEY'
    h.inp.grid = room()
    assert h.core.decide()['action'] == 'RETURN_BASE_WITH_GAPS'  # coverage unverified by default
    h.core.coverage_complete = lambda: True
    assert h.core.decide()['action'] == 'RETURN_BASE'
    h.inp.grid = room(walls=[(2.0, 0.0, 2.2, 8.0)])
    h.inp.robot = (5.0, 4.0, 0.0)
    assert h.core.decide()['action'] == 'AWAIT_OPERATOR'
    assert kinds(h.core.drain()) == []
