"""Nav phase N0: goal contract, pre-checks, tracker, motion gate, supervisor mapping (no ROS)."""
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

try:
    import numpy as np
    import yaml
    from PIL import Image
except ImportError:  # pragma: no cover
    np = None

if np is not None:
    from ultron.autonomy.config import load_sim_slam_config
    from ultron.autonomy.localization_state import LocalizationState
    from ultron.autonomy.live_grid import LiveMapMatcher
    from ultron.autonomy.nav2_adapter import Nav2Adapter
    from ultron.autonomy.navigation_contract import (NAV2_STATUS, GoalTracker, MapFootprint, MotionGate,
                                                     check_goal, decision_to_goal, make_goal)

MAP = ('map-A', 1)


def valid_state(now, state='VALID', map_id='map-A', version=1):
    return LocalizationState(stamp=now - 0.1, evaluated_at=now, map_id=map_id, map_version=version, state=state,
                             reason='test', age_limit_s=0.5)


@unittest.skipIf(np is None, 'NumPy/PyYAML/Pillow not installed')
class NavigationContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = load_sim_slam_config().navigation
        cls.tmp = tempfile.TemporaryDirectory()
        # 6 x 4 m map at 0.05 m: free room, a wall block, and an unknown strip.
        pixels = np.full((80, 120), 254, np.uint8)
        pixels[:, 60:64] = 0          # wall at x 3.0-3.2 m
        pixels[0:10, :] = 205         # unknown band at the top (y 3.5-4.0 m)
        Image.fromarray(pixels).save(Path(cls.tmp.name) / 'm.pgm')
        (Path(cls.tmp.name) / 'm.yaml').write_text(yaml.safe_dump({
            'image': 'm.pgm', 'mode': 'trinary', 'resolution': 0.05, 'origin': [0.0, 0.0, 0.0],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}))
        cls.footprint = MapFootprint(Path(cls.tmp.name) / 'm.yaml')

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def goal(self, x=1.5, y=1.5, now=10.0, **kw):
        return make_goal(x, y, 0.0, mission_id='M1', map_id=kw.pop('map_id', 'map-A'), map_version=kw.pop('version', 1),
                         params=self.params, reason='test', now=now, **kw)

    def live_grid(self, cells):
        orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        origin = SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0), orientation=orientation)
        info = SimpleNamespace(height=cells.shape[0], width=cells.shape[1], resolution=0.1, origin=origin)
        return SimpleNamespace(header=SimpleNamespace(frame_id='map'), info=info, data=cells.ravel())

    def test_live_target_rechecks_the_latest_occupancy(self):
        cells = np.zeros((40, 60), dtype=int)
        goal = self.goal()
        backend = SimpleNamespace(footprint=MapFootprint.from_grid(self.live_grid(cells)),
                                  config=SimpleNamespace(navigation=self.params), grid_stamp_ns=9_500_000_000,
                                  grid_history=[9_000_000_000, 9_500_000_000])
        displayed = 9_000_000_000
        self.assertTrue(Nav2Adapter._live_target(backend, goal, 10.0, displayed)[0])
        cells[15, 15] = 100
        backend.footprint = MapFootprint.from_grid(self.live_grid(cells))
        self.assertIn('OCCUPIED', Nav2Adapter._live_target(backend, goal, 10.0, displayed)[1])
        cells[15, 15] = -1
        backend.footprint = MapFootprint.from_grid(self.live_grid(cells))
        self.assertIn('UNKNOWN', Nav2Adapter._live_target(backend, goal, 10.0, displayed)[1])
        self.assertIn('REVISION_UNAVAILABLE', Nav2Adapter._live_target(backend, goal, 10.0, 11_000_000_000)[1])
        self.assertIn('REVISION_UNAVAILABLE', Nav2Adapter._live_target(backend, goal, 10.0, 9_100_000_000)[1])
        self.assertIn('REVISION_STALE', Nav2Adapter._live_target(backend, goal, 10.0, 7_000_000_000)[1])

    def test_live_scan_match_needs_support_and_counts_free_space_mismatches(self):
        cells = np.full((20, 20), -1)
        cells[5, 5] = 100
        cells[10, 10] = 0
        matcher = LiveMapMatcher(self.live_grid(cells), tolerance=0.12)
        matched = [0.55, 0.55]
        unknown = [1.55, 1.55]
        free = [1.05, 1.05]
        self.assertEqual(matcher.score((0, 0, 0), [matched, unknown, unknown]), 0.0)
        self.assertEqual(matcher.score((0, 0, 0), [matched, matched, unknown]), 1.0)
        self.assertAlmostEqual(matcher.score((0, 0, 0), [matched, matched, free]), 2.0 / 3.0)

    def test_active_live_goal_is_canceled_if_latest_footprint_becomes_occupied(self):
        cells = np.zeros((40, 60), dtype=int)
        tracker = Mock(status='ACTIVE', goal=self.goal())
        backend = SimpleNamespace(
            footprint=MapFootprint.from_grid(self.live_grid(cells)),
            config=SimpleNamespace(navigation=self.params), grid_stamp_ns=9_500_000_000, grid_history=[9_500_000_000],
            live_map=True, tracker=tracker, now=lambda: 10.0, operator_allowed=lambda _: True,
            _pose=lambda: (0.0, 0.0, 0.0), speed=0.0, yaw_rate=0.0, gate={'inhibited': False}, cancel=Mock(),
        )
        backend._live_target = lambda *args: Nav2Adapter._live_target(backend, *args)
        Nav2Adapter.tick(backend)
        backend.cancel.assert_not_called()
        cells[15, 15] = 100
        backend.footprint = MapFootprint.from_grid(self.live_grid(cells))
        Nav2Adapter.tick(backend)
        self.assertIn('TARGET_FOOTPRINT_OCCUPIED', backend.cancel.call_args.args[0])

    def check(self, goal, now=10.0, **kw):
        args = dict(map_id='map-A', map_version=1, localization=valid_state(now), now=now, operator_enabled=True,
                    footprint=self.footprint, clearance_m=self.params['target_clearance_m'])
        args.update(kw)
        return check_goal(goal, **args)

    def test_goal_record_and_validation(self):
        goal = self.goal()
        record = goal.to_dict()
        self.assertEqual(record['schema'], 'ultron.navigation_goal/v1')
        self.assertEqual((record['frame_id'], record['timeout_s']), ('map', self.params['default_timeout_s']))
        for bad in (dict(x=float('nan')), dict(timeout_s=0.0)):
            args = dict(x=1.5, timeout_s=None) | bad
            with self.assertRaises(ValueError):
                make_goal(args['x'], 1.5, 0.0, mission_id='M', map_id='a', map_version=1, params=self.params,
                          reason='r', now=0.0, timeout_s=args['timeout_s'])

    def test_prechecks(self):
        self.assertEqual(self.check(self.goal()), (True, 'OK'))
        cases = {
            'MAP_MISMATCH': self.check(self.goal(version=2)),
            'OPERATOR_MOTION_NOT_ENABLED': self.check(self.goal(), operator_enabled=False),
            'LOCALIZATION': self.check(self.goal(), localization=valid_state(10.0, 'LOST')),
            'OCCUPIED': self.check(self.goal(x=3.1)),
            'UNKNOWN': self.check(self.goal(y=3.3)),
            'OUTSIDE_MAP': self.check(self.goal(x=5.8)),
            'UNKNOWN_FRAME': self.check(self.goal(frame_id='odom')),
            'NO_MAP': self.check(self.goal(), footprint=None),
        }
        for expected, (ok, reason) in cases.items():
            self.assertFalse(ok, expected)
            self.assertIn(expected, reason)
        stale = self.check(self.goal(now=20.0), now=20.0, localization=valid_state(10.0))
        self.assertIn('STALE', stale[1])

    def test_nav2_status_mapping_is_explicit(self):
        self.assertEqual(NAV2_STATUS, {0: 'FAILED', 1: 'ACCEPTED', 2: 'ACTIVE', 3: 'ACTIVE', 4: 'REACHED',
                                       5: 'CANCELLED', 6: 'FAILED'})

    def test_tracker_lifecycle_and_late_results(self):
        tracker = GoalTracker(self.params)
        first = self.goal()
        tracker.submit(first, True, 'OK', 10.0)
        tracker.on_backend_status(first.goal_id, 2, 10.5)
        self.assertEqual(tracker.status, 'ACTIVE')
        self.assertTrue(tracker.request_cancel('OPERATOR', 11.0))
        self.assertFalse(tracker.request_cancel('OPERATOR', 11.1))      # one request only
        result = tracker.on_backend_status(first.goal_id, 5, 11.2)
        self.assertEqual(result.status, 'CANCELLED')
        second = self.goal(now=12.0)
        tracker.submit(second, True, 'OK', 12.0)
        self.assertIsNone(tracker.on_backend_status(first.goal_id, 4, 12.5))   # late SUCCEEDED of old goal
        self.assertEqual(tracker.status, 'ACCEPTED')
        self.assertEqual(tracker.ignored_late, 1)
        with self.assertRaises(RuntimeError):
            tracker.submit(self.goal(now=12.6), True, 'OK', 12.6)          # still one active goal
        self.assertEqual(tracker.on_backend_status(second.goal_id, 6, 13.0, 'planner failed').status, 'FAILED')

    def test_rejected_goal_never_reaches_backend(self):
        tracker = GoalTracker(self.params)
        result = tracker.submit(self.goal(), False, 'TARGET_FOOTPRINT_OCCUPIED', 10.0)
        self.assertEqual((result.status, tracker.status), ('REJECTED', 'REJECTED'))

    def test_timeout_cancels_and_reports_timeout(self):
        tracker = GoalTracker(self.params)
        goal = self.goal(timeout_s=5.0)
        tracker.submit(goal, True, 'OK', 10.0)
        self.assertIsNone(tracker.tick(14.9))
        self.assertEqual(tracker.tick(15.1), 'CANCEL')
        self.assertIsNone(tracker.tick(15.2))
        self.assertEqual(tracker.on_backend_status(goal.goal_id, 5, 15.4).status, 'TIMEOUT')

    def test_capture_ready_only_after_settle_with_valid_localization(self):
        tracker = GoalTracker(self.params)
        goal = self.goal()
        tracker.submit(goal, True, 'OK', 10.0)
        result = tracker.on_backend_status(goal.goal_id, 4, 20.0)
        self.assertEqual(result.status, 'REACHED')
        self.assertFalse(result.capture_ready)                    # arrival alone is not readiness
        settle = self.params['settle_time_s']
        self.assertFalse(tracker.settle(20.1, 0.3, 0.0, True, (1.5, 1.5, 0)))    # still moving
        self.assertFalse(tracker.settle(20.2, 0.0, 0.0, True, (1.5, 1.5, 0)))
        self.assertFalse(tracker.settle(20.2 + settle / 2, 0.0, 0.0, False, (1.5, 1.5, 0)))  # not VALID: resets
        self.assertFalse(tracker.settle(20.3 + settle / 2, 0.0, 0.0, True, (1.5, 1.5, 0)))
        self.assertTrue(tracker.settle(20.31 + settle * 1.5, 0.0, 0.0, True, (1.5, 1.5, 0)))
        self.assertTrue(result.capture_ready and result.stop_acknowledged)

    def test_motion_gate_defaults_to_inhibit(self):
        gate = MotionGate(self.params, *MAP)
        cmd = (0.3, 0.1)
        self.assertEqual(gate.decide(cmd, 10.0, 10.0), ((0.0, 0.0), 'OPERATOR_ENABLE_MISSING'))
        gate.inputs.operator_enabled, gate.inputs.operator_stamp = True, 10.0
        self.assertIn('LOCALIZATION', gate.decide(cmd, 10.0, 10.0)[1])            # no localization yet
        gate.inputs.localization = valid_state(10.0)
        self.assertEqual(gate.decide(cmd, 10.0, 10.0), (cmd, None))
        checks = {
            'COMMAND_STALE': lambda: gate.decide(cmd, 9.0, 10.0),
            'NO_COMMAND': lambda: gate.decide(None, None, 10.0),
            'COMMAND_NONFINITE': lambda: gate.decide((float('inf'), 0.0), 10.0, 10.0),
            'OPERATOR_ENABLE_STALE': lambda: gate.decide(cmd, 11.4, 11.4),
        }
        for expected, run in checks.items():
            out, reason = run()
            self.assertEqual(out, (0.0, 0.0), expected)
            self.assertIn(expected, reason)
        gate.inputs.operator_stamp = 11.4
        gate.inputs.localization = valid_state(11.4, 'DEGRADED')
        self.assertIn('NOT_VALID', gate.decide(cmd, 11.4, 11.4)[1])
        gate.inputs.localization = valid_state(11.4, map_id='other')
        self.assertIn('MAP_MISMATCH', gate.decide(cmd, 11.4, 11.4)[1])
        gate.inputs.localization = valid_state(11.4)
        gate.inputs.operator_enabled = False
        self.assertEqual(gate.decide(cmd, 11.4, 11.4)[1], 'OPERATOR_DISABLED')

    def test_supervisor_decisions(self):
        identity = lambda p: p
        args = dict(world_to_map=identity, mission_id='M1', map_id='map-A', map_version=1, params=self.params, now=5.0)
        goal, _ = decision_to_goal({'action': 'INSPECT', 'goal': (1.0, 2.0), 'yaw': 0.5, 'asset_id': 'LAD-001',
                                    'reason': 'view'}, **args)
        self.assertEqual((goal.x, goal.y, goal.asset_id), (1.0, 2.0, 'LAD-001'))
        self.assertAlmostEqual(goal.yaw, 0.5)
        goal, _ = decision_to_goal({'action': 'RETURN_BASE_WITH_GAPS', 'goal': [1.5, 1.5]}, **args)
        self.assertIn('RETURN_BASE_WITH_GAPS', goal.reason)
        for action in ('SURVEY', 'AWAIT_OPERATOR', 'RECORD_UNREACHABLE'):
            self.assertIsNone(decision_to_goal({'action': action}, **args)[0])
        shifted = decision_to_goal({'action': 'RETURN_BASE', 'goal': [2.0, 0.0]},
                                   **(args | {'world_to_map': lambda p: (p[0] - 1.0, p[1], p[2] + math.pi)}))[0]
        self.assertEqual(shifted.x, 1.0)


if __name__ == '__main__':
    unittest.main()
