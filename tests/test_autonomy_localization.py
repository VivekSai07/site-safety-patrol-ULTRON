"""Phase 4: LocalizationState monitor, consumer gate, time-correct lookup (no ROS) + opt-in run."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

try:
    import numpy as np
    import yaml
except ImportError:  # pragma: no cover
    np = None

if np is not None:
    from ultron.autonomy.config import CONFIG_DIR, ConfigError, load_scenario, load_sim_slam_config
    from ultron.autonomy.localization_monitor import confirm_label
    from ultron.autonomy.localization_state import (LocalizationMonitor, MapMatcher, TransformHistory,
                                                    TransformRejected, accept_pose)

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(np is None, 'NumPy/PyYAML not installed')
class MonitorTest(unittest.TestCase):
    def setUp(self):
        self.params = load_sim_slam_config().localization
        # A square room of occupied cells, 4 x 4 m, map frame.
        edge = np.arange(0, 4, 0.05)
        walls = np.vstack([np.column_stack((edge, np.zeros_like(edge))), np.column_stack((edge, np.full_like(edge, 4))),
                           np.column_stack((np.zeros_like(edge), edge)), np.column_stack((np.full_like(edge, 4), edge))])
        self.matcher = MapMatcher(walls, 0.05, self.params['match_tolerance_m'])
        self.walls = walls
        self.monitor = LocalizationMonitor(self.params, 'map-A', 1)
        self.t = 10.0

    def scan_from(self, pose):
        """Endpoints in base_link of the true walls seen from ``pose``."""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        d = self.walls - (x, y)
        return np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]))

    def feed(self, estimate, truth=None, n=1):
        for _ in range(n):
            self.t += 0.1
            self.monitor.update_scan(self.t, estimate, self.scan_from(truth or estimate), self.matcher)
            self.monitor.tick(self.t + 0.05)
        return self.monitor.state

    def test_matcher_scores(self):
        self.assertEqual(self.matcher.score((2, 2, 0), self.scan_from((2, 2, 0))), 1.0)
        self.assertLess(self.matcher.score((2.6, 2, 0), self.scan_from((2, 2, 0))), 0.6)
        self.assertEqual(self.matcher.score((2, 2, 0), np.zeros((0, 2))), 0.0)

    def test_valid_only_after_consecutive_matching_scans(self):
        self.assertEqual(self.monitor.state, 'INITIALIZING')
        self.assertEqual(self.feed((2, 2, 0), n=self.params['valid_consecutive'] - 1), 'INITIALIZING')
        self.assertEqual(self.feed((2, 2, 0)), 'VALID')

    def test_data_arrival_alone_never_validates(self):
        # Scans keep arriving but disagree with the map: must not become VALID.
        self.assertEqual(self.feed((2.7, 2.0, 0.3), truth=(2, 2, 0), n=30), 'LOST')

    def test_mismatch_degrades_then_loses(self):
        self.feed((2, 2, 0), n=5)
        self.assertEqual(self.feed((2, 2, 0), truth=(2.5, 2.5, 0.4)), 'DEGRADED')
        self.assertEqual(self.feed((2, 2, 0), truth=(2.5, 2.5, 0.4), n=self.params['lost_consecutive']), 'LOST')
        # Leaving LOST needs the longer recovery run.
        self.assertEqual(self.feed((2, 2, 0), n=self.params['recover_consecutive'] - 1), 'LOST')
        self.assertEqual(self.feed((2, 2, 0)), 'VALID')

    def test_stale_when_scans_stop_and_revalidates_after(self):
        self.feed((2, 2, 0), n=5)
        self.monitor.tick(self.t + self.params['scan_age_limit_s'] - 0.01)
        self.assertEqual(self.monitor.state, 'VALID')
        self.monitor.tick(self.t + self.params['scan_age_limit_s'] + 0.01)
        self.assertEqual(self.monitor.state, 'STALE')
        self.t += 3.0
        self.assertEqual(self.feed((2, 2, 0)), 'DEGRADED')
        self.assertEqual(self.feed((2, 2, 0), n=self.params['recover_consecutive']), 'VALID')

    def test_pose_jump_is_not_silently_accepted(self):
        self.feed((2, 2, 0), n=5)
        self.assertEqual(self.feed((2.0, 2.8, 0)), 'DEGRADED')  # 0.8 m jump with a matching scan
        self.assertIn('jumped', self.monitor.transitions[-1]['reason'])

    def test_missing_transform_degrades(self):
        self.feed((2, 2, 0), n=5)
        self.t += 0.1
        self.monitor.update_scan(self.t, None, self.scan_from((2, 2, 0)), self.matcher)
        self.assertEqual(self.monitor.state, 'DEGRADED')

    def test_consumer_gate(self):
        self.feed((2, 2, 0), n=5)
        now = self.t + 0.1
        state = self.monitor.snapshot(now)
        self.assertEqual(accept_pose(state, 'map-A', 1, now), (True, 'OK'))
        self.assertFalse(accept_pose(state, 'map-B', 1, now)[0])
        self.assertIn('MAP_MISMATCH', accept_pose(state, 'map-A', 2, now)[1])
        self.assertIn('STALE', accept_pose(state, 'map-A', 1, now + 1.0)[1])
        self.assertIn('FUTURE', accept_pose(state, 'map-A', 1, state.stamp - 0.5)[1])
        self.assertFalse(accept_pose(None, 'map-A', 1, now)[0])
        self.feed((2, 2, 0), truth=(2.6, 2.6, 0.5))
        self.assertIn('NOT_VALID', accept_pose(self.monitor.snapshot(self.t), 'map-A', 1, self.t)[1])
        record = state.to_dict()
        self.assertEqual(record['schema'], 'ultron.localization_state/v1')
        self.assertFalse(record['ground_truth_used'])
        for key in ('stamp', 'clock_id', 'frame_id', 'map_id', 'map_version', 'position', 'orientation_xyzw',
                    'state', 'age_s', 'age_limit_s', 'covariance_xy_yaw', 'covariance_source'):
            self.assertIn(key, record)


@unittest.skipIf(np is None, 'NumPy/PyYAML not installed')
class TransformHistoryTest(unittest.TestCase):
    def setUp(self):
        self.h = TransformHistory(horizon_s=2.0, max_gap_s=0.1)
        for i in range(11):
            self.h.add(10 + 0.02 * i, (0.1 * i, 0.0, math.radians(2 * i)), 'VALID')

    def test_interpolates_at_acquisition_time(self):
        x, y, yaw = self.h.transform_at(10.03)
        self.assertAlmostEqual(x, 0.15)
        self.assertAlmostEqual(math.degrees(yaw), 3.0)

    def test_rejects_extrapolation_old_gaps_and_invalid(self):
        with self.assertRaisesRegex(TransformRejected, 'EXTRAPOLATION'):
            self.h.transform_at(10.25)
        with self.assertRaisesRegex(TransformRejected, 'TOO_OLD'):
            self.h.transform_at(9.0)
        self.h.add(10.5, (1.0, 0, 0), 'VALID')
        with self.assertRaisesRegex(TransformRejected, 'GAP'):
            self.h.transform_at(10.3)
        self.h.add(10.52, (1.0, 0, 0), 'DEGRADED')
        with self.assertRaisesRegex(TransformRejected, 'NOT_VALID'):
            self.h.transform_at(10.51)
        with self.assertRaises(ValueError):
            self.h.add(10.0, (0, 0, 0), 'VALID')

    def test_poses_are_confirmed_only_by_later_scans(self):
        # Scan at 31.5 VALID; kidnap at 31.52; next scan 31.6 scores DEGRADED.
        scored = [(31.5, 'VALID'), (31.6, 'DEGRADED')]
        self.assertEqual(confirm_label(31.50, scored, now=32.0, max_wait=0.65), 'VALID')
        self.assertEqual(confirm_label(31.54, scored, now=32.0, max_wait=0.65), 'DEGRADED')
        self.assertIsNone(confirm_label(31.64, scored, now=31.8, max_wait=0.65))           # wait for a scan
        self.assertEqual(confirm_label(31.64, scored, now=32.5, max_wait=0.65), 'UNCONFIRMED')


@unittest.skipIf(np is None, 'NumPy/PyYAML not installed')
class ScenarioConfigTest(unittest.TestCase):
    def test_shipped_scenarios_load(self):
        for name in ('loc_reload', 'loc_blackout', 'loc_kidnap'):
            self.assertTrue(load_scenario(CONFIG_DIR / 'scenarios' / f'{name}.yaml').steps)

    def test_bad_steps_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            for step in ({'fault': 'gps_spoof', 'duration_s': 1}, {'teleport': [1, 2]}, {'dance': 1},
                         {'wait_s': -1}):
                path = Path(tmp) / 's.yaml'
                path.write_text(yaml.safe_dump({'schema': 'ultron.sim_scenario/v1', 'scene': 'x',
                                                'start_pose': [0, 0, 0], 'steps': [step]}))
                with self.assertRaises(ConfigError):
                    load_scenario(path)

    def test_localization_thresholds_validated(self):
        config = yaml.safe_load((CONFIG_DIR / 'sim_slam.yaml').read_text())
        config['localization']['score_lost_max'] = 0.9  # above score_valid_min
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / 'sim_slam.yaml').write_text(yaml.safe_dump(config))
            (Path(tmp) / 'lidar_helios32.yaml').write_text((CONFIG_DIR / 'lidar_helios32.yaml').read_text())
            with self.assertRaisesRegex(ConfigError, 'score_lost_max'):
                load_sim_slam_config(Path(tmp) / 'sim_slam.yaml')


@unittest.skipUnless(os.environ.get('ULTRON_SLAM_TESTS') == '1' and os.environ.get('ROS_LOCALHOST_ONLY') == '1',
                     'set ULTRON_SLAM_TESTS=1 and run inside scripts/sim-slam.sh exec (about 3 minutes)')
class LocalizationRunTest(unittest.TestCase):
    def test_map_then_kidnap_has_no_false_valid(self):
        with tempfile.TemporaryDirectory() as tmp:
            mapping, loc = Path(tmp) / 'map', Path(tmp) / 'loc'
            for cmd in ([sys.executable, '-m', 'ultron.autonomy.run_mapping', str(mapping), '--rate', '2', '--no-eval'],
                        [sys.executable, '-m', 'ultron.autonomy.run_localization', str(loc), '--map', str(mapping),
                         '--scenario', str(CONFIG_DIR / 'scenarios' / 'loc_kidnap.yaml'), '--rate', '2']):
                proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=600)
                self.assertEqual(proc.returncode, 0, proc.stdout[-2000:] + proc.stderr[-2000:])
            result = json.loads((loc / 'eval.json').read_text())
            self.assertEqual(result['verdict'], 'PASS', result['checks'])
            self.assertEqual(result['false_valid_samples'], 0)
            print(f"\n  kidnap detected after {result['kidnap']['detect_delay_s']} s; "
                  f"VALID rmse {result['valid_rmse_m']:.3f} m")


if __name__ == '__main__':
    unittest.main()
