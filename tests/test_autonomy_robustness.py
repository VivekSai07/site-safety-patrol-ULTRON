"""Phase 5: motion distortion, walking people, dropout fault, 20 Hz profile, resource tooling."""
import math
import os
import tempfile
import time
import unittest
from pathlib import Path

try:
    import mujoco
    import numpy as np
    import yaml
except ImportError:  # pragma: no cover
    mujoco = None

if mujoco is not None:
    from ultron.autonomy.config import CONFIG_DIR, ConfigError, load_lidar_profile, load_scenario, load_sim_slam_config
    from ultron.autonomy.hall_scene import WALKERS, HallSim
    from ultron.autonomy.latency_probe import ResourceSampler
    from ultron.autonomy.lidar import SimLidar
    from ultron.autonomy.run_mapping import latency_summary


@unittest.skipIf(mujoco is None, 'MuJoCo/NumPy/PyYAML not installed')
class DistortionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import dataclasses
        config = load_sim_slam_config()
        exact = dataclasses.replace(config.lidar, range_noise_std_m=0.0, dropout_probability=0.0)
        cls.sim = HallSim(exact.mount_xyz_m, exact.mount_rpy_rad)
        cls.lidar = SimLidar(cls.sim, exact, config.self_filter, seed=0)

    @classmethod
    def tearDownClass(cls):
        cls.lidar.close()

    def test_static_pose_function_matches_undistorted_scan(self):
        pose = (6.0, 4.6, 0.4)
        self.sim.place(*pose)
        plain = self.lidar.scan(0.0)
        sliced = self.lidar.scan(0.0, pose_at=lambda t: pose)
        np.testing.assert_allclose(sliced.points, plain.points, atol=1e-4)

    def test_each_slice_is_cast_from_its_own_pose(self):
        start = (6.0, 4.6, 0.0)
        rate = 0.8  # rad/s, the driver's maximum turn rate
        self.sim.place(*start)
        turning = self.lidar.scan(0.0, pose_at=lambda t: (start[0], start[1], start[2] + rate * t))
        key = lambda scan: {(int(r), round(float(t), 6)): p for r, t, p in zip(scan.ring, scan.time, scan.points)}
        distorted = key(turning)
        # Slices 4 and 6 cover -36..0 and 36..72 deg (open front sector; slice 0 is the blind rear).
        width = self.lidar.period / len(self.lidar._chunks)            # 0.01 s per 36-degree slice
        for index in (4, 6):
            t_mid = self.lidar._chunks[index][0]
            self.sim.place(start[0], start[1], start[2] + rate * t_mid)   # static scan at that slice's pose
            reference = key(self.lidar.scan(0.0))
            rays = [k for k in distorted if int(k[1] / width + 1e-6) == index and k in reference]
            self.assertGreater(len(rays), 1000)
            worst = max(float(np.linalg.norm(distorted[k] - reference[k])) for k in rays)
            self.assertLess(worst, 1e-3)
        # Slices really differ: the last slice's sensor is 0.072 rad from the first.
        self.assertAlmostEqual(rate * (self.lidar._chunks[-1][0] - self.lidar._chunks[0][0]), 0.072, places=3)
        self.sim.place(*start)


    def test_threaded_casting_is_deterministic(self):
        # Regression: concurrent mj_multiRay on one MjData corrupted 0-31 rays per distorted sweep.
        import dataclasses
        config = load_sim_slam_config()
        exact = dataclasses.replace(config.lidar, range_noise_std_m=0.0, dropout_probability=0.0)
        sim = HallSim(exact.mount_xyz_m, exact.mount_rpy_rad, people='walking')
        sim.place(6.0, 4.6, 0.0)
        sim.update_people(3.3)
        turn = lambda t: (6.0, 4.6, 0.8 * t)
        single = SimLidar(sim, exact, config.self_filter, threads=1)
        threaded = SimLidar(sim, exact, config.self_filter, threads=4)
        try:
            single.scan(0.0, pose_at=turn)
            for _ in range(5):
                threaded.scan(0.0, pose_at=turn)
                np.testing.assert_array_equal(threaded._dist, single._dist)
        finally:
            single.close()
            threaded.close()


@unittest.skipIf(mujoco is None, 'MuJoCo/NumPy/PyYAML not installed')
class PeopleTest(unittest.TestCase):
    def test_walkers_stay_on_their_segments_and_move(self):
        sim = HallSim(people='walking')
        for t in np.arange(0, 20, 0.37):
            for (x, y), (a, b) in zip(sim.walker_positions(t), WALKERS):
                d = abs((b[0] - a[0]) * (a[1] - y) - (a[0] - x) * (b[1] - a[1])) / math.dist(a, b)
                self.assertLess(d, 1e-9)
                self.assertLessEqual(math.dist(a, (x, y)), math.dist(a, b) + 1e-9)
        self.assertNotEqual(sim.walker_positions(0.0), sim.walker_positions(1.0))
        self.assertFalse(any(o.kind.startswith('PERSON_STATIC') for o in sim.objects))

    def test_people_modes(self):
        self.assertEqual(HallSim(people=False).people, 'none')
        with self.assertRaises(ValueError):
            HallSim(people='dancing')


@unittest.skipIf(mujoco is None, 'MuJoCo/NumPy/PyYAML not installed')
class ConfigAndToolsTest(unittest.TestCase):
    def test_dropout_scenario_and_20hz_profile(self):
        steps = load_scenario(CONFIG_DIR / 'scenarios' / 'loc_dropout.yaml').steps
        self.assertIn(('fault', 'lidar_dropout', 6.0, 0.6), steps)
        profile = load_lidar_profile(CONFIG_DIR / 'lidar_helios32_20hz.yaml')
        self.assertEqual((profile.scan_rate_hz, profile.columns), (20.0, 900))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 's.yaml'
            path.write_text(yaml.safe_dump({'schema': 'ultron.sim_scenario/v1', 'scene': 'x', 'start_pose': [0, 0, 0],
                                            'steps': [{'fault': 'lidar_blackout', 'duration_s': 1, 'probability': 0.5}]}))
            with self.assertRaises(ConfigError):  # probability only for dropout
                load_scenario(path)

    def test_resource_sampler_and_latency_summary(self):
        import subprocess
        import sys
        # As in the orchestrators: sample a separate process group while this thread waits.
        burner = subprocess.Popen([sys.executable, '-c', 'import time\nend=time.time()+1.5\nwhile time.time()<end: pass'],
                                  start_new_session=True)
        sampler = ResourceSampler({}, period=0.2)
        sampler.start()
        sampler.add('burner', burner.pid)   # added after start, as the orchestrators do
        burner.wait()
        stats = sampler.stop()['burner']
        self.assertGreater(stats['samples'], 2)
        self.assertGreater(stats['cpu_percent_mean'], 50.0)   # one busy core
        self.assertGreater(stats['rss_mb_max'], 1.0)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'latency.csv'
            path.write_text('scan_stamp,seen_at,latency_s\n' + ''.join(f'{i},{i},{v}\n' for i, v in enumerate((0.04, 0.06, 0.1))))
            summary = latency_summary(path)
            self.assertEqual((summary['scans'], summary['median_s'], summary['max_s']), (3, 0.06, 0.1))
            self.assertIsNone(latency_summary(Path(tmp) / 'missing.csv'))


if __name__ == '__main__':
    unittest.main()
