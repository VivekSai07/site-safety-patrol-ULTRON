"""Phase 1: simulated Helios-32 lidar and test hall (no ROS)."""
import dataclasses
import math
import os
import time
import unittest

try:
    import mujoco
    import numpy as np
    import yaml  # noqa: F401
except ImportError:  # pragma: no cover
    mujoco = None

if mujoco is not None:
    from ultron.autonomy.config import load_sim_slam_config
    from ultron.autonomy.hall_scene import HallSim, LIDAR_HOUSING_GROUP
    from ultron.autonomy.lidar import SimLidar, rotate


@unittest.skipIf(mujoco is None, 'MuJoCo/NumPy/PyYAML not installed')
class LidarTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_sim_slam_config()
        cls.sim = HallSim(cls.config.lidar.mount_xyz_m, cls.config.lidar.mount_rpy_rad)
        cls.lidar = SimLidar(cls.sim, cls.config.lidar, cls.config.self_filter, seed=3)
        exact = dataclasses.replace(cls.config.lidar, range_noise_std_m=0.0, dropout_probability=0.0)
        cls.exact = SimLidar(cls.sim, exact, cls.config.self_filter, seed=3)

    @classmethod
    def tearDownClass(cls):
        cls.lidar.close()
        cls.exact.close()

    def geoms(self, scan, prefix):
        names = [mujoco.mj_id2name(self.sim.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or '' for g in scan.eval_geom_id]
        return np.array([n.startswith(prefix) for n in names])

    def test_hall_contents(self):
        kinds = [o.kind for o in self.sim.objects]
        ladders = [k for k in kinds if 'LADDER' in k or k == 'TRESTLE']
        self.assertEqual(len(ladders), 7)
        for kind in ('LAYHER_A_FRAME_STEPLADDER', 'COMBINATION_LADDER_A_FRAME', 'LEANING_COMBINATION_LADDER', 'TRESTLE'):
            self.assertIn(kind, kinds)
        housing = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_GEOM, 'lidar_housing')
        self.assertEqual(self.sim.model.geom_group[housing], LIDAR_HOUSING_GROUP)

    def test_beam_pattern_matches_profile(self):
        profile = self.config.lidar
        self.assertEqual(self.lidar.rays, profile.channels * profile.columns)
        elevation = np.degrees(np.arcsin(self.lidar.directions[:, 2]))
        expected = np.repeat(profile.vertical_angles_deg, profile.columns)
        np.testing.assert_allclose(elevation, expected, atol=1e-9)
        self.sim.place(6.0, 4.6, 0.4)
        scan = self.lidar.scan(1.25)
        self.assertEqual(scan.stamp, 1.25)
        self.assertEqual(scan.points.dtype, np.float32)
        self.assertTrue(np.isfinite(scan.points).all())
        self.assertEqual(len({len(scan.points), len(scan.ring), len(scan.time), len(scan.intensity)}), 1)
        self.assertLess(scan.ring.max(), profile.channels)
        self.assertTrue(((scan.time >= 0) & (scan.time < 1 / profile.scan_rate_hz)).all())
        accounted = len(scan.points) + scan.self_filtered + scan.no_return + scan.dropped
        self.assertLessEqual(accounted, scan.rays_total)  # remainder: noisy ranges outside limits

    def test_exact_ranges_on_known_planes(self):
        self.sim.place(10.0, 3.0, 0.0)  # facing the east wall face at x = 12.0
        scan = self.exact.scan(0.0)
        world = self.exact.lidar_to_world(scan.points.astype(np.float64))
        wall = self.geoms(scan, 'wall_east_a')
        self.assertGreater(wall.sum(), 100)
        np.testing.assert_allclose(world[wall, 0], 12.0, atol=1e-4)
        floor = self.geoms(scan, 'floor')
        self.assertGreater(floor.sum(), 1000)
        np.testing.assert_allclose(world[floor, 2], 0.0, atol=1e-4)

    def test_noise_statistics(self):
        self.sim.place(10.0, 3.0, 0.0)
        scan = self.lidar.scan(0.0)
        world = self.lidar.lidar_to_world(scan.points.astype(np.float64))
        wall = self.geoms(scan, 'wall_east_a')
        # Range noise projects onto x through the beam direction cosine.
        directions = scan.points[wall] / np.linalg.norm(scan.points[wall], axis=1, keepdims=True)
        origin_x = self.sim.data.site_xpos[self.sim.lidar_site][0]
        ranges_true = (12.0 - origin_x) / directions[:, 0]
        residual = np.linalg.norm(scan.points[wall], axis=1) - ranges_true
        sigma = self.config.lidar.range_noise_std_m
        self.assertLess(abs(residual.mean()), 3 * sigma / math.sqrt(len(residual)) + 1e-3)
        self.assertAlmostEqual(residual.std(), sigma, delta=0.2 * sigma)
        self.assertTrue((np.abs(world[wall, 0] - 12.0) < 6 * sigma).all())

    def test_seeded_scans_are_reproducible(self):
        self.sim.place(6.0, 4.6, 0.4)
        a = SimLidar(self.sim, self.config.lidar, self.config.self_filter, seed=11)
        b = SimLidar(self.sim, self.config.lidar, self.config.self_filter, seed=11)
        try:
            np.testing.assert_array_equal(a.scan(0).points, b.scan(0).points)
        finally:
            a.close(); b.close()

    def test_robot_self_occlusion_and_filter(self):
        for pose in ((6.0, 4.6, 0.4), (1.6, 1.5, math.pi / 2)):
            self.sim.place(*pose)
            scan = self.exact.scan(0.0)
            base = rotate(scan.points.astype(np.float64), self.exact.mount_R) + self.exact.mount_t
            inside = np.all((base >= self.exact.crop_min) & (base <= self.exact.crop_max), axis=1)
            self.assertFalse(inside.any(), 'self-filter crop box must remove robot returns')
            self.assertGreater(scan.self_filtered, 1000)
            azimuth = np.degrees(np.arctan2(scan.points[:, 1], scan.points[:, 0]))
            # Rear is blocked by the compute box; front is open.
            self.assertFalse((np.abs(azimuth) > 170).any())
            self.assertTrue(scan.blind_columns[np.abs(scan.azimuth_deg) > 170].all())
            self.assertFalse(scan.blind_columns[np.abs(scan.azimuth_deg) < 60].any())
            self.assertGreater(scan.blind_fraction, 0.1)
            self.assertLess(scan.blind_fraction, 0.5)

    def test_batch_cast_agrees_with_exact_single_rays(self):
        group = np.ones(6, dtype=np.uint8)
        group[LIDAR_HOUSING_GROUP] = 0
        out = np.zeros(1, dtype=np.int32)
        mismatches = total = 0
        for pose in ((6.0, 4.6, 0.4), (1.6, 1.5, math.pi / 2), (9.0, 8.0, 3.0), (6.0, 8.5, 1.6)):
            self.sim.place(*pose)
            self.exact.scan(0.0)
            lidar = self.exact
            combined = np.where((lidar._robot_geom >= 0) & ((lidar._geomid < 0) | (lidar._robot_dist <= lidar._dist)),
                                lidar._robot_geom, lidar._geomid)
            origin = self.sim.data.site_xpos[self.sim.lidar_site].copy()
            world = rotate(lidar.directions, np.array(self.sim.data.site_xmat[self.sim.lidar_site].reshape(3, 3)))
            for i in range(0, lidar.rays, 23):
                mujoco.mj_ray(self.sim.model, self.sim.data, origin, world[i], group, 1, -1, out)
                total += 1
                mismatches += int(out[0] != combined[i])
        self.assertLessEqual(mismatches / total, 0.001, f'{mismatches}/{total} rays disagree')

    @unittest.skipUnless(os.environ.get('ULTRON_PERF_TESTS') == '1', 'set ULTRON_PERF_TESTS=1 for timing')
    def test_scan_time_budget(self):
        times = []
        for i in range(30):
            self.sim.place(2 + 0.25 * i, 3.0 + 0.1 * i, 0.2 * i)
            start = time.perf_counter()
            self.lidar.scan(0.1 * i)
            times.append(time.perf_counter() - start)
        median = float(np.median(times[1:]))
        print(f'scan median {1e3 * median:.1f} ms, p95 {1e3 * np.percentile(times[1:], 95):.1f} ms')
        self.assertLessEqual(median, 0.050)


if __name__ == '__main__':
    unittest.main()
