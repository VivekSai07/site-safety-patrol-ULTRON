"""Phase 2 logic without ROS: odometry drift, 2D scan projection, kinematic driver, route."""
import math
import os
import unittest
from unittest import mock

try:
    import numpy as np
    import yaml  # noqa: F401
except ImportError:  # pragma: no cover
    np = None

if np is not None:
    from ultron.autonomy.config import CONFIG_DIR, load_route, load_sim_slam_config
    from ultron.autonomy.driver import KinematicDriver, route_segments
    from ultron.autonomy.odometry import OdometrySimulator
    from ultron.autonomy.scan2d import ScanGeometry, open_sector, project

DRIVER = {'max_linear_m_s': 0.6, 'max_angular_rad_s': 0.8, 'max_linear_accel_m_s2': 0.5,
          'max_angular_accel_rad_s2': 1.0, 'command_timeout_s': 0.5, 'footprint_radius_m': 0.55}


@unittest.skipIf(np is None, 'NumPy/PyYAML not installed')
class OdometryTest(unittest.TestCase):
    params = {'linear_scale_error': 0.03, 'yaw_rate_bias_rad_s': 0.004, 'linear_noise_std': 0.02,
              'angular_noise_std': 0.02, 'seed': 7}

    def test_drift_is_bounded_and_nonzero(self):
        odom = OdometrySimulator(self.params)
        for _ in range(500):  # 10 m straight at 1 m/s, 0.02 s steps
            odom.step(1.0, 0.0, 0.02)
        self.assertAlmostEqual(odom.distance, 10.3, delta=0.15)       # 3 % scale error
        self.assertGreater(abs(odom.yaw), 0.01)                         # yaw bias while moving
        self.assertLess(abs(odom.yaw), 0.1)

    def test_no_drift_while_stationary(self):
        odom = OdometrySimulator(self.params)
        for _ in range(1000):
            odom.step(0.0, 0.0, 0.02)
        self.assertEqual((odom.x, odom.y, odom.yaw), (0.0, 0.0, 0.0))

    def test_exact_when_error_free(self):
        odom = OdometrySimulator({k: 0.0 for k in self.params} | {'seed': 1})
        for _ in range(100):
            odom.step(0.0, math.pi / 2, 0.01)  # quarter turn
        for _ in range(100):
            odom.step(1.0, 0.0, 0.01)
        self.assertAlmostEqual(odom.x, 0.0, places=9)
        self.assertAlmostEqual(odom.y, 1.0, places=9)


@unittest.skipIf(np is None, 'NumPy/PyYAML not installed')
class ScanProjectionTest(unittest.TestCase):
    geometry = ScanGeometry(-math.pi / 2, math.pi / 2, math.radians(1.0), 0.3, 30.0, -0.45, 0.6, 0.18)

    def test_nearest_return_per_bin_and_height_band(self):
        points = np.array([[2.0, 0.0, 0.0],     # in band, bin 0 deg
                           [3.0, 0.0, 0.0],     # farther, same bin
                           [1.0, 0.0, -0.7],    # floor: below band (base z -0.52)
                           [0.0, 1.5, 0.1],     # 90 deg
                           [-1.0, 0.0, 0.0]])   # 180 deg: outside the open sector
        ranges = project(points, self.geometry)
        self.assertEqual(len(ranges), 181)
        self.assertAlmostEqual(float(ranges[90]), 2.0, places=5)
        self.assertAlmostEqual(float(ranges[180]), 1.5, places=5)
        self.assertEqual(int(np.isfinite(ranges).sum()), 2)
        self.assertTrue(np.isinf(ranges[0]))

    def test_open_sector_excludes_rear_blind_columns(self):
        azimuth = np.arange(1800) * 0.2 - 180.0
        blind = np.abs(azimuth) > 125
        low, high = open_sector(blind, azimuth, margin_deg=2.0)
        self.assertAlmostEqual(math.degrees(low), -123.0, delta=0.25)
        self.assertAlmostEqual(math.degrees(high), 123.0, delta=0.25)
        with self.assertRaises(ValueError):
            open_sector(np.ones(1800, bool), azimuth)
        wrapped = np.abs(azimuth) < 40  # open sector would straddle 180 deg
        with self.assertRaisesRegex(ValueError, 'wraps'):
            open_sector(wrapped, azimuth)


@unittest.skipIf(np is None, 'NumPy/PyYAML not installed')
class DriverTest(unittest.TestCase):
    def test_acceleration_limit_and_timeout_stop(self):
        driver = KinematicDriver(DRIVER, (0, 0, 0), lambda x, y: True)
        driver.set_command(0.5, 0.0, now=0.0)
        state = driver.step(0.02, 0.02)
        self.assertAlmostEqual(state.vx, 0.01)  # 0.5 m/s^2 * 0.02 s
        t = 0.02
        while t < 0.47:  # stay clear of the 0.5 s timeout boundary (float accumulation)
            t += 0.02
            driver.step(t, 0.02)
        self.assertGreater(driver.state.vx, 0.2)
        state = driver.step(0.52, 0.02)          # 0.52 s after the only command
        self.assertEqual((state.vx, state.wz, state.last_event), (0.0, 0.0, 'COMMAND_TIMEOUT'))
        x = state.x
        driver.step(0.6, 0.02)
        self.assertEqual(driver.state.x, x)
        self.assertEqual(driver.state.timeouts, 1)

    def test_speed_clamped_and_nonfinite_rejected(self):
        driver = KinematicDriver(DRIVER, (0, 0, 0), lambda x, y: True)
        driver.set_command(5.0, -9.0, now=0.0)
        for i in range(200):
            driver.step(0.0 + 1e-6, 0.02)  # keep the command fresh
        self.assertAlmostEqual(driver.state.vx, 0.6)
        self.assertAlmostEqual(driver.state.wz, -0.8)
        with self.assertRaises(ValueError):
            driver.set_command(float('nan'), 0.0, now=0.0)

    def test_collision_stops_and_cancels_route(self):
        wall = lambda x, y: x < 1.0
        driver = KinematicDriver(DRIVER, (0, 0, 0), wall)
        driver.load_route([(0.5, 0.0, 5.0)])
        t = 0.0
        for _ in range(300):
            t += 0.02
            driver.step(t, 0.02)
        self.assertLess(driver.state.x, 1.0)
        self.assertEqual(driver.state.collisions, 1)
        self.assertFalse(driver.route_active)
        with self.assertRaises(ValueError):
            KinematicDriver(DRIVER, (2, 0, 0), wall)

    def test_route_reproduces_waypoints(self):
        route = load_route(CONFIG_DIR / 'routes' / 'hall_survey_loop.yaml')
        segments = route_segments(route.start_pose, route.waypoints, route.final_yaw)
        driver = KinematicDriver(DRIVER | {'max_linear_m_s': 1.0}, route.start_pose, lambda x, y: True)
        driver.load_route(segments)
        t, visited = 0.0, []
        while driver.route_active:
            t += 0.02
            s = driver.step(t, 0.02)
            visited.append((s.x, s.y))
        for wx, wy in route.waypoints:
            self.assertLess(min(math.hypot(wx - x, wy - y) for x, y in visited), 0.02)
        self.assertAlmostEqual(driver.state.x, route.start_pose[0], places=3)
        self.assertAlmostEqual(driver.state.y, route.start_pose[1], places=3)
        self.assertAlmostEqual(math.cos(driver.state.yaw - route.final_yaw), 1.0, places=5)

    def test_survey_route_is_collision_free_in_hall(self):
        try:
            from ultron.autonomy.hall_scene import build_hall, footprint_free
        except ImportError:  # pragma: no cover
            self.skipTest('hall scene unavailable')
        _, objects = build_hall()
        route = load_route(CONFIG_DIR / 'routes' / 'hall_survey_loop.yaml')
        driver = KinematicDriver(DRIVER | {'max_linear_m_s': 1.0}, route.start_pose,
                                 lambda x, y: footprint_free(objects, x, y, DRIVER['footprint_radius_m']))
        driver.load_route(route_segments(route.start_pose, route.waypoints, route.final_yaw))
        t = 0.0
        while driver.route_active:
            t += 0.02
            driver.step(t, 0.02)
        self.assertEqual(driver.state.collisions, 0)
        self.assertGreater(t, 60)


@unittest.skipIf(np is None, 'NumPy/PyYAML not installed')
class IsolationTest(unittest.TestCase):
    def test_bridge_refuses_without_isolated_domain(self):
        try:
            from ultron.autonomy.sim_bridge import check_isolation
        except ImportError:  # pragma: no cover
            self.skipTest('bridge dependencies unavailable')
        config = load_sim_slam_config()
        for env in ({}, {'ROS_DOMAIN_ID': '71'}, {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'}):
            with mock.patch.dict(os.environ, env, clear=True), self.assertRaises(SystemExit):
                check_isolation(config)
        with mock.patch.dict(os.environ, {'ROS_DOMAIN_ID': '71', 'ROS_LOCALHOST_ONLY': '1'}, clear=True):
            check_isolation(config)


if __name__ == '__main__':
    unittest.main()
