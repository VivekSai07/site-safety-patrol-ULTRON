import unittest

from ultron import preflight
from ultron.preflight import CAMERA_ONLY, evaluate, table
from ultron.recording import COLOR, COLOR_INFO, DEPTH

CAMERA = {COLOR: 30.0, DEPTH: 30.0, COLOR_INFO: 30.0}
HALL = {**CAMERA, "/scan_cloud": 10.0, "/imu/data": 100.0, "/tf": 100.0}


class PreflightTest(unittest.TestCase):
    def test_hall_setup_is_ready(self):
        ok, rows = evaluate(HALL)
        self.assertTrue(ok)
        self.assertIn("[OK  ] lidar", table(rows))

    def test_todays_camera_only_bags_would_have_been_stopped(self):
        ok, rows = evaluate(CAMERA)
        self.assertFalse(ok)
        text = table(rows)
        self.assertIn("[FAIL] lidar", text)
        self.assertIn("[FAIL] imu", text)
        self.assertIn("rslidar_sdk", text)

    def test_camera_only_mode(self):
        self.assertTrue(evaluate(CAMERA, CAMERA_ONLY)[0])
        self.assertFalse(evaluate({COLOR: 30.0}, CAMERA_ONLY)[0])  # depth missing

    def test_driver_default_lidar_name_counts(self):
        rates = {**HALL, "/rslidar_points": 10.0}
        del rates["/scan_cloud"]
        self.assertTrue(evaluate(rates)[0])

    def test_slow_topic_and_missing_type(self):
        self.assertFalse(evaluate({**HALL, "/scan_cloud": 1.0})[0])
        ok, rows = evaluate(HALL, missing_types={"/sportmodestate"})
        self.assertTrue(ok)  # unitree is optional
        self.assertIn("NO TYPE", table(rows))

    def test_every_profile_topic_listed_once(self):
        topics = preflight.all_topics()
        self.assertEqual(len(topics), len(set(topics)))


if __name__ == "__main__":
    unittest.main()
