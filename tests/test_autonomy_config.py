import tempfile
import unittest
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover - CI installs the autonomy extra
    yaml = None

from ultron.autonomy.config import CONFIG_DIR, ConfigError, load_lidar_profile, load_sim_slam_config


@unittest.skipIf(yaml is None, 'PyYAML (autonomy extra) not installed')
class AutonomyConfigTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.lidar = yaml.safe_load((CONFIG_DIR / 'lidar_helios32.yaml').read_text())
        self.sim = yaml.safe_load((CONFIG_DIR / 'sim_slam.yaml').read_text())

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, lidar=None, sim=None):
        (self.dir / 'lidar_helios32.yaml').write_text(yaml.safe_dump(lidar or self.lidar))
        (self.dir / 'sim_slam.yaml').write_text(yaml.safe_dump(sim or self.sim))
        return self.dir / 'sim_slam.yaml'

    def test_shipped_configuration_loads(self):
        config = load_sim_slam_config()
        lidar = config.lidar
        self.assertEqual(lidar.channels, 32)
        self.assertEqual(len(lidar.vertical_angles_deg), 32)
        self.assertAlmostEqual(lidar.vertical_angles_deg[0], -55.0)
        self.assertAlmostEqual(lidar.vertical_angles_deg[-1], 15.0)
        self.assertEqual(lidar.columns, 1800)
        self.assertEqual(config.frames['base'], 'base_link')
        self.assertEqual(len(config.sha256), 64)

    def test_unconfirmed_values_are_reported(self):
        lidar = load_lidar_profile()
        self.assertIn('variant', lidar.unconfirmed)
        self.assertIn('mount_xyz_m', lidar.unconfirmed)
        self.assertNotIn('channels', lidar.unconfirmed)

    def test_unknown_and_missing_keys_are_rejected(self):
        self.sim['driver']['max_speed_typo'] = 1.0
        with self.assertRaisesRegex(ConfigError, 'unknown'):
            load_sim_slam_config(self.write())
        del self.sim['driver']['max_speed_typo']
        del self.sim['odometry']['seed']
        with self.assertRaisesRegex(ConfigError, 'missing'):
            load_sim_slam_config(self.write())

    def test_topic_outside_simulation_namespace_is_rejected(self):
        for topic in ('/cmd_vel', '/ultron/cmd_vel', '/api/sport/request'):
            self.sim['topics']['cmd_vel'] = topic
            with self.assertRaisesRegex(ConfigError, 'simulation namespace'):
                load_sim_slam_config(self.write())

    def test_network_isolation_and_sim_time_are_required(self):
        self.sim['ros']['localhost_only'] = False
        with self.assertRaisesRegex(ConfigError, 'localhost_only'):
            load_sim_slam_config(self.write())

    def test_lidar_value_without_valid_source_is_rejected(self):
        self.lidar['scan_rate_hz']['source'] = 'guess'
        with self.assertRaisesRegex(ConfigError, 'source'):
            load_sim_slam_config(self.write())
        self.lidar['scan_rate_hz'] = 10.0
        with self.assertRaises(ConfigError):
            load_sim_slam_config(self.write())

    def test_invalid_numbers_are_rejected(self):
        self.lidar['range_min_m']['value'] = 200.0
        with self.assertRaisesRegex(ConfigError, 'range_min_m'):
            load_lidar_profile(self.write() and self.dir / 'lidar_helios32.yaml')
        self.lidar['range_min_m']['value'] = 0.2
        self.lidar['horizontal_resolution_deg']['value'] = 0.7
        with self.assertRaisesRegex(ConfigError, 'divide 360'):
            load_lidar_profile(self.write() and self.dir / 'lidar_helios32.yaml')
        self.lidar['horizontal_resolution_deg']['value'] = 0.2
        self.sim['driver']['command_timeout_s'] = 0
        with self.assertRaisesRegex(ConfigError, 'command_timeout_s'):
            load_sim_slam_config(self.write())
        self.sim['driver']['command_timeout_s'] = float('nan')
        with self.assertRaisesRegex(ConfigError, 'finite'):
            load_sim_slam_config(self.write())


if __name__ == '__main__':
    unittest.main()
