"""Record profile regexes (Agent F): what each profile captures and never captures."""
from pathlib import Path
import re

from ultron_b2_bringup.record_profiles import load_profiles, profile_regex

PROFILES = load_profiles(Path(__file__).resolve().parents[1] / 'config' / 'record' / 'profiles.yaml')
NS = '/ultron/sim'


def _matches(profile, topic, use_sim_time=True):
    return re.match(profile_regex(PROFILES, profile, NS, use_sim_time), topic) is not None


def test_minimal_is_small_and_namespaced():
    for t in ('/tf', '/tf_static', '/clock', f'{NS}/state/odom', f'{NS}/sensors/scan', f'{NS}/map'):
        assert _matches('minimal', t), t
    for t in (f'{NS}/sensors/lidar/points', f'{NS}/sensors/cam_head/color/image_raw', f'{NS}/viz/points',
              '/ultron/replay/state/odom', f'{NS}/sensors/scan_extra', f'/x{NS}/map'):
        assert not _matches('minimal', t), t
    assert not _matches('minimal', '/clock', use_sim_time=False)


def test_debug_extends_minimal_without_raw_sensors():
    for t in (f'{NS}/state/odom', f'{NS}/viz/points', f'{NS}/sensors/imu/data', f'{NS}/sim/ground_truth/ladders'):
        assert _matches('debug', t), t
    for t in (f'{NS}/sensors/lidar/points', f'{NS}/sensors/cam_head/color/image_raw'):
        assert not _matches('debug', t), t


def test_full_records_raw_in_namespace_only():
    assert _matches('full', f'{NS}/sensors/lidar/points')
    assert not _matches('full', '/ultron/live/sensors/lidar/points')
