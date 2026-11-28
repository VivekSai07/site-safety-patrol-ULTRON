import importlib.util
from pathlib import Path

import pytest


path = Path(__file__).parents[1] / 'launch' / 'slam.launch.py'
spec = importlib.util.spec_from_file_location('replay_slam_launch', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_replay_options_do_not_change_sim_or_live():
    assert module.replay_slam_options('sim', '10', 'true') == ([], [], {'replay_clock_watchdog': False})
    assert module.replay_slam_options('live', '10', 'true') == ([], [], {'replay_clock_watchdog': False})
    params, args, monitor = module.replay_slam_options('replay', '10', 'true')
    assert params == [{'scan_queue_size': 10}]
    assert 'tf2_ros_message_filter:=debug' in args
    assert monitor == {'replay_clock_watchdog': True}
    for bad in ('0', '101', '-1', 'invalid'):
        with pytest.raises(ValueError):
            module.replay_slam_options('replay', bad, 'false')
