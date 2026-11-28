"""Replay remap table and bag-play command (no processes started)."""
import importlib.util
from pathlib import Path

from ultron_b2_bringup import replay_topics as rt

CONTRACT_SENSORS = {'sensors/lidar/points', 'sensors/imu/data', 'sensors/cam_head/color/image_raw',
                    'sensors/cam_head/color/camera_info', 'sensors/cam_head/depth/image_rect_raw',
                    'sensors/cam_head/depth/camera_info'}


def _drivers():
    path = Path(__file__).resolve().parents[1] / 'launch' / 'sub' / 'drivers.launch.py'
    spec = importlib.util.spec_from_file_location('drivers_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_vendor_topics_map_to_contract():
    table = rt.replay_table()
    assert table['/scan_cloud'] == 'sensors/lidar/points'
    assert table['/imu/data'] == 'sensors/imu/data'
    assert CONTRACT_SENSORS <= set(table.values())
    assert all(src.startswith('/camera_depth/head/') for src, dst in table.items() if 'cam_head' in dst)
    assert all(not dst.startswith('/') for dst in table.values()), 'contract names are relative'


def test_bag_tf_is_moved_off_the_tf_tree_and_static_kept():
    remaps = rt.bag_remaps('/ultron/replay')
    assert '/tf:=/ultron/replay/bag/tf_xsens' in remaps
    assert not any(r.endswith(':=/tf') or r.endswith(':=/tf_static') for r in remaps)
    assert '/tf_static' in rt.bag_topics() and '/tf_static' not in rt.replay_table()


def test_remap_targets_are_absolute_under_namespace():
    for rule in rt.bag_remaps('/ultron/replay/'):
        src, dst = rule.split(':=')
        assert src.startswith('/') and dst.startswith('/ultron/replay/') and '//' not in dst


def test_cameras_optional_and_heavy_topics_not_played():
    no_cam = rt.bag_topics(cameras=False)
    assert not any(t.startswith('/camera_depth') for t in no_cam)
    assert {'/scan_cloud', '/imu/data', '/tf', '/tf_static'} <= set(no_cam)
    assert not set(rt.NOT_REPLAYED) & set(rt.bag_topics())


def test_replay_command_has_clock_topics_and_no_control():
    cmd = _drivers().replay_command('/bags/x', '/ultron/replay', rate='0.5', loop=True)
    assert cmd[:4] == ['ros2', 'bag', 'play', '/bags/x']
    assert cmd[cmd.index('--clock') + 1] == '100' and cmd[cmd.index('--rate') + 1] == '0.5'
    assert '--loop' in cmd and '--disable-keyboard-controls' in cmd
    joined = ' '.join(cmd)
    assert 'cmd_vel' not in joined and '/api/' not in joined
