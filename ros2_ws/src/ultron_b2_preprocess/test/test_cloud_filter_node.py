"""cloud_filter node on synthetic PointCloud2 messages: direct processing and one pub/sub round trip.

Uses the domain from the environment (dev container: 71/72, localhost only; never 0).
"""
import os
import time

import numpy as np
import pytest
import rclpy
import rclpy.executors
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from ultron_b2_preprocess import filter_core as fc
from ultron_b2_preprocess.cloud_filter import CloudFilter

STAMP = 1795823322.5


@pytest.mark.parametrize('module_name,node_name', [('cloud_filter', 'CloudFilter'), ('cloud_to_scan', 'CloudToScan')])
def test_shutdown_suppresses_only_known_conversion_race(monkeypatch, module_name, node_name):
    import importlib
    module = importlib.import_module(f'ultron_b2_preprocess.{module_name}')

    class Node:
        def destroy_node(self):
            pass

    monkeypatch.setattr(module, node_name, Node)
    monkeypatch.setattr(rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(rclpy, 'try_shutdown', lambda: None)
    message = 'Unable to convert call argument to Python object (compile in debug mode for details)'
    monkeypatch.setattr(rclpy, 'spin', lambda _: (_ for _ in ()).throw(RuntimeError(message)))
    monkeypatch.setattr(rclpy, 'ok', lambda: False)
    module.main()
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    with pytest.raises(RuntimeError, match='Unable to convert'):
        module.main()
    monkeypatch.setattr(rclpy, 'ok', lambda: False)
    monkeypatch.setattr(rclpy, 'spin', lambda _: (_ for _ in ()).throw(RuntimeError('other error')))
    with pytest.raises(RuntimeError, match='other error'):
        module.main()


def bag_cloud(xyz, frame='lidar_link', dt=0.1):
    """PointCloud2 in the recorded rslidar_sdk layout (x,y,z,intensity f32, ring u16, timestamp f64)."""
    dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'timestamp'],
                      'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'],
                      'offsets': [0, 4, 8, 12, 16, 18], 'itemsize': 26})
    xyz = np.asarray(xyz, float).reshape(-1, 3)
    a = np.zeros(len(xyz), dtype)
    if len(xyz):
        a['x'], a['y'], a['z'] = xyz.T
        a['timestamp'] = STAMP + np.linspace(0, dt, len(xyz))
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(STAMP), int(round((STAMP % 1) * 1e9))
    msg.height, msg.width = len(xyz), 1
    msg.fields = [PointField(name='x', offset=0, datatype=7, count=1), PointField(name='y', offset=4, datatype=7, count=1),
                  PointField(name='z', offset=8, datatype=7, count=1),
                  PointField(name='intensity', offset=12, datatype=7, count=1),
                  PointField(name='ring', offset=16, datatype=4, count=1),
                  PointField(name='timestamp', offset=18, datatype=8, count=1)]
    msg.point_step, msg.row_step, msg.is_dense = 26, 26, True
    msg.data = a.tobytes()
    return msg


def decode(msg):
    fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
    n = msg.width * msg.height
    if n == 0:
        return np.zeros(0, fc.OUT_DTYPE)
    return np.frombuffer(bytes(msg.data), fc.structured_dtype(fields, msg.point_step), count=n)


@pytest.fixture(scope='module')
def ros():
    assert os.environ.get('ROS_DOMAIN_ID', '') != '0', 'never run on domain 0'
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    n = CloudFilter(parameter_overrides=[Parameter('voxel_size', value=0.1), Parameter('stats_period_s', value=0.0)])
    yield n
    n.destroy_node()


def test_crop_self_box_and_voxel(node):
    rng = np.random.default_rng(3)
    wall = np.column_stack([np.full(2000, 4.0), rng.uniform(-2, 2, 2000), rng.uniform(-0.5, 1.5, 2000)])
    body = rng.uniform([-0.2, -0.3, -0.5], [0.15, 0.3, 0.0], size=(300, 3))
    far = [[60.0, 0, 0]]
    out, stats, cpu_s = node.process(bag_cloud(np.vstack([wall, body, far])))
    pts = decode(out)
    xyz = np.stack([pts['x'], pts['y'], pts['z']], 1).astype(float)
    assert out.header.frame_id == 'lidar_link' and out.header.stamp.sec == int(STAMP)
    assert out.width * out.height == len(pts) == stats.n_out
    assert not fc.inside_box(xyz, node.cfg.self_box_min, node.cfg.self_box_max).any()
    assert stats.n_self + stats.n_range >= 300 and np.all(np.linalg.norm(xyz, axis=1) <= 40.0)
    assert len(pts) < 2000 / 2  # 0.1 m voxels on a 4 x 2 m wall
    assert [f.name for f in out.fields] == ['x', 'y', 'z', 'intensity', 'ring', 'time']
    assert np.all((pts['time'] >= 0) & (pts['time'] <= 0.1 + 1e-6))
    assert cpu_s < 1.0


def test_empty_cloud_publishes_empty(node):
    out, stats, _ = node.process(bag_cloud(np.zeros((0, 3))))
    assert out.width * out.height == 0 and stats.n_out == 0 and len(out.data) == 0


def test_nan_points_removed(node):
    msg = bag_cloud([[np.nan, 1, 1], [3.0, 0, 0], [np.inf, 0, 0]])
    msg.is_dense = False
    out, stats, _ = node.process(msg)
    pts = decode(out)
    assert stats.n_nonfinite == 2 and len(pts) == 1 and pts['x'][0] == pytest.approx(3.0)


def test_cloud_without_xyz_rejected(node):
    msg = bag_cloud([[1.0, 0, 0]])
    msg.fields = [f for f in msg.fields if f.name != 'z']
    with pytest.raises(ValueError):
        node.process(msg)


def test_pub_sub_round_trip(node):
    helper = rclpy.create_node('cloud_filter_test_helper')
    got = []
    helper.create_subscription(PointCloud2, 'sensors/lidar/points_filtered', got.append, 10)
    pub = helper.create_publisher(PointCloud2, 'sensors/lidar/points', 10)  # reliable offer, like the bag
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    try:
        deadline = time.monotonic() + 15.0
        while not got and time.monotonic() < deadline:
            pub.publish(bag_cloud([[3.0, 0.0, 0.0], [0.0, 0.0, -0.3]]))
            executor.spin_once(timeout_sec=0.2)
        assert got, 'no filtered cloud received'
        assert got[0].width * got[0].height == 1
    finally:
        executor.shutdown()
        helper.destroy_node()
