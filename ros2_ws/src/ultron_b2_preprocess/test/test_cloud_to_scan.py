"""cloud_to_scan: projection core and one RELIABLE pub/sub round trip (domain from env; never 0)."""
import math
import os
import time

import numpy as np
import pytest
import rclpy
import rclpy.executors
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from ultron_b2_preprocess import scan_core as sc
from ultron_b2_preprocess.cloud_to_scan import CloudToScan

from test_cloud_filter_node import bag_cloud

LIFT = np.array([[1, 0, 0, 0.342], [0, 1, 0, 0], [0, 0, 1, 0.83], [0, 0, 0, 1.0]])  # base_footprint<-lidar (sim)


def test_nearest_return_per_bin_and_height_band():
    cfg = sc.ScanConfig(angle_increment=math.radians(1.0))
    pts = [(2.0, 0.0, 0.0), (3.0, 0.0, 0.0),          # same bin: nearest (in target frame) wins
           (0.0, 4.0, 0.0),                           # +90 deg
           (5.0, 0.0, 1.0),                           # above band (z 1.83 m)
           (0.1, 0.0, 0.0)]                           # 0.442 m from base_footprint: kept (>= 0.35)
    r = sc.project(np.array(pts), LIFT, cfg)
    assert cfg.n_bins == 360 and len(r) == 360
    front = int((0.0 + math.pi) / cfg.angle_increment)
    assert r[front] == pytest.approx(0.442, abs=1e-4)
    left = int((math.atan2(4.0, 0.342) - cfg.angle_min) / cfg.angle_increment)
    assert r[left] == pytest.approx(math.hypot(0.342, 4.0), abs=1e-4)
    assert np.isfinite(r).sum() == 2


def test_range_limits_and_empty():
    cfg = sc.ScanConfig(range_min=1.0, range_max=3.0)
    assert np.isinf(sc.project(np.zeros((0, 3)), LIFT, cfg)).all()
    r = sc.project(np.array([(0.2, 0.0, 0.0), (5.0, 0.0, 0.0), (1.5, 0.0, 0.0)]), LIFT, cfg)
    assert np.isfinite(r).sum() == 1 and float(r[np.isfinite(r)][0]) == pytest.approx(1.842, abs=1e-4)


def test_quaternion_matrix_yaw():
    m = sc.quaternion_matrix((1, 2, 3), (0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)))
    assert np.allclose(m[:3, :3] @ [1, 0, 0], [0, 1, 0]) and np.allclose(m[:3, 3], [1, 2, 3])


def test_reliable_round_trip():
    assert os.environ.get('ROS_DOMAIN_ID', '') != '0', 'never run on domain 0'
    rclpy.init()
    try:
        node = CloudToScan()
        helper = rclpy.create_node('cloud_to_scan_test')
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_footprint', 'lidar_link'
        tf.transform.translation.x, tf.transform.translation.z, tf.transform.rotation.w = 0.342, 0.83, 1.0
        static = StaticTransformBroadcaster(helper)
        static.sendTransform(tf)
        rel = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        pub = helper.create_publisher(type(bag_cloud([])), 'sensors/lidar/points_filtered', rel)
        got = []
        helper.create_subscription(LaserScan, 'sensors/scan', got.append, rel)
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(helper)
        deadline = time.monotonic() + 15.0
        while not got and time.monotonic() < deadline:
            pub.publish(bag_cloud([(3.0, 0.0, 0.0), (0.0, -2.0, 0.0)]))
            for _ in range(10):
                ex.spin_once(timeout_sec=0.02)
        assert got, 'no scan received'
        scan = got[0]
        assert scan.header.frame_id == 'base_footprint'
        finite = sorted(round(v, 3) for v in scan.ranges if math.isfinite(v))
        assert finite == [round(math.hypot(0.342, -2.0), 3), 3.342]
        info = helper.get_publishers_info_by_topic('/sensors/scan') or helper.get_publishers_info_by_topic('sensors/scan')
        assert all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in info)
        node.destroy_node()
        helper.destroy_node()
    finally:
        rclpy.shutdown()
