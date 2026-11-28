#!/usr/bin/env python3
"""Eval-only PointCloud2 -> LaserScan (stand-in for Agent B's preprocessing; not installed).

pointcloud_to_laserscan subscribes best-effort; with ~1.5 MB Helios clouds it received 0.3-79 % of the bag
scans in our runs, which corrupted the SLAM/monitor evaluation. This converter subscribes RELIABLE, keeps
points in a base_link z band, and publishes the nearest return per angle bin with the cloud stamp.
"""
import argparse
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener

from ultron_b2_localization.geometry import transform_matrix
from ultron_b2_localization.kiss_icp_node import cloud_to_arrays


class CloudToScan(Node):
    def __init__(self, a):
        super().__init__('eval_cloud_to_scan', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.a, self.cache = a, {}
        self.n_bins = int(round(2 * math.pi / a.angle_increment))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(LaserScan, a.scan, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(PointCloud2, a.cloud, self._on_cloud, QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

    def _on_cloud(self, msg):
        frame = msg.header.frame_id
        if frame not in self.cache:
            try:
                t = self.buffer.lookup_transform(self.a.target_frame, frame, Time())
            except Exception as exc:  # noqa: BLE001 - static TF not received yet
                self.get_logger().warn(f'waiting for {self.a.target_frame}<-{frame}: {exc}', throttle_duration_sec=2.0)
                return
            tr, q = t.transform.translation, t.transform.rotation
            self.cache[frame] = transform_matrix((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        m = self.cache[frame]
        xyz, _ = cloud_to_arrays(msg)
        p = xyz @ m[:3, :3].T + m[:3, 3]
        p = p[(p[:, 2] >= self.a.min_height) & (p[:, 2] <= self.a.max_height)]
        r = np.hypot(p[:, 0], p[:, 1])
        keep = (r >= self.a.range_min) & (r <= self.a.range_max)
        p, r = p[keep], r[keep]
        idx = ((np.arctan2(p[:, 1], p[:, 0]) + math.pi) / self.a.angle_increment).astype(int) % self.n_bins
        ranges = np.full(self.n_bins, np.inf, dtype=np.float32)
        np.minimum.at(ranges, idx, r.astype(np.float32))
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = msg.header.stamp, self.a.target_frame
        scan.angle_min, scan.angle_increment = -math.pi, self.a.angle_increment
        scan.angle_max = -math.pi + (self.n_bins - 1) * self.a.angle_increment
        scan.scan_time, scan.range_min, scan.range_max = 0.1, self.a.range_min, self.a.range_max
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cloud', default='sensors/lidar/points_filtered')
    ap.add_argument('--scan', default='sensors/scan')
    ap.add_argument('--target-frame', default='base_link')
    ap.add_argument('--min-height', type=float, default=-0.35)
    ap.add_argument('--max-height', type=float, default=1.0)
    ap.add_argument('--range-min', type=float, default=1.0)
    ap.add_argument('--range-max', type=float, default=30.0)
    ap.add_argument('--angle-increment', type=float, default=0.0087)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = CloudToScan(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
