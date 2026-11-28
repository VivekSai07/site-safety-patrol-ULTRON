"""cloud_to_scan: sensors/lidar/points_filtered -> sensors/scan (owner: preprocess; integration).

Replaces pointcloud_to_laserscan (Humble), which subscribes best-effort with no QoS parameter: on hosts with a
small socket receive buffer (208 KiB here) best-effort readers lose most fragmented ~1 MB clouds (21-99.7 %
measured by Agent C). This node subscribes RELIABLE (depth 2) and publishes RELIABLE, so it is the single
``sensors/scan`` publisher in sim and replay. The cloud->target transform is a static mount and is cached.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2

from . import filter_core as fc
from . import scan_core as sc


class CloudToScan(Node):
    def __init__(self, **kwargs):
        super().__init__('cloud_to_scan', **kwargs)
        p = self.declare_parameter
        self.target_frame = p('target_frame', 'base_footprint').value
        self.scan_time = p('scan_time', 0.1).value
        self.cfg = sc.ScanConfig(
            min_height=p('min_height', 0.45).value, max_height=p('max_height', 1.00).value,
            angle_min=p('angle_min', -math.pi).value, angle_max=p('angle_max', math.pi).value,
            angle_increment=p('angle_increment', math.radians(0.2)).value,
            range_min=p('range_min', 0.35).value, range_max=p('range_max', 12.0).value)
        input_topic = p('input_topic', 'sensors/lidar/points_filtered').value
        output_topic = p('output_topic', 'sensors/scan').value
        input_reliability = p('input_reliability', 'reliable').value
        if input_reliability not in ('reliable', 'best_effort'):
            raise ValueError(f'input_reliability must be reliable or best_effort, got {input_reliability!r}')
        in_qos = qos_profile_sensor_data if input_reliability == 'best_effort' else \
            QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        from tf2_ros import Buffer, TransformListener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.cache = {}
        self.pub = self.create_publisher(LaserScan, output_topic, QoSProfile(
            depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
        self.sub = self.create_subscription(PointCloud2, input_topic, self.on_cloud, in_qos)
        self.get_logger().info(f'{input_topic} -> {output_topic} in {self.target_frame}: band '
                               f'{self.cfg.min_height}..{self.cfg.max_height} m, range {self.cfg.range_min}-'
                               f'{self.cfg.range_max} m, {self.cfg.n_bins} bins (RELIABLE in/out; PROPOSED values)')

    def transform_for(self, frame):
        if frame not in self.cache:
            t = self.buffer.lookup_transform(self.target_frame, frame, Time())
            tr, q = t.transform.translation, t.transform.rotation
            self.cache[frame] = sc.quaternion_matrix((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        return self.cache[frame]

    def convert(self, msg, transform):
        fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
        n = msg.width * msg.height
        xyz = fc.parse(msg.data, fields, msg.point_step, n, 0.0, msg.is_bigendian)[0]
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = msg.header.stamp, self.target_frame
        scan.angle_min, scan.angle_increment = self.cfg.angle_min, self.cfg.angle_increment
        scan.angle_max = self.cfg.angle_min + (self.cfg.n_bins - 1) * self.cfg.angle_increment
        scan.scan_time, scan.range_min, scan.range_max = float(self.scan_time), self.cfg.range_min, self.cfg.range_max
        scan.ranges = sc.project(xyz, transform, self.cfg).tolist()
        return scan

    def on_cloud(self, msg):
        try:
            transform = self.transform_for(msg.header.frame_id)
        except Exception as exc:  # noqa: BLE001 - static TF not received yet
            self.get_logger().warning(f'waiting for {self.target_frame}<-{msg.header.frame_id}: {exc}',
                                      throttle_duration_sec=2.0)
            return
        self.pub.publish(self.convert(msg, transform))


def main(args=None):
    rclpy.init(args=args)
    node = CloudToScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok() or 'Unable to convert call argument to Python object' not in str(exc):
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
