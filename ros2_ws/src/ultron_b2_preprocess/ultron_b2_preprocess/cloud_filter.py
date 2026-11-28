"""cloud_filter node: sensors/lidar/points -> sensors/lidar/points_filtered.

Self-filter crop box, range and height crop, voxel downsample (first point per voxel, attributes kept) and
optional rotation-only deskew from IMU angular velocity. Output fields x,y,z,intensity (f32), ring (u16),
time (f32, seconds after header stamp; zeros after deskew, when the stamp becomes the reference time).
QoS: subscribes best-effort (accepts reliable and best-effort offers), publishes reliable.
"""
import array
import collections
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField

from . import filter_core as fc


def stamp_to_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def s_to_stamp(t, msg_stamp):
    msg_stamp.sec = int(np.floor(t))
    msg_stamp.nanosec = int(round((t - np.floor(t)) * 1e9)) % 1_000_000_000
    return msg_stamp


class CloudFilter(Node):
    def __init__(self, **kwargs):
        super().__init__('cloud_filter', **kwargs)
        p = self.declare_parameter
        self.expected_frame = p('expected_frame', 'lidar_link').value
        self.cfg = fc.FilterConfig(
            range_min=p('range_min', 0.3).value, range_max=p('range_max', 40.0).value,
            z_min=p('z_min', -1.5).value, z_max=p('z_max', 3.0).value,
            self_box_enabled=p('self_box_enabled', True).value,
            self_box_min=tuple(p('self_box_min', [-0.25, -0.35, -0.60]).value),
            self_box_max=tuple(p('self_box_max', [0.20, 0.35, 0.05]).value),
            voxel_size=p('voxel_size', 0.05).value)
        self.deskew = p('deskew', False).value
        self.deskew_reference = p('deskew_reference', 'end').value
        self.imu_time_offset_s = p('imu_time_offset_s', 0.0).value
        imu_rpy = p('imu_to_cloud_rpy', [0.0, 0.0, 0.0]).value
        self.imu_to_cloud = fc.rpy_to_matrix(*imu_rpy)
        self.stats_period_s = p('stats_period_s', 10.0).value
        input_topic = p('input_topic', 'sensors/lidar/points').value
        output_topic = p('output_topic', 'sensors/lidar/points_filtered').value
        imu_topic = p('imu_topic', 'sensors/imu/data').value
        # Reliable by default: a best-effort reader dropped every ~1 MB replayed cloud (lost UDP fragments
        # are not retransmitted; host net.core.rmem_max 208 KiB). Use best_effort for best-effort drivers.
        input_reliability = p('input_reliability', 'reliable').value

        out_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, output_topic, out_qos)
        if input_reliability not in ('reliable', 'best_effort'):
            raise ValueError(f'input_reliability must be reliable or best_effort, got {input_reliability!r}')
        in_qos = qos_profile_sensor_data if input_reliability == 'best_effort' else \
            QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(PointCloud2, input_topic, self.on_cloud, in_qos)
        self.imu = collections.deque(maxlen=400)
        if self.deskew:
            self.imu_sub = self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.fields = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in fc.OUT_FIELDS]
        self._frame_warned = False
        self._window = []
        self._window_start = time.monotonic()
        self.totals = collections.Counter()
        self.get_logger().info(
            f'{input_topic} -> {output_topic}: range {self.cfg.range_min}-{self.cfg.range_max} m, '
            f'z {self.cfg.z_min}..{self.cfg.z_max} m, self box {self.cfg.self_box_enabled} '
            f'{list(self.cfg.self_box_min)}..{list(self.cfg.self_box_max)}, voxel {self.cfg.voxel_size} m, '
            f'deskew {self.deskew} (frame {self.expected_frame}; PROPOSED values)')

    def on_imu(self, msg):
        w = self.imu_to_cloud @ np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.imu.append((stamp_to_s(msg.header.stamp) + self.imu_time_offset_s, *w))

    def process(self, msg):
        """Filter one PointCloud2; returns (output PointCloud2, FilterStats, seconds of CPU)."""
        t_cpu = time.process_time()
        stamp = stamp_to_s(msg.header.stamp)
        if msg.header.frame_id != self.expected_frame and not self._frame_warned:
            self.get_logger().warning(f'cloud frame {msg.header.frame_id!r} != expected {self.expected_frame!r}; '
                                      'self box is applied in the cloud frame')
            self._frame_warned = True
        fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
        n = msg.width * msg.height
        if n and not {'x', 'y', 'z'} <= {f[0] for f in fields}:
            raise ValueError('PointCloud2 without x/y/z fields')
        xyz, intensity, ring, t, has_time = fc.parse(msg.data, fields, msg.point_step, n, stamp, msg.is_bigendian)
        omega, t_ref = None, None
        if self.deskew and has_time and n:
            finite_t = t[np.isfinite(t)]
            span = float(finite_t.max()) if finite_t.size else 0.0
            omega = fc.mean_angular_velocity(list(self.imu), stamp, stamp + span)
            t_ref = span if self.deskew_reference == 'end' else 0.0
            if omega is None:
                self.totals['deskew_skipped_no_imu'] += 1
        packed, stats = fc.filter_cloud(xyz, intensity, ring, t, self.cfg, omega, t_ref)
        out = PointCloud2()
        out.header.frame_id = msg.header.frame_id
        if stats.deskewed:
            s_to_stamp(stamp + t_ref, out.header.stamp)
        else:
            out.header.stamp = msg.header.stamp
        out.height, out.width = 1, len(packed)
        out.fields = self.fields
        out.is_bigendian = False
        out.point_step = fc.OUT_DTYPE.itemsize
        out.row_step = out.point_step * out.width
        out.data = array.array('B', packed.tobytes())
        out.is_dense = True
        return out, stats, time.process_time() - t_cpu

    def on_cloud(self, msg):
        try:
            out, stats, cpu_s = self.process(msg)
        except ValueError as exc:
            self.totals['rejected'] += 1
            self.get_logger().error(f'rejected cloud: {exc}', throttle_duration_sec=5.0)
            return
        self.pub.publish(out)
        self.totals['frames'] += 1
        self._window.append((stats.n_in, stats.n_out, stats.n_self, cpu_s))
        now = time.monotonic()
        if self.stats_period_s > 0 and now - self._window_start >= self.stats_period_s:
            w = np.asarray(self._window, dtype=float)
            self.get_logger().info(
                f'{len(w)} frames in {now - self._window_start:.1f} s: points in {w[:, 0].mean():.0f} '
                f'-> out {w[:, 1].mean():.0f} (self box removed {w[:, 2].mean():.0f}); '
                f'cpu/frame mean {1e3 * w[:, 3].mean():.1f} ms max {1e3 * w[:, 3].max():.1f} ms')
            self._window, self._window_start = [], now


def main(args=None):
    rclpy.init(args=args)
    node = CloudFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok() or str(exc) != 'Unable to convert call argument to Python object (compile in debug mode for details)':
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
