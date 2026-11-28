"""KISS-ICP lidar odometry node: PointCloud2 -> odom->base_link TF + nav_msgs/Odometry.

Alternative to MOLA-LO with identical outputs (``state/odom``, TF ``odom -> base_link``).
Uses the kiss-icp 1.2.3 Python package (pip; MIT). Points are transformed into ``base_frame``
with the static TF before registration, so the estimated pose is the base_link pose and the
odom origin is base_link at the first scan. With ``planar`` (default) the published pose keeps
x, y, yaw only (z = roll = pitch = 0) for the 2D stack; the 3D estimate is kept internally.

Frames are processed latest-only in a worker thread; skipped frames are counted and logged.
After a clock discontinuity the stateful estimator must be restarted before publishing again.
"""
import math
import threading
import time

import numpy as np

from .geometry import flatten_pose, matrix_to_quat, transform_matrix, twist_from_delta

_NP_TYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16, 5: np.int32, 6: np.uint32, 7: np.float32,
             8: np.float64}
TIME_FIELDS = ('timestamp', 'time', 't')


def cloud_to_arrays(msg):
    """PointCloud2 -> (xyz float64 (N,3), per-point times float64 (N,) or empty), NaNs removed."""
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.datatype not in _NP_TYPES or f.count != 1:
            continue
        names.append(f.name)
        formats.append(np.dtype(_NP_TYPES[f.datatype]).newbyteorder('>' if msg.is_bigendian else '<'))
        offsets.append(f.offset)
    dtype = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': msg.point_step})
    n = msg.width * msg.height
    data = np.frombuffer(bytes(msg.data), dtype=dtype, count=n) if msg.row_step == msg.width * msg.point_step \
        else np.concatenate([np.frombuffer(bytes(msg.data[r * msg.row_step:(r * msg.row_step) + msg.width * msg.point_step]),
                                           dtype=dtype) for r in range(msg.height)])
    xyz = np.column_stack([data['x'], data['y'], data['z']]).astype(np.float64)
    t_field = next((f for f in TIME_FIELDS if f in names), None)
    times = data[t_field].astype(np.float64) if t_field else np.empty(0)
    ok = np.isfinite(xyz).all(axis=1)
    return xyz[ok], (times[ok] if len(times) else times)


def normalize_times(times):
    """Per-point times -> [0, 1] over the sweep (KISS-ICP deskew convention); empty if unusable."""
    if len(times) == 0 or not np.isfinite(times).all():
        return np.empty(0)
    lo, hi = float(times.min()), float(times.max())
    if hi - lo <= 1e-9:
        return np.empty(0)
    return (times - lo) / (hi - lo)


def range_filter(xyz, times, min_range, max_range):
    r = np.linalg.norm(xyz, axis=1)
    keep = (r >= min_range) & (r <= max_range)
    return xyz[keep], (times[keep] if len(times) else times)


def make_kiss(voxel_size, max_points_per_voxel, deskew, initial_threshold, min_motion_th):
    from kiss_icp.config.config import AdaptiveThresholdConfig, DataConfig, MappingConfig
    from kiss_icp.config.parser import KISSConfig
    from kiss_icp.kiss_icp import KissICP
    config = KISSConfig(data=DataConfig(deskew=deskew, max_range=1e4, min_range=0.0),
                        mapping=MappingConfig(voxel_size=voxel_size, max_points_per_voxel=max_points_per_voxel),
                        adaptive_threshold=AdaptiveThresholdConfig(initial_threshold=initial_threshold,
                                                                   min_motion_th=min_motion_th))
    return KissICP(config=config)


class KissOdometryNode:
    def __init__(self, node):
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.clock import JumpThreshold
        from rclpy.duration import Duration
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from tf2_ros import Buffer, TransformBroadcaster, TransformListener
        self.node = node
        p = lambda name, default: node.declare_parameter(name, default).value
        self.points_topic = p('points_topic', 'sensors/lidar/points_filtered')
        self.odom_frame = p('odom_frame', 'odom')
        self.base_frame = p('base_frame', 'base_link')
        self.publish_tf = p('publish_tf', True)
        self.planar = p('planar', True)
        self.min_range = p('min_range', 1.0)
        self.max_range = p('max_range', 30.0)
        self.max_future_s = float(p('max_future_s', 0.5))
        if not math.isfinite(self.max_future_s) or self.max_future_s < 0:
            raise ValueError('max_future_s must be finite and nonnegative')
        self.kiss = make_kiss(p('voxel_size', 0.3), p('max_points_per_voxel', 20), p('deskew', True),
                              p('initial_threshold', 2.0), p('min_motion_th', 0.1))
        self.Odometry, self.TransformStamped = Odometry, TransformStamped
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.tf_pub = TransformBroadcaster(node)
        self.odom_pub = node.create_publisher(Odometry, p('odom_topic', 'state/odom'),
                                              QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        reliable = p('points_reliable', True)
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE) if reliable else qos_profile_sensor_data
        node.create_subscription(PointCloud2, self.points_topic, self._on_cloud, qos)
        self.sensor_to_base = {}
        self.latest, self.cv = None, threading.Condition()
        self.received = self.processed = self.skipped = 0
        self.proc_times = []
        self.prev = None          # (stamp_s, 4x4 published pose)
        self.state_lock = threading.Lock()
        self.clock_invalidated = False
        self.clock_jump = node.get_clock().create_jump_callback(
            JumpThreshold(min_forward=Duration(seconds=1), min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True),
            pre_callback=self._invalidate_clock)
        self.running = True
        self.worker = threading.Thread(target=self._work, daemon=True)
        self.worker.start()
        node.create_timer(10.0, self._report)
        node.get_logger().info(f'KISS-ICP odometry: {self.points_topic} -> {self.odom_frame}->{self.base_frame} '
                               f'(planar={self.planar})')

    def _on_cloud(self, msg):
        with self.cv:
            self.received += 1
            if self.latest is not None:
                self.skipped += 1
            self.latest = msg
            self.cv.notify()

    def _sensor_pose(self, frame):
        if frame not in self.sensor_to_base:
            from rclpy.time import Time
            tf = self.buffer.lookup_transform(self.base_frame, frame, Time())
            t, q = tf.transform.translation, tf.transform.rotation
            self.sensor_to_base[frame] = transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        return self.sensor_to_base[frame]

    def _work(self):
        while self.running:
            with self.cv:
                while self.latest is None and self.running:
                    self.cv.wait(0.5)
                msg, self.latest = self.latest, None
            if msg is None:
                continue
            try:
                with self.state_lock:
                    if self.clock_invalidated:
                        raise ValueError('clock jumped; restart KISS-ICP odometry')
                    self._process(msg)
            except Exception as exc:  # keep running; a TF not yet available is expected at start
                self.node.get_logger().warn(f'KISS-ICP frame skipped: {exc}', throttle_duration_sec=5.0)

    def _invalidate_clock(self):
        with self.state_lock:
            if self.prev is not None:
                self.clock_invalidated = True

    def _process(self, msg):
        start = time.perf_counter()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if now <= 0 or stamp > now + self.max_future_s:
            raise ValueError('clock unavailable or cloud timestamp in the future')
        if self.prev is not None and stamp <= self.prev[0]:
            raise ValueError('non-increasing cloud timestamp')
        to_base = self._sensor_pose(msg.header.frame_id)
        xyz, times = cloud_to_arrays(msg)
        xyz, times = range_filter(xyz, times, self.min_range, self.max_range)
        if len(xyz) < 100:
            raise ValueError(f'only {len(xyz)} points after range filter')
        pts = xyz @ to_base[:3, :3].T + to_base[:3, 3]
        self.kiss.register_frame(pts, normalize_times(times))
        pose = flatten_pose(self.kiss.last_pose) if self.planar else self.kiss.last_pose.copy()
        if self.prev is None:
            self.prev = (stamp, pose)
            return
        self._publish(msg.header.stamp, stamp, pose)
        self.processed += 1
        self.proc_times.append(time.perf_counter() - start)

    def _publish(self, stamp_msg, stamp, pose):
        qx, qy, qz, qw = matrix_to_quat(pose[:3, :3])
        x, y, z = pose[:3, 3]
        if self.publish_tf:
            tf = self.TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp_msg, self.odom_frame, self.base_frame
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
            r = tf.transform.rotation
            r.x, r.y, r.z, r.w = qx, qy, qz, qw
            self.tf_pub.sendTransform(tf)
        odom = self.Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp_msg, self.odom_frame, self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = x, y, z
        o = odom.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        if self.prev is not None:
            lin, ang = twist_from_delta(self.prev[1], pose, stamp - self.prev[0])
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = lin
            odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = ang
        # KISS-ICP provides no covariance; -1 on the diagonal marks it unknown (REP-145 style).
        odom.pose.covariance[0] = odom.twist.covariance[0] = -1.0
        self.odom_pub.publish(odom)
        self.prev = (stamp, pose)

    def _report(self):
        if not self.proc_times:
            return
        t = sorted(self.proc_times)
        self.node.get_logger().info(
            f'KISS-ICP: received {self.received}, processed {self.processed}, skipped {self.skipped}; '
            f'process time median {1e3 * t[len(t) // 2]:.0f} ms, max {1e3 * t[-1]:.0f} ms')
        self.proc_times = self.proc_times[-200:]

    def close(self):
        self.running = False
        with self.cv:
            self.cv.notify()
        self.worker.join(timeout=2.0)


def main(argv=None):
    from .node_runtime import run
    run('kiss_icp_odometry', KissOdometryNode, argv)
