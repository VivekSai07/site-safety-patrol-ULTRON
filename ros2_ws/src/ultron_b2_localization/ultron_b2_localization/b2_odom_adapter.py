"""B2 sport-mode odometry adapter (fallback ``odom_source:=b2``): unitree_go/SportModeState ->
nav_msgs/Odometry ``state/odom`` + TF ``odom -> base_link``.

MOCK-TESTED ONLY. Never run against the robot without the authorization and safeguards in
AGENTS.md. Field conventions are TO_VERIFY on the B2 firmware: ``position`` (m) and
``velocity`` (m/s) in the robot's odometry frame, ``imu_state.quaternion`` as [w, x, y, z],
``yaw_speed`` (rad/s), ``stamp`` frequently zero (then the receive time is used).
"""
import math

import numpy as np

from .geometry import matrix_to_quat, quat_to_matrix


class SportModeRejected(ValueError):
    pass


def sportmode_to_odometry(msg, planar=True):
    """Pure conversion. Returns dict(position, orientation_xyzw, linear_body, angular_body, stamp_s or None)."""
    pos = [float(v) for v in msg.position]
    vel = [float(v) for v in msg.velocity]
    w, x, y, z = (float(v) for v in msg.imu_state.quaternion)
    values = pos + vel + [w, x, y, z, float(msg.yaw_speed)]
    if len(pos) != 3 or len(vel) != 3 or not all(math.isfinite(v) for v in values):
        raise SportModeRejected('non-finite or malformed SportModeState')
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if abs(norm - 1.0) > 0.05:
        raise SportModeRejected(f'quaternion norm {norm:.3f} is not ~1')
    r = quat_to_matrix(x, y, z, w)
    if planar:
        yaw = math.atan2(r[1, 0], r[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        pos[2] = 0.0
        vel[2] = 0.0
    linear_body = r.T @ np.asarray(vel)          # Odometry twist is in child_frame_id
    angular_body = [0.0, 0.0, float(msg.yaw_speed)]
    stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
    return {'position': pos, 'orientation_xyzw': list(matrix_to_quat(r)), 'linear_body': linear_body.tolist(),
            'angular_body': angular_body, 'stamp_s': stamp if stamp > 0 else None}


class B2OdomAdapter:
    def __init__(self, node):
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from tf2_ros import TransformBroadcaster
        try:
            from unitree_go.msg import SportModeState
        except ImportError as exc:
            raise SystemExit('b2_odom_adapter: unitree_go messages not found (build unitree_ros2); '
                             f'odom_source:=b2 unavailable: {exc}')
        p = lambda name, default: node.declare_parameter(name, default).value
        self.node = node
        self.odom_frame, self.base_frame = p('odom_frame', 'odom'), p('base_frame', 'base_link')
        self.planar, self.publish_tf = p('planar', True), p('publish_tf', True)
        self.use_msg_stamp = p('use_message_stamp', False)
        self.Odometry, self.TransformStamped = Odometry, TransformStamped
        self.tf_pub = TransformBroadcaster(node)
        self.pub = node.create_publisher(Odometry, p('odom_topic', 'state/odom'),
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        node.create_subscription(SportModeState, p('sportmode_topic', '/sportmodestate'), self._on_state,
                                 qos_profile_sensor_data)
        self.rejected = 0
        node.get_logger().warn('b2_odom_adapter: MOCK-TESTED ONLY; field conventions TO_VERIFY on the robot')

    def _on_state(self, msg):
        try:
            d = sportmode_to_odometry(msg, self.planar)
        except SportModeRejected as exc:
            self.rejected += 1
            self.node.get_logger().warn(f'SportModeState rejected ({self.rejected}): {exc}',
                                        throttle_duration_sec=2.0)
            return
        from rclpy.time import Time
        stamp = (Time(seconds=d['stamp_s']).to_msg() if self.use_msg_stamp and d['stamp_s'] is not None
                 else self.node.get_clock().now().to_msg())
        odom = self.Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, self.odom_frame, self.base_frame
        p, o = odom.pose.pose.position, odom.pose.pose.orientation
        p.x, p.y, p.z = d['position']
        o.x, o.y, o.z, o.w = d['orientation_xyzw']
        t = odom.twist.twist
        t.linear.x, t.linear.y, t.linear.z = d['linear_body']
        t.angular.x, t.angular.y, t.angular.z = d['angular_body']
        odom.pose.covariance[0] = odom.twist.covariance[0] = -1.0   # unknown
        self.pub.publish(odom)
        if self.publish_tf:
            tf = self.TransformStamped()
            tf.header, tf.child_frame_id = odom.header, self.base_frame
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = d['position']
            r = tf.transform.rotation
            r.x, r.y, r.z, r.w = d['orientation_xyzw']
            self.tf_pub.sendTransform(tf)


def main(argv=None):
    from .node_runtime import run
    run('b2_odom_adapter', B2OdomAdapter, argv)
