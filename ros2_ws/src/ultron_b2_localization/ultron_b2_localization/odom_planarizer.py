"""Planar relay for a 3D odometry backend (used for MOLA-LO).

MOLA-LO's ``enforce_planar_motion`` only constrains its state-estimator prediction; the published ICP pose
keeps z/roll/pitch drift (bag replay: |z| up to 0.11 m, roll/pitch up to 2.6 deg in 36 s on a flat floor).
The stack contract is a planar ``odom -> base_link`` (x, y, yaw), identical for every ``odom_source``, so the
MOLA TF is remapped away and this node republishes the flattened pose as the single TF authority plus
``state/odom``. Same stamp as the input; no buffering.
"""
import math

from .geometry import flatten_pose, matrix_to_quat, transform_matrix


def planarize(position, orientation_xyzw):
    """Return ((x, y, 0), (0, 0, qz, qw)) keeping only the heading of a 3D pose."""
    m = flatten_pose(transform_matrix(position, orientation_xyzw))
    qx, qy, qz, qw = matrix_to_quat(m[:3, :3])
    if not all(math.isfinite(v) for v in (*m[:3, 3], qz, qw)):
        raise ValueError('non-finite pose')
    return (float(m[0, 3]), float(m[1, 3]), 0.0), (0.0, 0.0, float(qz), float(qw))


class OdomPlanarizer:
    def __init__(self, node):
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from tf2_ros import TransformBroadcaster

        self.node = node
        self.TransformStamped = TransformStamped
        p = lambda name, default: node.declare_parameter(name, default).value  # noqa: E731
        self.odom_frame = p('odom_frame', 'odom')
        self.base_frame = p('base_frame', 'base_link')
        self.publish_tf = bool(p('publish_tf', True))
        self.tf_pub = TransformBroadcaster(node)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = node.create_publisher(Odometry, p('odom_out', 'state/odom'), qos)
        node.create_subscription(Odometry, p('odom_in', 'mola/odom_raw'), self._on_odom, qos)
        self.max_dz = self.max_tilt = 0.0
        node.create_timer(10.0, self._report)

    def _on_odom(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        try:
            (x, y, z), (_, _, qz, qw) = planarize((pose.position.x, pose.position.y, pose.position.z),
                                                  (q.x, q.y, q.z, q.w))
        except ValueError as exc:
            self.node.get_logger().warn(f'odometry dropped: {exc}', throttle_duration_sec=5.0)
            return
        self.max_dz = max(self.max_dz, abs(pose.position.z))
        self.max_tilt = max(self.max_tilt, 2 * math.degrees(math.asin(min(1.0, math.hypot(q.x, q.y)))))
        msg.header.frame_id, msg.child_frame_id = self.odom_frame, self.base_frame
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        q.x, q.y, q.z, q.w = 0.0, 0.0, qz, qw
        tw = msg.twist.twist
        tw.linear.z = tw.angular.x = tw.angular.y = 0.0
        if self.publish_tf:
            tf = self.TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = msg.header.stamp, self.odom_frame, self.base_frame
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tf_pub.sendTransform(tf)
        self.pub.publish(msg)

    def _report(self):
        self.node.get_logger().info(f'planarized: dropped |z| up to {self.max_dz:.3f} m, '
                                    f'tilt up to {self.max_tilt:.2f} deg', throttle_duration_sec=60.0)


def main(argv=None):
    from .node_runtime import run
    run('odom_planarizer', OdomPlanarizer, argv)


if __name__ == '__main__':
    main()
