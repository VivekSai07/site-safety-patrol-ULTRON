"""tf_authority_check: duplicate publishers / parents / ground-truth frames must fail (exit 1)."""
import os
import subprocess
import time

import pytest

from _graph import executable, ros_env

DOMAIN = 73   # test-only localhost domain, isolated from sim (71) / replay (72)


@pytest.fixture(scope='module')
def ros():
    os.environ.update(ros_env(DOMAIN))
    import rclpy
    rclpy.init()
    yield rclpy
    rclpy.shutdown()


def _tf(parent, child, x=0.0):
    from geometry_msgs.msg import TransformStamped
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x = x
    t.transform.rotation.w = 1.0
    return t


def run_checker(rclpy_mod, publishers, duration_s=4.0):
    """publishers: list of (topic, [(parent, child)]); each gets its own node + publisher."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    checker = subprocess.Popen([executable('tf_authority_check'), '--ros-args', '-p', f'duration_s:={duration_s}',
                                '-p', 'report_period_s:=0.2'],
                               env=ros_env(DOMAIN), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    nodes, pubs = [], []
    for i, (topic, edges) in enumerate(publishers):
        node = rclpy_mod.create_node(f'tf_test_pub_{i}_{time.monotonic_ns()}')
        qos = QoSProfile(depth=10)
        if topic == '/tf_static':
            qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        nodes.append(node)
        pubs.append((node.create_publisher(TFMessage, topic, qos), edges, topic))
    try:
        deadline = time.monotonic() + duration_s + 10
        while checker.poll() is None and time.monotonic() < deadline:
            for pub, edges, topic in pubs:
                msg = TFMessage(transforms=[_tf(p, c) for p, c in edges])
                for t in msg.transforms:
                    t.header.stamp = nodes[0].get_clock().now().to_msg()
                pub.publish(msg)
            time.sleep(0.05)
        if checker.poll() is None:
            checker.kill()
        out = checker.communicate()[0]
    finally:
        for node in nodes:
            node.destroy_node()
    return checker.returncode, out


def test_two_publishers_of_odom_to_base_link_fail(ros):
    code, out = run_checker(ros, [('/tf', [('odom', 'base_link')]), ('/tf', [('odom', 'base_link')])])
    assert code == 1, out
    assert 'MULTIPLE_PUBLISHERS' in out and 'base_link' in out


def test_single_authority_per_frame_passes(ros):
    code, out = run_checker(ros, [('/tf', [('odom', 'base_link')]), ('/tf', [('map', 'odom')]),
                                  ('/tf_static', [('base_link', 'lidar_link'), ('base_link', 'imu_link')])])
    assert code == 0, out
    assert 'TF_AUTHORITY_VIOLATION' not in out


def test_ground_truth_frame_fails(ros):
    code, out = run_checker(ros, [('/tf', [('ultron_sim_world', 'base_link')])])
    assert code == 1 and 'GROUND_TRUTH_FRAME' in out, out


def test_multiple_parents_fail_even_from_one_publisher(ros):
    code, out = run_checker(ros, [('/tf', [('odom', 'base_link'), ('map', 'base_link')])])
    assert code == 1 and 'MULTIPLE_PARENTS' in out, out


def test_static_and_dynamic_authority_conflict_fails(ros):
    code, out = run_checker(ros, [('/tf_static', [('base_link', 'lidar_link')]), ('/tf', [('base_link', 'lidar_link')])])
    assert code == 1 and ('STATIC_DYNAMIC_CONFLICT' in out or 'MULTIPLE_PUBLISHERS' in out), out
