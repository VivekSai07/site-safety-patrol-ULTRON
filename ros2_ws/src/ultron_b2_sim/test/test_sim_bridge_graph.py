"""Graph test: the contract sim bridge on domain 71 publishes contract topics, reacts to cmd_vel,
renders valid images and serves scenario hooks. SIMULATION ONLY.

The test publishes a Twist directly on ``/ultron/sim/sim/cmd_vel``: in the integrated stack only the control
sim_driver (fed by motion_gate's gated ``cmd_vel``) publishes it; this isolated test stands in for it.
"""
import json
import os
import signal
import subprocess
import time

import pytest

DOMAIN = '71'
NS = '/ultron/sim'


def _env():
    from ament_index_python.packages import get_package_share_directory
    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1', PYTHONUNBUFFERED='1')
    try:
        env['CYCLONEDDS_URI'] = 'file://' + os.path.join(
            get_package_share_directory('ultron_b2_bringup'), 'config', 'dds', 'cyclonedds_localhost.xml')
    except Exception:  # noqa: BLE001
        pass
    return env


@pytest.fixture(scope='module')
def bridge():
    try:
        import rclpy  # noqa: F401
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('ultron_b2_sim')
    except Exception as exc:  # noqa: BLE001
        pytest.skip(f'ROS workspace not sourced: {exc}')
    env = _env()
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY')})
    if 'CYCLONEDDS_URI' in env:
        os.environ['CYCLONEDDS_URI'] = env['CYCLONEDDS_URI']
    proc = subprocess.Popen(['ros2', 'launch', 'ultron_b2_sim', 'sim_bridge.launch.py', f'namespace:={NS}',
                             'people:=none'], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node('sim_bridge_graph_test')
    try:
        yield proc, node
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        out = proc.stdout.read()
        assert 'Traceback' not in out, out[-4000:]


def _spin(node, seconds, cond=None):
    import rclpy
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if cond is not None and cond():
            return True
    return cond() if cond else True


def test_contract_topics_rates_images_and_cmd_vel(bridge):
    proc, node = bridge
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from ultron_b2_interfaces.srv import SimScenario

    specs = {'/clock': Clock, f'{NS}/sensors/lidar/points': PointCloud2, f'{NS}/sensors/scan': LaserScan,
             f'{NS}/sensors/imu/data': Imu, f'{NS}/state/odom': Odometry,
             f'{NS}/sensors/cam_head/color/image_raw': Image, f'{NS}/sensors/cam_head/color/camera_info': CameraInfo,
             f'{NS}/sensors/cam_head/depth/image_rect_raw': Image,
             f'{NS}/sensors/cam_head/depth/camera_info': CameraInfo,
             f'{NS}/sim/oracle/cam_head/ladder_labels': Image}
    stamps = {t: [] for t in specs}
    last = {}
    reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    for topic, typ in specs.items():
        def cb(msg, topic=topic):
            stamps[topic].append(time.monotonic())
            last[topic] = msg
        node.create_subscription(typ, topic, cb, reliable)
    tf_children = set()
    node.create_subscription(TFMessage, '/tf', lambda m: tf_children.update(
        (t.header.frame_id, t.child_frame_id) for t in m.transforms), qos_profile_sensor_data)
    truth = []
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, f'{NS}/sim/ground_truth/ladders', lambda m: truth.append(json.loads(m.data)),
                             latched)

    assert _spin(node, 60, lambda: all(stamps.values())), {t: len(v) for t, v in stamps.items()}
    for v in stamps.values():
        v.clear()
    _spin(node, 5.0)
    rates = {t: (len(v) - 1) / (v[-1] - v[0]) if len(v) > 1 else 0.0 for t, v in stamps.items()}
    print('RATES', json.dumps(rates, indent=1))
    expected = {'/clock': 50, f'{NS}/sensors/lidar/points': 10, f'{NS}/sensors/scan': 10,
                f'{NS}/sensors/imu/data': 50, f'{NS}/state/odom': 50,
                f'{NS}/sensors/cam_head/color/image_raw': 10, f'{NS}/sensors/cam_head/depth/image_rect_raw': 10}
    for topic, hz in expected.items():
        assert rates[topic] > 0.6 * hz, (topic, rates[topic])

    color = last[f'{NS}/sensors/cam_head/color/image_raw']
    depth = last[f'{NS}/sensors/cam_head/depth/image_rect_raw']
    info = last[f'{NS}/sensors/cam_head/color/camera_info']
    assert (color.encoding, color.width, color.height) == ('rgb8', 640, 360)
    assert len(color.data) == color.step * color.height and color.header.frame_id == 'cam_head_color_optical_frame'
    assert (depth.encoding, depth.width, depth.height) == ('16UC1', 424, 240)
    assert info.k[0] == pytest.approx(321.4505) and info.header.frame_id == color.header.frame_id
    import numpy as np
    d = np.frombuffer(bytes(depth.data), np.uint16)
    assert 0.2 < (d > 0).mean() <= 1.0 and d.max() <= 10000
    assert np.frombuffer(bytes(color.data), np.uint8).std() > 5

    # TF: odom->base_link only; no ground-truth frame and no base_link->lidar_link from the bridge.
    assert ('odom', 'base_link') in tf_children, tf_children
    assert not any('ultron_sim_world' in p for pair in tf_children for p in pair)
    assert ('base_link', 'lidar_link') not in tf_children
    assert truth and truth[-1]['provenance'] == 'SIMULATION_GROUND_TRUTH_EVALUATION_ONLY'

    # sim/cmd_vel reaction (test stand-in for sim_driver).
    odom_topic = f'{NS}/state/odom'
    pub = node.create_publisher(Twist, f'{NS}/sim/cmd_vel', reliable)
    start = last[odom_topic].pose.pose.position
    x0, y0 = start.x, start.y
    cmd = Twist()
    cmd.linear.x = 0.3
    t_end = time.monotonic() + 3.0
    while time.monotonic() < t_end:
        pub.publish(cmd)
        _spin(node, 0.1)
    moved_pos = last[odom_topic].pose.pose.position
    moved = ((moved_pos.x - x0) ** 2 + (moved_pos.y - y0) ** 2) ** 0.5
    print('CMD_VEL_MOVED_M', moved, 'TWIST_VX', last[odom_topic].twist.twist.linear.x)
    assert moved > 0.3, moved
    _spin(node, 1.5)   # command timeout -> stop
    assert abs(last[odom_topic].twist.twist.linear.x) < 0.05

    # Scenario hook: hide a ladder -> truth republished with present=False.
    client = node.create_client(SimScenario, f'{NS}/sim/scenario')
    assert client.wait_for_service(timeout_sec=10)
    req = SimScenario.Request(schema_version=SimScenario.Request.SCHEMA_VERSION, command='ladder_hide',
                              target='ladder_07')
    fut = client.call_async(req)
    assert _spin(node, 10, fut.done)
    assert fut.result().accepted, fut.result().message
    assert _spin(node, 5, lambda: any(not l['present'] for l in truth[-1]['ladders'] if l['name'] == 'ladder_07'))
    bad = client.call_async(SimScenario.Request(command='teleport_to_mars'))
    assert _spin(node, 10, bad.done) and not bad.result().accepted
    assert proc.poll() is None
