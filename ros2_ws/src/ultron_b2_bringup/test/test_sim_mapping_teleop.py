"""Simulation-only initial mapping teleop; never connects to a robot."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace

import pytest
from launch import LaunchContext

from _graph import ros_env, start, stop


def test_mapping_scan_validation():
    from sensor_msgs.msg import LaserScan
    from ultron_b2_navigation.motion_gate import MotionGateNode

    node = SimpleNamespace(gate=SimpleNamespace(scan_stamp=None))
    scan = LaserScan()
    scan.header.frame_id = 'base_footprint'
    scan.header.stamp.sec = 10
    scan.range_min, scan.range_max = 0.1, 10.0
    scan.ranges = [float('inf'), 2.0]
    MotionGateNode._on_scan(node, scan)
    assert node.gate.scan_stamp == 10.0
    scan.ranges = [float('nan'), float('inf')]
    MotionGateNode._on_scan(node, scan)
    assert node.gate.scan_stamp is None
    scan.ranges = [2.0]
    scan.header.frame_id = 'wrong_frame'
    MotionGateNode._on_scan(node, scan)
    assert node.gate.scan_stamp is None


@pytest.mark.parametrize(('run_mode', 'mode', 'expected'), [
    ('sim', 'mapping', 'false'),
    ('sim', 'mission', 'true'),
    ('live', 'mapping', 'true'),
])
def test_mapping_teleop_override_is_sim_only(run_mode, mode, expected):
    path = Path(__file__).resolve().parents[1] / 'launch' / 'sub' / 'control.launch.py'
    spec = importlib.util.spec_from_file_location('control_sublaunch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ctx = LaunchContext()
    ctx.launch_configurations.update(run_mode=run_mode, mode=mode, sim_teleop_without_localization='true')
    include = module._include(ctx)[0]
    assert dict(include.launch_arguments)['teleop_requires_localization'] == expected
    assert dict(include.launch_arguments)['teleop_requires_fresh_scan'] == ('true' if expected == 'false' else 'false')


def test_mapping_teleop_moves_only_with_operator_heartbeat(tmp_path):
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from ultron_b2_interfaces.msg import MotionGateStatus, OperatorEnable
    from ultron_b2_interfaces.srv import SaveMap, SimScenario

    env = ros_env(71)
    env.update(HOME=str(tmp_path), ROS_HOME=str(tmp_path / '.ros'), ROS_LOG_DIR=str(tmp_path / 'ros-log'))
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'CYCLONEDDS_URI')})
    proc = start(['ros2', 'launch', 'ultron_b2_bringup', 'sim.launch.py', 'mode:=mapping',
                  'odom_source:=sim', 'sim_teleop_without_localization:=true'], env)
    rclpy.init()
    node = Node('sim_mapping_teleop_probe', namespace='/ultron/sim')
    op_pub = node.create_publisher(OperatorEnable, 'ops/operator_enable', 10)
    cmd_pub = node.create_publisher(Twist, 'control/in/teleop', 10)
    odom, status = [], []
    truth, bridge = [], []
    node.create_subscription(Odometry, 'state/odom', odom.append, 10)
    node.create_subscription(MotionGateStatus, 'safety/motion_gate', status.append, 10)
    node.create_subscription(PoseStamped, 'sim/ground_truth/base_pose', truth.append, 10)
    node.create_subscription(String, 'sim/bridge_status', lambda msg: bridge.append(json.loads(msg.data)),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    cmd = Twist()
    cmd.linear.x = 0.1
    heartbeat = OperatorEnable(schema_version=OperatorEnable.SCHEMA_VERSION, enabled=True,
                               operator_id='simulation-test')
    try:
        deadline = time.monotonic() + 40
        start_x = None
        while time.monotonic() < deadline and proc.poll() is None:
            op_pub.publish(heartbeat)
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.1)
            if odom and start_x is None:
                start_x = odom[-1].pose.pose.position.x
            if (start_x is not None and odom[-1].pose.pose.position.x - start_x > 0.06
                    and any(s.open and s.active_source == 'teleop'
                            and not s.teleop_requires_localization for s in status)):
                break
        assert start_x is not None and odom[-1].pose.pose.position.x - start_x > 0.06
        assert any(s.open and s.active_source == 'teleop'
                   and not s.teleop_requires_localization for s in status)

        scenario = node.create_client(SimScenario, 'sim/scenario')
        assert scenario.wait_for_service(timeout_sec=5)
        future = scenario.call_async(SimScenario.Request(command='lidar_blackout', values=[4.0]))
        status.clear()
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and proc.poll() is None:
            op_pub.publish(heartbeat)
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.1)
            if future.done() and any(s.inhibited_reason == 'SCAN_MISSING_OR_STALE'
                                     and s.cmd_out.linear.x == 0.0 for s in status):
                break
        assert future.done() and future.result().accepted
        assert any(s.inhibited_reason == 'SCAN_MISSING_OR_STALE' and s.cmd_out.linear.x == 0.0 for s in status)

        status.clear()
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and proc.poll() is None:
            op_pub.publish(heartbeat)
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(s.open and s.active_source == 'teleop' for s in status):
                break
        assert any(s.open and s.active_source == 'teleop' for s in status)

        status.clear()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and proc.poll() is None:
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(s.inhibited_reason == 'OPERATOR_ENABLE_STALE' and s.cmd_out.linear.x == 0.0 for s in status):
                break
        assert any(s.inhibited_reason == 'OPERATOR_ENABLE_STALE' and s.cmd_out.linear.x == 0.0 for s in status)

        capture = tmp_path / 'manual-capture'
        snapshot = subprocess.run(['ros2', 'run', 'ultron_b2_tools', 'sim_snapshot',
                                   '--output', str(capture)], env=env, capture_output=True, text=True, timeout=30)
        assert snapshot.returncode == 0, snapshot.stdout + snapshot.stderr
        meta = json.loads((capture / 'metadata.json').read_text())
        assert meta['settled'] == 'NOT_ASSESSED' and meta['provenance'] == 'SIM_TOPIC_UNVERIFIED'
        from PIL import Image
        with Image.open(capture / 'color.png') as png:
            assert png.size == (meta['width'], meta['height']) and png.mode == 'RGB'
            assert any(low < high for low, high in png.getextrema())

        save = node.create_client(SaveMap, 'map/save')
        assert save.wait_for_service(timeout_sec=10), 'map/save not available'
        request = SaveMap.Request(schema_version=SaveMap.Request.SCHEMA_VERSION, map_id='sim-teleop-test',
                                  directory=str(tmp_path / 'maps'), save_3d=False)
        future = save.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        assert future.done() and future.result().success, future.result() if future.done() else 'save timed out'
        assert (tmp_path / 'maps/sim-teleop-test/v1/map.posegraph').is_file()

        assert bridge, 'sim bridge status unavailable'
        before_collisions = bridge[-1]['collisions']
        yaw = 2.2521810496021537
        import math
        x, y = 5.559030494681202 - 0.25 * math.cos(yaw), 8.5499715535162 - 0.25 * math.sin(yaw)
        future = scenario.call_async(SimScenario.Request(command='kidnap', values=[x, y, yaw]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done() and future.result().accepted
        status.clear()
        truth.clear()
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline and proc.poll() is None:
            op_pub.publish(heartbeat)
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.1)
        assert truth and bridge
        velocity = [s.cmd_out.linear.x for s in status if s.open and s.active_source == 'teleop']
        moving = next((i for i, v in enumerate(velocity) if v > 0.01), None)
        assert moving is not None and any(v < 0.05 for v in velocity[moving + 1:]), velocity[-20:]
        assert math.hypot(truth[-1].pose.position.x - x, truth[-1].pose.position.y - y) > 0.01
        end = truth[-1].header.stamp.sec + truth[-1].header.stamp.nanosec * 1e-9
        cmd.linear.x = 0.0
        deadline = time.monotonic() + 4
        while bridge[-1]['sim_time_s'] < end and time.monotonic() < deadline and proc.poll() is None:
            op_pub.publish(heartbeat)
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.1)
        assert bridge[-1]['sim_time_s'] >= end
        assert bridge[-1]['collisions'] == before_collisions, bridge[-1]
    finally:
        node.destroy_node()
        rclpy.shutdown()
        output = stop(proc)
    assert 'Traceback' not in output, output
