"""mission sub-launch: placeholder outside mode:=mission; perception + mission nodes in mode:=mission."""
from pathlib import Path
import importlib.util

import pytest
from launch import LaunchContext
from launch.actions import OpaqueFunction

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'sub' / 'mission.launch.py'


def _entities(mode, run_mode):
    spec = importlib.util.spec_from_file_location('mission_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()
    ctx = LaunchContext()
    for arg in ld.get_launch_arguments():
        ctx.launch_configurations[arg.name] = ''.join(s.perform(ctx) for s in arg.default_value)
    ctx.launch_configurations.update(mode=mode, run_mode=run_mode)
    opaque = [e for e in ld.entities if isinstance(e, OpaqueFunction)]
    assert len(opaque) == 1
    return opaque[0].execute(ctx)


def _flat(node):
    """Parameter files and dict values of a launch Node, performed to plain text."""
    ctx = LaunchContext()
    out = []
    for p in node._Node__parameters:
        if isinstance(p, dict):
            for k, v in p.items():
                value = ''.join(s.perform(ctx) for s in v) if isinstance(v, tuple) else str(v)
                out += [''.join(s.perform(ctx) for s in k), value]
        else:
            out.append(str(p.param_file[0].perform(ctx) if isinstance(p.param_file, list) else p.param_file))
    return ' '.join(out)




@pytest.mark.parametrize('mode', ['mapping', 'localization', 'nav'])
def test_non_mission_modes_keep_placeholder(mode):
    nodes = _entities(mode, 'sim')
    assert [n.node_executable for n in nodes] == ['placeholder']


def test_sim_mission_mode_starts_perception_and_mission_with_oracle():
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from ultron_b2_navigation.nav_contract_server import NAV_DEFAULTS

    pytest.importorskip('ament_index_python')
    try:
        nodes = _entities('mission', 'sim')
    except Exception as exc:  # packages not installed in this environment
        pytest.skip(f'mission/perception packages not installed: {exc}')
    exe = sorted(n.node_executable for n in nodes)
    assert exe == ['capture_coordinator', 'ladder_detector', 'ladder_localizer', 'ladder_tracker', 'mission_manager']
    det = next(n for n in nodes if n.node_executable == 'ladder_detector')
    flat = _flat(det)
    assert 'oracle_sim' in flat and 'oracle_hall_ladders.yaml' in flat
    localizer = next(n for n in nodes if n.node_executable == 'ladder_localizer')
    assert 'depth_registered_to_color True' in _flat(localizer)
    assert 'oracle_depth_masks True' in _flat(localizer)
    manager = next(n for n in nodes if n.node_executable == 'mission_manager')
    assert 'max_inspections_before_explore 3' in _flat(manager)
    assert 'frontier_goal_search_m 2.0' in _flat(manager)
    params = _flat(manager).split()
    clearance = float(params[params.index('clearance_m') + 1])
    footprint_path = Path(get_package_share_directory('ultron_b2_description')) / 'config' / 'footprint.yaml'
    footprint = yaml.safe_load(footprint_path.read_text())['sim']
    assert clearance == footprint['planning_radius_m']
    assert clearance >= footprint['radius_m']
    collision_path = (Path(get_package_share_directory('ultron_b2_control')) / 'config'
                      / 'collision_monitor_sim_override.yaml')
    collision = yaml.safe_load(collision_path.read_text())['/**/collision_monitor']['ros__parameters']
    assert clearance > collision['FootprintApproach.radius']
    assert clearance >= NAV_DEFAULTS['target_clearance_m']


def test_sim_nav_and_mission_use_same_conservative_clearance(tmp_path, monkeypatch):
    import yaml
    from ament_index_python.packages import get_package_share_directory

    nav_path = Path(get_package_share_directory('ultron_b2_navigation')) / 'launch' / 'navigation.launch.py'
    spec = importlib.util.spec_from_file_location('navigation_launch', nav_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv('ROS_HOME', str(tmp_path))
    path = module.render_params('sim', '/ultron/sim', '/ultron/sim/map', False)
    config = yaml.safe_load(Path(path).read_text())
    nodes = _entities('mission', 'sim')
    params = _flat(next(n for n in nodes if n.node_executable == 'mission_manager')).split()
    clearance = float(params[params.index('clearance_m') + 1])
    for name in ('local_costmap', 'global_costmap'):
        costmap = config[f'/ultron/sim/{name}'][name]['ros__parameters']
        assert costmap['robot_radius'] == clearance


def test_sim_nav_goal_checker_is_no_looser_than_mission():
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from ultron_b2_mission.mission_core import MissionConfig

    nav = Path(get_package_share_directory('ultron_b2_navigation')) / 'config' / 'nav2_sim.yaml'
    goal = yaml.safe_load(nav.read_text())['controller_server']['ros__parameters']['general_goal_checker']
    mission = MissionConfig()
    assert goal['xy_goal_tolerance'] <= mission.nav_xy_tolerance_m
    assert goal['yaw_goal_tolerance'] <= mission.nav_yaw_tolerance_rad


@pytest.mark.parametrize(('radius', 'planning'), [(0.55, 0.4), (0.55, float('nan')), (float('nan'), 0.9)])
def test_nav_refuses_invalid_sim_clearance(tmp_path, monkeypatch, radius, planning):
    import yaml
    from ament_index_python.packages import get_package_share_directory

    nav_path = Path(get_package_share_directory('ultron_b2_navigation')) / 'launch' / 'navigation.launch.py'
    spec = importlib.util.spec_from_file_location('navigation_launch_invalid', nav_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config' / 'footprint.yaml').write_text(yaml.safe_dump({
        'sim': {'type': 'circle', 'radius_m': radius, 'planning_radius_m': planning}}))
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    with pytest.raises(RuntimeError):
        module.planning_radius('sim')


def test_live_mission_mode_never_uses_oracle():
    try:
        nodes = _entities('mission', 'live')
    except Exception as exc:
        pytest.skip(f'mission/perception packages not installed: {exc}')
    det = next(n for n in nodes if n.node_executable == 'ladder_detector')
    flat = _flat(det)
    assert 'oracle_sim' not in flat and 'oracle_hall_ladders' not in flat and 'yolo' in flat
    localizer = next(n for n in nodes if n.node_executable == 'ladder_localizer')
    assert 'depth_registered_to_color False' in _flat(localizer)
    assert 'oracle_depth_masks False' in _flat(localizer)
    manager = next(n for n in nodes if n.node_executable == 'mission_manager')
    assert 'max_inspections_before_explore 0' in _flat(manager)
    assert 'clearance_m' not in _flat(manager)
    assert 'frontier_goal_search_m' not in _flat(manager)


def test_localizer_rejects_unregistered_depth_on_ros_graph():
    import json
    import os
    import time
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from ultron_b2_interfaces.msg import LadderCandidateArray, LadderDetection, LadderDetectionArray, LocalizationState
    from _graph import executable, ros_env, start, stop

    env = ros_env(71)
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'CYCLONEDDS_URI')})
    proc = start([executable('ladder_localizer', 'ultron_b2_perception'),
                  '--ros-args', '-r', '__ns:=/ultron/sim'], env)
    rclpy.init()
    node = Node('unregistered_depth_probe', namespace='/ultron/sim')
    loc_pub = node.create_publisher(LocalizationState, 'state/localization', 10)
    det_pub = node.create_publisher(LadderDetectionArray, 'perception/ladder_detections', 10)
    stats, observations = [], []
    node.create_subscription(String, 'perception/ladder_localizer/stats',
                             lambda m: stats.append(json.loads(m.data)), 10)
    node.create_subscription(LadderCandidateArray, 'perception/ladder_observations',
                             observations.append, 10)
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and proc.poll() is None:
            stamp = node.get_clock().now().to_msg()
            loc = LocalizationState(state=LocalizationState.STATE_VALID)
            loc.header.stamp = stamp
            det = LadderDetectionArray(detections=[LadderDetection()])
            det.header.stamp = stamp
            loc_pub.publish(loc)
            det_pub.publish(det)
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(s['drops'].get('DEPTH_NOT_REGISTERED', 0) for s in stats):
                break
        assert any(s['drops'].get('DEPTH_NOT_REGISTERED', 0) for s in stats), stats
        assert not observations
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop(proc)


def test_oracle_mask_rejected_outside_sim():
    import subprocess
    from _graph import executable, ros_env

    result = subprocess.run([executable('ladder_localizer', 'ultron_b2_perception'), '--ros-args',
                             '-p', 'run_mode:=live', '-p', 'oracle_depth_masks:=true'],
                            env=ros_env(71), capture_output=True, text=True, timeout=15)
    assert result.returncode != 0
    assert 'oracle depth masks are restricted to simulation' in result.stderr


def test_oracle_localizer_waits_for_matching_frames_and_uses_mask():
    import json
    import os
    import time
    import numpy as np
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster
    from ultron_b2_interfaces.msg import (LadderCandidate, LadderCandidateArray, LadderDetection,
                                          LadderDetectionArray, LocalizationState)
    from ultron_b2_perception.ros_util import numpy_to_image
    from _graph import executable, ros_env, start, stop

    env = ros_env(71)
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'CYCLONEDDS_URI')})
    proc = start([executable('ladder_localizer', 'ultron_b2_perception'), '--ros-args',
                  '-r', '__ns:=/ultron/sim', '-p', 'run_mode:=sim', '-p', 'oracle_depth_masks:=true',
                  '-p', 'depth_registered_to_color:=true', '-p', 'max_image_age_s:=10.0',
                  '-p', 'localization_max_age_s:=10.0'], env)
    rclpy.init()
    node = Node('oracle_mask_probe', namespace='/ultron/sim')
    loc_pub = node.create_publisher(LocalizationState, 'state/localization', 10)
    det_pub = node.create_publisher(LadderDetectionArray, 'perception/ladder_detections', 10)
    label_pub = node.create_publisher(Image, 'sim/oracle/cam_head/ladder_labels', 10)
    depth_pub = node.create_publisher(Image, 'sensors/cam_head/depth/image_rect_raw', 10)
    color_info_pub = node.create_publisher(CameraInfo, 'sensors/cam_head/color/camera_info', 10)
    depth_info_pub = node.create_publisher(CameraInfo, 'sensors/cam_head/depth/camera_info', 10)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    candidate_pub = node.create_publisher(LadderCandidateArray, 'perception/ladder_candidates', latched)
    observations, stats = [], []
    node.create_subscription(LadderCandidateArray, 'perception/ladder_observations', observations.append, 10)
    node.create_subscription(String, 'perception/ladder_localizer/stats',
                             lambda m: stats.append(json.loads(m.data)), 10)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'camera'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    try:
        pubs = (loc_pub, det_pub, label_pub, depth_pub, color_info_pub, depth_info_pub, candidate_pub)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not all(p.get_subscription_count() for p in pubs):
            rclpy.spin_once(node, timeout_sec=0.1)
        assert all(p.get_subscription_count() for p in pubs), 'localizer did not subscribe'
        stamp = node.get_clock().now().to_msg()
        loc = LocalizationState(state=LocalizationState.STATE_VALID)
        loc.header.stamp = stamp
        info = CameraInfo(width=100, height=100, k=[100.0, 0.0, 49.5, 0.0, 100.0, 49.5, 0.0, 0.0, 1.0])
        info.header.stamp, info.header.frame_id = stamp, 'camera'
        labels = np.zeros((100, 100), dtype=np.uint16)
        labels[20:80, 20:80] = 3
        labels[30:70, 30:70] = 0
        depth = np.full((100, 100), 6.0, dtype=np.float32)
        depth[labels == 3] = 4.0
        depth[40:60, 40:60] = 2.0
        label_msg = numpy_to_image(labels, '16UC1', stamp, 'camera')
        depth_msg = numpy_to_image(depth, '32FC1', stamp, 'camera')
        wrong_stamp = Time(sec=stamp.sec + (stamp.nanosec + 1) // 1_000_000_000,
                           nanosec=(stamp.nanosec + 1) % 1_000_000_000)
        wrong_depth = numpy_to_image(depth, '32FC1', wrong_stamp, 'camera')
        detection = LadderDetection(backend='oracle_sim', is_oracle=True, label='ORACLE_SIM:TRESTLE',
                                    image_width=100, image_height=100, bbox_x_min=20.0, bbox_y_min=20.0,
                                    bbox_x_max=80.0, bbox_y_max=80.0)
        det = LadderDetectionArray(detections=[detection])
        det.header.stamp, det.header.frame_id = stamp, 'camera'
        loc_pub.publish(loc)
        color_info_pub.publish(info)
        depth_info_pub.publish(info)
        label_pub.publish(label_msg)
        depth_pub.publish(wrong_depth)
        det_pub.publish(det)
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert not observations, 'accepted a depth frame with the wrong timestamp'
        depth_pub.publish(depth_msg)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not observations:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert observations and len(observations[0].candidates) == 1
        candidate = observations[0].candidates[0]
        assert candidate.from_oracle and candidate.position.z == pytest.approx(4.0)

        bad_info = CameraInfo(width=100, height=100, k=list(info.k))
        bad_info.k[2] = float('nan')
        color_info_pub.publish(bad_info)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        stamp = node.get_clock().now().to_msg()
        loc.header.stamp, det.header.stamp = stamp, stamp
        loc_pub.publish(loc)
        depth_pub.publish(numpy_to_image(depth, '32FC1', stamp, 'camera'))
        label_pub.publish(numpy_to_image(labels, '16UC1', stamp, 'camera'))
        det_pub.publish(det)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not any(s['drops'].get('NO_CAMERA_INFO', 0) for s in stats):
            rclpy.spin_once(node, timeout_sec=0.1)
        assert any(s['drops'].get('NO_CAMERA_INFO', 0) for s in stats)
        assert len(observations) == 1, 'invalid calibration reused a previous camera matrix'
        color_info_pub.publish(info)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)

        known = LadderCandidate(candidate_id='known')
        known.position.x, known.position.z = 1.0, 4.0
        candidate_pub.publish(LadderCandidateArray(candidates=[known]))
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        stamp = node.get_clock().now().to_msg()
        loc.header.stamp = stamp
        det.header.stamp = stamp
        det.detections[0].bbox_x_min = 19.0  # no instance has this box
        loc_pub.publish(loc)
        depth_pub.publish(numpy_to_image(depth, '32FC1', stamp, 'camera'))
        label_pub.publish(numpy_to_image(labels, '16UC1', stamp, 'camera'))
        det_pub.publish(det)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and len(observations) < 2:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert len(observations) >= 2
        assert not any(c.association_state == 'VIEW_CHECK' for c in observations[-1].candidates), \
            'an unlocalized detection cannot justify absence checks'
    finally:
        node.destroy_node()
        rclpy.shutdown()
        out = stop(proc)
    assert 'Traceback' not in out, out


def test_sim_mission_mode_graph_starts_mission_nodes_without_errors():
    """Graph check: perception + mission nodes come up under /ultron/sim with no tracebacks.

    C/D/F are stubs here, so the mission just idles (no map/localization => no goals)."""
    import time

    from _graph import node_list, ros_env, start, stop
    expected = {'ladder_detector', 'ladder_localizer', 'ladder_tracker', 'mission_manager', 'capture_coordinator'}
    env = ros_env(71)
    env['HOME'] = str(Path(__file__).resolve().parent / '.artifacts' / 'home')  # registry/missions scratch
    proc = start(['ros2', 'launch', 'ultron_b2_bringup', 'sim.launch.py', 'mode:=mission'], env)
    time.sleep(6.0)
    ns = '/ultron/sim/'
    nodes = node_list(env, until=lambda n: proc.poll() is not None
                      or expected <= {x[len(ns):] for x in n if x.startswith(ns)}) if proc.poll() is None else []
    alive = proc.poll() is None
    out = stop(proc)
    started = {x[len(ns):] for x in nodes if x.startswith(ns)}
    assert alive, out
    assert expected <= started and 'mission_placeholder' not in started, (nodes, out)
    assert 'Traceback' not in out and '[ERROR]' not in out, out
    assert 'ORACLE' in out, out  # the detector announces its oracle labelling in sim


def test_sim_mission_detector_receives_rendered_ladder_labels():
    """The actual simulator publishes synchronized labels consumed by the detector."""
    import json
    import os
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from ultron_b2_interfaces.msg import LadderDetectionArray
    from ament_index_python.packages import get_package_share_directory
    from ultron_b2_perception.ladder_detector import frame_stamp_ns, load_oracle_ladders, oracle_segmentation_ids
    from ultron_b2_perception.oracle import segmentation_boxes
    from ultron_b2_perception.ros_util import image_to_numpy
    from _graph import ros_env, start, stop

    env = ros_env(71)
    env['HOME'] = str(Path(__file__).resolve().parent / '.artifacts' / 'segmentation_home')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'CYCLONEDDS_URI')})
    proc = start(['ros2', 'launch', 'ultron_b2_bringup', 'sim.launch.py', 'mode:=mission'], env)
    rclpy.init()
    node = Node('sim_detector_probe', namespace='/ultron/sim')
    detections, labels = {}, {}
    node.create_subscription(LadderDetectionArray, 'perception/ladder_detections',
                             lambda m: detections.update({frame_stamp_ns(m): m}), 10)
    node.create_subscription(Image, 'sim/oracle/cam_head/ladder_labels',
                             lambda m: labels.update({frame_stamp_ns(m): image_to_numpy(m)}), 10)
    truth = []
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, 'sim/ground_truth/ladders',
                             lambda m: truth.append(json.loads(m.data)), latched)
    cfg = Path(get_package_share_directory('ultron_b2_perception')) / 'config/oracle_hall_ladders.yaml'
    ids = oracle_segmentation_ids(load_oracle_ladders(cfg))
    try:
        deadline = time.monotonic() + 40
        matched = []
        while time.monotonic() < deadline and proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            matched = [stamp for stamp in detections.keys() & labels.keys() if detections[stamp].detections]
            if matched and truth:
                break
        assert matched, 'no synchronized simulator oracle detections received'
        assert truth, 'no simulator ladder truth received'
        assert {item['label']: (item['name'], item['kind']) for item in truth[-1]['ladders']} == \
            {label: (item['name'], item['type']) for label, item in ids.items()}
        stamp = matched[0]
        expected = segmentation_boxes(labels[stamp], ids)
        actual = detections[stamp].detections
        assert [(d.label, (d.bbox_x_min, d.bbox_y_min, d.bbox_x_max, d.bbox_y_max)) for d in actual] == \
            [(d['label'], d['bbox']) for d in expected]
        assert all(d.is_oracle and d.label.startswith('ORACLE_SIM:')
                   for m in detections.values() for d in m.detections)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        out = stop(proc)
    assert 'Traceback' not in out, out[-4000:]
