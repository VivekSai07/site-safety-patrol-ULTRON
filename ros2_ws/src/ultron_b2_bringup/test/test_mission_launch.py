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
    manager = next(n for n in nodes if n.node_executable == 'mission_manager')
    assert 'max_inspections_before_explore 3' in _flat(manager)
    params = _flat(manager).split()
    clearance = float(params[params.index('clearance_m') + 1])
    footprint_path = Path(get_package_share_directory('ultron_b2_description')) / 'config' / 'footprint.yaml'
    assert clearance >= yaml.safe_load(footprint_path.read_text())['sim']['radius_m']
    assert clearance >= NAV_DEFAULTS['target_clearance_m']


def test_live_mission_mode_never_uses_oracle():
    try:
        nodes = _entities('mission', 'live')
    except Exception as exc:
        pytest.skip(f'mission/perception packages not installed: {exc}')
    det = next(n for n in nodes if n.node_executable == 'ladder_detector')
    flat = _flat(det)
    assert 'oracle_sim' not in flat and 'oracle_hall_ladders' not in flat and 'yolo' in flat
    manager = next(n for n in nodes if n.node_executable == 'mission_manager')
    assert 'max_inspections_before_explore 0' in _flat(manager)
    assert 'clearance_m' not in _flat(manager)


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
