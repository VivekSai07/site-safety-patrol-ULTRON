"""Sub-launch `mission` (owner: Agent E).

mode:=mission starts perception (ladder_detector, ladder_localizer, ladder_tracker) and the mission
nodes (mission_manager, capture_coordinator). Any other mode keeps the silent `mission_placeholder`
so mapping/localization/nav graphs are unchanged. Relative topic names (mode namespace).

Detector backend: sim => oracle_sim (simulation ground truth, every detection labelled ORACLE);
replay/live => yolo, which reports UNAVAILABLE unless weights are supplied locally (not in repo).
The mission never publishes velocities; it moves only through nav/navigate_to_view (Agent D).
Extra arguments (zone_file, route_file, memory_db, run_dir, evidence_dir, ladder_backend, weights)
can only be set when this file is launched directly; the stack passes the sub-launch contract only.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration as LC
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from ultron_b2_bringup.launch_common import PKG, SUB_ARGS

EXTRA_ARGS = {
    'zone_file': '',            # empty: sim uses the hall zone; live needs mission/set_patrol_zone
    'route_file': '',
    'memory_db': os.path.join('~', '.ultron', 'memory', 'ultron_registry.sqlite'),
    'run_dir': os.path.join('~', '.ultron', 'missions'),
    'evidence_dir': os.path.join('~', '.ultron', 'evidence'),
    'ladder_backend': '',       # empty: oracle_sim in sim, yolo otherwise
    'weights': '',
    'model_dir': '',
}


def _share(pkg, *parts):
    return os.path.join(get_package_share_directory(pkg), *parts)


def _nodes(context):
    get = context.perform_substitution
    mode, run_mode = get(LC('mode')), get(LC('run_mode')) or 'sim'
    if mode != 'mission':
        return [Node(package=PKG, executable='placeholder', name='mission_placeholder', output='screen',
                     parameters=[{'component': 'mission', 'owner': 'E'}])]
    expand = lambda key: os.path.expanduser(get(LC(key)))  # noqa: E731
    backend = get(LC('ladder_backend')) or ('oracle_sim' if run_mode == 'sim' else 'yolo')
    zone_file = expand('zone_file') or (_share('ultron_b2_mission', 'config', 'hall_zone.yaml')
                                        if run_mode == 'sim' else '')
    memory_db = expand('memory_db')
    perception_cfg = _share('ultron_b2_perception', 'config', 'perception_sim.yaml')
    mission_cfg = _share('ultron_b2_mission', 'config', 'mission_sim.yaml')
    detector_params = {'backend': backend, 'run_mode': run_mode, 'weights': expand('weights'),
                       'model_dir': expand('model_dir')}
    if backend == 'oracle_sim':
        detector_params['oracle_ladders_file'] = _share('ultron_b2_perception', 'config', 'oracle_hall_ladders.yaml')
    return [
        Node(package='ultron_b2_perception', executable='ladder_detector', name='ladder_detector', output='screen',
             parameters=[perception_cfg, detector_params]),
        Node(package='ultron_b2_perception', executable='ladder_localizer', name='ladder_localizer', output='screen',
             parameters=[perception_cfg]),
        Node(package='ultron_b2_perception', executable='ladder_tracker', name='ladder_tracker', output='screen',
             parameters=[perception_cfg, {'memory_db': memory_db}]),
        Node(package='ultron_b2_mission', executable='mission_manager', name='mission_manager', output='screen',
             parameters=[mission_cfg, {'zone_file': zone_file, 'route_file': expand('route_file'),
                                       'memory_db': memory_db, 'run_dir': expand('run_dir')}]),
        Node(package='ultron_b2_mission', executable='capture_coordinator', name='capture_coordinator',
             output='screen', parameters=[{'evidence_dir': expand('evidence_dir')}]),
    ]


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
    declared += [DeclareLaunchArgument(k, default_value=v) for k, v in EXTRA_ARGS.items()]
    return LaunchDescription(declared + [OpaqueFunction(function=_nodes)])
