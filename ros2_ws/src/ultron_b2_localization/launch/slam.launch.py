"""2D SLAM / localization + LocalizationState + map manager (owner: Agent C).

- ``mode:=mapping``: slam_toolbox online-async mapping; localization_monitor scores scans against
  the live map (consistency only); map_manager saves on ``map/save``.
- ``mode:=localization|nav|mission``: slam_toolbox localization on ``map:=`` (``map_id``,
  ``map_id:version`` or a map directory / map.yaml under ``map_root``). The map's hashes are
  verified here before slam_toolbox gets it; map_manager announces it on ``map/info`` and
  localization_monitor scores against it. Without ``map:=`` no localization backend starts and
  the state stays INITIALIZING ("No map loaded") until ``map/load`` is called.

Outputs (relative): ``map`` (OccupancyGrid), TF ``map -> odom``, ``state/localization``,
``map/info``; services ``map/save``, ``map/load``.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'ultron_b2_localization'


def resolve_map(ref, map_root):
    from ultron_b2_localization import map_store
    from ultron_b2_localization.map_manager_node import parse_map_ref
    map_id, version, path = parse_map_ref(ref)
    if path and os.path.isdir(path):
        path = os.path.join(path, 'map.yaml')
    directory = map_store.resolve(map_root, map_id, version, path)
    problems = map_store.verify(directory)
    if problems:
        raise RuntimeError(f'map {directory} failed its integrity check: {problems}')
    return directory, map_store.info_dict(directory)


def _nodes(context):
    from ultron_b2_localization.map_manager_node import default_map_root
    cfg = lambda name: os.path.join(get_package_share_directory(PKG), 'config', name)
    mode = LaunchConfiguration('mode').perform(context)
    run_mode = LaunchConfiguration('run_mode').perform(context)
    map_ref = LaunchConfiguration('map').perform(context)
    map_root = os.path.expanduser(LaunchConfiguration('map_root').perform(context) or default_map_root())
    pose = [float(v) for v in LaunchConfiguration('initial_pose').perform(context).split(',')]
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'}
    provenance = LaunchConfiguration('provenance').perform(context)
    manager_params = {'map_root': map_root, 'initial_pose': pose, 'provenance': provenance,
                      'source': f'slam_toolbox via odom_source={LaunchConfiguration("odom_source").perform(context)}'}
    actions = []
    if mode == 'mapping':
        actions += [
            Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
                 parameters=[cfg('slam_toolbox_mapping.yaml'), sim_time]),
            Node(package=PKG, executable='localization_monitor', name='localization_monitor', output='screen',
                 parameters=[cfg('localization_monitor.yaml'), sim_time,
                             {'map_source': 'live_grid', 'source': 'SLAM_TOOLBOX_MAPPING'}]),
        ]
    else:
        if map_ref:
            directory, info = resolve_map(map_ref, map_root)
            if not info['posegraph_path']:
                raise RuntimeError(f'map {directory} has no slam_toolbox pose graph')
            manager_params['initial_map'] = info['yaml_path']
            # Simulation scans can arrive in bursts before the corresponding MOLA odometry TF.
            sim_scan_queue = {'scan_queue_size': 10} if run_mode == 'sim' else {}
            actions += [
                LogInfo(msg=f'[slam] localization on {info["map_id"]} v{info["map_version"]} ({directory})'),
                Node(package='slam_toolbox', executable='localization_slam_toolbox_node', name='slam_toolbox',
                     output='screen',
                     parameters=[cfg('slam_toolbox_localization.yaml'), sim_time, sim_scan_queue,
                                 {'map_file_name': info['posegraph_path'], 'map_start_pose': pose}])]
        else:
            actions.append(LogInfo(msg='[slam] WARNING: no map:= given; localization backend not started '
                                       '(state INITIALIZING until map/load)'))
        actions.append(Node(package=PKG, executable='localization_monitor', name='localization_monitor',
                            output='screen', parameters=[cfg('localization_monitor.yaml'), sim_time,
                                                         {'map_source': 'map_info'}]))
    actions.append(Node(package=PKG, executable='map_manager', name='map_manager', output='screen',
                        parameters=[manager_params, sim_time]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localization', 'nav', 'mission']),
        DeclareLaunchArgument('run_mode', default_value='', choices=['', 'sim', 'replay', 'live']),
        DeclareLaunchArgument('map', default_value='', description='map_id, map_id:version, or map dir/yaml'),
        DeclareLaunchArgument('map_root', default_value='', description='default $ULTRON_MAP_ROOT or $ROS_HOME/ultron_maps'),
        DeclareLaunchArgument('initial_pose', default_value='0,0,0', description='x,y,yaw in the map frame'),
        DeclareLaunchArgument('odom_source', default_value='mola'),
        DeclareLaunchArgument('provenance', default_value='SLAM_TOOLBOX_ESTIMATE'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_nodes),
    ])
