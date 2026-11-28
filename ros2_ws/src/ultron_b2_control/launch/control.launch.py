"""Common command chain (every source, one path):

  control/in/teleop, control/in/autonomy, ops/hold
    -> twist_mux (HOLD 255 > teleop 100 > autonomy 10) -> control/cmd_vel_muxed
    -> collision_monitor -> control/cmd_vel_safe
    -> motion_gate (sole cmd_vel publisher) -> cmd_vel
    -> sim_driver (driver:=sim -> sim/cmd_vel) | b2_sport_bridge (driver:=bridge; MOCK ONLY in this repo)

Args: namespace, push_namespace, driver sim|bridge|none, hardware_enable (bridge only, default false),
bridge_target mock|robot, teleop_requires_localization (default true), map_meta, map_id, map_version,
use_sim_time.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace, SetParameter

PKG = 'ultron_b2_control'


def _setup(context):
    c = {k: context.launch_configurations.get(k, '') for k in (
        'namespace', 'push_namespace', 'driver', 'hardware_enable', 'bridge_target', 'teleop_requires_localization',
        'teleop_requires_fresh_scan',
        'map_meta', 'map_id', 'map_version', 'use_sim_time')}
    share = get_package_share_directory(PKG)
    nav_share = get_package_share_directory('ultron_b2_navigation')
    mux_yaml = f'{share}/config/twist_mux.yaml'
    ns = '/' + c['namespace'].strip('/') if c['namespace'].strip('/') else ''
    actions = []
    if c['push_namespace'] == 'true' and ns:
        actions.append(PushRosNamespace(ns))
    if c['use_sim_time']:
        actions.append(SetParameter(name='use_sim_time', value=c['use_sim_time'] == 'true'))
    actions += [
        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=[mux_yaml], remappings=[('cmd_vel_out', 'control/cmd_vel_muxed')]),
        Node(package=PKG, executable='active_source_monitor', name='active_source_monitor', output='screen',
             parameters=[mux_yaml]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             output='screen', parameters=[f'{nav_share}/config/collision_monitor.yaml',
                                          *([f'{share}/config/collision_monitor_sim_override.yaml']
                                            if c['driver'] == 'sim' else [])]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_control',
             output='screen', parameters=[{'autostart': True, 'bond_timeout': 4.0,
                                           'node_names': ['collision_monitor']}]),
        Node(package='ultron_b2_navigation', executable='motion_gate', name='motion_gate', output='screen',
             parameters=[{'teleop_requires_localization': c['teleop_requires_localization'] != 'false',
                          'teleop_requires_fresh_scan': c['teleop_requires_fresh_scan'] == 'true',
                          'map_meta_path': c['map_meta'], 'map_id': c['map_id'],
                          'map_version': int(c['map_version'] or 0)}]),
    ]
    if c['driver'] == 'sim':
        actions.append(Node(package=PKG, executable='sim_driver', name='sim_driver', output='screen'))
    elif c['driver'] == 'bridge':
        actions.append(Node(package=PKG, executable='b2_sport_bridge', name='b2_sport_bridge', output='screen',
                            parameters=[f'{share}/config/sport_bridge.yaml',
                                        {'hardware_enable': c['hardware_enable'] == 'true',
                                         'target': c['bridge_target'] or 'mock'}]))
    return [GroupAction(actions, scoped=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/ultron/sim'),
        DeclareLaunchArgument('push_namespace', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('driver', default_value='sim', choices=['sim', 'bridge', 'none']),
        DeclareLaunchArgument('hardware_enable', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('bridge_target', default_value='mock', choices=['mock', 'robot']),
        DeclareLaunchArgument('teleop_requires_localization', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('teleop_requires_fresh_scan', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('map_meta', default_value=''),
        DeclareLaunchArgument('map_id', default_value=''),
        DeclareLaunchArgument('map_version', default_value='0'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false', '']),
        OpaqueFunction(function=_setup),
    ])
