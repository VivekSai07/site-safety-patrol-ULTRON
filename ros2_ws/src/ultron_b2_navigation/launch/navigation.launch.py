"""Nav2 + ULTRON navigation contract for the B2 stack (SIMULATION-validated; real profile PROPOSED).

Nav2 controller/behaviors -> nav/cmd_vel_raw -> velocity_smoother -> control/in/autonomy.
Nav2 is NOT autostarted: nav_contract_server requests lifecycle STARTUP once the clock runs and
map->base_link exists (no Nav2 activation without localization TF). Nothing here publishes cmd_vel; the control launch carries twist_mux -> collision_monitor -> motion_gate.

Args: namespace (absolute, default /ultron/sim), push_namespace (true when run standalone),
profile sim|real, map_yaml (optional saved map -> map_server on nav/map; else live `map`),
map_meta (map_meta.json or its folder: map id/version for contract checks), zone_file (PatrolZone YAML),
use_stvl, use_sim_time.
"""
import math
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace, SetParameter

PKG = 'ultron_b2_navigation'
NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother']


def footprint_radius(profile):
    path = Path(get_package_share_directory('ultron_b2_description')) / 'config' / 'footprint.yaml'
    data = yaml.safe_load(path.read_text())[profile]
    radius = data.get('radius_m')
    if (data.get('type') != 'circle' or not isinstance(radius, (int, float))
            or not math.isfinite(radius) or radius <= 0):
        raise RuntimeError(f'footprint "{profile}" is {data.get("type")} (radius {data.get("radius_m")}): '
                           'Nav2 refuses to start until the footprint is measured (TO_VERIFY)')
    return float(radius)


def planning_radius(profile):
    radius = footprint_radius(profile)
    if profile != 'sim':
        return radius
    path = Path(get_package_share_directory('ultron_b2_description')) / 'config' / 'footprint.yaml'
    value = yaml.safe_load(path.read_text())['sim'].get('planning_radius_m')
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value < radius:
        raise RuntimeError('simulation planning radius must be finite and at least the footprint radius')
    return float(value)


def render_params(profile, ns, map_topic, use_stvl):
    """Substitute tokens, optionally add STVL, write a params file under $ROS_HOME (never /tmp)."""
    share = Path(get_package_share_directory(PKG))
    text = (share / 'config' / f'nav2_{profile}.yaml').read_text()
    subs = {'${NS}': ns, '${MAP_TOPIC}': map_topic, '${ROBOT_RADIUS}': str(planning_radius(profile)),
            '${BT_XML}': str(share / 'config' / 'bt' / 'navigate_no_motion_recovery.xml')}
    for k, v in subs.items():
        text = text.replace(k, v)
    params = yaml.safe_load(text)
    local = params['local_costmap']['local_costmap']['ros__parameters']
    if use_stvl:
        local['plugins'].insert(local['plugins'].index('inflation_layer'), 'stvl_layer')
    else:
        local.pop('stvl_layer', None)
    if ns:   # rcl matches param-file keys against fully qualified node names (costmaps: <ns>/<name>/<name>)
        params = {f'{ns}/{k}': v for k, v in params.items()}
    out_dir = Path(os.environ.get('ROS_HOME', Path.home() / '.ros')) / 'ultron_b2_navigation'
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f'nav2_{profile}_{ns.strip("/").replace("/", "_") or "root"}_{os.getpid()}.yaml'
    out.write_text(yaml.safe_dump(params, sort_keys=False))
    return str(out)


def _setup(context):
    cfg = {k: context.launch_configurations.get(k, '') for k in
           ('namespace', 'push_namespace', 'profile', 'map_yaml', 'map_meta', 'zone_file', 'use_stvl',
            'use_sim_time')}
    ns = '/' + cfg['namespace'].strip('/') if cfg['namespace'].strip('/') else ''
    map_yaml = cfg['map_yaml']
    map_topic = f'{ns}/nav/map' if map_yaml else f'{ns}/map'
    params = render_params(cfg['profile'], ns, map_topic, cfg['use_stvl'] == 'true')
    nav_out = [('cmd_vel', 'nav/cmd_vel_raw')]
    actions = []
    if cfg['push_namespace'] == 'true' and ns:
        actions.append(PushRosNamespace(ns))
    if cfg['use_sim_time']:
        actions.append(SetParameter(name='use_sim_time', value=cfg['use_sim_time'] == 'true'))
    if map_yaml:
        actions += [
            Node(package='nav2_map_server', executable='map_server', name='nav_map_server', output='screen',
                 parameters=[{'yaml_filename': map_yaml, 'topic_name': 'nav/map', 'frame_id': 'map'}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_nav_map',
                 output='screen', parameters=[{'autostart': True, 'node_names': ['nav_map_server']}]),
        ]
    actions += [
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[params], remappings=nav_out),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[params], remappings=nav_out),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[params]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=[params],
             remappings=[('cmd_vel', 'nav/cmd_vel_raw'), ('cmd_vel_smoothed', 'control/in/autonomy')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[params, {'autostart': False}]),   # STARTUP by nav_contract_server
        Node(package=PKG, executable='zone_mask_server', name='zone_mask_server', output='screen',
             parameters=[{'zone_file': cfg['zone_file'], 'map_yaml': map_yaml, 'map_meta_path': cfg['map_meta']}]),
        Node(package=PKG, executable='nav_contract_server', name='nav_contract_server', output='screen',
             parameters=[{'map_meta_path': cfg['map_meta'], 'map_yaml': map_yaml}]),
    ]
    return [GroupAction(actions, scoped=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/ultron/sim'),
        DeclareLaunchArgument('push_namespace', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('profile', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('map_meta', default_value=''),
        DeclareLaunchArgument('zone_file', default_value=''),
        DeclareLaunchArgument('use_stvl', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false', '']),
        OpaqueFunction(function=_setup),
    ])
