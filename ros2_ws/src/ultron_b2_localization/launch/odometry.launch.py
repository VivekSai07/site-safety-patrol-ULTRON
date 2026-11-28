"""odom->base_link authority selected by ``odom_source`` (owner: Agent C).

- ``sim``  : nothing here; the simulation driver publishes odom->base_link and ``state/odom``.
- ``mola`` : MOLA-LO (``mola-cli``, separate GPLv3 process) on ``sensors/lidar/points_filtered``, flattened
             by ``odom_planarizer`` (MOLA's own TF is remapped to ``mola/tf_raw``).
- ``kiss`` : KISS-ICP node on the same input.
- ``b2``   : B2 sport-mode odometry adapter (live only; MOCK-TESTED ONLY).

All outputs: TF ``odom -> base_link`` (planar) + nav_msgs/Odometry ``state/odom`` (relative names;
the including launch pushes the namespace). Exactly one of them runs.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'ultron_b2_localization'
SOURCES = ('sim', 'mola', 'kiss', 'b2')


def _config(name):
    with open(os.path.join(get_package_share_directory(PKG), 'config', name)) as f:
        return yaml.safe_load(f)


def mola_environment(profile, points_topic=''):
    cfg = _config('mola_lo.yaml')
    env = dict(cfg['common'])
    env.update(cfg['profiles'].get(profile) or {})
    share = get_package_share_directory('mola_lidar_odometry')
    for key in ('MOLA_STATE_ESTIMATOR_YAML', 'MOLA_ODOMETRY_PIPELINE_YAML'):
        if not os.path.isabs(env[key]):
            env[key] = os.path.join(share, env[key])
    if points_topic:
        env['MOLA_LIDAR_TOPIC'] = points_topic
    return {k: str(v) for k, v in env.items()}


def kiss_parameters(profile, points_topic=''):
    cfg = _config('kiss_icp.yaml')
    params = dict(cfg['parameters'])
    params.update(cfg['profiles'].get(profile) or {})
    if points_topic:
        params['points_topic'] = points_topic
    return params


def _nodes(context):
    source = LaunchConfiguration('odom_source').perform(context)
    profile = LaunchConfiguration('calibration_profile').perform(context) or 'sim'
    points = LaunchConfiguration('points_topic').perform(context)
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'}
    if source not in SOURCES:
        raise RuntimeError(f'odom_source must be one of {SOURCES}, not {source!r}')
    if source == 'sim':
        return [LogInfo(msg='[state_estimation] odom_source=sim: simulation driver owns odom->base_link')]
    if source == 'mola':
        share = get_package_share_directory('mola_lidar_odometry')
        # MOLA's ICP pose is 3D even with enforce_planar_motion; its TF and odometry go to private topics and
        # odom_planarizer is the single odom->base_link authority (see odom_planarizer.py).
        return [Node(package='mola_launcher', executable='mola-cli', name='mola_lidar_odometry', output='screen',
                     arguments=[os.path.join(share, 'mola-cli-launchs', 'lidar_odometry_ros2.yaml')],
                     additional_env=mola_environment(profile, points), parameters=[sim_time],
                     remappings=[('lidar_odometry/pose', 'mola/odom_raw'), ('/tf', 'mola/tf_raw')]),
                Node(package=PKG, executable='odom_planarizer', name='mola_odom_planarizer', output='screen',
                     parameters=[{'odom_in': 'mola/odom_raw', 'odom_out': 'state/odom'}, sim_time])]
    if source == 'kiss':
        return [Node(package=PKG, executable='kiss_icp_odometry', name='kiss_icp_odometry', output='screen',
                     parameters=[kiss_parameters(profile, points), sim_time])]
    return [Node(package=PKG, executable='b2_odom_adapter', name='b2_odom_adapter', output='screen',
                 parameters=[sim_time])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_source', default_value='mola', choices=list(SOURCES)),
        DeclareLaunchArgument('calibration_profile', default_value='real', choices=['sim', 'real']),
        DeclareLaunchArgument('points_topic', default_value='', description='Override input cloud topic'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_nodes),
    ])
