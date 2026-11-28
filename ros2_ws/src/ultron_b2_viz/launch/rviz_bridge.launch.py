"""domain_bridge: stack domain (sim 71 / replay 72 / live 0) -> viz domain 42, whitelist only.

Topics arrive in domain 42 as /ultron/viz/* (mode independent) plus /tf, /tf_static; the laptop
RViz config is rviz/ultron_viz42.rviz. ``zenoh:=true`` additionally starts the zenoh-bridge-ros2dds
fallback on the Orin side (UNRUN unless installed). See docs/b2_ros_stack/VIZ_GUIDE.md.
"""
import os
import shutil

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction

from ultron_b2_viz.whitelist import MODE_DOMAINS, VIZ_DOMAIN


def _bridge(context):
    mode = context.launch_configurations.get('run_mode', 'sim')
    if mode not in MODE_DOMAINS:
        raise RuntimeError(f'run_mode must be one of {sorted(MODE_DOMAINS)}')
    env_domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if env_domain != str(MODE_DOMAINS[mode]):
        raise RuntimeError(f'rviz_bridge for {mode} expects ROS_DOMAIN_ID={MODE_DOMAINS[mode]} (got {env_domain})')
    share = get_package_share_directory('ultron_b2_viz')
    actions = [ExecuteProcess(name='viz_domain_bridge', output='screen',
                              cmd=[os.path.join(get_package_prefix('domain_bridge'), 'lib', 'domain_bridge', 'domain_bridge'),
                                   os.path.join(share, 'config', f'domain_bridge_{mode}.yaml')])]
    if context.launch_configurations.get('zenoh', 'false') == 'true':
        exe = shutil.which('zenoh-bridge-ros2dds')
        cfg = os.path.join(share, 'config', 'zenoh_bridge_orin.json5')
        if exe is None:
            actions.append(LogInfo(msg='[ultron_b2_viz] zenoh-bridge-ros2dds not installed: zenoh fallback UNRUN'))
        else:
            actions.append(ExecuteProcess(name='viz_zenoh_bridge', output='screen', cmd=[exe, '-c', cfg],
                                          additional_env={'ROS_DOMAIN_ID': str(VIZ_DOMAIN)}))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_mode', default_value='sim', choices=sorted(MODE_DOMAINS)),
        DeclareLaunchArgument('zenoh', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_bridge),
    ])
