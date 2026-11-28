"""Sub-launch `control` (Agent D): twist_mux -> collision_monitor -> motion_gate (sole cmd_vel publisher)
-> sim_driver (sim) | b2_sport_bridge (live; hardware_enable from the stack, default false).

Runs inside the mode namespace pushed by the parent. Optional parent launch arguments: map_meta (default: the `map:=` directory),
teleop_requires_localization (default true, fail-closed).
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ultron_b2_bringup.launch_common import SUB_ARGS, saved_map_paths
from ultron_b2_bringup.modes import MODES


def _include(context):
    cfg = context.launch_configurations
    run_mode = cfg.get('run_mode') or 'sim'
    ns = cfg.get('namespace') or MODES[run_mode]['namespace']
    live = run_mode == 'live'
    sim_mapping_teleop = (run_mode == 'sim' and cfg.get('mode') == 'mapping'
                          and cfg.get('sim_teleop_without_localization') == 'true')
    src = PythonLaunchDescriptionSource(
        get_package_share_directory('ultron_b2_control') + '/launch/control.launch.py')
    return [IncludeLaunchDescription(src, launch_arguments={
        'namespace': ns, 'push_namespace': 'false',
        'driver': 'bridge' if live else 'sim',
        'hardware_enable': 'true' if live and cfg.get('hardware_enable') == 'true' else 'false',
        'bridge_target': 'robot' if live else 'mock',
        'teleop_requires_localization': ('false' if sim_mapping_teleop else
                                        cfg.get('teleop_requires_localization', 'true') or 'true'),
        'teleop_requires_fresh_scan': 'true' if sim_mapping_teleop else 'false',
        'map_meta': saved_map_paths(cfg)[1],
        'use_sim_time': cfg.get('use_sim_time', ''),
    }.items())]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
                             + [OpaqueFunction(function=_include)])
