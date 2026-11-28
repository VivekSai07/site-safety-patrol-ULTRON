"""Sub-launch `nav` (Agent D): Nav2 + velocity_smoother + zone_mask_server + nav_contract_server (NavigateToView).

Nav2 -> nav/cmd_vel_raw -> velocity_smoother -> control/in/autonomy. Runs inside the mode namespace
pushed by the parent; relative topic names. Optional parent launch arguments (not part of SUB_ARGS):
map_yaml (saved map for map_server on nav/map; default: the `map:=` saved map, else live `map`), map_meta
(map_meta.json / folder; default: the `map:=` directory),
zone_file (PatrolZone YAML), use_stvl.
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
    map_yaml, map_meta = saved_map_paths(cfg)
    src = PythonLaunchDescriptionSource(
        get_package_share_directory('ultron_b2_navigation') + '/launch/navigation.launch.py')
    return [IncludeLaunchDescription(src, launch_arguments={
        'namespace': ns, 'push_namespace': 'false',
        'profile': cfg.get('calibration_profile') or ('sim' if run_mode == 'sim' else 'real'),
        'map_yaml': map_yaml, 'map_meta': map_meta,
        'zone_file': cfg.get('zone_file', ''), 'use_stvl': cfg.get('use_stvl', 'false'),
        'use_sim_time': cfg.get('use_sim_time', ''),
    }.items())]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
                             + [OpaqueFunction(function=_include)])
