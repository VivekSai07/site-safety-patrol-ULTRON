"""Sub-launch `drivers` -- sim: Agent F (ultron_b2_sim); live/replay: Agent B (placeholder until merged).

Receives the sub-launch contract args (ultron_b2_bringup.launch_common.SUB_ARGS) and runs inside the
mode namespace. In sim the MuJoCo contract bridge publishes /clock, sensors/*, state/odom (only with
odom_source:=sim) and sim/ground_truth/* and consumes the gated `cmd_vel`. It never publishes
base_link->sensor TF (robot_state_publisher owns it).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ultron_b2_bringup.launch_common import SUB_ARGS, placeholder_launch


def _drivers(context):
    cfg = context.launch_configurations
    if cfg.get('run_mode') != 'sim':
        return placeholder_launch('drivers', owner='B (live/replay)').entities
    source = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('ultron_b2_sim'), 'launch', 'sim_bridge.launch.py'))
    args = {'odom_source': cfg.get('odom_source', 'sim'),
            'calibration_profile': cfg.get('calibration_profile', 'sim')}
    for key in ('sim_rate', 'sim_people', 'sim_route', 'sim_scenario', 'sim_hidden_ladders', 'sim_scan_topic',
                'sim_camera'):
        if cfg.get(key):
            args[key[len('sim_'):]] = cfg[key]
    return [IncludeLaunchDescription(source, launch_arguments=list(args.items()))]


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
    return LaunchDescription(declared + [OpaqueFunction(function=_drivers)])
