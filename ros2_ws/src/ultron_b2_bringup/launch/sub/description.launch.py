"""Sub-launch `description` (Agent A): robot_state_publisher for base_link -> sensor frames.

calibration_profile sim|real selects ultron_b2_description/calibration/<profile>.yaml.
publish_description:=false disables it (e.g. if a replayed /tf_static already carries the mounts;
tf_authority_check flags any duplicate authority).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ultron_b2_bringup.launch_common import SUB_ARGS


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ultron_b2_description'), 'launch', 'description.launch.py'])),
        launch_arguments=[('calibration_profile', LaunchConfiguration('calibration_profile')),
                          ('use_sim_time', LaunchConfiguration('use_sim_time'))],
        condition=IfCondition(LaunchConfiguration('publish_description')))
    return LaunchDescription(declared + [rsp])
