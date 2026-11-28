"""Sub-launch `state_estimation` (owner: Agent C).

Sole odom->base_link publisher selected by `odom_source` (ultron_b2_localization/launch/odometry.launch.py):
sim -> nothing (simulation driver owns it), mola -> MOLA-LO, kiss -> KISS-ICP, b2 -> sport-mode adapter.
Outputs (relative, in the mode namespace): TF odom->base_link + nav_msgs/Odometry `state/odom`.
Optional extra launch argument `points_topic:=` overrides the input cloud (default
`sensors/lidar/points_filtered`).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ultron_b2_bringup.launch_common import SUB_ARGS


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ultron_b2_localization'), 'launch',
                                                            'odometry.launch.py'])),
        launch_arguments=[('odom_source', LaunchConfiguration('odom_source')),
                          ('calibration_profile', LaunchConfiguration('calibration_profile')),
                          ('use_sim_time', LaunchConfiguration('use_sim_time')),
                          ('points_topic', LaunchConfiguration('points_topic', default=''))])
    return LaunchDescription(declared + [odometry])
