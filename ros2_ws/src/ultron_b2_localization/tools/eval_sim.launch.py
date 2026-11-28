"""Eval-only: odometry + slam_toolbox mapping under /ultron/sim, fed by tools/eval_sim_bridge.py.

No robot_state_publisher: the bridge publishes its own base_link->lidar_link static TF (the sim
calibration in ultron_b2_description must match it; bringup integration is Agent F's sim package).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    share = os.path.join(get_package_share_directory('ultron_b2_localization'), 'launch')
    src = LaunchConfiguration('odom_source')
    return LaunchDescription([
        DeclareLaunchArgument('odom_source', choices=['sim', 'mola', 'kiss']),
        GroupAction([
            PushRosNamespace('/ultron/sim'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'odometry.launch.py')),
                                     launch_arguments={'odom_source': src, 'calibration_profile': 'sim',
                                                       'use_sim_time': 'true'}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'slam.launch.py')),
                                     launch_arguments={'mode': 'mapping', 'odom_source': src, 'use_sim_time': 'true',
                                                       'provenance': 'SIM_EVAL_SLAM_TOOLBOX_ESTIMATE'}.items()),
        ]),
    ])
