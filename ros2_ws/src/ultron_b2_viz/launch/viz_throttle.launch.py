"""viz_throttle in the current namespace (bringup pushes /ultron/<mode>)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('ultron_b2_viz'), 'config', 'viz_throttle.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default),
        Node(package='ultron_b2_viz', executable='viz_throttle', name='viz_throttle', output='screen',
             parameters=[LaunchConfiguration('params')]),
    ])
