"""Sub-launch `preprocess` (owner: Agent B).

cloud_filter: sensors/lidar/points -> sensors/lidar/points_filtered (self box, range, height, voxel,
optional deskew). pointcloud_to_laserscan: points_filtered -> sensors/scan in base_footprint.
Parameter files are chosen by calibration_profile (sim|real) from ultron_b2_preprocess/config/.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ultron_b2_bringup.launch_common import SUB_ARGS


def _config(stem):
    profile = LaunchConfiguration('calibration_profile')
    return PathJoinSubstitution([FindPackageShare('ultron_b2_preprocess'), 'config',
                                 PythonExpression(["'", stem, "_' + '", profile, "' + '.yaml'"])])


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
    cloud_filter = Node(package='ultron_b2_preprocess', executable='cloud_filter', name='cloud_filter',
                        output='screen', parameters=[_config('cloud_filter')])
    to_scan = Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                   name='pointcloud_to_laserscan', output='screen', parameters=[_config('pointcloud_to_laserscan')],
                   remappings=[('cloud_in', 'sensors/lidar/points_filtered'), ('scan', 'sensors/scan')])
    return LaunchDescription(declared + [cloud_filter, to_scan])
