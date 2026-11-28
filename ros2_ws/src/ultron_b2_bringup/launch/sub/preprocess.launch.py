"""Sub-launch `preprocess` (owner: Agent B).

cloud_filter: sensors/lidar/points -> sensors/lidar/points_filtered (self box, range, height, voxel,
optional deskew). cloud_to_scan: points_filtered -> sensors/scan in base_footprint, RELIABLE in and out
(the only sensors/scan publisher; upstream pointcloud_to_laserscan is best-effort only and lost most clouds).
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
    to_scan = Node(package='ultron_b2_preprocess', executable='cloud_to_scan', name='cloud_to_scan',
                   output='screen', parameters=[_config('cloud_to_scan')])
    return LaunchDescription(declared + [cloud_filter, to_scan])
