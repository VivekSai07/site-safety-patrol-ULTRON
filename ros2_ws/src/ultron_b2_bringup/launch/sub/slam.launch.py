"""Sub-launch `slam` (owner: Agent C).

slam_toolbox mapping (mode:=mapping) or localization on a saved map (other modes), localization_monitor
(`state/localization`) and map_manager (`map/save`, `map/load`, `map/info`); see
ultron_b2_localization/launch/slam.launch.py. Extra launch arguments: `map:=<map_id[:version]|dir|yaml>`
(localization), `map_root:=`, `initial_pose:=x,y,yaw`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ultron_b2_bringup.launch_common import SUB_ARGS


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ultron_b2_localization'), 'launch',
                                                            'slam.launch.py'])),
        launch_arguments=[('mode', LaunchConfiguration('mode')),
                          ('run_mode', LaunchConfiguration('run_mode')),
                          ('odom_source', LaunchConfiguration('odom_source')),
                          ('use_sim_time', LaunchConfiguration('use_sim_time')),
                          ('provenance', ['ULTRON_B2_', LaunchConfiguration('run_mode'), '_SLAM_TOOLBOX_ESTIMATE']),
                          ('map', LaunchConfiguration('map', default='')),
                          ('map_root', LaunchConfiguration('map_root', default='')),
                          ('initial_pose', LaunchConfiguration('initial_pose', default='0,0,0'))])
    return LaunchDescription(declared + [slam])
