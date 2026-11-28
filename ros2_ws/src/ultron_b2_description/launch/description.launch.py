"""robot_state_publisher for the B2 description (relative topics; caller sets the namespace)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('ultron_b2_description')
    profile = LaunchConfiguration('calibration_profile')
    calibration = PathJoinSubstitution([share, 'calibration', PythonExpression(["'", profile, "' + '.yaml'"])])
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([share, 'urdf', 'b2.urdf.xacro']),
                 ' calibration:=', calibration, ' legs:=', LaunchConfiguration('legs')]),
        value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('calibration_profile', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument('legs', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
