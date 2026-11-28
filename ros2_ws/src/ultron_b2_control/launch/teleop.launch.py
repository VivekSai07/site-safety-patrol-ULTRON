"""Teleop sources. Publish to control/in/teleop ONLY (never cmd_vel): the command still passes
twist_mux -> collision_monitor -> motion_gate. Scales PROPOSED to stay inside the gate limits
(vx 0.6, wz 0.8); the gate rejects over-limit commands rather than clamping them.

  ros2 launch ultron_b2_control teleop.launch.py [joy:=true] [keyboard:=true keyboard_prefix:='xterm -e']
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

TELEOP = [('cmd_vel', 'control/in/teleop')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/ultron/sim'),
        DeclareLaunchArgument('joy', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('keyboard', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('keyboard_prefix', default_value='xterm -e',
                              description='teleop_twist_keyboard needs a TTY'),
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             condition=IfCondition(LaunchConfiguration('joy'))),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy', output='screen',
             condition=IfCondition(LaunchConfiguration('joy')), remappings=TELEOP,
             parameters=[{'axis_linear.x': 1, 'scale_linear.x': 0.5, 'scale_linear_turbo.x': 0.5,
                          'axis_angular.yaw': 0, 'scale_angular.yaw': 0.6, 'scale_angular_turbo.yaw': 0.6,
                          'enable_button': 4, 'require_enable_button': True}]),
        Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard', name='teleop_twist_keyboard',
             output='screen', prefix=LaunchConfiguration('keyboard_prefix'),
             condition=IfCondition(LaunchConfiguration('keyboard')), remappings=TELEOP,
             parameters=[{'speed': 0.3, 'turn': 0.6}]),
    ])
