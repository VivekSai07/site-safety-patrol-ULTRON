"""foxglove_bridge with the ULTRON topic whitelist (Agent F).

Binds 127.0.0.1:8765 by default: view from a laptop through an SSH tunnel
(``ssh -L 8765:127.0.0.1:8765 <orin>``; docs/b2_ros_stack/VIZ_GUIDE.md). Binding another address
exposes the stack beyond loopback and needs user approval. Read-only: no client publishing,
services or parameters; only whitelisted (throttled) topics are advertised.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ultron_b2_viz.whitelist import FOXGLOVE_PORT, foxglove_whitelist

NEVER = ['(?!)']  # never-matching regex: an empty list does not round-trip as a string array


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('foxglove_port', default_value=str(FOXGLOVE_PORT)),
        DeclareLaunchArgument('foxglove_address', default_value='127.0.0.1',
                              description='Bind address; anything but loopback needs user approval'),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge', output='screen',
             parameters=[{
                 'port': LaunchConfiguration('foxglove_port'),
                 'address': LaunchConfiguration('foxglove_address'),
                 'topic_whitelist': foxglove_whitelist(),
                 'service_whitelist': NEVER,
                 'param_whitelist': NEVER,
                 'client_topic_whitelist': NEVER,
                 'capabilities': ['connectionGraph', 'assets'],
                 'sysinfo': False,
                 'remote_access': False,
                 'include_hidden': False,
                 'max_qos_depth': 5,
                 'send_buffer_limit': 10000000,
             }]),
    ])
