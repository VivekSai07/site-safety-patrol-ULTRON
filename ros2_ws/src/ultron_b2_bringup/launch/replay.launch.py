"""Real-bag replay (domain 72, ROS_LOCALHOST_ONLY=1, namespace /ultron/replay). No control/nav/mission nodes.

Example:
  ros2_ws/scripts/ultron_launch.sh replay mode:=mapping bag:=/abs/path/to/bag_dir replay_rate:=1.0
Replay-only args: ultron_b2_bringup.replay_topics.REPLAY_ARGS (consumed by launch/sub/drivers.launch.py).
"""
from launch.actions import DeclareLaunchArgument

from ultron_b2_bringup.launch_common import generate
from ultron_b2_bringup.replay_topics import REPLAY_ARGS


def generate_launch_description():
    ld = generate('replay')
    for name, (default, help_text) in REPLAY_ARGS.items():
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=help_text))
    return ld
