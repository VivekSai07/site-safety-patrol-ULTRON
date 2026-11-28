"""Real-bag replay (domain 72, ROS_LOCALHOST_ONLY=1, namespace /ultron/replay). No control/nav/mission nodes.

Example: ros2 launch ultron_b2_bringup replay.launch.py mode:=mapping viz:=none
"""
from ultron_b2_bringup.launch_common import generate


def generate_launch_description():
    return generate('replay')
