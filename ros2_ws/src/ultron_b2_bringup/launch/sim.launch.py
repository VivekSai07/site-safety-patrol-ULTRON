"""Simulation (domain 71, ROS_LOCALHOST_ONLY=1, namespace /ultron/sim).

Example: ros2 launch ultron_b2_bringup sim.launch.py mode:=mapping viz:=none
"""
from ultron_b2_bringup.launch_common import generate


def generate_launch_description():
    return generate('sim')
