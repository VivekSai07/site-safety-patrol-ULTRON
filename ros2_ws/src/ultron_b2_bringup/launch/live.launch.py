"""LIVE robot (domain 0, namespace /ultron/live). Only with explicit user/operator authorization.
hardware_enable:=true additionally needs the operator hardware token; agents never run this file.

Example: ros2 launch ultron_b2_bringup live.launch.py mode:=mapping viz:=none
"""
from ultron_b2_bringup.launch_common import generate


def generate_launch_description():
    return generate('live')
