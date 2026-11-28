"""Sub-launch `control` -- STUB owned by Agent D.

Responsibility: twist_mux -> collision_monitor -> motion_gate (sole cmd_vel publisher) -> sim_driver | b2_sport_bridge.
Receives the sub-launch contract args (ultron_b2_bringup.launch_common.SUB_ARGS) and runs inside the
mode namespace. Replace the placeholder with the real nodes; keep relative topic names.
"""
from ultron_b2_bringup.launch_common import placeholder_launch


def generate_launch_description():
    return placeholder_launch('control', owner='D')
