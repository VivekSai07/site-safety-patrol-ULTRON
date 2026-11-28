"""Sub-launch `state_estimation` -- STUB owned by Agent C.

Responsibility: MOLA-LO / KISS-ICP / B2-odom adapter: sole odom->base_link publisher selected by odom_source.
Receives the sub-launch contract args (ultron_b2_bringup.launch_common.SUB_ARGS) and runs inside the
mode namespace. Replace the placeholder with the real nodes; keep relative topic names.
"""
from ultron_b2_bringup.launch_common import placeholder_launch


def generate_launch_description():
    return placeholder_launch('state_estimation', owner='C')
