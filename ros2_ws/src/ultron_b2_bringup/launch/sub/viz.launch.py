"""Sub-launch `viz` -- STUB owned by Agent F.

Responsibility: foxglove_bridge whitelist, viz_throttle, domain_bridge to viz domain 42.
Receives the sub-launch contract args (ultron_b2_bringup.launch_common.SUB_ARGS) and runs inside the
mode namespace. Replace the placeholder with the real nodes; keep relative topic names.
"""
from ultron_b2_bringup.launch_common import placeholder_launch


def generate_launch_description():
    return placeholder_launch('viz', owner='F')
