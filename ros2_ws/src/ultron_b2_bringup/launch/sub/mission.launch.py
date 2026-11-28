"""Sub-launch `mission` -- STUB owned by Agent E.

Responsibility: mission_manager, frontier_explorer, perception, view planning, memory bridge, coverage.
Receives the sub-launch contract args (ultron_b2_bringup.launch_common.SUB_ARGS) and runs inside the
mode namespace. Replace the placeholder with the real nodes; keep relative topic names.
"""
from ultron_b2_bringup.launch_common import placeholder_launch


def generate_launch_description():
    return placeholder_launch('mission', owner='E')
