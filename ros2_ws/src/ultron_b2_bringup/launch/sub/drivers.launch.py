"""Sub-launch `drivers` -- STUB owned by Agent B (live/replay) / F (sim).

Responsibility: Sensor drivers or bag playback; remap vendor topics into sensors/*.
Receives the sub-launch contract args (ultron_b2_bringup.launch_common.SUB_ARGS) and runs inside the
mode namespace. Replace the placeholder with the real nodes; keep relative topic names.
"""
from ultron_b2_bringup.launch_common import placeholder_launch


def generate_launch_description():
    return placeholder_launch('drivers', owner='B (live/replay) / F (sim)')
