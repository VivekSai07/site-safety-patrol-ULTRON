"""Sub-launch `slam` -- STUB owned by Agent C.

Responsibility: slam_toolbox mapping/localization (map->odom), localization_monitor, map_manager.
Receives the sub-launch contract args (ultron_b2_bringup.launch_common.SUB_ARGS) and runs inside the
mode namespace. Replace the placeholder with the real nodes; keep relative topic names.
"""
from ultron_b2_bringup.launch_common import placeholder_launch


def generate_launch_description():
    return placeholder_launch('slam', owner='C')
