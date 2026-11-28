"""Sub-launch `nav` -- STUB owned by Agent D.

Responsibility: Nav2 + nav_contract_server (NavigateToView) -> nav/cmd_vel_raw -> velocity_smoother -> control/in/autonomy.
Receives the sub-launch contract args (ultron_b2_bringup.launch_common.SUB_ARGS) and runs inside the
mode namespace. Replace the placeholder with the real nodes; keep relative topic names.
"""
from ultron_b2_bringup.launch_common import placeholder_launch


def generate_launch_description():
    return placeholder_launch('nav', owner='D')
