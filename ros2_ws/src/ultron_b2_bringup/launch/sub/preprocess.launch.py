"""Sub-launch `preprocess` -- STUB owned by Agent B.

Responsibility: Self-filter, crop, voxel -> sensors/lidar/points_filtered; pointcloud_to_laserscan -> sensors/scan.
Receives the sub-launch contract args (ultron_b2_bringup.launch_common.SUB_ARGS) and runs inside the
mode namespace. Replace the placeholder with the real nodes; keep relative topic names.
"""
from ultron_b2_bringup.launch_common import placeholder_launch


def generate_launch_description():
    return placeholder_launch('preprocess', owner='B')
