"""Isaac Sim backend behind the same topic contract -- STUB, UNRUN (M7, post-event; Agent F).

Not on the critical path (docs/ISAAC_SIM_FEASIBILITY.md). This launch declares the same arguments
and contract as ``sim_bridge.launch.py`` but refuses to start: the Isaac-side contract publisher
(Helios-32 emitter pattern, USD hall, kinematic B2, odometry/IMU/camera/ground truth) is not
written yet. ``smoke:=true`` only runs the existing RTX-lidar smoke test, which publishes
``sim/isaac/points`` (NOT a contract topic). See ``isaac/README.md``.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from ament_index_python.packages import get_package_share_directory

# Contract the Isaac backend must publish (relative to /ultron/sim), identical to the MuJoCo bridge.
CONTRACT = ('/clock', 'sensors/lidar/points', 'sensors/scan', 'sensors/imu/data', 'state/odom',
            'sensors/cam_head/color/image_raw', 'sensors/cam_head/color/camera_info',
            'sensors/cam_head/depth/image_rect_raw', 'sensors/cam_head/depth/camera_info',
            'sim/ground_truth/base_pose', 'sim/ground_truth/ladders', 'sim/scenario')
INPUTS = ('cmd_vel',)


def _isaac(context):
    cfg = context.launch_configurations
    isaac = cfg.get('isaac_sim') or os.environ.get('ISAAC_SIM', os.path.expanduser('~/isaac-sim'))
    if cfg.get('smoke', 'false') != 'true':
        raise RuntimeError('isaac.launch.py is an UNRUN stub: the Isaac contract publisher is not implemented '
                           f'(must publish {", ".join(CONTRACT)}). Use sim_bridge.launch.py (MuJoCo).')
    if not os.path.isfile(os.path.join(isaac, 'python.sh')):
        raise RuntimeError(f'Isaac Sim not found at {isaac!r} (set isaac_sim:=... or $ISAAC_SIM)')
    if os.environ.get('ROS_DOMAIN_ID') != '71' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Isaac smoke needs ROS_DOMAIN_ID=71 and ROS_LOCALHOST_ONLY=1')
    share = os.path.realpath(os.path.join(get_package_share_directory('ultron_b2_sim'), 'package.xml'))
    script = None
    p = os.path.dirname(share)
    while p != '/':
        cand = os.path.join(p, 'scripts', 'isaac', 'run-smoke.sh')
        if os.path.isfile(cand):
            script = cand
            break
        p = os.path.dirname(p)
    if script is None:
        raise RuntimeError('scripts/isaac/run-smoke.sh not found (symlink-install from the repo required)')
    return [ExecuteProcess(cmd=[script, '--seconds', cfg.get('seconds', '20')], output='screen',
                           additional_env={'ISAAC_SIM': isaac})]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('isaac_sim', default_value='', description='Isaac Sim install (default $ISAAC_SIM)'),
        DeclareLaunchArgument('smoke', default_value='false', choices=['true', 'false'],
                              description='Run only the RTX-lidar smoke test (non-contract topic)'),
        DeclareLaunchArgument('seconds', default_value='20'),
        DeclareLaunchArgument('odom_source', default_value='sim'),
        DeclareLaunchArgument('namespace', default_value=''),
        OpaqueFunction(function=_isaac),
    ])
