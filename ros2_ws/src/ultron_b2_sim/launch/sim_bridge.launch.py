"""MuJoCo sim bridge on the B2 topic contract (Agent F). SIMULATION ONLY; domain 71, localhost.

Run inside the stack namespace (bringup's drivers sub-launch pushes ``/ultron/sim``) or standalone:

    ros2 launch ultron_b2_sim sim_bridge.launch.py namespace:=/ultron/sim

Velocity input is the gated ``cmd_vel`` (motion_gate is its sole publisher in the stack).
robot_state_publisher owns ``base_link -> sensor`` TF, so this launch never publishes it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

ARGS = {
    'namespace': ('', 'Push this namespace (empty: inherit, e.g. from bringup)'),
    'odom_source': ('sim', 'sim: bridge publishes state/odom + odom->base_link; mola/kiss: sim/leg_odom only'),
    'calibration_profile': ('sim', 'ultron_b2_description calibration profile (must be sim)'),
    'camera_config': ('', 'Head camera YAML (default: share/ultron_b2_sim/config/head_camera_sim.yaml)'),
    'camera': ('true', 'Render the MuJoCo RGB-D head camera'),
    'imu': ('true', 'Publish the kinematic synthetic IMU'),
    'oracle_segmentation': ('true', 'Publish the ORACLE ladder label image'),
    'scan_topic': ('contract', 'contract: publish sensors/scan; sim: sim/scan (preprocess owns sensors/scan)'),
    'rate': ('1.0', 'Simulation speed relative to wall time'),
    'people': ('static', 'static | walking | none'),
    'route': ('', 'Scripted route YAML (empty: cmd_vel control)'),
    'scenario': ('', 'Scenario YAML (route/wait/teleport/fault steps)'),
    'hidden_ladders': ('', 'Comma-separated ladder names hidden at start (e.g. ladder_07)'),
    'seed': ('', 'Noise seed override'),
    'out': ('', 'New folder for run metadata/evaluation logs (under runs/)'),
}


def _bridge(context):
    get = lambda k: context.launch_configurations.get(k, ARGS[k][0])  # noqa: E731
    if get('calibration_profile') not in ('', 'sim'):
        raise RuntimeError('ultron_b2_sim requires calibration_profile:=sim')
    share = get_package_share_directory('ultron_b2_sim')
    camera_cfg = get('camera_config') or os.path.join(share, 'config', 'head_camera_sim.yaml')
    calibration = os.path.join(get_package_share_directory('ultron_b2_description'), 'calibration', 'sim.yaml')
    args = ['--odom-source', get('odom_source'), '--calibration', calibration, '--camera-config', camera_cfg,
            '--rate', get('rate'), '--people', get('people'), '--scan-topic', get('scan_topic')]
    for key, flag in (('camera', '--no-camera'), ('imu', '--no-imu'),
                      ('oracle_segmentation', '--no-oracle-segmentation')):
        if get(key).lower() != 'true':
            args.append(flag)
    for key in ('route', 'scenario', 'hidden_ladders', 'seed', 'out'):
        if get(key):
            args += ['--' + key.replace('_', '-'), get(key)]
    node = Node(package='ultron_b2_sim', executable='sim_contract_bridge', name='ultron_sim_bridge',
                output='screen', arguments=args, parameters=[{'use_sim_time': False}])
    actions = [node]
    if get('namespace'):
        actions = [PushRosNamespace(get('namespace'))] + actions
    return [GroupAction(actions, scoped=True)]


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()]
    return LaunchDescription(declared + [OpaqueFunction(function=_bridge)])
