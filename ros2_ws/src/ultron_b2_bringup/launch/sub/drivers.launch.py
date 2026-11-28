"""Sub-launch `drivers` (owner: Agent B for replay/live, Agent F for sim).

replay: `ros2 bag play --clock` of the real recording with vendor topics remapped into the contract
        (ultron_b2_bringup.replay_topics). The player is a plain process (node `/rosbag2_player`, root
        namespace); its remap targets are absolute under the mode namespace. Without `bag:=` nothing plays.
live:   vendor drivers (rslidar_sdk Helios-32, Xsens MTi, RealSense D457 head) with configs in
        config/drivers/. WRITTEN, NEVER RUN against hardware; live needs explicit operator authority.
sim:    the MuJoCo contract bridge (ultron_b2_sim) publishes /clock, sensors/*, state/odom (only with
        odom_source:=sim) and sim/ground_truth/* and consumes `sim/cmd_vel` from the sim_driver. It never
        publishes base_link->sensor TF (robot_state_publisher owns it).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ultron_b2_bringup.launch_common import PKG, SUB_ARGS
from ultron_b2_bringup.replay_topics import REPLAY_ARGS, bag_remaps, bag_topics

CLOCK_HZ = '100'
LIVE_CAMERA_TOPICS = ('color/image_raw', 'color/camera_info', 'color/metadata', 'depth/image_rect_raw',
                      'depth/camera_info', 'depth/metadata', 'extrinsics/depth_to_color')


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes')


def replay_command(bag, namespace, rate='1.0', start_offset='0.0', delay='2.0', loop=False, cameras=True):
    cmd = ['ros2', 'bag', 'play', os.path.expanduser(bag), '--clock', CLOCK_HZ, '--rate', rate,
           '--start-offset', start_offset, '--delay', delay, '--disable-keyboard-controls',
           '--topics', *bag_topics(cameras), '--remap', *bag_remaps(namespace, cameras)]
    if loop:
        cmd.append('--loop')
    return cmd


def _replay(context):
    cfg = context.launch_configurations
    bag = cfg.get('bag', '').strip()
    if not bag:
        return [LogInfo(msg='[drivers] replay: no bag:=<path> given; nothing is played')]
    namespace = cfg.get('ros_namespace', '/ultron/replay')
    cameras = _truthy(cfg.get('replay_cameras', 'true'))
    cmd = replay_command(bag, namespace, cfg.get('replay_rate', '1.0'), cfg.get('replay_start_offset', '0.0'),
                         cfg.get('replay_delay', '2.0'), _truthy(cfg.get('replay_loop', 'false')), cameras)
    return [LogInfo(msg=f'[drivers] replay: {bag} -> {namespace} (cameras={cameras})'),
            ExecuteProcess(name='bag_player', cmd=cmd, output='screen')]


def _live(_context):
    cfg_dir = os.path.join(get_package_share_directory(PKG), 'config', 'drivers')
    lidar = Node(package='rslidar_sdk', executable='rslidar_sdk_node', name='rslidar_sdk', output='screen',
                 parameters=[{'config_path': os.path.join(cfg_dir, 'rslidar_helios32.yaml')}],
                 remappings=[('/scan_cloud', 'sensors/lidar/points'),
                             ('/rslidar_packets', 'sensors/lidar/packets')])
    imu = Node(package='xsens_mti_ros2_driver', executable='xsens_mti_node', name='xsens_mti', output='screen',
               parameters=[os.path.join(cfg_dir, 'xsens_mti.yaml')],
               remappings=[('imu/data', 'sensors/imu/data'), ('/imu/data', 'sensors/imu/data')])
    camera = Node(package='realsense2_camera', executable='realsense2_camera_node', name='cam_head', output='screen',
                  parameters=[os.path.join(cfg_dir, 'realsense_d457_head.yaml')],
                  remappings=[(f'~/{t}', f'sensors/cam_head/{t}') for t in LIVE_CAMERA_TOPICS])
    return [LogInfo(msg='[drivers] live: vendor drivers (configs UNVERIFIED on hardware)'), lidar, imu, camera]


def _sim(context):
    cfg = context.launch_configurations
    source = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('ultron_b2_sim'), 'launch', 'sim_bridge.launch.py'))
    args = {'odom_source': cfg.get('odom_source', 'sim'),
            'calibration_profile': cfg.get('calibration_profile', 'sim'),
            # preprocess cloud_to_scan is the single sensors/scan publisher; the bridge's own scan goes to sim/scan.
            'scan_topic': 'sim'}
    for key in ('sim_rate', 'sim_people', 'sim_route', 'sim_scenario', 'sim_hidden_ladders', 'sim_scan_topic',
                'sim_camera'):
        if cfg.get(key):
            args[key[len('sim_'):]] = cfg[key]
    return [IncludeLaunchDescription(source, launch_arguments=list(args.items()))]


def _dispatch(context):
    run_mode = context.launch_configurations.get('run_mode', '')
    return {'replay': _replay, 'live': _live}.get(run_mode, _sim)(context)


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
    declared += [DeclareLaunchArgument(k, default_value=d, description=h) for k, (d, h) in REPLAY_ARGS.items()]
    return LaunchDescription(declared + [OpaqueFunction(function=_dispatch)])
