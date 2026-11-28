"""Topic table for real-bag replay and live drivers -> ULTRON topic contract (owner: Agent B).

Source of the vendor names: docs/b2_ros_stack/BAG_FACTS.md (recording rosbag2_2026_11_28-00_48_42).
Contract names are relative (PLAN.md section 3.2); the mode namespace is prepended at launch.

TF decision (replay): the bag's only dynamic edge is the Xsens filter orientation `world -> imu_link`
(zero translation, not odometry). Playing it on /tf would give `imu_link` a second parent next to the
description's `base_link -> imu_link`, so /tf is remapped off the TF tree to `bag/tf_xsens` (kept for
diagnostics). The bag's /tf_static holds only RealSense-internal frames (`head_link -> head_*`); it is
played unchanged and the description attaches `head_link` under `cam_head_link`.
"""

LIDAR = {'/scan_cloud': 'sensors/lidar/points'}
IMU = {'/imu/data': 'sensors/imu/data'}
CAMERA = {
    '/camera_depth/head/color/image_raw': 'sensors/cam_head/color/image_raw',
    '/camera_depth/head/color/camera_info': 'sensors/cam_head/color/camera_info',
    '/camera_depth/head/color/metadata': 'sensors/cam_head/color/metadata',
    '/camera_depth/head/depth/image_rect_raw': 'sensors/cam_head/depth/image_rect_raw',
    '/camera_depth/head/depth/camera_info': 'sensors/cam_head/depth/camera_info',
    '/camera_depth/head/depth/metadata': 'sensors/cam_head/depth/metadata',
    '/camera_depth/head/extrinsics/depth_to_color': 'sensors/cam_head/extrinsics/depth_to_color',
}
# Bag /tf (Xsens world->imu_link, orientation only) is moved off the TF tree.
BAG_TF = {'/tf': 'bag/tf_xsens'}
# Played unchanged (absolute): RealSense-internal static frames.
PASSTHROUGH = ('/tf_static',)
# Recorded but never replayed by default: raw packets (58 % complete, /scan_cloud is the source of
# record), Xsens auxiliary outputs and /status (definition not available in the dev image).
NOT_REPLAYED = ('/rslidar_packets', '/imu/acceleration', '/imu/angular_velocity', '/imu/mag', '/imu/time_ref',
                '/filter/quaternion', '/filter/euler', '/status', '/rosout')

# Replay-only launch arguments: name -> (default, description).
REPLAY_ARGS = {
    'replay_rate': ('1.0', 'Playback rate'),
    'replay_loop': ('false', 'Loop the bag (sim time jumps back on each loop)'),
    'replay_start_offset': ('0.0', 'Start this many seconds into the bag'),
    'replay_delay': ('2.0', 'Seconds to wait before playing so subscribers can discover the player'),
    'replay_cameras': ('true', 'Also play the head RealSense topics (~110 MB/s at 1x)'),
}


def _absolute(namespace, relative):
    namespace = '/' + namespace.strip('/') if namespace.strip('/') else ''
    return f'{namespace}/{relative}'


def replay_table(cameras=True):
    """{bag topic: relative contract topic} for the topics that are remapped during replay."""
    table = dict(LIDAR)
    table.update(IMU)
    if cameras:
        table.update(CAMERA)
    table.update(BAG_TF)
    return table


def bag_topics(cameras=True):
    """Topics selected for `ros2 bag play --topics`."""
    return sorted(replay_table(cameras)) + list(PASSTHROUGH)


def bag_remaps(namespace, cameras=True):
    """`old:=new` rules for `ros2 bag play --remap`; targets are absolute (the player is not namespaced)."""
    return [f'{src}:={_absolute(namespace, dst)}' for src, dst in sorted(replay_table(cameras).items())]
