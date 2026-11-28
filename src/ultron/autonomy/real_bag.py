"""Prepare a derived ROS bag from the recorded B2 LiDAR for offline SLAM replay."""

import array
import csv
from importlib.metadata import version
from itertools import chain
import json
import math
from pathlib import Path

import numpy as np

CLOUD = '/scan_cloud'
SCAN = '/ultron/real/scan'
ODOM = '/ultron/real/odom'
LEVEL_FRAME = 'ultron_lidar_level'
BINS = 720
ANGLE_INC = math.tau / BINS
HEIGHT_BAND = (0.25, 1.6)
MIN_RANGE, MAX_RANGE = 0.3, 20.0
CALIBRATION_CLOUDS = 10
NANOSECONDS = 1_000_000_000


def cloud_xyz(msg):
    """Decode named FLOAT32 XYZ fields, including organized clouds with row padding."""
    fields = {field.name: field for field in msg.fields}
    dtype = '>f4' if msg.is_bigendian else '<f4'
    data = memoryview(msg.data)
    if (msg.width <= 0 or msg.height <= 0 or msg.point_step <= 0
            or msg.row_step < msg.width * msg.point_step
            or len(data) != msg.row_step * msg.height):
        raise ValueError('Invalid PointCloud2 dimensions or strides')
    columns = []
    for axis in 'xyz':
        field = fields.get(axis)
        if field is None or field.datatype != 7 or field.count != 1 or not 0 <= field.offset <= msg.point_step - 4:
            raise ValueError(f'PointCloud2 requires a scalar FLOAT32 {axis} field')
        columns.append(np.ndarray((msg.height, msg.width), dtype=dtype, buffer=data,
                                  offset=field.offset, strides=(msg.row_step, msg.point_step)).ravel())
    points = np.column_stack(columns)
    return points[np.isfinite(points).all(axis=1)].astype(np.float64)


def fit_floor(points):
    """Estimate LiDAR height and a levelling rotation from the lowest dense nearby layer."""
    near = points[np.hypot(points[:, 0], points[:, 1]) < 4.0]
    if len(near) < 500:
        raise ValueError('Too few points near the lidar to find the floor')
    z0 = np.percentile(near[:, 2], 3)
    floor = near[np.abs(near[:, 2] - z0) < 0.12]
    if len(floor) < 300:
        raise ValueError('No dense floor layer found')
    centroid = floor.mean(axis=0)
    _, singular_values, axes = np.linalg.svd(floor - centroid, full_matrices=False)
    normal = axes[2] if axes[2, 2] > 0 else -axes[2]
    height = -float(centroid @ normal)
    if normal[2] < 0.85 or singular_values[1] / math.sqrt(len(floor)) < 0.15 or height <= 0:
        raise ValueError('Floor candidate is steep, degenerate, or above the lidar')
    x, y, z = np.cross(normal, (0.0, 0.0, 1.0))
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    rotation = np.eye(3) + skew + skew @ skew / (1 + normal[2])
    return normal, height, rotation, len(floor)


def flatten(points, height):
    """Nearest return along each horizontal ray, restricted to the above-floor band."""
    ranges = np.hypot(points[:, 0], points[:, 1])
    z = points[:, 2] + height
    keep = ((z >= HEIGHT_BAND[0]) & (z <= HEIGHT_BAND[1])
            & (ranges >= MIN_RANGE) & (ranges <= MAX_RANGE))
    angles = np.arctan2(points[keep, 1], points[keep, 0])
    bins = np.round((angles + math.pi) / ANGLE_INC).astype(int) % BINS
    result = np.full(BINS, np.inf, dtype=np.float32)
    np.minimum.at(result, bins, ranges[keep])
    return result


def read_clouds(source):
    from rclpy.serialization import deserialize_message
    import rosbag2_py as bag
    from sensor_msgs.msg import PointCloud2

    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(source), storage_id='sqlite3'), bag.ConverterOptions('', ''))
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if topics.get(CLOUD) != 'sensor_msgs/msg/PointCloud2':
        raise ValueError(f'Expected {CLOUD} as sensor_msgs/msg/PointCloud2 in {source}')
    reader.set_filter(bag.StorageFilter(topics=[CLOUD]))
    previous = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp
        timestamp = stamp.sec * NANOSECONDS + stamp.nanosec
        if msg.header.frame_id != 'lidar_link' or timestamp <= previous:
            raise ValueError(f'Expected increasing acquisition timestamps in lidar_link at {timestamp}')
        previous = timestamp
        points = cloud_xyz(msg)
        if not len(points):
            raise ValueError(f'Missing finite LiDAR points at {timestamp}')
        yield stamp, timestamp, points


def make_replay(source, out):
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    import rosbag2_py as bag
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    from ..protocol.bag import LidarOdometry

    clouds = read_clouds(source)
    samples = []
    for cloud in clouds:
        samples.append(cloud)
        if len(samples) == CALIBRATION_CLOUDS:
            break
    if len(samples) < CALIBRATION_CLOUDS:
        raise ValueError(f'Need at least {CALIBRATION_CLOUDS} clouds for floor estimation')
    normal, height, rotation, floor_points = fit_floor(np.vstack([sample[2] for sample in samples]))
    odometry = LidarOdometry()
    writer = bag.SequentialWriter()
    writer.open(bag.StorageOptions(uri=str(out / 'replay'), storage_id='sqlite3'), bag.ConverterOptions('', ''))
    for name, msgtype in ((SCAN, 'sensor_msgs/msg/LaserScan'), (ODOM, 'nav_msgs/msg/Odometry'),
                          ('/tf', 'tf2_msgs/msg/TFMessage'), ('/tf_static', 'tf2_msgs/msg/TFMessage')):
        writer.create_topic(bag.TopicMetadata(name, msgtype, 'cdr'))

    first_stamp, first_ns, _ = samples[0]
    mount = TransformStamped()
    mount.header.frame_id, mount.child_frame_id = 'base_link', LEVEL_FRAME
    mount.header.stamp = first_stamp
    mount.transform.translation.z = height
    mount.transform.rotation.w = 1.0
    writer.write('/tf_static', serialize_message(TFMessage(transforms=[mount])), first_ns)

    with (out / 'odometry.csv').open('w', newline='') as handle:
        log = csv.writer(handle)
        log.writerow(('t', 'x', 'y', 'yaw'))
        for count, (stamp, timestamp, points) in enumerate(chain(samples, clouds), start=1):
            levelled = points @ rotation.T
            x, y, yaw = odometry.update(levelled)
            if not all(map(math.isfinite, (x, y, yaw))):
                raise ValueError(f'Nonfinite odometry at {timestamp}')
            pose = TransformStamped()
            pose.header.stamp, pose.header.frame_id, pose.child_frame_id = stamp, 'odom', 'base_link'
            pose.transform.translation.x, pose.transform.translation.y = x, y
            pose.transform.rotation.z, pose.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            writer.write('/tf', serialize_message(TFMessage(transforms=[pose])), timestamp)
            odom = Odometry(header=pose.header, child_frame_id=pose.child_frame_id)
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            odom.pose.pose.orientation = pose.transform.rotation
            writer.write(ODOM, serialize_message(odom), timestamp)
            log.writerow((f'{stamp.sec}.{stamp.nanosec:09d}', x, y, yaw))

            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, LEVEL_FRAME
            scan.angle_min, scan.angle_increment = -math.pi, ANGLE_INC
            scan.angle_max = -math.pi + (BINS - 1) * ANGLE_INC
            scan.range_min, scan.range_max = MIN_RANGE, MAX_RANGE
            scan.scan_time = (timestamp - previous_ns) / NANOSECONDS if count > 1 else 0.0
            scan.ranges = array.array('f', flatten(levelled, height))
            writer.write(SCAN, serialize_message(scan), timestamp)
            previous_ns = timestamp
    del writer  # finalize and write rosbag2 metadata before starting playback

    provenance = {
        'source_bag': str(source.resolve()), 'source_topic': CLOUD, 'source_frame': 'lidar_link',
        'clock': 'recorded lidar header acquisition timestamps',
        'clouds': count, 'duration_s': (previous_ns - first_ns) / NANOSECONDS,
        'mean_cloud_hz': (count - 1) * NANOSECONDS / (previous_ns - first_ns),
        'floor_normal_lidar_frame': normal.tolist(), 'lidar_to_level_rotation': rotation.tolist(),
        'estimated_lidar_height_m': height, 'floor_points': floor_points,
        'calibration_clouds': CALIBRATION_CLOUDS, 'height_band_m': HEIGHT_BAND,
        'calibration_source': 'floor fit on the beginning of this recording',
        'odom_source': 'KISS-ICP on levelled lidar clouds', 'kiss_icp_version': version('kiss-icp'),
        'base_frame_origin': 'vertically below the lidar, projected onto the estimated floor',
        'ground_truth_available': False,
    }
    (out / 'frontend_calibration.json').write_text(json.dumps(provenance, indent=2) + '\n')
    return provenance
