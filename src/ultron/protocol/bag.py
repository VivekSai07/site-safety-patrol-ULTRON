"""Read a recorded ROS 2 bag from the B2 without a ROS installation (pip package `rosbags`).

Topic names and frames are those of the hackathon robot's recordings:
    /camera_depth/head/color/image_raw        rgb8, with camera_info
    /camera_depth/head/depth/image_rect_raw   16UC1 millimetres, its own camera and resolution
    /camera_depth/head/extrinsics/depth_to_color
    /tf  world -> imu_link                    orientation only

    /scan_cloud                               PointCloud2 in lidar_link, 10 Hz

The recordings carry no robot pose and no camera-to-lidar transform. Poses therefore come
from lidar odometry over /scan_cloud (pip package `kiss-icp`), interpolated to each colour
frame, with the camera assumed to look along the lidar's +x from the lidar's position.
`pose=imu` instead gives (0, 0, IMU yaw): only right while the robot turns on the spot.
`lidar=True` (source option `lidar=1`) also gives each frame the latest scan as `frame.cloud`: Nx3
points in the odometry frame, moved there with that scan's own odometry pose (z up, the floor about
0.74 m below the first scan's lidar). Not `frame.points`, which would make `lift` range from lidar.
"""
import math
from pathlib import Path

import numpy as np
from PIL import Image

from .frames import Frame
from .lidar import Calibration

COLOR, DEPTH = "/camera_depth/head/color/image_raw", "/camera_depth/head/depth/image_rect_raw"
COLOR_INFO, DEPTH_INFO = "/camera_depth/head/color/camera_info", "/camera_depth/head/depth/camera_info"
DEPTH_TO_COLOR, TF = "/camera_depth/head/extrinsics/depth_to_color", "/tf"
SCAN = "/scan_cloud"
LIDAR_MIN_RANGE, LIDAR_MAX_RANGE = 0.6, 30.0  # metres; closer returns are the robot's own body
MAX_DEPTH_AGE = 0.1  # seconds: an older depth image is not paired with a colour frame
DEPTH_SCALE = 2      # registered depth is stored at 1/DEPTH_SCALE of the colour resolution


def register_depth(depth_m, depth_k, color_k, color_size, rotation, translation, scale=DEPTH_SCALE):
    """Depth image re-projected into the colour camera, in metres; 0 where nothing was measured.

    `depth_k` and `color_k` are (fx, fy, cx, cy); rotation (3x3) and translation (3,) map
    depth-camera points into the colour camera. Output is colour size / scale.
    """
    height, width = depth_m.shape
    v, u = np.nonzero(depth_m > 0)
    z = depth_m[v, u]
    fx, fy, cx, cy = depth_k
    points = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], axis=1) @ np.asarray(rotation).T + translation
    fx, fy, cx, cy = color_k
    front = points[:, 2] > 0.05
    points = points[front]
    out_w, out_h = color_size[0] // scale, color_size[1] // scale
    col = np.round((fx * points[:, 0] / points[:, 2] + cx) / scale).astype(int)
    row = np.round((fy * points[:, 1] / points[:, 2] + cy) / scale).astype(int)
    inside = (col >= 0) & (col < out_w) & (row >= 0) & (row < out_h)
    registered = np.full(out_h * out_w, np.inf, dtype=np.float32)
    np.minimum.at(registered, row[inside] * out_w + col[inside], points[inside, 2].astype(np.float32))
    registered[~np.isfinite(registered)] = 0.0
    return registered.reshape(out_h, out_w)


def _typestore():
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    store = get_typestore(Stores.ROS2_HUMBLE)
    store.register(get_types_from_msg("float64[9] rotation\nfloat64[3] translation\n",
                                      "realsense2_camera_msgs/msg/Extrinsics"))
    return store


def _intrinsics(info):
    return (info.k[0], info.k[4], info.k[2], info.k[5])


def calibration(path):
    """Colour camera `Calibration` from the bag's camera_info. No lidar transform is recorded."""
    from rosbags.rosbag2 import Reader
    store = _typestore()
    with Reader(Path(path)) as reader:
        for connection, _, raw in reader.messages([c for c in reader.connections if c.topic == COLOR_INFO]):
            info = store.deserialize_cdr(raw, connection.msgtype)
            return Calibration(_intrinsics(info), (info.width, info.height))
    raise ValueError(f"No {COLOR_INFO} in {path}")


class LidarOdometry:
    """Robot pose from successive lidar scans (KISS-ICP), in the frame of the first scan.

    A stand-in for SLAM: it drifts over long distances and has no map, but needs nothing
    except the scans. On the hall recording a 17 m loop closed to within 15 cm.
    """

    def __init__(self):
        from kiss_icp.config import KISSConfig
        from kiss_icp.kiss_icp import KissICP
        config = KISSConfig()
        config.data.min_range, config.data.max_range, config.data.deskew = LIDAR_MIN_RANGE, LIDAR_MAX_RANGE, False
        config.mapping.voxel_size = LIDAR_MAX_RANGE / 100
        self._odometry = KissICP(config)

    def update(self, points):
        """Feed one scan (Nx3, lidar frame). Returns the pose (x, y, yaw) after it."""
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        self._odometry.register_frame(points, np.zeros(len(points)))
        pose = self.matrix
        return float(pose[0, 3]), float(pose[1, 3]), math.atan2(pose[1, 0], pose[0, 0])

    @property
    def matrix(self):
        """The full 4x4 pose after the last scan (roll, pitch and height included)."""
        return np.array(self._odometry.last_pose, dtype=np.float64)


def _scan_points(message):
    cloud = np.frombuffer(message.data, np.uint8).reshape(-1, message.point_step)
    return np.ascontiguousarray(cloud[:, :12]).view(np.float32).reshape(-1, 3)


def lidar_trajectory(path, matrices=None):
    """Lidar odometry over the whole bag: arrays (stamps, x, y, unwrapped yaw) in the first scan's frame.
    A dict passed as `matrices` is filled with {bag time in ns: 4x4 pose} per scan."""
    from rosbags.rosbag2 import Reader
    store = _typestore()
    odometry, rows = LidarOdometry(), []
    with Reader(Path(path)) as reader:
        for connection, nanoseconds, raw in reader.messages([c for c in reader.connections if c.topic == SCAN]):
            message = store.deserialize_cdr(raw, connection.msgtype)
            rows.append((nanoseconds / 1e9, *odometry.update(_scan_points(message))))
            if matrices is not None:
                matrices[nanoseconds] = odometry.matrix
    if not rows:
        raise ValueError(f"No {SCAN} in {path}")
    stamps, x, y, yaw = np.array(rows).T
    return stamps, x, y, np.unwrap(yaw)


def frames(path, rate=2.0, pose="lidar", lidar=False):
    """Colour frames at about `rate` per second, each with registered depth and a pose.

    pose: "lidar" (odometry from the scans), "imu" (heading only) or "none".
    lidar: also attach the latest scan as `frame.cloud` in the odometry frame (needs pose="lidar").
    """
    from rosbags.rosbag2 import Reader
    from ..pipeline.discover import world_cloud
    store = _typestore()
    matrices = {} if lidar and pose == "lidar" else None
    trajectory = lidar_trajectory(path, matrices) if pose == "lidar" else None
    wanted = (COLOR, DEPTH, COLOR_INFO, DEPTH_INFO, DEPTH_TO_COLOR, TF) + ((SCAN,) if matrices is not None else ())
    scan = None  # (bag time in ns, raw message) of the latest scan: deserialised only when a frame takes it
    color_info = depth_info = None
    rotation, translation = np.eye(3), np.zeros(3)
    depth, depth_stamp, yaw, last, count = None, None, None, None, 0
    with Reader(Path(path)) as reader:
        for connection, nanoseconds, raw in reader.messages([c for c in reader.connections if c.topic in wanted]):
            topic, stamp = connection.topic, nanoseconds / 1e9
            if topic == COLOR and last is not None and stamp - last < 1 / rate:
                continue  # skip before paying for deserialisation
            if topic == SCAN:
                scan = (nanoseconds, connection, raw)
                continue
            if topic in (COLOR_INFO, DEPTH_INFO) and (color_info if topic == COLOR_INFO else depth_info) is not None:
                continue
            message = store.deserialize_cdr(raw, connection.msgtype)
            if topic == COLOR_INFO:
                color_info = message
            elif topic == DEPTH_INFO:
                depth_info = message
            elif topic == DEPTH_TO_COLOR:
                rotation = np.asarray(message.rotation).reshape(3, 3, order="F")  # librealsense is column-major
                translation = np.asarray(message.translation)
            elif topic == TF:
                for transform in message.transforms:
                    if transform.child_frame_id == "imu_link":
                        q = transform.transform.rotation
                        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            elif topic == DEPTH:
                millimetres = np.frombuffer(message.data, np.uint16).reshape(message.height, message.width)
                depth, depth_stamp = millimetres.astype(np.float32) / 1000, stamp
            elif topic == COLOR:
                pixels = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
                pixels = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
                if message.encoding == "bgr8":
                    pixels = pixels[:, :, ::-1]
                registered = None
                if (depth is not None and color_info is not None and depth_info is not None
                        and stamp - depth_stamp <= MAX_DEPTH_AGE):
                    registered = register_depth(depth, _intrinsics(depth_info), _intrinsics(color_info),
                                                (message.width, message.height), rotation, translation)
                last, count = stamp, count + 1
                robot = None
                if trajectory is not None:
                    robot = tuple(float(np.interp(stamp, trajectory[0], values)) for values in trajectory[1:])
                elif pose == "imu" and yaw is not None:
                    robot = (0.0, 0.0, yaw)
                camera = Calibration(_intrinsics(color_info), (message.width, message.height)) if color_info else None
                frame = Frame(Image.fromarray(pixels), name=f"bag-{count:05}", stamp=stamp, pose=robot,
                              pose_frame="odometry" if robot else None,
                              depth=registered, camera=camera)
                if scan is not None and scan[0] in matrices:
                    points = _scan_points(store.deserialize_cdr(scan[2], scan[1].msgtype))
                    points = points[np.isfinite(points).all(axis=1)]
                    reach = np.linalg.norm(points[:, :2], axis=1)
                    points = points[(reach >= LIDAR_MIN_RANGE) & (reach <= LIDAR_MAX_RANGE)]  # not the robot's body
                    frame.cloud = world_cloud(points, matrices[scan[0]])
                yield frame
