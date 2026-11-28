"""Where frames come from. Every source is an iterable of `Frame`.

    folder:PATH                    images in name order (recorded data, the nesseler photos)
    bag:PATH[,rate=2][,pose=lidar][,lidar=1] recorded ROS 2 bag from the B2: colour, registered depth, lidar odometry
                                   (lidar=1: each frame also carries the scan as `cloud`, odometry frame)
    relay[:PORT]                   live from the robot through scripts/ros_relay.py (default port 5600)
    cv:0 | cv:rtsp://... | cv:FILE OpenCV capture: webcam, network stream or video file
    ros2:/image[,pose=/odom][,cloud=/points]   ROS 2 topics (Image or CompressedImage)
    unitree[:INTERFACE]            B2 front camera through unitree_sdk2py

Live sources always hand out the newest frame and drop the ones nobody had time for.
Only reading: nothing here publishes a topic or sends a command to the robot.
The ROS 2 and Unitree sources could not be run without the robot; check topic names
and message types against `ros2 topic list -t` on the first hardware session.
"""
import math
from pathlib import Path
import threading
import time

import numpy as np
from PIL import Image

from .frames import Frame

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


class Latest:
    """A one-slot mailbox: writers overwrite, the reader waits for something new."""

    def __init__(self):
        self._condition = threading.Condition()
        self._item, self._closed = None, False

    def put(self, item):
        with self._condition:
            self._item = item
            self._condition.notify()

    def close(self):
        with self._condition:
            self._closed = True
            self._condition.notify()

    def __iter__(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._item is not None or self._closed)
                if self._item is None:
                    return
                item, self._item = self._item, None
            yield item


def folder(path, pace=0.0):
    """Images of a directory in name order, `pace` seconds apart."""
    files = sorted(p for p in Path(path).iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    if not files:
        raise ValueError(f"No images in {path}")
    for file in files:
        yield Frame.from_file(file)
        time.sleep(pace)


def opencv(target):
    """Webcam index, stream URL, GStreamer pipeline or video file."""
    import cv2
    capture = cv2.VideoCapture(int(target) if str(target).isdigit() else target)
    if not capture.isOpened():
        raise ValueError(f"OpenCV cannot open {target}")
    is_file = Path(str(target)).is_file()
    mailbox, count = Latest(), 0

    def read():
        nonlocal count
        while True:
            ok, bgr = capture.read()
            if not ok:
                break
            count += 1
            stamp = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000 if is_file else time.time()
            mailbox.put(Frame(Image.fromarray(bgr[:, :, ::-1]), name=f"frame-{count:06}", stamp=stamp))
            if is_file:
                time.sleep(1 / (capture.get(cv2.CAP_PROP_FPS) or 30))
        mailbox.close()

    threading.Thread(target=read, daemon=True).start()
    return mailbox


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def image_from_ros(message):
    """sensor_msgs/Image or CompressedImage to a PIL image."""
    if hasattr(message, "format"):  # CompressedImage
        return Frame.from_bytes(bytes(message.data)).image
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(message.encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding {message.encoding}")
    rows = np.frombuffer(bytes(message.data), np.uint8).reshape(message.height, message.step)
    pixels = rows[:, :message.width * channels].reshape(message.height, message.width, channels)
    if message.encoding.startswith("bgr"):
        pixels = pixels[:, :, [2, 1, 0]]
    return Image.fromarray(pixels[:, :, 0] if channels == 1 else pixels[:, :, :3]).convert("RGB")


def ros2(image_topic, pose=None, cloud=None):
    """Subscribe to an image topic, optionally with a pose topic and a lidar point cloud.

    Pose may be nav_msgs/Odometry, geometry_msgs/PoseStamped or Unitree's SportModeState.
    """
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = rclpy.create_node("ultron_protocol_reader")
    mailbox, state, count = Latest(), {"pose": None, "points": None}, [0]

    def topic_type(topic):
        for _ in range(50):  # discovery takes a moment
            types = dict(node.get_topic_names_and_types()).get(topic)
            if types:
                return get_message(types[0])
            rclpy.spin_once(node, timeout_sec=0.1)
        raise ValueError(f"No publisher on {topic}")

    def on_image(message):
        count[0] += 1
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        mailbox.put(Frame(image_from_ros(message), name=f"frame-{count[0]:06}", stamp=stamp or time.time(),
                          pose=state["pose"], points=state["points"]))

    def on_pose(message):
        if hasattr(message, "imu_state"):  # unitree_go/SportModeState
            state["pose"] = (message.position[0], message.position[1], message.imu_state.rpy[2])
            return
        p = message.pose.pose if hasattr(message.pose, "pose") else message.pose
        state["pose"] = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_cloud(message):
        from sensor_msgs_py import point_cloud2
        points = point_cloud2.read_points_numpy(message, ["x", "y", "z"], skip_nans=True)
        state["points"] = np.asarray(points, dtype=np.float32)

    node.create_subscription(topic_type(image_topic), image_topic, on_image, qos_profile_sensor_data)
    if pose:
        node.create_subscription(topic_type(pose), pose, on_pose, qos_profile_sensor_data)
    if cloud:
        node.create_subscription(topic_type(cloud), cloud, on_cloud, qos_profile_sensor_data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    return mailbox


def unitree(interface=None, rate=5.0):
    """B2 front camera JPEGs through Unitree's video service."""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.b2.front_video.front_video_client import FrontVideoClient

    ChannelFactoryInitialize(0, interface) if interface else ChannelFactoryInitialize(0)
    client = FrontVideoClient()
    client.SetTimeout(3.0)
    client.Init()
    count = 0
    while True:
        code, data = client.GetImageSample()
        if code == 0:
            count += 1
            yield Frame.from_bytes(bytes(data), name=f"frame-{count:06}")
        time.sleep(1 / rate)


def relay(port, host="0.0.0.0", log=print):
    """Frames from the robot-side relay (`scripts/ros_relay.py`) over TCP.

    The relay sends colour, depth, intrinsics and, if TF has it, the robot pose in the map.
    Without a TF pose the lidar clouds it also sends are turned into odometry here. Returns
    when the relay disconnects.
    """
    import socket

    from .bag import LidarOdometry, register_depth
    from .lidar import Calibration
    from .relay_wire import unpack

    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, int(port)))
    server.listen(1)
    log(f"Waiting for the relay on port {port} ...")
    mailbox = Latest()

    def serve():
        connection, peer = server.accept()
        log(f"Relay connected from {peer[0]}")
        odometry, lidar_pose, count = None, None, 0
        try:
            while True:
                header, blobs = unpack(connection.recv)
                if header["kind"] == "cloud":
                    odometry = odometry or LidarOdometry()
                    lidar_pose = odometry.update(np.frombuffer(blobs["points"], np.float32).reshape(-1, 3))
                    continue
                color = np.frombuffer(blobs["color"], np.uint8).reshape(header["color_shape"])
                depth = np.frombuffer(blobs["depth"], np.uint16).reshape(header["depth_shape"]).astype(np.float32) / 1000
                size = (color.shape[1], color.shape[0])
                registered = register_depth(depth, header["depth_k"], header["color_k"], size,
                                            np.asarray(header["rotation"]).reshape(3, 3), np.asarray(header["translation"]))
                count += 1
                pose = header["pose"] or lidar_pose  # a pose in the SLAM map beats local odometry
                mailbox.put(Frame(Image.fromarray(color), name=f"live-{count:05}", stamp=header["stamp"],
                                  pose=tuple(pose) if pose else None, depth=registered,
                                  pose_frame="map" if header["pose"] else ("odometry" if pose else None),
                                  camera=Calibration(header["color_k"], size)))
        except (EOFError, OSError, ValueError) as error:
            log(f"Relay stream ended: {error}")
        finally:
            connection.close()
            server.close()
            mailbox.close()

    threading.Thread(target=serve, daemon=True).start()
    return mailbox


def open_source(spec, pace=0.0):
    """Frame iterable for a source spec such as "folder:photos" or "ros2:/camera/image_raw"."""
    kind, _, rest = spec.partition(":")
    if kind == "folder":
        return folder(rest, pace)
    if kind == "bag":
        from . import bag
        path, *options = rest.split(",")
        settings = dict(option.split("=", 1) for option in options)
        return bag.frames(path, float(settings.get("rate", 2.0)), settings.get("pose", "lidar"),
                          lidar=settings.get("lidar") == "1")
    if kind == "sim":
        from . import sim
        path, *options = rest.split(",")
        settings = dict(option.split("=", 1) for option in options)
        error = tuple(float(v) for v in settings["error"].split("/")) if "error" in settings else None
        depth = {"noisy": "depth_noisy"}.get(settings.get("depth"), "depth")
        return sim.frames(path, float(settings.get("rate", 2.0)), error, depth, lidar=settings.get("lidar") == "1")
    if kind == "relay":
        return relay(rest or 5600)
    if kind == "cv":
        return opencv(rest)
    if kind == "ros2":
        topic, *options = rest.split(",")
        return ros2(topic, **dict(option.split("=", 1) for option in options))
    if kind == "unitree":
        return unitree(rest or None)
    raise ValueError(f"Unknown source {spec!r}")
