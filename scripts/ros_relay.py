#!/usr/bin/env python3
"""Robot side of the ULTRON pipeline: forward camera, depth, lidar and pose over TCP. Read-only.

Run where the robot's ROS 2 topics are visible (Humble or Jazzy), in a shell with ROS sourced:

    python3 scripts/ros_relay.py --to GPU_MACHINE:5600

It needs only rclpy, tf2_ros and numpy. It subscribes and sends; it never publishes a topic,
calls a service or commands the robot. With `--record DIR` it also starts `ros2 bag record`
(`ultron.recording`) as a separate, lower-priority process that writes a bag of the robot's
topics on this machine; the recorder only subscribes (rosbag2 itself announces split events on
/events/write_split), and if it fails the relay carries on. On the other end: `python -m ultron.pipeline patrol
--source relay:5600`. Topic defaults are those of the hackathon B2's recordings.

Each colour frame goes out with the newest depth image, both cameras' intrinsics, the
depth-to-colour transform from TF, and the robot pose from TF (map <- base frame) when
localisation publishes one. While TF has no pose, lidar clouds are forwarded too so the
pipeline can compute odometry itself.
"""
import argparse
import logging
import math
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from ultron.protocol.relay_wire import cloud_message, frame_message
from ultron.recording import add_arguments as add_record_arguments, recorder_from_args

MAX_DEPTH_AGE = 0.1  # seconds: an older depth image is not sent with a colour frame


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Relay(Node):
    def __init__(self, args):
        super().__init__("ultron_relay")
        self.args, self.socket = args, None
        self.color_k = self.depth_k = self.depth = self.depth_stamp = self.depth_frame = None
        self.extrinsics, self.last_sent, self.sent, self.pose_seen = None, 0.0, 0, False
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(CameraInfo, args.color_info, lambda m: setattr(self, "color_k", self.intrinsics(m)), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.depth_info, lambda m: setattr(self, "depth_k", self.intrinsics(m)), qos_profile_sensor_data)
        self.create_subscription(Image, args.depth, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Image, args.color, self.on_color, qos_profile_sensor_data)
        if args.cloud:
            self.create_subscription(PointCloud2, args.cloud, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(5.0, lambda: self.get_logger().info(
            f"sent {self.sent} frames, pose from TF: {'yes' if self.pose_seen else 'no (forwarding lidar)'}"))

    @staticmethod
    def intrinsics(info):
        return (info.k[0], info.k[4], info.k[2], info.k[5])

    def send(self, data):
        """Send or drop: a pipeline that is down must not block the robot's ROS callbacks."""
        try:
            if self.socket is None:
                host, port = self.args.to.rsplit(":", 1)
                self.socket = socket.create_connection((host, int(port)), timeout=3)
                self.get_logger().info(f"connected to {self.args.to}")
            self.socket.sendall(data)
            return True
        except OSError as error:
            self.get_logger().warn(f"pipeline not reachable ({error}); retrying", throttle_duration_sec=5.0)
            self.socket = None
            return False

    def pose(self):
        """Robot (x, y, yaw) in the map from TF, or None while localisation is not publishing."""
        try:
            t = self.tf.lookup_transform(self.args.map_frame, self.args.base_frame, Time(), Duration(seconds=0.0)).transform
        except TransformException:
            return None
        q = t.rotation
        self.pose_seen = True
        return (t.translation.x, t.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def depth_to_color(self, color_frame):
        """Rotation and translation taking depth-camera points into the colour camera, from TF."""
        if self.extrinsics is None and self.depth_frame:
            try:
                t = self.tf.lookup_transform(color_frame, self.depth_frame, Time(), Duration(seconds=0.0)).transform
                self.extrinsics = (rotation_matrix(t.rotation), np.array([t.translation.x, t.translation.y, t.translation.z]))
            except TransformException:
                return np.eye(3), np.zeros(3)  # not yet known: the cameras sit a few centimetres apart
        return self.extrinsics or (np.eye(3), np.zeros(3))

    def on_depth(self, message):
        if message.encoding != "16UC1":
            self.get_logger().warn(f"depth encoding {message.encoding} not supported", once=True)
            return
        rows = np.frombuffer(message.data, np.uint16).reshape(message.height, message.step // 2)
        self.depth, self.depth_stamp = rows[:, :message.width], stamp_seconds(message.header)
        self.depth_frame = message.header.frame_id

    def on_color(self, message):
        stamp = stamp_seconds(message.header)
        if stamp - self.last_sent < 1 / self.args.rate or self.color_k is None or self.depth_k is None:
            return
        if self.depth is None or abs(stamp - self.depth_stamp) > MAX_DEPTH_AGE or message.encoding not in ("rgb8", "bgr8"):
            return
        pixels = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        pixels = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
        if message.encoding == "bgr8":
            pixels = pixels[:, :, ::-1]
        rotation, translation = self.depth_to_color(message.header.frame_id)
        if self.send(frame_message(stamp, pixels, self.depth, self.color_k, self.depth_k, rotation, translation, self.pose())):
            self.last_sent, self.sent = stamp, self.sent + 1

    def on_cloud(self, message):
        if self.pose_seen:
            return  # localisation gives the pose: no need to ship clouds
        offsets = {field.name: field.offset for field in message.fields}
        if not {"x", "y", "z"} <= set(offsets) or (offsets["y"], offsets["z"]) != (offsets["x"] + 4, offsets["x"] + 8):
            self.get_logger().warn("point cloud without consecutive float32 x, y, z: not forwarded", once=True)
            return
        cloud = np.frombuffer(message.data, np.uint8).reshape(-1, message.point_step)
        points = np.ascontiguousarray(cloud[:, offsets["x"]:offsets["x"] + 12]).view(np.float32).reshape(-1, 3)
        self.send(cloud_message(stamp_seconds(message.header), points[np.isfinite(points).all(axis=1)]))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--to", required=True, help="HOST:PORT of the machine running the pipeline")
    parser.add_argument("--rate", type=float, default=8.0, help="colour frames per second to forward")
    parser.add_argument("--color", default="/camera_depth/head/color/image_raw")
    parser.add_argument("--color-info", default="/camera_depth/head/color/camera_info")
    parser.add_argument("--depth", default="/camera_depth/head/depth/image_rect_raw")
    parser.add_argument("--depth-info", default="/camera_depth/head/depth/camera_info")
    parser.add_argument("--cloud", default="/scan_cloud", help="lidar cloud for odometry; empty to disable")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--record", metavar="DIR", default=None,
                        help="also record a ROS 2 bag into DIR/<time>/ (on this machine's disk)")
    add_record_arguments(parser, prefix="record-")
    args, ros_args = parser.parse_known_args()
    recorder = None
    if args.record:  # before rclpy starts its threads: the recorder is a forked child
        logging.basicConfig(level=logging.INFO, format="[recorder] %(levelname)s %(message)s")
        recorder = recorder_from_args(args.record, args, prefix="record_")
        recorder.start()
    rclpy.init(args=ros_args)
    node = Relay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if recorder is not None:
            recorder.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
