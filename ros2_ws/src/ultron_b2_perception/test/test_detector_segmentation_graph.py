"""Simulation-only detector graph: detections require matching instance-label frames."""
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from ament_index_python.packages import get_package_prefix  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from ultron_b2_interfaces.msg import LadderDetectionArray  # noqa: E402
from ultron_b2_perception.ros_util import numpy_to_image  # noqa: E402


def test_segmentation_detector_requires_exact_image_pair():
    if os.environ.get('ROS_DOMAIN_ID') != '71' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        pytest.skip('graph test requires isolated simulation domain 71')
    cfg = Path(__file__).resolve().parents[1] / 'config' / 'oracle_hall_ladders.yaml'
    executable = Path(get_package_prefix('ultron_b2_perception')) / 'lib/ultron_b2_perception/ladder_detector'
    ns = '/ultron/segmentation_test'
    proc = subprocess.Popen(
        [str(executable), '--ros-args', '-r', f'__ns:={ns}', '-p', 'run_mode:=sim',
         '-p', 'backend:=oracle_sim', '-p', 'oracle_source:=segmentation',
         '-p', f'oracle_ladders_file:={cfg}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = Node('segmentation_test', namespace=ns)
    color_pub = node.create_publisher(Image, 'sensors/cam_head/color/image_raw', 10)
    label_pub = node.create_publisher(Image, 'sim/oracle/cam_head/ladder_labels', 10)
    received = []
    node.create_subscription(LadderDetectionArray, 'perception/ladder_detections', received.append, 10)

    def spin_until(condition, seconds=10):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not condition():
            rclpy.spin_once(node, timeout_sec=0.05)
        return condition()

    try:
        assert spin_until(lambda: color_pub.get_subscription_count() and label_pub.get_subscription_count())
        stamp = node.get_clock().now().to_msg()
        color = numpy_to_image(np.zeros((30, 30, 3), dtype=np.uint8), 'rgb8', stamp, 'camera')
        labels = np.zeros((30, 30), dtype=np.uint16)
        labels[5:20, 6:18] = 1
        label = numpy_to_image(labels, '16UC1', stamp, 'camera')
        color_pub.publish(color)
        assert not spin_until(lambda: received, 0.5)
        label_pub.publish(label)
        assert spin_until(lambda: received)
        assert len(received[0].detections) == 1
        detection = received[0].detections[0]
        assert detection.is_oracle and detection.label == 'ORACLE_SIM:LAYHER_A_FRAME_STEPLADDER'
        assert (detection.bbox_x_min, detection.bbox_y_min,
                detection.bbox_x_max, detection.bbox_y_max) == (6.0, 5.0, 18.0, 20.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
