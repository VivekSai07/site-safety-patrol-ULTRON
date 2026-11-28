"""ladder_detector node.

Input: ``sensors/cam_head/color/image_raw`` (+ camera_info). Output: ``perception/ladder_detections``
(LadderDetectionArray) and a latched ``perception/ladder_detector/status`` string.

``backend:=oracle_sim`` is simulation ground truth: every detection has ``backend=oracle_sim``,
``is_oracle=true`` and label ``ORACLE_SIM:<type>``. It is refused outside ``run_mode:=sim``.
Learned backends need locally supplied weights; without them the node reports UNAVAILABLE and
publishes no detections (no oracle fallback).
"""
import hashlib
import sys

import numpy as np
import yaml

from .backends import BackendUnavailable, make_backend
from .geometry import intrinsics
from .oracle import project_ladders, segmentation_boxes
from .ros_util import image_to_numpy, pose_to_matrix, run_node, sensor_qos, stamp_to_float, tf_to_matrix


def load_oracle_ladders(path):
    with open(path) as fh:
        data = yaml.safe_load(fh)
    if data.get('source') != 'SIMULATION_GROUND_TRUTH_ORACLE':
        raise ValueError('oracle ladder file must declare source: SIMULATION_GROUND_TRUTH_ORACLE')
    return data['ladders']


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from ultron_b2_interfaces.msg import LadderDetection, LadderDetectionArray

    class LadderDetector(Node):
        def __init__(self):
            super().__init__('ladder_detector')
            p = self.declare_parameter
            self.backend_name = p('backend', 'oracle_sim').value
            self.run_mode = p('run_mode', 'sim').value
            self.max_rate = float(p('max_rate_hz', 5.0).value)            # PROPOSED
            self.oracle_source = p('oracle_source', 'projection').value  # projection | segmentation
            self.oracle_file = p('oracle_ladders_file', '').value
            self.sensor_qos = sensor_qos(self)
            self.oracle_pose_tol = float(p('oracle_pose_tolerance_s', 0.1).value)
            self.base_frame = p('base_frame', 'base_link').value
            self.max_range = float(p('max_range_m', 8.0).value)            # PROPOSED
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_pub = self.create_publisher(String, 'perception/ladder_detector/status', latched)
            self.pub = self.create_publisher(LadderDetectionArray, 'perception/ladder_detections', 10)
            self.K = None
            self.gt_poses = []
            self.last_t = -1e9
            self.seq = 0
            self.depth = None
            try:
                self.backend = make_backend(self.backend_name, self.run_mode,
                                            weights=p('weights', '').value, model_dir=p('model_dir', '').value,
                                            threshold=float(p('score_threshold', 0.25).value))
                if self.backend.is_oracle:
                    self._init_oracle()
            except (BackendUnavailable, OSError, ValueError, KeyError) as exc:
                self.backend = None
                self._status(f'UNAVAILABLE backend={self.backend_name}: {exc}')
                self.get_logger().error(f'ladder detector UNAVAILABLE: {exc} (no detections will be published)')
                return
            tag = ' ORACLE (simulation ground truth, not learned perception)' if self.backend.is_oracle else ''
            self._status(f'AVAILABLE backend={self.backend_name}{tag}')
            self.get_logger().info(f'ladder detector AVAILABLE backend={self.backend_name}{tag}')
            self.create_subscription(CameraInfo, p('camera_info_topic', 'sensors/cam_head/color/camera_info').value,
                                     self._on_info, self.sensor_qos)
            self.create_subscription(Image, p('image_topic', 'sensors/cam_head/color/image_raw').value,
                                     self._on_image, self.sensor_qos)

        def _status(self, text):
            self.status_pub.publish(String(data=text))

        def _init_oracle(self):
            if self.oracle_source == 'projection':
                self.ladders = load_oracle_ladders(self.oracle_file)
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
                # EVALUATION_ONLY ground-truth pose: used solely to render ORACLE boxes.
                self.create_subscription(PoseStamped, self.declare_parameter(
                    'oracle_pose_topic', 'sim/ground_truth/base_pose').value, self._on_gt, 50)
                depth_topic = self.declare_parameter('oracle_depth_topic', '').value
                if depth_topic:
                    self.create_subscription(Image, depth_topic, self._on_depth, self.sensor_qos)
            elif self.oracle_source == 'segmentation':
                with open(self.declare_parameter('oracle_segmentation_ids_file', '').value) as fh:
                    data = yaml.safe_load(fh)
                self.seg_ids = {int(k): v for k, v in data['instances'].items()}
                self.seg = {}
                self.create_subscription(Image, self.declare_parameter(
                    'oracle_segmentation_topic', 'sensors/cam_head/oracle/segmentation').value, self._on_seg,
                    self.sensor_qos)
            else:
                raise ValueError(f'unknown oracle_source {self.oracle_source}')

        def _on_info(self, msg):
            try:
                self.K, self.size = intrinsics(msg.k), (msg.width, msg.height)
            except ValueError as exc:
                self.get_logger().warn(str(exc), throttle_duration_sec=5.0)

        def _on_gt(self, msg):
            self.gt_poses = (self.gt_poses + [(stamp_to_float(msg.header.stamp), pose_to_matrix(msg.pose))])[-200:]

        def _on_depth(self, msg):
            from .localizer_core import depth_to_metres  # noqa: PLC0415
            self.depth = (stamp_to_float(msg.header.stamp), depth_to_metres(image_to_numpy(msg), msg.encoding))

        def _on_seg(self, msg):
            self.seg[stamp_to_float(msg.header.stamp)] = image_to_numpy(msg)
            if len(self.seg) > 30:
                self.seg.pop(min(self.seg))

        def _on_image(self, msg):
            t = stamp_to_float(msg.header.stamp)
            if t - self.last_t < 1.0 / self.max_rate:
                return
            self.last_t = t
            try:
                found = self._oracle(msg, t) if self.backend.is_oracle else self._learned(msg)
            except Exception as exc:  # one bad frame must not kill the node
                self.get_logger().warn(f'frame skipped: {exc}', throttle_duration_sec=5.0)
                return
            if found is None:
                return
            out = LadderDetectionArray(schema_version=LadderDetectionArray.SCHEMA_VERSION, header=msg.header)
            digest = hashlib.sha256(bytes(msg.data)).hexdigest()[:16]
            for item in found:
                self.seq += 1
                d = LadderDetection(schema_version=LadderDetection.SCHEMA_VERSION, header=msg.header,
                                    detection_id=f'{self.backend_name}-{t:.3f}-{self.seq}',
                                    backend=self.backend_name, is_oracle=self.backend.is_oracle,
                                    label=item['label'], confidence=float(item.get('confidence', 1.0)),
                                    image_width=msg.width, image_height=msg.height,
                                    evidence_ref=f'image_sha256_16:{digest}')
                d.bbox_x_min, d.bbox_y_min, d.bbox_x_max, d.bbox_y_max = (float(v) for v in item['bbox'])
                out.detections.append(d)
            self.pub.publish(out)

        def _oracle(self, msg, t):
            if self.oracle_source == 'segmentation':
                if not self.seg:
                    return None
                key = min(self.seg, key=lambda s: abs(s - t))
                if abs(key - t) > self.oracle_pose_tol:
                    return None
                return segmentation_boxes(self.seg[key], self.seg_ids)
            if self.K is None or not self.gt_poses:
                return None
            gt_t, T_world_base = min(self.gt_poses, key=lambda tp: abs(tp[0] - t))
            if abs(gt_t - t) > self.oracle_pose_tol:
                return None
            T_base_opt = tf_to_matrix(self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, rclpy.time.Time()))  # static mount
            depth = None
            if self.depth is not None and abs(self.depth[0] - t) <= self.oracle_pose_tol \
                    and self.depth[1].shape == (msg.height, msg.width):
                depth = self.depth[1]
            return project_ladders(self.ladders, T_world_base @ T_base_opt, self.K, msg.width, msg.height,
                                   depth_m=depth, max_range_m=self.max_range)

        def _learned(self, msg):
            rgb = image_to_numpy(msg)
            if rgb.ndim == 3 and rgb.shape[2] == 4:
                rgb = rgb[..., :3]
            return self.backend.detect(np.ascontiguousarray(rgb))

    return run_node(LadderDetector, argv)

if __name__ == '__main__':
    sys.exit(main())
