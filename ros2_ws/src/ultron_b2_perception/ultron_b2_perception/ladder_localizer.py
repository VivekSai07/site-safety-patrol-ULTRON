"""ladder_localizer node: detections + aligned depth + TF at the image stamp -> map-frame observations.

Subscribes ``perception/ladder_detections``, depth image/camera_info, colour camera_info,
``state/localization`` and ``perception/ladder_candidates`` (for unoccluded view checks).
Publishes ``perception/ladder_observations`` (LadderCandidateArray, header = image stamp, frame
``map``): entries with ``association_state=OBSERVATION`` are localized detections; entries with
``association_state=VIEW_CHECK`` name known candidates that were in unoccluded view in that frame
(used by the tracker for MISSING). Stats JSON on ``perception/ladder_localizer/stats``.

Drops (counted, never approximated): TF not available at the image stamp (stale/extrapolated),
depth not synchronized, localization not VALID, insufficient depth, out of range, old image.
"""
import json
import sys

import numpy as np

from .localizer_core import (DropCounter, Rejected, color_box_to_depth, depth_to_metres, localize, oracle_depth_mask,
                             require_registered_depth, visible_candidates)
from .geometry import intrinsics
from .ros_util import image_to_numpy, ladder_type_from_label, run_node, sensor_qos, stamp_to_float, tf_to_matrix

OBSERVATION, VIEW_CHECK = 'OBSERVATION', 'VIEW_CHECK'


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def lookup_at_stamp(buffer, target, source, stamp_s, timeout_s):
    """Transform target<-source at exactly ``stamp_s``; any TF failure raises Rejected('STALE_TF')."""
    from rclpy.duration import Duration  # noqa: PLC0415
    from rclpy.time import Time  # noqa: PLC0415
    from tf2_ros import TransformException  # noqa: PLC0415
    try:
        tf = buffer.lookup_transform(target, source, Time(nanoseconds=int(round(stamp_s * 1e9))),
                                     timeout=Duration(seconds=timeout_s))
    except TransformException as exc:
        raise Rejected('STALE_TF', str(exc)) from exc
    return tf_to_matrix(tf)


def main(argv=None):
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from ultron_b2_interfaces.msg import (LadderCandidate, LadderCandidateArray, LadderDetectionArray,
                                          LocalizationState)

    class LadderLocalizer(Node):
        def __init__(self):
            super().__init__('ladder_localizer')
            p = self.declare_parameter
            self.map_frame = p('map_frame', 'map').value
            self.tf_timeout = float(p('tf_timeout_s', 0.1).value)          # PROPOSED
            self.sync_tol = float(p('depth_sync_tolerance_s', 0.05).value)  # PROPOSED
            self.max_age = float(p('max_image_age_s', 1.0).value)           # PROPOSED
            self.loc_max_age = float(p('localization_max_age_s', 1.0).value)
            self.depth_scale = float(p('depth_scale_16u', 0.001).value)     # RealSense default, TO_VERIFY
            self.depth_registered_to_color = p('depth_registered_to_color', False).value
            self.oracle_masks = p('oracle_depth_masks', False).value
            if self.oracle_masks and p('run_mode', '').value != 'sim':
                raise ValueError('oracle depth masks are restricted to simulation')
            self.max_range = float(p('max_range_m', 8.0).value)
            camera_qos = sensor_qos(self)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
            self.depths = []
            self.oracle_labels = {}
            self.pending_detections = {}
            self.K_depth = self.K_color = None
            self.loc = None
            self.candidates = []
            self.drops = DropCounter()
            self.published = 0
            self.pub = self.create_publisher(LadderCandidateArray, 'perception/ladder_observations', 10)
            self.stats_pub = self.create_publisher(String, 'perception/ladder_localizer/stats', 10)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Image, p('depth_topic', 'sensors/cam_head/depth/image_rect_raw').value,
                                     self._on_depth, camera_qos)
            self.create_subscription(CameraInfo, p('depth_info_topic', 'sensors/cam_head/depth/camera_info').value,
                                     self._on_depth_info, camera_qos)
            self.create_subscription(CameraInfo, p('color_info_topic', 'sensors/cam_head/color/camera_info').value,
                                     self._on_color_info, camera_qos)
            self.create_subscription(LocalizationState, 'state/localization', self._on_loc, 10)
            self.create_subscription(LadderCandidateArray, 'perception/ladder_candidates', self._on_cands, latched)
            self.create_subscription(LadderDetectionArray, 'perception/ladder_detections', self._on_dets, 10)
            if self.oracle_masks:
                self.create_subscription(Image, p('oracle_segmentation_topic',
                                                  'sim/oracle/cam_head/ladder_labels').value,
                                         self._on_labels, camera_qos)
                self.create_timer(0.1, self._drain_pending)
            self.create_timer(1.0, self._stats)

        def _on_depth(self, msg):
            try:
                depth = depth_to_metres(image_to_numpy(msg), msg.encoding, self.depth_scale)
            except (Rejected, ValueError):
                self.drops.drop('NO_DEPTH')
                return
            self.depths = (self.depths + [(stamp_to_float(msg.header.stamp), msg.header.frame_id,
                                          depth, stamp_ns(msg))])[-30:]
            if self.oracle_masks:
                self._drain_pending()

        def _on_labels(self, msg):
            try:
                if msg.encoding not in ('16UC1', 'mono16'):
                    return
                labels = image_to_numpy(msg)
            except ValueError:
                return
            self.oracle_labels[stamp_ns(msg)] = (msg.header.frame_id, labels)
            if len(self.oracle_labels) > 30:
                self.oracle_labels.pop(min(self.oracle_labels))
            self._drain_pending()

        def _on_depth_info(self, msg):
            try:
                self.K_depth = intrinsics(msg.k)
            except ValueError:
                self.K_depth = None

        def _on_color_info(self, msg):
            try:
                self.K_color = intrinsics(msg.k)
            except ValueError:
                self.K_color = None

        def _on_loc(self, msg):
            self.loc = msg

        def _on_cands(self, msg):
            self.candidates = [{'candidate_id': c.candidate_id,
                                'position': (c.position.x, c.position.y, c.position.z)}
                               for c in msg.candidates if c.association_state != 'MISSING']

        def _loc_valid(self, t):
            loc = self.loc
            return (loc is not None and loc.state == LocalizationState.STATE_VALID
                    and abs(t - stamp_to_float(loc.header.stamp)) <= self.loc_max_age)

        def _on_dets(self, msg):
            if self.oracle_masks:
                self.pending_detections[stamp_ns(msg)] = msg
                if len(self.pending_detections) > 30:
                    old = self.pending_detections.pop(min(self.pending_detections))
                    self.drops.update({'ORACLE_FRAME_NOT_SYNCED': max(1, len(old.detections))})
                self._drain_pending()
                return
            self._process_dets(msg)

        def _drain_pending(self):
            now = self.get_clock().now().nanoseconds
            for key, msg in list(self.pending_detections.items()):
                if now - key > self.max_age * 1e9:
                    self.pending_detections.pop(key)
                    self.drops.update({'ORACLE_FRAME_NOT_SYNCED': max(1, len(msg.detections))})
                    continue
                depth = next((d for d in self.depths if d[3] == key), None)
                labels = self.oracle_labels.get(key)
                if depth is not None and labels is not None:
                    self.pending_detections.pop(key)
                    if labels[0] != msg.header.frame_id:
                        self.drops.update({'NO_ORACLE_MASK': max(1, len(msg.detections))})
                        continue
                    self._process_dets(msg, depth, labels[1])

        def _process_dets(self, msg, exact_depth=None, labels=None):
            t = stamp_to_float(msg.header.stamp)
            n = max(1, len(msg.detections))
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - t > self.max_age:
                return self.drops.update({'IMAGE_TOO_OLD': n})
            if not self._loc_valid(t):
                return self.drops.update({'LOCALIZATION_NOT_VALID': n})
            try:
                require_registered_depth(self.depth_registered_to_color)
            except Rejected as exc:
                return self.drops.update({exc.reason: n})
            if self.K_depth is None or (self.K_color is None and msg.detections):
                return self.drops.update({'NO_CAMERA_INFO': n})
            if not self.depths:
                return self.drops.update({'NO_DEPTH': n})
            dt, frame, depth, _ = exact_depth or min(self.depths, key=lambda d: abs(d[0] - t))
            if abs(dt - t) > self.sync_tol:
                return self.drops.update({'DEPTH_NOT_SYNCED': n})
            try:
                # TF at the *image* stamp; never the latest transform.
                T = lookup_at_stamp(self.tf_buffer, self.map_frame, frame, dt, self.tf_timeout)
            except Rejected as exc:
                self.get_logger().warn(f'drop: {exc}', throttle_duration_sec=5.0)
                return self.drops.update({'STALE_TF': n})
            out = LadderCandidateArray(schema_version=LadderCandidateArray.SCHEMA_VERSION)
            out.header.stamp, out.header.frame_id = msg.header.stamp, self.map_frame
            all_localized = True
            for det in msg.detections:
                try:
                    color_box = (det.bbox_x_min, det.bbox_y_min, det.bbox_x_max, det.bbox_y_max)
                    mask = None
                    if self.oracle_masks:
                        if not det.is_oracle or det.backend != 'oracle_sim' or labels.shape != (
                                det.image_height, det.image_width):
                            raise Rejected('NO_ORACLE_MASK', 'non-oracle detection or inconsistent image')
                        mask = oracle_depth_mask(labels, color_box, depth.shape, self.K_color, self.K_depth)
                    box = color_box_to_depth(color_box, self.K_color, self.K_depth)
                    pos, cov, _rng = localize(box, depth, self.K_depth, T, max_range_m=self.max_range, mask=mask)
                except Rejected as exc:
                    self.drops.drop(exc.reason)
                    all_localized = False
                    continue
                out.candidates.append(self._entry(out.header, OBSERVATION, pos, cov,
                                                  ladder_type_from_label(det.label), det.detection_id,
                                                  det.is_oracle))
            checks = visible_candidates(self.candidates, depth, self.K_depth, T) if all_localized else ()
            for cid in checks:
                e = self._entry(out.header, VIEW_CHECK, (0.0, 0.0, 0.0), np.zeros((3, 3)), '', '', False)
                e.candidate_id = cid
                out.candidates.append(e)
            self.published += 1
            self.pub.publish(out)

        def _entry(self, header, kind, pos, cov, ladder_type, det_id, oracle):
            c = LadderCandidate(schema_version=LadderCandidate.SCHEMA_VERSION, header=header,
                                association_state=kind, ladder_type=ladder_type, from_oracle=bool(oracle),
                                map_id=self.loc.map_id, map_version=self.loc.map_version, observation_count=1)
            c.position.x, c.position.y, c.position.z = (float(v) for v in pos)
            c.position_covariance = [float(v) for v in np.asarray(cov).reshape(9)]
            c.first_seen = c.last_seen = header.stamp
            if det_id:
                c.detection_ids = [det_id]
            return c

        def _stats(self):
            self.stats_pub.publish(String(data=json.dumps({'published_frames': self.published,
                                                           'drops': dict(self.drops)})))

    return run_node(LadderLocalizer, argv)


if __name__ == '__main__':
    sys.exit(main())
