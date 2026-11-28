"""capture_coordinator node: CaptureViews server ``mission/capture_views``. Never moves the robot.

For each goal it waits for colour/depth frames received *after* the goal was accepted (so the
robot has already settled at the view pose; NavigateToView returned settled=true), stores them
under ``evidence_dir/<mission_id>/<capture_id>/`` as ``.npy`` plus a JSON manifest, and returns
``sha256:<digest>:<path>`` evidence refs. Evidence is local runtime data: never commit it.
Captured frames are evidence for later assessment and human review, not a verdict.
"""
import hashlib
import json
import os
import pathlib
import sys
import threading
import time

import numpy as np


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def write_evidence(directory, name, array, meta):
    directory = pathlib.Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f'{name}.npy'
    np.save(path, array)
    digest = sha256_file(path)
    (directory / f'{name}.json').write_text(json.dumps(dict(meta, sha256=digest), indent=2, default=str))
    return f'sha256:{digest}:{path}'


def main(argv=None):
    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from ultron_b2_interfaces.action import CaptureViews
    from ultron_b2_perception.ros_util import image_to_numpy, stamp_to_float

    class CaptureCoordinator(Node):
        def __init__(self):
            super().__init__('capture_coordinator')
            p = self.declare_parameter
            self.evidence_dir = p('evidence_dir', os.path.expanduser('~/.ultron/evidence')).value
            self.frame_timeout_s = float(p('frame_timeout_s', 3.0).value)   # PROPOSED
            topics = {'color': p('color_topic', 'sensors/cam_head/color/image_raw').value,
                      'depth': p('depth_topic', 'sensors/cam_head/depth/image_rect_raw').value}
            self.lock = threading.Lock()
            self.latest = {}
            cb = ReentrantCallbackGroup()
            for modality, topic in topics.items():
                self.create_subscription(Image, topic, lambda m, k=modality: self._on_image(k, m),
                                         qos_profile_sensor_data, callback_group=cb)
            self.server = ActionServer(self, CaptureViews, 'mission/capture_views', execute_callback=self._execute,
                                       goal_callback=self._goal_cb, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                       callback_group=cb)

        def _on_image(self, modality, msg):
            with self.lock:
                self.latest[modality] = (time.monotonic(), msg)

        def _goal_cb(self, goal):
            if not goal.capture_id or not goal.view_poses:
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

        def _execute(self, gh):
            g = gh.request
            accepted = time.monotonic()
            modalities = [m for m in (g.modalities or ['color', 'depth'])]
            result = CaptureViews.Result(schema_version='ultron.capture_views_result/v1')
            out_dir = pathlib.Path(self.evidence_dir) / (g.mission_id or 'mission') / g.capture_id
            refs, failed, reasons = [], 0, []
            for idx, view in enumerate(g.view_poses):
                for modality in modalities:
                    frame = self._wait_frame(modality, accepted, gh)
                    if gh.is_cancel_requested:
                        gh.canceled()
                        result.status, result.reason = 'CANCELLED', 'cancelled'
                        result.evidence_refs = refs
                        return result
                    if frame is None:
                        failed += 1
                        reasons.append(f'view {idx} {modality}: no fresh frame within {self.frame_timeout_s:.1f} s')
                        continue
                    try:
                        arr = image_to_numpy(frame)
                        meta = {'schema': 'ultron.capture_evidence/v1', 'capture_id': g.capture_id,
                                'mission_id': g.mission_id, 'asset_id': g.asset_id, 'map_id': g.map_id,
                                'map_version': int(g.map_version), 'modality': modality, 'view_index': idx,
                                'view_pose_map': [view.pose.position.x, view.pose.position.y],
                                'image_stamp': stamp_to_float(frame.header.stamp),
                                'frame_id': frame.header.frame_id, 'encoding': frame.encoding,
                                'verdict': 'NONE_EVIDENCE_ONLY', 'human_review_required': True}
                        refs.append(write_evidence(out_dir, f'view{idx}_{modality}', arr, meta))
                    except Exception as exc:
                        failed += 1
                        reasons.append(f'view {idx} {modality}: {exc}')
                gh.publish_feedback(CaptureViews.Feedback(views_done=idx + 1, current_view=idx, status='CAPTURING'))
            total = len(g.view_poses) * len(modalities)
            result.views_captured = (total - failed) // max(1, len(modalities))
            result.views_failed = len(g.view_poses) - result.views_captured
            result.evidence_refs = refs
            result.status = 'COMPLETE' if failed == 0 else ('PARTIAL' if refs else 'FAILED')
            result.reason = '; '.join(reasons) or 'captured'
            if result.status == 'FAILED':
                gh.abort()
            else:
                gh.succeed()
            return result

        def _wait_frame(self, modality, after, gh):
            deadline = time.monotonic() + self.frame_timeout_s
            while time.monotonic() < deadline and not gh.is_cancel_requested:
                with self.lock:
                    item = self.latest.get(modality)
                if item is not None and item[0] > after:
                    return item[1]
                time.sleep(0.02)
            return None

    rclpy.init(args=argv)
    node = CaptureCoordinator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
