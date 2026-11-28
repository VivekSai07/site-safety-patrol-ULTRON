"""ORACLE projection labels, backend degradation without weights, localization geometry, stale TF drop."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ultron_b2_perception import backends  # noqa: E402
from ultron_b2_perception.geometry import BODY_FROM_OPTICAL, intrinsics, planar  # noqa: E402
from ultron_b2_perception.ladder_detector import FramePairs, frame_stamp_ns, oracle_segmentation_ids  # noqa: E402
from ultron_b2_perception.localizer_core import (Rejected, color_box_to_depth, localize, oracle_depth_mask,  # noqa: E402
                                                  require_registered_depth, visible_candidates)
from ultron_b2_perception.oracle import ORACLE_LABEL_PREFIX, project_ladders, segmentation_boxes  # noqa: E402

K = np.array([[643.6, 0.0, 643.5], [0.0, 642.9, 362.0], [0.0, 0.0, 1.0]])
W, H = 1280, 720


def cam_at(x, y, yaw, z=0.6):
    return planar(x, y, yaw, z) @ BODY_FROM_OPTICAL


def test_oracle_projection_labels_and_fov():
    ladders = [{'name': 'ahead', 'type': 'TRESTLE', 'position': [4.0, 0.0], 'yaw': 0.0},
               {'name': 'behind', 'type': 'TRESTLE', 'position': [-4.0, 0.0], 'yaw': 0.0},
               {'name': 'far', 'type': 'TRESTLE', 'position': [30.0, 0.0], 'yaw': 0.0}]
    dets = project_ladders(ladders, cam_at(0, 0, 0), K, W, H)
    assert [d['name'] for d in dets] == ['ahead']
    assert dets[0]['label'] == ORACLE_LABEL_PREFIX + 'TRESTLE'
    u = 0.5 * (dets[0]['bbox'][0] + dets[0]['bbox'][2])
    assert abs(u - K[0, 2]) < 5


def test_oracle_projection_rejects_occluded_ladder():
    ladders = [{'name': 'L', 'type': 'TRESTLE', 'position': [5.0, 0.0], 'yaw': 0.0}]
    wall = np.full((H, W), 2.0, dtype=np.float32)
    assert project_ladders(ladders, cam_at(0, 0, 0), K, W, H, depth_m=wall) == []
    clear = np.full((H, W), 6.0, dtype=np.float32)
    assert len(project_ladders(ladders, cam_at(0, 0, 0), K, W, H, depth_m=clear)) == 1


def test_segmentation_boxes_are_oracle_labelled():
    img = np.zeros((100, 100), dtype=np.uint16)
    img[10:60, 20:40] = 7
    boxes = segmentation_boxes(img, {7: {'name': 'ladder_06', 'type': 'TRESTLE'}})
    assert boxes[0]['bbox'] == (20.0, 10.0, 40.0, 60.0) and boxes[0]['label'].startswith(ORACLE_LABEL_PREFIX)


def test_segmentation_pairs_exact_frames_in_either_arrival_order():
    pairs = FramePairs(limit=2)
    assert pairs.add('image', 1, 'image1') is None
    assert pairs.add('labels', 2, 'labels2') is None
    assert pairs.add('labels', 1, 'labels1') == ('image1', 'labels1')
    assert pairs.add('image', 2, 'image2') == ('image2', 'labels2')
    for t in (3, 4, 5):
        assert pairs.add('image', t, f'image{t}') is None
    assert pairs.add('labels', 3, 'stale') is None
    assert pairs.add('labels', 5, 'labels5') == ('image5', 'labels5')


def test_segmentation_ids_follow_sorted_simulator_ladder_names():
    ladders = [{'name': 'ladder_07', 'type': 'TRESTLE'},
               {'name': 'ladder_01', 'type': 'LAYHER'}]
    ids = oracle_segmentation_ids(ladders)
    assert [ids[i]['name'] for i in ids] == ['ladder_01', 'ladder_07']
    with pytest.raises(ValueError):
        oracle_segmentation_ids(ladders + [ladders[0]])


def test_frame_pair_keys_preserve_nanoseconds_at_large_timestamps():
    from types import SimpleNamespace
    stamp = SimpleNamespace(sec=1_700_000_000, nanosec=1)
    msg = SimpleNamespace(header=SimpleNamespace(stamp=stamp))
    assert frame_stamp_ns(msg) == 1_700_000_000_000_000_001


def test_oracle_backend_refused_outside_sim_and_learned_backends_unavailable_without_weights():
    with pytest.raises(backends.BackendUnavailable):
        backends.make_backend('oracle_sim', 'hardware')
    assert isinstance(backends.make_backend('oracle_sim', 'sim'), backends.OracleBackend)
    with pytest.raises(backends.BackendUnavailable):
        backends.make_backend('yolo', 'sim', weights='does/not/exist/missing.pt')
    with pytest.raises(backends.BackendUnavailable):
        backends.make_backend('owlv2', 'sim', model_dir='does/not/exist/owlv2')
    with pytest.raises(backends.BackendUnavailable):
        backends.make_backend('magic', 'sim')


def test_localize_box_centre_to_map_point():
    depth = np.full((H, W), 6.0, dtype=np.float32)
    depth[300:420, 600:690] = 4.0
    T_map_opt = cam_at(1.0, 2.0, math.pi / 2)
    pos, cov, rng = localize((600, 300, 690, 420), depth, K, T_map_opt)
    assert abs(pos[0] - 1.0) < 0.15 and abs(pos[1] - 6.0) < 0.1
    assert cov.shape == (3, 3) and np.all(np.linalg.eigvalsh(cov) > 0)
    with pytest.raises(Rejected):
        localize((600, 300, 690, 420), np.full((H, W), np.nan, dtype=np.float32), K, T_map_opt)


def test_unregistered_depth_is_rejected():
    with pytest.raises(Rejected) as exc:
        require_registered_depth(False)
    assert exc.value.reason == 'DEPTH_NOT_REGISTERED'
    require_registered_depth(True)


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_nonfinite_camera_intrinsics_are_rejected(bad):
    matrix = K.copy()
    matrix[0, 2] = bad
    with pytest.raises(ValueError):
        intrinsics(matrix)


def test_oracle_mask_excludes_foreground_and_background_in_box():
    k_color = np.array([[100.0, 0, 49.5], [0, 100.0, 49.5], [0, 0, 1]])
    k_depth = np.array([[50.0, 0, 24.5], [0, 50.0, 24.5], [0, 0, 1]])
    labels = np.zeros((100, 100), dtype=np.uint16)
    labels[20:80, 20:80] = 3
    labels[30:70, 30:70] = 0
    box = (20.0, 20.0, 80.0, 80.0)
    mask = oracle_depth_mask(labels, box, (50, 50), k_color, k_depth)
    depth = np.full((50, 50), 6.0, dtype=np.float32)
    depth[mask] = 4.0
    depth[20:30, 20:30] = 2.0
    dbox = color_box_to_depth(box, k_color, k_depth)
    pos, _, _ = localize(dbox, depth, k_depth, np.eye(4), mask=mask)
    assert pos[2] == pytest.approx(4.0)
    with pytest.raises(Rejected, match='NO_ORACLE_MASK'):
        oracle_depth_mask(labels, (19.0, 20.0, 80.0, 80.0), depth.shape, k_color, k_depth)


def test_visible_candidates_excludes_occluded_and_out_of_view():
    depth = np.full((H, W), 8.0, dtype=np.float32)
    T = cam_at(0, 0, 0)
    cands = [{'candidate_id': 'in', 'position': [4.0, 0.0, 0.6]},
             {'candidate_id': 'behind', 'position': [-4.0, 0.0, 0.6]},
             {'candidate_id': 'side', 'position': [0.5, 4.0, 0.6]}]
    assert visible_candidates(cands, depth, K, T) == ['in']
    depth[:] = 2.0  # wall in front of the candidate
    assert visible_candidates(cands, depth, K, T) == []
    depth[:] = np.nan  # no depth return is not evidence of an unoccluded view
    assert visible_candidates(cands, depth, K, T) == []
    depth[:] = 0.0
    assert visible_candidates(cands, depth, K, T) == []


def test_stale_tf_is_rejected_at_image_stamp():
    pytest.importorskip('rclpy')
    tf2_ros = pytest.importorskip('tf2_ros')
    from builtin_interfaces.msg import Time as TimeMsg
    from geometry_msgs.msg import TransformStamped
    from rclpy.duration import Duration
    from ultron_b2_perception.ladder_localizer import lookup_at_stamp

    buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))

    def add(t):
        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'cam_optical'
        tf.header.stamp = TimeMsg(sec=int(t), nanosec=int((t % 1) * 1e9))
        tf.transform.translation.x = float(t)
        tf.transform.rotation.w = 1.0
        buf.set_transform(tf, 'test')
    add(10.0)
    add(11.0)
    T = lookup_at_stamp(buf, 'map', 'cam_optical', 10.5, 0.0)
    assert abs(T[0, 3] - 10.5) < 1e-6  # interpolated at the image stamp, not latest
    with pytest.raises(Rejected) as exc:
        lookup_at_stamp(buf, 'map', 'cam_optical', 12.5, 0.0)  # newer than TF => stale TF, dropped
    assert exc.value.reason == 'STALE_TF'
    with pytest.raises(Rejected):
        lookup_at_stamp(buf, 'map', 'cam_optical', 5.0, 0.0)  # older than buffer
