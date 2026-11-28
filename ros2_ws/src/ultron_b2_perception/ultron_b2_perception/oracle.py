"""ORACLE ladder detections for simulation only.

Two oracle sources, both simulation ground truth (never learned perception):

* ``projection``: project configured ground-truth ladder boxes (world frame) into the head camera
  using the simulator's EVALUATION_ONLY ground-truth pose; optional depth occlusion test.
* ``segmentation``: bounding boxes from a simulator instance-segmentation image.

Every detection is labelled ``ORACLE_SIM:<type>`` with ``is_oracle=True``. Ground truth is used
only to produce these labelled detections; it is never fed to the estimator or put on TF.
"""
import math

import numpy as np

from .geometry import apply, inverse, project

ORACLE_LABEL_PREFIX = 'ORACLE_SIM:'
# Approximate box sizes per hall ladder type (simulation geometry from hall_scene.py).
TYPE_DEFAULTS = {
    'LAYHER_A_FRAME_STEPLADDER': {'height': 2.0, 'half_x': 0.40, 'half_y': 0.70},
    'COMBINATION_LADDER_A_FRAME': {'height': 2.5, 'half_x': 0.30, 'half_y': 0.60},
    'LEANING_COMBINATION_LADDER': {'height': 2.9, 'half_x': 0.30, 'half_y': 0.45},
    'TRESTLE': {'height': 0.8, 'half_x': 0.50, 'half_y': 0.35},
}


def ladder_corners(ladder):
    """8 world-frame corners of the ladder's oriented bounding box."""
    d = TYPE_DEFAULTS.get(ladder['type'], {'height': 2.0, 'half_x': 0.4, 'half_y': 0.6})
    h = float(ladder.get('height', d['height']))
    hx, hy = float(ladder.get('half_x', d['half_x'])), float(ladder.get('half_y', d['half_y']))
    x, y = ladder['position'][:2]
    c, s = math.cos(ladder.get('yaw', 0.0)), math.sin(ladder.get('yaw', 0.0))
    out = []
    for dx in (-hx, hx):
        for dy in (-hy, hy):
            for z in (0.0, h):
                out.append((x + c * dx - s * dy, y + s * dx + c * dy, z))
    return out


def project_ladders(ladders, T_world_optical, K, width, height, *, depth_m=None, max_range_m=8.0,
                    min_box_px=12, occlusion_margin_m=0.6, min_visible_fraction=0.3):
    """Return oracle boxes for ladders visible in the image.

    ``T_world_optical`` maps optical-frame points into the world frame. ``depth_m`` (optional,
    metres, same intrinsics) is used to reject ladders hidden behind closer geometry.
    """
    T_optical_world = inverse(T_world_optical)
    results = []
    for ladder in ladders:
        corners = [apply(T_optical_world, c) for c in ladder_corners(ladder)]
        h = float(ladder.get('height', TYPE_DEFAULTS.get(ladder['type'], {'height': 2.0})['height']))
        centre = apply(T_optical_world, (*ladder['position'][:2], 0.5 * h))
        if centre[2] <= 0.2 or float(np.linalg.norm(centre)) > max_range_m:
            continue
        pixels = [project(K, c) for c in corners if c[2] > 0.1]
        if len(pixels) < 4:
            continue
        us, vs = [p[0] for p in pixels], [p[1] for p in pixels]
        raw = (min(us), min(vs), max(us), max(vs))
        box = (max(0.0, raw[0]), max(0.0, raw[1]), min(float(width), raw[2]), min(float(height), raw[3]))
        if box[2] - box[0] < min_box_px or box[3] - box[1] < min_box_px:
            continue
        raw_area = max(1e-6, (raw[2] - raw[0]) * (raw[3] - raw[1]))
        if (box[2] - box[0]) * (box[3] - box[1]) / raw_area < min_visible_fraction:
            continue
        if depth_m is not None and _occluded(depth_m, box, float(np.min([c[2] for c in corners])),
                                            occlusion_margin_m):
            continue
        results.append({'name': ladder['name'], 'type': ladder['type'], 'bbox': box,
                        'label': ORACLE_LABEL_PREFIX + ladder['type'], 'range_m': float(np.linalg.norm(centre))})
    return results


def _occluded(depth_m, box, nearest_z, margin):
    h, w = depth_m.shape
    x0, y0 = int(box[0]), int(box[1])
    x1, y1 = int(math.ceil(box[2])), int(math.ceil(box[3]))
    patch = depth_m[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
    valid = patch[np.isfinite(patch) & (patch > 0.05)]
    if valid.size < 10:
        return False
    # Hidden when most of the box sees something clearly in front of the ladder's nearest corner.
    return float(np.mean(valid < nearest_z - margin)) > 0.8


def segmentation_boxes(instance_image, id_to_ladder, *, min_pixels=50):
    """Bounding boxes per instance id from an oracle segmentation image."""
    boxes = []
    for inst_id, ladder in sorted(id_to_ladder.items()):
        v, u = np.nonzero(instance_image == int(inst_id))
        if u.size < min_pixels:
            continue
        boxes.append({'name': ladder['name'], 'type': ladder['type'],
                      'bbox': (float(u.min()), float(v.min()), float(u.max() + 1), float(v.max() + 1)),
                      'label': ORACLE_LABEL_PREFIX + ladder['type'], 'pixels': int(u.size)})
    return boxes
