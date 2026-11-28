"""Compare box and oracle-mask depth localization on the synthetic hall; not a real accuracy test."""
import math
from collections import Counter

import numpy as np

from ultron.autonomy.hall_scene import BASE_HEIGHT_M, HallSim
from ultron_b2_perception.geometry import BODY_FROM_OPTICAL, planar, transform
from ultron_b2_perception.localizer_core import Rejected, color_box_to_depth, localize, oracle_depth_mask
from ultron_b2_perception.oracle import segmentation_boxes
from ultron_b2_sim.camera import HeadCamera, StreamSpec


def matrix(spec):
    p = spec.intrinsics()
    return np.array([[p['fx'], 0, p['cx']], [0, p['fy'], p['cy']], [0, 0, 1]])


def main():
    sim = HallSim()
    color = StreamSpec(640, 360, 321.4505, 'color')
    depth = StreamSpec(424, 240, 215.5275, 'depth')
    mount, pitch = (0.45, 0.0, 0.13), -0.2219
    cam = HeadCamera(sim, mount, (0.0, pitch, 0.0), color, depth)
    kc, kd = matrix(color), matrix(depth)
    objects = {o.name: o for o in sim.objects if o.name.startswith('ladder_')}
    ids = {i: {'name': name, 'type': objects[name].kind} for name, i in cam.ladder_labels.items()}
    rng = np.random.default_rng(42)
    samples, rejected = [], Counter()
    try:
        for _ in range(80):
            x, y = rng.uniform(0.7, 11.3), rng.uniform(0.7, 9.3)
            if not sim.footprint_free(x, y, 0.65):
                continue
            target = objects[rng.choice(sorted(objects))]
            yaw = math.atan2(target.position[1] - y, target.position[0] - x)
            sim.place(x, y, yaw)
            _, raw, labels = cam.render()
            depth_m = raw.astype(float) / 1000.0
            depth_m[raw == 0] = np.nan
            T = planar(x, y, yaw, BASE_HEIGHT_M) @ transform(mount, (0, math.sin(pitch / 2), 0,
                                                                      math.cos(pitch / 2))) @ BODY_FROM_OPTICAL
            for det in segmentation_boxes(labels, ids):
                box = color_box_to_depth(det['bbox'], kc, kd)
                try:
                    mask = oracle_depth_mask(labels, det['bbox'], raw.shape, kc, kd)
                    old, _, _ = localize(box, depth_m, kd, T)
                    new, _, _ = localize(box, depth_m, kd, T, mask=mask)
                except Rejected as exc:
                    rejected[exc.reason] += 1
                    continue
                truth = np.asarray(objects[det['name']].position)
                samples.append((np.linalg.norm(old[:2] - truth), np.linalg.norm(new[:2] - truth)))
    finally:
        cam.close()
    print('paired samples:', len(samples), 'rejected:', dict(rejected))
    if samples:
        a = np.asarray(samples)
        for i, method in enumerate(('box', 'mask')):
            print(method, 'median_m', round(float(np.median(a[:, i])), 3),
                  'max_m', round(float(np.max(a[:, i])), 3), 'over_0.65m', int(np.sum(a[:, i] > 0.65)))


if __name__ == '__main__':
    main()
