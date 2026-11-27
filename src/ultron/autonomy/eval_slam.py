"""Evaluate a simulated SLAM run against simulator ground truth. EVALUATION ONLY.

Trajectory: the SLAM estimate (map frame) is placed in the world with the known start pose
only (map origin = odom origin = start pose); no best-fit alignment, so errors cannot be
hidden. A least-squares SE(2) fit is reported separately for reference.

Map: reference surfaces are the points a noise-free copy of the simulated lidar hits along
the TRUE trajectory, within the 2D scan's height band and open sector, i.e. what the sensor
could observe. Precision = occupied map cells within tolerance of a reference surface.
Recall = reference surface cells (all / walls only) with an occupied map cell within tolerance.

Acceptance thresholds are fixed in docs/SAHIL_SLAM_PLAN.md (Phase 3) before evaluation.
"""
import argparse
import csv
import dataclasses
import json
import math
from pathlib import Path

import numpy as np

THRESHOLDS = {'ate_rmse_m': 0.10, 'ate_max_m': 0.25, 'yaw_rmse_deg': 2.0, 'wall_recall': 0.85,
              'odometry_rmse_ratio_min': 2.0, 'tolerance_m': 0.10}


def read_csv(path):
    rows = list(csv.DictReader(Path(path).open()))
    return {round(float(r['t']), 3): (float(r['x']), float(r['y']), float(r['yaw'])) for r in rows}


def compose(base, pose):
    bx, by, byaw = base
    x, y, yaw = pose
    c, s = math.cos(byaw), math.sin(byaw)
    return bx + c * x - s * y, by + s * x + c * y, byaw + yaw


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def trajectory_errors(estimate, truth, start, align=False):
    stamps = sorted(set(estimate) & set(truth))
    if not stamps:
        raise ValueError('No common timestamps between estimate and ground truth')
    est = np.array([compose(start, estimate[t]) for t in stamps])
    ref = np.array([truth[t] for t in stamps])
    if align:  # least-squares SE(2) (Umeyama without scale) of estimate onto truth
        mu_e, mu_r = est[:, :2].mean(0), ref[:, :2].mean(0)
        H = (est[:, :2] - mu_e).T @ (ref[:, :2] - mu_r)
        theta = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        R = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
        est[:, :2] = (est[:, :2] - mu_e) @ R.T + mu_r
        est[:, 2] += theta
    dxy = np.hypot(est[:, 0] - ref[:, 0], est[:, 1] - ref[:, 1])
    dyaw = np.degrees(np.abs(np.arctan2(np.sin(est[:, 2] - ref[:, 2]), np.cos(est[:, 2] - ref[:, 2]))))
    return {'samples': len(stamps), 'rmse_m': float(np.sqrt(np.mean(dxy ** 2))), 'max_m': float(dxy.max()),
            'final_m': float(dxy[-1]), 'yaw_rmse_deg': float(np.sqrt(np.mean(dyaw ** 2))),
            'yaw_max_deg': float(dyaw.max())}


def load_map(yaml_path):
    import yaml
    from PIL import Image
    meta = yaml.safe_load(Path(yaml_path).read_text())
    image = np.array(Image.open(Path(yaml_path).parent / meta['image']))
    if meta.get('negate', 0):
        image = 255 - image
    occupancy = (255 - image.astype(float)) / 255.0
    occupied = occupancy > meta['occupied_thresh']
    free = occupancy < meta['free_thresh']
    rows, cols = np.nonzero(occupied)
    res = float(meta['resolution'])
    ox, oy = meta['origin'][:2]
    if meta['origin'][2]:
        raise ValueError('Rotated map origins are not supported')
    height = image.shape[0]
    xy = np.column_stack((ox + (cols + 0.5) * res, oy + (height - 1 - rows + 0.5) * res))
    unknown_grey = (255 - 205) / 255.0  # map_saver trinary unknown pixel
    return {'occupied_xy': xy, 'resolution': res, 'occupied_cells': int(occupied.sum()),
            'unknown_reads_as_free': bool(unknown_grey < meta['free_thresh']),
            'free_cells': int(free.sum()), 'unknown_cells': int((~occupied & ~free).sum())}


def reference_surfaces(truth, config, stride_s=0.5):
    """Noise-free lidar hits along the TRUE trajectory, in the scan band and open sector."""
    from .hall_scene import HallSim
    from .lidar import SimLidar
    from .scan2d import geometry_from
    import mujoco
    exact = dataclasses.replace(config.lidar, range_noise_std_m=0.0, dropout_probability=0.0)
    sim = HallSim(exact.mount_xyz_m, exact.mount_rpy_rad)
    lidar = SimLidar(sim, exact, config.self_filter, seed=0)
    stamps = sorted(truth)
    first = lidar.scan(0.0)
    geometry = geometry_from(config, first.blind_columns, first.azimuth_deg)
    points, walls = [], []
    next_t = stamps[0]
    try:
        for t in stamps:
            if t < next_t:
                continue
            next_t = t + stride_s
            sim.place(*truth[t])
            scan = lidar.scan(t)
            p = scan.points.astype(np.float64)
            z_base = p[:, 2] + geometry.mount_z
            rng = np.hypot(p[:, 0], p[:, 1])
            angle = np.arctan2(p[:, 1], p[:, 0])
            keep = ((z_base >= geometry.z_min) & (z_base <= geometry.z_max) & (rng >= geometry.range_min)
                    & (rng <= geometry.range_max) & (angle >= geometry.angle_min) & (angle <= geometry.angle_max))
            world = lidar.lidar_to_world(p[keep])
            names = [mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or '' for g in scan.eval_geom_id[keep]]
            points.append(world[:, :2])
            walls.append(np.array([n.startswith('wall_') for n in names], dtype=bool))
    finally:
        lidar.close()
    return np.vstack(points), np.concatenate(walls)


def _cells(xy, res, origin):
    return set(map(tuple, np.floor((xy - origin) / res).astype(np.int64)))


def _near(query_cells, target_cells, radius_cells):
    offsets = [(dx, dy) for dx in range(-radius_cells, radius_cells + 1) for dy in range(-radius_cells, radius_cells + 1)
               if dx * dx + dy * dy <= radius_cells * radius_cells]
    return sum(any((cx + dx, cy + dy) in target_cells for dx, dy in offsets) for cx, cy in query_cells)


def map_scores(map_xy_world, reference_xy, wall_mask, res=0.05, tolerance=0.10):
    origin = np.array([-2.0, -2.0])
    occupied = _cells(map_xy_world, res, origin)
    reference = _cells(reference_xy, res, origin)
    wall_ref = _cells(reference_xy[wall_mask], res, origin)
    radius = int(round(tolerance / res))
    return {'occupied_cells': len(occupied), 'reference_cells': len(reference), 'reference_wall_cells': len(wall_ref),
            'precision': _near(occupied, reference, radius) / max(1, len(occupied)),
            'recall_all': _near(reference, occupied, radius) / max(1, len(reference)),
            'recall_walls': _near(wall_ref, occupied, radius) / max(1, len(wall_ref)),
            'tolerance_m': tolerance, 'cell_m': res}


def overlay_png(run_dir, out_png, map_yaml=None, scale=60):
    """Review image (world frame): SLAM occupied cells (black) over reference surfaces (green);
    trajectories: truth (blue), SLAM (orange), odometry only (red)."""
    from PIL import Image, ImageDraw
    from .config import load_sim_slam_config
    run_dir = Path(run_dir)
    start = tuple(json.loads((run_dir / 'sim' / 'run_meta.json').read_text())['start_pose_world'])
    truth = read_csv(run_dir / 'sim' / 'ground_truth.csv')
    x0, y0, x1, y1 = -1.0, -1.0, 13.0, 11.0
    image = Image.new('RGB', (int((x1 - x0) * scale), int((y1 - y0) * scale)), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    px = lambda x, y: ((x - x0) * scale, (y1 - y) * scale)
    reference, _ = reference_surfaces(truth, load_sim_slam_config(), stride_s=1.0)
    for x, y in reference[::3]:
        draw.point(px(x, y), fill=(60, 190, 90))
    grid = load_map(Path(map_yaml) if map_yaml else run_dir / 'map.yaml')
    half = grid['resolution'] / 2 * scale
    for x, y in grid['occupied_xy']:
        wx, wy, _ = compose(start, (x, y, 0.0))
        cx, cy = px(wx, wy)
        draw.rectangle((cx - half, cy - half, cx + half, cy + half), fill=(20, 20, 20))
    def path(poses, color):
        pts = [px(*p[:2]) for _, p in sorted(poses.items())[::10]]
        draw.line(pts, fill=color, width=3)
    path(truth, (40, 90, 230))
    path({t: compose(start, p) for t, p in read_csv(run_dir / 'estimate.csv').items()}, (250, 150, 0))
    path({t: compose(start, p) for t, p in read_csv(run_dir / 'sim' / 'odometry.csv').items()}, (220, 30, 30))
    draw.text((10, 10), 'black: SLAM occupied | green: reference surfaces | blue: truth | orange: SLAM | red: odometry only',
              fill=(0, 0, 0))
    image.save(out_png)
    return out_png


def evaluate(run_dir, map_yaml=None):
    from .config import load_sim_slam_config
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / 'sim' / 'run_meta.json').read_text())
    start = tuple(meta['start_pose_world'])
    truth = read_csv(run_dir / 'sim' / 'ground_truth.csv')
    odometry = read_csv(run_dir / 'sim' / 'odometry.csv')
    estimate = read_csv(run_dir / 'estimate.csv')
    result = {'provenance': 'SIMULATION_GROUND_TRUTH_EVALUATION', 'thresholds': THRESHOLDS,
              'estimate_coverage': len(set(estimate) & set(truth)) / max(1, len(truth)),
              'slam_start_anchored': trajectory_errors(estimate, truth, start),
              'slam_best_fit_reference_only': trajectory_errors(estimate, truth, start, align=True),
              'odometry_only': trajectory_errors(odometry, truth, start)}
    map_yaml = Path(map_yaml) if map_yaml else run_dir / 'map.yaml'
    if map_yaml.exists():
        config = load_sim_slam_config()
        grid = load_map(map_yaml)
        world = np.array([compose(start, (x, y, 0.0))[:2] for x, y in grid['occupied_xy']])
        reference, walls = reference_surfaces(truth, config)
        result['map'] = {**{k: v for k, v in grid.items() if k != 'occupied_xy'},
                         **map_scores(world, reference, walls, tolerance=THRESHOLDS['tolerance_m'])}
    slam, odo = result['slam_start_anchored'], result['odometry_only']
    checks = {'ate_rmse': slam['rmse_m'] <= THRESHOLDS['ate_rmse_m'],
              'ate_max': slam['max_m'] <= THRESHOLDS['ate_max_m'],
              'yaw_rmse': slam['yaw_rmse_deg'] <= THRESHOLDS['yaw_rmse_deg'],
              'odometry_clearly_worse': odo['rmse_m'] >= THRESHOLDS['odometry_rmse_ratio_min'] * slam['rmse_m'],
              'estimate_coverage': result['estimate_coverage'] >= 0.95}
    if 'map' in result:
        checks['wall_recall'] = result['map']['recall_walls'] >= THRESHOLDS['wall_recall']
    result['checks'] = checks
    result['verdict'] = 'PASS' if all(checks.values()) and 'map' in result else 'FAIL' if not all(checks.values()) else 'INCOMPLETE'
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description='Evaluate a simulated SLAM run (evaluation only)')
    parser.add_argument('run', type=Path)
    parser.add_argument('--map', type=Path)
    parser.add_argument('--png', action='store_true', help='Also write overlay.png')
    args = parser.parse_args(argv)
    result = evaluate(args.run, args.map)
    if args.png:
        overlay_png(args.run, args.run / 'overlay.png', args.map)
    (args.run / 'eval.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({'verdict': result['verdict'], 'checks': result['checks'],
                      'slam': result['slam_start_anchored'], 'odometry_only': result['odometry_only'],
                      'map': {k: result.get('map', {}).get(k) for k in ('precision', 'recall_all', 'recall_walls', 'occupied_cells')}},
                     indent=2))


if __name__ == '__main__':
    main()
