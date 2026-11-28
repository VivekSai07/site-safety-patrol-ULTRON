"""Run offline LiDAR-only 3D SLAM on the recorded B2 PointCloud2 stream."""

import argparse
import csv
from importlib.metadata import version
import json
from pathlib import Path
import time

import numpy as np

from .real_bag import CLOUD, NANOSECONDS, read_clouds

CLOSURE_DISTANCE_M = 0.20


def trajectory_metrics(poses):
    xyz = poses[:, :3, 3]
    delta = xyz[-1] - xyz[0]
    return {
        'path_length_3d_m': float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()),
        'endpoint_separation_3d_m': float(np.linalg.norm(delta)),
        'endpoint_separation_xy_m': float(np.linalg.norm(delta[:2])),
        'height_span_m': float(np.ptp(xyz[:, 2])),
    }


def write_trajectory(path, stamps, poses):
    names = [f'm{row}{col}' for row in range(3) for col in range(4)]
    with path.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['lidar_stamp_ns', *names])
        for timestamp, matrix in zip(stamps, poses, strict=True):
            writer.writerow([timestamp, *matrix[:3, :].ravel()])


def run(source, out):
    import open3d as o3d
    from kiss_icp.kiss_icp import KissICP
    from kiss_slam.config import KissSLAMConfig
    from kiss_slam.slam import KissSLAM

    config = KissSLAMConfig(
        out_dir=str(out),
        odometry={
            'preprocessing': {'min_range': 0.6, 'max_range': 25.0, 'deskew': False},
            'mapping': {'voxel_size': 0.25},
            'registration': {'max_num_threads': 4},
        },
        local_mapper={'splitting_distance': 2.5, 'voxel_size': 0.15},
        loop_closer={'detector': {'density_map_resolution': 0.2}},
    )
    slam = KissSLAM(config)
    odometry = KissICP(config.kiss_icp_config())
    stamps, baseline = [], []
    started = time.monotonic()
    for _, timestamp, points in read_clouds(source):
        point_times = np.zeros(len(points))
        slam.process_scan(points, point_times)
        odometry.register_frame(points, point_times)
        baseline.append(odometry.last_pose.copy())
        stamps.append(timestamp)
    if len(stamps) < 2:
        raise ValueError('At least two lidar scans are required')
    # Finalize the last submap and let the backend attempt its closure, as its CLI does.
    slam.generate_new_node()
    slam.local_map_graph.erase_last_local_map()
    elapsed = time.monotonic() - started
    poses, baseline = np.asarray(slam.poses), np.asarray(baseline)
    for trajectory in (poses, baseline):
        if trajectory.shape != (len(stamps), 4, 4) or not np.isfinite(trajectory).all():
            raise RuntimeError('Backend poses must match all acquisition timestamps and be finite')
    write_trajectory(out / 'slam.csv', stamps, poses)
    write_trajectory(out / 'odometry.csv', stamps, baseline)
    slam.optimizer.write_graph(str(out / 'submaps.g2o'))

    submaps = list(slam.local_map_graph.local_maps())
    clouds = {}
    for submap in submaps:
        xyz = submap.pcd.point.positions.numpy()
        xyz_world = xyz @ submap.keypose[:3, :3].T + submap.keypose[:3, 3]
        clouds[submap.id] = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz_world))

    closures = []
    for i, j in slam.get_closures():
        # Check both directions in the final global frame. These are bounded fractions,
        # independent of the backend's voxel-overlap heuristic.
        source_distance = np.asarray(clouds[i].compute_point_cloud_distance(clouds[j]))
        target_distance = np.asarray(clouds[j].compute_point_cloud_distance(clouds[i]))
        closures.append({'submap_ids': [int(i), int(j)], 'distance_m': CLOSURE_DISTANCE_M,
                         'source_fraction': float(np.mean(source_distance <= CLOSURE_DISTANCE_M)),
                         'target_fraction': float(np.mean(target_distance <= CLOSURE_DISTANCE_M))})

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.vstack([np.asarray(pcd.points) for pcd in clouds.values()]))
    cloud = cloud.voxel_down_sample(config.local_mapper.voxel_size)
    if not o3d.io.write_point_cloud(str(out / 'map.ply'), cloud):
        raise RuntimeError('Could not export the global point cloud')

    report = {
        'source': str(source.resolve()), 'source_topic': CLOUD, 'source_frame': 'lidar_link',
        'map_frame': 'initial lidar pose, metres, acquisition stamps in nanoseconds',
        'ground_truth_available': False, 'scans': len(stamps),
        'lidar_duration_s': (stamps[-1] - stamps[0]) / NANOSECONDS,
        'elapsed_s_including_odometry_baseline': round(elapsed, 3),
        'packages': {name: version(name) for name in ('kiss-slam', 'kiss-icp', 'map-closures', 'open3d', 'numpy')},
        'submaps': len(submaps), 'accepted_closures': closures,
        'keyposes_xyz': [s.keypose[:3, 3].tolist() for s in submaps],
        'map_points': len(cloud.points),
        'slam': trajectory_metrics(poses),
        'odometry': trajectory_metrics(baseline),
    }
    (out / 'config.json').write_text(config.model_dump_json(indent=2) + '\n')
    (out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='Extracted ROS 2 SQLite bag folder')
    parser.add_argument('out', type=Path, help='New output folder')
    args = parser.parse_args(argv)
    if not (args.source / 'metadata.yaml').is_file():
        parser.error('source must be an extracted rosbag2 folder with metadata.yaml')
    args.out.mkdir(parents=True, exist_ok=False)
    run(args.source, args.out)


if __name__ == '__main__':
    main()
