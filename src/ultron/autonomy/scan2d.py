"""Flatten a 3D lidar sweep into a 2D LaserScan (replaces pointcloud_to_laserscan here).

Points within a height band of base_link are binned by azimuth in the lidar frame (yaw-only
mount, so lidar_link's XY plane is horizontal); each bin keeps its nearest return. The
scan covers only the sector not blocked by the robot's own body: a blind sector published
as "no return" would let SLAM clear map cells it never observed.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ScanGeometry:
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    z_min: float      # base_link heights
    z_max: float
    mount_z: float

    @property
    def bins(self):
        return int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1


def open_sector(blind_columns, azimuth_deg, margin_deg=2.0):
    """Largest contiguous unblocked azimuth sector (radians), shrunk by a safety margin."""
    free = ~np.asarray(blind_columns, dtype=bool)
    if free.all():
        return -math.pi, math.pi
    if not free.any():
        raise ValueError('Lidar is blocked in every direction')
    n = len(free)
    start = int(np.argmin(free))  # a blocked column; walk the ring from there
    best, run, run_start = (0, 0), 0, None
    for k in range(1, n + 1):
        i = (start + k) % n
        if free[i]:
            if run == 0:
                run_start = k
            run += 1
            if run > best[1]:
                best = (run_start, run)
        else:
            run = 0
    first = (start + best[0]) % n
    last = (first + best[1] - 1) % n
    if last < first:
        raise ValueError('Open sector wraps through 180 deg; unsupported scan layout')
    low = float(azimuth_deg[first]) + margin_deg
    high = float(azimuth_deg[last]) - margin_deg
    if high <= low:
        raise ValueError('Open sector narrower than the margin')
    return math.radians(low), math.radians(high)


def geometry_from(config, blind_columns, azimuth_deg):
    projection = config.scan_projection
    low, high = open_sector(blind_columns, azimuth_deg)
    increment = math.radians(projection['angle_increment_deg'])
    high = low + math.floor((high - low) / increment) * increment
    return ScanGeometry(low, high, increment, projection['range_min_m'], projection['range_max_m'],
                        projection['z_min_m'], projection['z_max_m'], config.lidar.mount_xyz_m[2])


def project(points, geometry):
    """Return float32 ranges; ``inf`` marks bins with no return inside range_max."""
    pts = np.asarray(points, dtype=np.float64)
    z_base = pts[:, 2] + geometry.mount_z
    rng = np.hypot(pts[:, 0], pts[:, 1])
    angle = np.arctan2(pts[:, 1], pts[:, 0])
    keep = ((z_base >= geometry.z_min) & (z_base <= geometry.z_max) & (rng >= geometry.range_min)
            & (rng <= geometry.range_max) & (angle >= geometry.angle_min - geometry.angle_increment / 2)
            & (angle <= geometry.angle_max + geometry.angle_increment / 2))
    index = np.rint((angle[keep] - geometry.angle_min) / geometry.angle_increment).astype(np.int64)
    ranges = np.full(geometry.bins, np.inf)
    np.minimum.at(ranges, np.clip(index, 0, geometry.bins - 1), rng[keep])
    return ranges.astype(np.float32)
