"""Simulated spinning lidar (Helios-32 profile) using MuJoCo batch ray casting.

SIMULATION ONLY. Ranges come from ray intersections with the MuJoCo scene, plus
configured Gaussian noise and dropout. Returns from the robot's own body are produced
by exact per-ray intersection and then removed by a base_link crop box, as a real
self-filter would. The robot's legs are held in a fixed standing pose in this
simulation, so its occlusion in the lidar frame is constant and computed once.
Intensity is a constant placeholder (no reflectivity model). The whole sweep is cast
from one pose; per-point ``time`` offsets are recorded for later motion-distortion work.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import math
import os

import numpy as np

from .hall_scene import LIDAR_HOUSING_GROUP, ROBOT_GROUP


@dataclass
class LidarScan:
    stamp: float                 # simulation time at the end of the sweep (s)
    points: np.ndarray           # (N, 3) float32, lidar_link frame, metres
    ring: np.ndarray             # (N,) uint16 channel index (0 = lowest beam)
    time: np.ndarray             # (N,) float32 offset from sweep start (s)
    intensity: np.ndarray        # (N,) float32 placeholder
    rays_total: int
    self_filtered: int
    dropped: int
    no_return: int
    blind_columns: np.ndarray    # (columns,) bool: azimuth columns mostly blocked by the robot
    azimuth_deg: np.ndarray      # (columns,) column azimuths in lidar frame
    # EVALUATION ONLY: true hit geom per returned point. Never pass to an estimator.
    eval_geom_id: np.ndarray

    @property
    def blind_fraction(self):
        return float(self.blind_columns.mean())


def rotate(points, R, out=None):
    """points @ R.T for (N,3) points without BLAS.

    numpy's ``@`` dispatches to OpenBLAS, whose worker threads busy-wait after each call
    and measured to slow the following mj_multiRay ~2.5x on this laptop. Element-wise
    arithmetic avoids that contention.
    """
    if out is None:
        out = np.empty_like(points, dtype=np.float64)
    for row in range(3):
        np.multiply(points[:, 0], R[row, 0], out=out[:, row])
        out[:, row] += points[:, 1] * R[row, 1]
        out[:, row] += points[:, 2] * R[row, 2]
    return out


def _rotation_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class SimLidar:
    def __init__(self, sim, profile, self_filter, *, seed=0, threads=None):
        self.sim = sim
        self.profile = profile
        self.rng = np.random.default_rng(seed)
        elevation = np.radians(np.asarray(profile.vertical_angles_deg))
        columns = profile.columns
        self.azimuth_deg = np.arange(columns) * profile.horizontal_resolution_deg - 180.0
        azimuth = np.radians(self.azimuth_deg)
        el, az = np.meshgrid(elevation, azimuth, indexing='ij')   # (channels, columns)
        self.directions = np.stack((np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)),
                                   axis=-1).reshape(-1, 3)        # lidar frame, unit vectors
        self.ring = np.repeat(np.arange(profile.channels, dtype=np.uint16), columns)
        self.column = np.tile(np.arange(columns), profile.channels)
        self.period = 1.0 / profile.scan_rate_hz
        self.time = (self.column / columns * self.period).astype(np.float32)
        self.mount_R = _rotation_rpy(*profile.mount_rpy_rad)
        self.mount_t = np.asarray(profile.mount_xyz_m, dtype=float)
        self.crop_min = np.asarray(self_filter['min_xyz_m'], dtype=float)
        self.crop_max = np.asarray(self_filter['max_xyz_m'], dtype=float)
        # Batch cast: scene only (not the lidar housing, not the robot).
        self.geomgroup = np.ones(6, dtype=np.uint8)
        self.geomgroup[LIDAR_HOUSING_GROUP] = 0
        self.geomgroup[ROBOT_GROUP] = 0
        n = len(self.directions)
        self._robot_geom, self._robot_dist = self._robot_occlusion()
        self._geomid = np.zeros(n, dtype=np.int32)
        self._dist = np.zeros(n, dtype=np.float64)
        # mj_multiRay releases the GIL, so contiguous ray chunks run in parallel threads.
        # Buffers are allocated once and reused: passing freshly allocated direction
        # arrays to the binding measured ~2.5x slower per call (mujoco 3.3.7).
        threads = threads or min(4, os.cpu_count() or 1)  # 4 measured fastest here; 8 slower
        bounds = np.linspace(0, n, threads + 1).astype(int)
        self._world_dirs = np.empty((n, 3), dtype=np.float64)
        self._chunks = [(a, b, self.directions[a:b], self._world_dirs[a:b], self._world_dirs[a:b].reshape(-1),
                         self._geomid[a:b], self._dist[a:b]) for a, b in zip(bounds[:-1], bounds[1:]) if b > a]
        self._pool = ThreadPoolExecutor(len(self._chunks)) if len(self._chunks) > 1 else None

    def _robot_occlusion(self):
        """Exact distance from the lidar to the robot's own body for every beam.

        mj_multiRay (3.3.7) misses geoms very close to the ray origin (measured), so the
        robot is intersected with exact single-ray mj_ray once. Valid while the robot is
        rigid relative to the lidar (fixed standing legs, as in HallSim).
        """
        mujoco, model, data = self.sim.mujoco, self.sim.model, self.sim.data
        site = self.sim.lidar_site
        origin = data.site_xpos[site].copy()
        world = rotate(self.directions, np.array(data.site_xmat[site].reshape(3, 3)))
        group = np.zeros(6, dtype=np.uint8)
        group[ROBOT_GROUP] = 1
        geom = np.full(len(world), -1, dtype=np.int32)
        dist = np.full(len(world), -1.0)
        out = np.zeros(1, dtype=np.int32)
        for i, direction in enumerate(world):
            d = mujoco.mj_ray(model, data, origin, direction, group, 1, -1, out)
            if out[0] >= 0:
                geom[i], dist[i] = out[0], d
        return geom, dist

    @property
    def rays(self):
        return len(self.directions)

    def scan(self, stamp):
        mujoco = self.sim.mujoco
        model, data = self.sim.model, self.sim.data
        site = self.sim.lidar_site
        origin = data.site_xpos[site].copy()
        R_world = data.site_xmat[site].reshape(3, 3)
        R_world = np.array(R_world)

        def cast(chunk):
            _, _, local, world, flat, geom, dist = chunk
            rotate(local, R_world, out=world)
            mujoco.mj_multiRay(model, data, origin, flat, self.geomgroup, 1, -1, geom, dist,
                               len(geom), self.profile.range_max_m)

        if self._pool is None:
            cast(self._chunks[0])
        else:
            list(self._pool.map(cast, self._chunks))
        dist = self._dist.copy()
        geom = self._geomid.copy()
        scene_hit = (geom >= 0) & (dist >= 0)
        robot_hit = self._robot_geom >= 0
        use_robot = robot_hit & (~scene_hit | (self._robot_dist <= dist))
        dist = np.where(use_robot, self._robot_dist, dist)
        geom = np.where(use_robot, self._robot_geom, geom)
        hit = scene_hit | robot_hit
        no_return = int((~hit).sum())
        # Self-filter in base_link: computed on true geometry before noise, like a fixed crop box.
        lidar_pts = self.directions * np.where(hit, dist, 0.0)[:, None]
        base_pts = rotate(lidar_pts, self.mount_R) + self.mount_t
        inside = np.all((base_pts >= self.crop_min) & (base_pts <= self.crop_max), axis=1) & hit
        # Blind column: most channels of that azimuth are blocked by the robot itself.
        blocked = inside.reshape(self.profile.channels, -1).mean(axis=0) > 0.5
        keep = hit & ~inside
        noisy = dist + self.rng.normal(0.0, self.profile.range_noise_std_m, dist.shape)
        keep &= (noisy >= self.profile.range_min_m) & (noisy <= self.profile.range_max_m)
        dropout = self.rng.random(dist.shape) < self.profile.dropout_probability
        dropped = int((keep & dropout).sum())
        keep &= ~dropout
        points = (self.directions[keep] * noisy[keep, None]).astype(np.float32)
        return LidarScan(stamp=float(stamp), points=points, ring=self.ring[keep],
                         time=self.time[keep], intensity=np.full(len(points), 100.0, np.float32),
                         rays_total=self.rays, self_filtered=int(inside.sum()), dropped=dropped,
                         no_return=no_return, blind_columns=blocked, azimuth_deg=self.azimuth_deg,
                         eval_geom_id=geom[keep])

    def close(self):
        if self._pool is not None:
            self._pool.shutdown()

    def lidar_to_world(self, points):
        """Transform lidar-frame points using the TRUE pose (evaluation/visualisation only)."""
        data = self.sim.data
        R = data.site_xmat[self.sim.lidar_site].reshape(3, 3)
        return rotate(points, R) + data.site_xpos[self.sim.lidar_site]


def topdown_png(scan_world, objects, pose, path, bounds=(-0.5, -0.5, 12.5, 10.5), scale=60):
    """Top-down review image: true object footprints (grey) and scan points coloured by height."""
    from PIL import Image, ImageDraw
    x0, y0, x1, y1 = bounds
    width, height = int((x1 - x0) * scale), int((y1 - y0) * scale)
    image = Image.new('RGB', (width, height), (250, 250, 250))
    draw = ImageDraw.Draw(image)
    to_px = lambda x, y: ((x - x0) * scale, (y1 - y) * scale)
    for obj in objects:
        if obj.footprint:
            draw.polygon([to_px(*c) for c in obj.footprint], outline=(170, 170, 170))
    z = scan_world[:, 2]
    for (x, y, _), level in zip(scan_world, np.clip(z / 2.5, 0, 1)):
        px, py = to_px(x, y)
        draw.point((px, py), fill=(int(255 * level), 40, int(255 * (1 - level))))
    rx, ry, yaw = pose
    px, py = to_px(rx, ry)
    draw.ellipse((px - 8, py - 8, px + 8, py + 8), outline=(0, 140, 0), width=3)
    draw.line((px, py, *to_px(rx + 0.5 * math.cos(yaw), ry + 0.5 * math.sin(yaw))), fill=(0, 140, 0), width=3)
    image.save(path)
    return path
