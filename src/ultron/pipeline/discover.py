"""Discovery from the lidar: upright, ladder-sized things worth pointing the camera at.

The camera sees one direction; the lidar sees all around. This finds candidates the detector
can then confirm or dismiss (`OnlinePatrol(candidates=LidarCandidates())`):

    cloud in the map frame ─▶ keep points 0.3-2.8 m above the floor, within reach
                           ─▶ 2D grid (CELL), cells whose points span some height
                           ─▶ connected blobs ─▶ keep blobs ladder-sized from above and tall enough

Walls, containers and fences are larger than any ladder from above and are dropped; so is a
ladder leaning on one, which joins its blob. The camera's detector still finds those.

A frame carries its cloud as `frame.cloud`: Nx3 points in the map frame (z up, floor at
`floor` or estimated). `protocol/sim.py` provides it with `lidar=1`; the relay does not yet.
"""
import math

import numpy as np

CELL = 0.2


class LidarCandidates:
    def __init__(self, floor=None, low=0.3, high=2.8, min_top=1.0, min_span=0.6, size=(0.2, 1.8),
                 reach=(1.0, 10.0), every=1.0, min_points=15):
        self.floor = floor            # floor height in the map frame; None: estimated per cloud
        self.low, self.high = low, high  # height band kept above the floor
        self.min_top = min_top        # a blob must reach this high above the floor ...
        self.min_span = min_span      # ... and its points span at least this much height
        self.size = size              # longest side of the blob from above, metres
        self.reach = reach            # distance band from the robot in which clouds are dense enough
        self.every = every            # seconds between clouds looked at
        self.min_points = min_points
        self._last = None

    def __call__(self, frame):
        cloud = getattr(frame, "cloud", None)
        if cloud is None or frame.pose is None:
            return []
        if self._last is not None and 0 <= frame.stamp - self._last < self.every:
            return []
        self._last = frame.stamp
        return self.find(np.asarray(cloud, dtype=np.float32), frame.pose)

    def find(self, cloud, pose):
        x, y = cloud[:, 0] - pose[0], cloud[:, 1] - pose[1]
        distance = np.hypot(x, y)
        near = (distance >= self.reach[0]) & (distance <= self.reach[1])
        floor = self.floor if self.floor is not None else float(np.percentile(cloud[near, 2], 5)) if near.any() else 0.0
        height = cloud[:, 2] - floor
        keep = near & (height > self.low) & (height < self.high)
        points, height = cloud[keep], height[keep]
        if len(points) < self.min_points:
            return []
        cells = np.floor(points[:, :2] / CELL).astype(np.int64)
        origin = cells.min(axis=0)
        cells -= origin
        shape = cells.max(axis=0) + 1
        flat = cells[:, 0] * shape[1] + cells[:, 1]
        top = np.full(shape[0] * shape[1], -np.inf)
        bottom = np.full(shape[0] * shape[1], np.inf)
        count = np.zeros(shape[0] * shape[1], dtype=np.int64)
        np.maximum.at(top, flat, height)
        np.minimum.at(bottom, flat, height)
        np.add.at(count, flat, 1)
        occupied = (count >= 2).reshape(shape)
        labels, blobs = _label(occupied)
        found = []
        for blob in range(1, blobs + 1):
            members = np.flatnonzero(labels.ravel() == blob)
            i, j = np.divmod(members, shape[1])
            longest = max(i.max() - i.min() + 1, j.max() - j.min() + 1) * CELL
            span = top[members].max() - bottom[members].min()
            if not (self.size[0] <= longest <= self.size[1]) or top[members].max() < self.min_top:
                continue
            if span < self.min_span or count[members].sum() < self.min_points:
                continue
            weights = count[members]
            cx = (np.average(i, weights=weights) + origin[0] + 0.5) * CELL
            cy = (np.average(j, weights=weights) + origin[1] + 0.5) * CELL
            found.append((round(float(cx), 2), round(float(cy), 2)))
        return found


def _label(grid):
    """Connected components (8-neighbour) of a boolean grid: (labels, count). Pure numpy/python."""
    labels = np.zeros(grid.shape, dtype=np.int32)
    count = 0
    for start in zip(*np.nonzero(grid)):
        if labels[start]:
            continue
        count += 1
        labels[start] = count
        stack = [start]
        while stack:
            a, b = stack.pop()
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    p, q = a + da, b + db
                    if 0 <= p < grid.shape[0] and 0 <= q < grid.shape[1] and grid[p, q] and not labels[p, q]:
                        labels[p, q] = count
                        stack.append((p, q))
    return labels, count


def world_cloud(points, transform):
    """Nx3 sensor-frame points to the map frame with a 4x4 transform."""
    transform = np.asarray(transform, dtype=np.float64)
    return (np.asarray(points, dtype=np.float64) @ transform[:3, :3].T + transform[:3, 3]).astype(np.float32)


def bearing(pose, point):
    return math.atan2(point[1] - pose[1], point[0] - pose[0])
