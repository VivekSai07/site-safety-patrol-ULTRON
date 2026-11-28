"""Discovery from the lidar: upright, ladder-sized things worth pointing the camera at.

The camera sees one direction; the lidar sees all around. This finds candidates the detector
can then confirm or dismiss (`OnlinePatrol(candidates=LidarCandidates())`):

    cloud in the map frame ─▶ keep points 0.3-2.8 m above the floor, within reach
                           ─▶ 2D grid (CELL), cells whose points span some height
                           ─▶ connected blobs ─▶ keep blobs ladder-sized from above and tall enough

Walls, containers and fences are larger than any ladder from above and are dropped; so is a
ladder leaning on one, which joins its blob. The camera's detector still finds those.

A frame carries its cloud as `frame.cloud`: Nx3 points in the map frame (z up, floor at
`floor` or estimated). `protocol/sim.py` and `protocol/bag.py` provide it with `lidar=1`; the
relay does not yet.

Ladders lying on the floor (`lying=True`, on by default) are a second kind of candidate: low
points 4-25 cm above the local floor, with nothing taller over them, forming a long narrow blob.
The B2's lidar sees the floor densely (hall recording: floor returns from 0.6 m out to 8 m and
more, field of view -53 to +15 degrees), so such a blob shows up in it although Nav2's 2D costmap
may not. Candidates are (x, y) tuples; a lying one has `.lying` True.
"""
import math

import numpy as np

CELL = 0.2
FLAT_SHARE = 0.25   # of a cell's near-floor returns that must be raised for it to count
TILE = 1.0          # metres: the floor is estimated per tile, as odometry tilts it over a room


class Candidate(tuple):
    """(x, y) of a candidate; `lying` when it is a low, long blob on the floor; `cells` the map
    grid cells (CELL metres, [[i, j], ...]) it covers, for an obstacle layer."""
    lying = False
    cells = ()

    def __new__(cls, x, y, lying=False, cells=()):
        made = super().__new__(cls, (x, y))
        made.lying, made.cells = lying, cells
        return made


class LidarCandidates:
    def __init__(self, floor=None, low=0.3, high=2.8, min_top=1.0, min_span=0.6, size=(0.2, 1.8),
                 reach=(1.0, 6.0), every=1.0, min_points=15, lying=True, flat=(0.04, 0.25),
                 flat_reach=6.0, flat_size=(1.0, 6.0), flat_width=0.9):
        self.floor = floor            # floor height in the map frame; None: estimated per cloud
        self.low, self.high = low, high  # height band kept above the floor
        self.min_top = min_top        # a blob must reach this high above the floor ...
        self.min_span = min_span      # ... and its points span at least this much height
        self.size = size              # longest side of the blob from above, metres
        # Distance band from the robot. 6 m covers a 10 x 8 m room from its middle; on the hall
        # recording both free-standing ladders were found at 10 m and at 6 m, with 50 and 23
        # unexplained upright candidates (sim yard: ladders found within 8 m of the route 11 of 11
        # at 10 m, 9 of 11 at 6 m).
        self.reach = reach
        self.every = every            # seconds between clouds looked at
        self.min_points = min_points
        self.lying = lying            # also look for low, long blobs: ladders lying on the floor
        self.flat = flat              # their height band above the local floor, metres
        self.flat_reach = flat_reach  # further out the floor rings are too far apart to join a blob
        self.flat_size = flat_size    # longest side, metres (ladders 1.5-6 m; mats and cables are kept out by height)
        self.flat_width = flat_width  # widest across, metres (a ladder 0.3-0.6 m, plus a ring's smear)
        self._last = None
        self._floor = {}  # map cell -> [returns near the floor, of those raised into the `flat` band]
        self._tall = set()  # map cells that ever had a return above the band

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
            cells = (np.stack([i, j], axis=1) + origin).tolist()
            found.append(Candidate(round(float(cx), 2), round(float(cy), 2), cells=cells))
        if self.lying:
            found += self.find_lying(cloud, pose, floor)
        return found

    def find_lying(self, cloud, pose, floor):
        """Low, long, narrow blobs on the floor with nothing taller over them."""
        x, y = cloud[:, 0] - pose[0], cloud[:, 1] - pose[1]
        distance = np.hypot(x, y)
        near = (distance >= self.reach[0]) & (distance <= self.flat_reach) & (np.abs(cloud[:, 2] - floor) < 0.5)
        points = cloud[near]
        if len(points) < self.min_points:
            return []
        # Local floor: the low tenth of each tile's returns (a ladder covers a small part of a tile).
        tiles = np.floor(points[:, :2] / TILE).astype(np.int64)
        keys, inverse = np.unique(tiles, axis=0, return_inverse=True)
        inverse = inverse.ravel()
        local = np.full(len(keys), floor)
        for k in range(len(keys)):
            z = points[inverse == k, 2]
            if len(z) >= 20:
                local[k] = np.percentile(z, 10)
        height = points[:, 2] - local[inverse]
        cells = np.floor(points[:, :2] / CELL).astype(np.int64)
        # Kept over clouds in the map frame: one sweep hits a ladder pointing away only where its
        # floor rings cross it (2-5 m out they lie 0.3-1.3 m apart); walking on fills the gaps.
        self._tall.update(map(tuple, cells[height >= self.flat[1] + 0.05]))
        for c, is_low in zip(map(tuple, cells[height < self.flat[1]]), height[height < self.flat[1]] > self.flat[0]):
            seen = self._floor.setdefault(c, [0, 0])
            seen[0] += 1
            seen[1] += is_low
        # Raised over a share of the cell's returns (lidar noise alone puts a few floor returns
        # 4 cm up), and with nothing ever taller over it or beside it (a wall's foot, a box, legs).
        flat = [c for c, (n, raised) in self._floor.items() if raised >= 2 and raised >= FLAT_SHARE * n and not any(
            (c[0] + a, c[1] + b) in self._tall for a in (-1, 0, 1) for b in (-1, 0, 1))]
        if not flat:
            return []
        flat = np.array(flat)
        origin = flat.min(axis=0) - 2
        grid = np.zeros(tuple(flat.max(axis=0) - origin + 3), dtype=bool)
        grid[tuple((flat - origin).T)] = True
        # Rails and rungs leave floor between them: cells one empty cell apart are joined.
        joined = grid.copy()
        for a in (-1, 0, 1):
            for b in (-1, 0, 1):
                joined |= np.roll(np.roll(grid, a, axis=0), b, axis=1)
        labels, blobs = _label(joined)
        found = []
        for blob in range(1, blobs + 1):
            members = np.argwhere((labels == blob) & grid)
            if len(members) < 3:
                continue
            centres = (members + origin + 0.5) * CELL
            spread = centres - centres.mean(axis=0)
            _, axes = np.linalg.eigh(spread.T @ spread)
            along, across = spread @ axes[:, 1], spread @ axes[:, 0]
            length, width = np.ptp(along) + CELL, np.ptp(across) + CELL
            if self.flat_size[0] <= length <= self.flat_size[1] and width <= self.flat_width:
                # The middle of its extent: near rings hit its near end more often than far ones.
                cx, cy = centres.mean(axis=0) + axes[:, 1] * (along.min() + along.max()) / 2 \
                    + axes[:, 0] * (across.min() + across.max()) / 2
                found.append(Candidate(round(float(cx), 2), round(float(cy), 2), lying=True,
                                       cells=(members + origin).tolist()))
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
