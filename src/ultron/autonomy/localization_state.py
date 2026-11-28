"""LocalizationState contract, scan-to-map validity monitor and time-correct pose lookup.

No ROS dependency. A pose becomes VALID only after consecutive scans agree with the loaded
map (match score), never merely because data arrived. Ground truth is never used here.

Match score: fraction of 2D scan endpoints, placed with the estimated pose, that land within
``match_tolerance_m`` of an occupied cell of the saved map. It measures consistency with the
map, not accuracy; a symmetric environment can still produce a confident wrong pose.
"""
from bisect import bisect_left
from collections import deque
from dataclasses import asdict, dataclass, field
import math

import numpy as np

SCHEMA = 'ultron.localization_state/v1'
STATES = ('INITIALIZING', 'VALID', 'DEGRADED', 'LOST', 'STALE')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class LocalizationState:
    stamp: float                      # time the pose refers to (scan stamp), s
    evaluated_at: float               # time the state was computed, s
    map_id: str
    map_version: int
    state: str
    reason: str
    position: list = field(default_factory=lambda: [0.0, 0.0, 0.0])   # metres, map frame
    orientation_xyzw: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    match_score: float | None = None
    covariance_xy_yaw: list | None = None
    covariance_source: str = 'UNAVAILABLE: backend does not provide one; use state and match_score'
    age_s: float = 0.0
    age_limit_s: float = 0.5
    schema: str = SCHEMA
    clock_id: str = 'ros_sim_time'
    frame_id: str = 'map'
    child_frame_id: str = 'base_link'
    source: str = 'SLAM_TOOLBOX_LOCALIZATION'
    ground_truth_used: bool = False

    @property
    def yaw(self):
        x, y, z, w = self.orientation_xyzw
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def to_dict(self):
        return asdict(self) | {'schema': SCHEMA}


def accept_pose(state, map_id, map_version, now, *, tolerance_s=1e-3):
    """Gate for consumers (navigation, perception). Returns (ok, reason)."""
    if state is None:
        return False, 'NO_LOCALIZATION_STATE'
    if state.map_id != map_id or state.map_version != map_version:
        return False, f'MAP_MISMATCH: state {state.map_id} v{state.map_version}, requested {map_id} v{map_version}'
    if state.state != 'VALID':
        return False, f'NOT_VALID: {state.state} ({state.reason})'
    if state.stamp > now + tolerance_s:
        return False, 'STAMP_IN_FUTURE'
    if now - state.stamp > state.age_limit_s:
        return False, f'STALE: age {now - state.stamp:.3f} s > {state.age_limit_s} s'
    return True, 'OK'


class MapMatcher:
    """Occupied-cell proximity mask of a saved occupancy grid (map frame)."""

    def __init__(self, occupied_xy, resolution, tolerance_m):
        self.res = float(resolution)
        xy = np.asarray(occupied_xy, dtype=float)
        if not len(xy):
            raise ValueError('Map has no occupied cells')
        self.origin = xy.min(axis=0) - tolerance_m - self.res
        size = np.ceil((xy.max(axis=0) + tolerance_m + self.res - self.origin) / self.res).astype(int) + 1
        mask = np.zeros(size[::-1], dtype=bool)                       # rows = y, cols = x
        cells = np.floor((xy - self.origin) / self.res).astype(int)
        radius = int(math.ceil(tolerance_m / self.res))
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if (dx * dx + dy * dy) * self.res * self.res <= tolerance_m * tolerance_m + 1e-12:
                    c = cells + (dx, dy)
                    mask[np.clip(c[:, 1], 0, size[1] - 1), np.clip(c[:, 0], 0, size[0] - 1)] = True
        self.mask = mask

    @classmethod
    def from_map_yaml(cls, path, tolerance_m):
        from .eval_slam import load_map
        grid = load_map(path)
        return cls(grid['occupied_xy'], grid['resolution'], tolerance_m)

    def score(self, pose, endpoints_base):
        """Fraction of base_link endpoints (N,2) near occupied cells after placing them at ``pose``."""
        pts = np.asarray(endpoints_base, dtype=float)
        if not len(pts):
            return 0.0
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.column_stack((x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]))
        cells = np.floor((world - self.origin) / self.res).astype(int)
        inside = (cells[:, 0] >= 0) & (cells[:, 1] >= 0) & (cells[:, 0] < self.mask.shape[1]) & (cells[:, 1] < self.mask.shape[0])
        hits = np.zeros(len(pts), dtype=bool)
        hits[inside] = self.mask[cells[inside, 1], cells[inside, 0]]
        return float(hits.mean())


class LocalizationMonitor:
    """State machine: INITIALIZING -> VALID <-> DEGRADED -> LOST; STALE when inputs stop."""

    def __init__(self, params, map_id, map_version):
        self.p = params
        self.map_id, self.map_version = map_id, int(map_version)
        self.state = 'INITIALIZING'
        self.reason = 'Waiting for consecutive scans that match the map'
        self.required = params['valid_consecutive']
        self.good_run = self.lost_run = 0
        self.degraded_since = None
        self.last_scan_stamp = None
        self.last_pose = None
        self.last_pose_stamp = None
        self.last_score = None
        self.transitions = []

    def _set(self, state, reason, t):
        if state != self.state:
            self.transitions.append({'t': round(t, 3), 'from': self.state, 'to': state, 'reason': reason})
            if state in ('LOST', 'STALE'):
                self.required = self.p['recover_consecutive']
            elif state == 'DEGRADED' and self.state == 'VALID':
                self.required = self.p['valid_consecutive']
            if state == 'DEGRADED':
                self.degraded_since = t
            if state == 'STALE':
                self.good_run = 0
        self.state, self.reason = state, reason

    def update_scan(self, stamp, pose, endpoints_base, matcher):
        """Process one scan whose estimated pose (map frame) at ``stamp`` is ``pose`` or None."""
        self.last_scan_stamp = stamp
        if pose is None:
            self.good_run = 0
            self.lost_run += 1
            self.last_score = None
            if self.state == 'VALID':
                self._set('DEGRADED', 'No map->base_link transform at scan time', stamp)
            return self.state
        jumped = False
        if self.last_pose is not None:
            dist = math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
            dyaw = abs(wrap(pose[2] - self.last_pose[2]))
            jumped = dist > self.p['jump_threshold_m'] or dyaw > self.p['jump_threshold_rad']
        self.last_pose, self.last_pose_stamp = tuple(pose), stamp
        score = matcher.score(pose, endpoints_base)
        self.last_score = score
        good = score >= self.p['score_valid_min'] and not jumped
        self.good_run = self.good_run + 1 if good else 0
        self.lost_run = self.lost_run + 1 if score <= self.p['score_lost_max'] else 0
        if jumped:
            self._set('DEGRADED' if self.state != 'LOST' else 'LOST',
                      f'Pose jumped between scans; revalidating', stamp)
        elif self.state == 'STALE':
            self._set('DEGRADED', 'Inputs resumed after STALE; revalidating', stamp)
        if self.state in ('INITIALIZING', 'DEGRADED', 'LOST'):
            if self.good_run >= self.required:
                self._set('VALID', f'{self.good_run} consecutive scans match the map (score {score:.2f})', stamp)
            elif self.lost_run >= self.p['lost_consecutive']:
                self._set('LOST', f'{self.lost_run} consecutive scans do not match the map (score {score:.2f})', stamp)
            elif (self.state == 'DEGRADED' and self.degraded_since is not None
                  and stamp - self.degraded_since > self.p['degraded_timeout_s']):
                self._set('LOST', f'DEGRADED for more than {self.p["degraded_timeout_s"]} s', stamp)
        elif self.state == 'VALID' and not good:
            self._set('DEGRADED', f'Scan/map match score {score:.2f} below {self.p["score_valid_min"]}', stamp)
        return self.state

    def tick(self, now):
        """Age checks; call periodically with the current clock."""
        if self.last_scan_stamp is None:
            if self.state != 'INITIALIZING':
                self._set('STALE', 'No scan received', now)
            return self.state
        if now - self.last_scan_stamp > self.p['scan_age_limit_s']:
            self._set('STALE', f'No scan for {now - self.last_scan_stamp:.2f} s (> {self.p["scan_age_limit_s"]} s)', now)
        elif self.last_pose_stamp is None or now - self.last_pose_stamp > self.p['pose_age_limit_s']:
            if self.state != 'INITIALIZING':
                self._set('STALE', 'No map->base_link transform within the age limit', now)
        return self.state

    def snapshot(self, now):
        pose = self.last_pose or (0.0, 0.0, 0.0)
        stamp = self.last_pose_stamp if self.last_pose_stamp is not None else now
        return LocalizationState(stamp=stamp, evaluated_at=now, map_id=self.map_id, map_version=self.map_version,
                                 state=self.state, reason=self.reason, position=[pose[0], pose[1], 0.0],
                                 orientation_xyzw=[0.0, 0.0, math.sin(pose[2] / 2), math.cos(pose[2] / 2)],
                                 match_score=self.last_score, age_s=max(0.0, now - stamp),
                                 age_limit_s=self.p['pose_age_limit_s'])


class TransformRejected(ValueError):
    pass


class TransformHistory:
    """map->base_link samples with their localization state; time-correct lookup for observations."""

    def __init__(self, horizon_s=10.0, max_gap_s=0.1):
        self.horizon, self.max_gap = horizon_s, max_gap_s
        self.samples = deque()   # (t, x, y, yaw, state)

    def add(self, t, pose, state):
        if self.samples and t <= self.samples[-1][0]:
            raise ValueError('Samples must be added in increasing time')
        self.samples.append((float(t), *map(float, pose), state))
        while self.samples and self.samples[-1][0] - self.samples[0][0] > self.horizon:
            self.samples.popleft()

    def transform_at(self, t):
        """Pose valid at acquisition time ``t`` (interpolated), or TransformRejected."""
        if not self.samples:
            raise TransformRejected('NO_SAMPLES')
        times = [s[0] for s in self.samples]
        if t > times[-1] + 1e-9:
            raise TransformRejected(f'EXTRAPOLATION: {t - times[-1]:.3f} s after newest sample')
        if t < times[0] - 1e-9:
            raise TransformRejected(f'TOO_OLD: {times[0] - t:.3f} s before oldest retained sample')
        i = bisect_left(times, t)
        if i < len(times) and abs(times[i] - t) <= 1e-9:
            a = b = self.samples[i]
        else:
            a, b = self.samples[i - 1], self.samples[i]
        if b[0] - a[0] > self.max_gap:
            raise TransformRejected(f'GAP: {b[0] - a[0]:.3f} s between samples')
        if a[4] != 'VALID' or b[4] != 'VALID':
            raise TransformRejected(f'NOT_VALID_AT_STAMP: {a[4]}/{b[4]}')
        f = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), wrap(a[3] + f * wrap(b[3] - a[3])))
