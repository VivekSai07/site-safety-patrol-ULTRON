"""LocalizationState monitor logic for the ROS 2 stack (no ROS dependency).

Ported verbatim from the validated simulation code ``src/ultron/autonomy/localization_state.py``
(LocalizationMonitor, MapMatcher; docs/SAHIL_VALIDATION.md Phase 4/5) and
``localization_monitor.py`` (confirm_label), so the installed package does not depend on the
repository source tree. ``test/test_monitor_core.py`` checks parity with the original when the
repository is available. A pose becomes VALID only after consecutive scans agree with the map,
never merely because data arrived. Ground truth is never used.
"""
from dataclasses import asdict, dataclass, field
import math

import numpy as np

SCHEMA = 'ultron.localization_state/v1'
STATES = ('INITIALIZING', 'VALID', 'DEGRADED', 'LOST', 'STALE')
PARAM_KEYS = ('scan_age_limit_s', 'pose_age_limit_s', 'match_tolerance_m', 'score_valid_min', 'score_lost_max',
              'valid_consecutive', 'recover_consecutive', 'lost_consecutive', 'degraded_timeout_s',
              'jump_threshold_m', 'jump_threshold_rad')
# Validated simulation values (config/autonomy/sim_slam.yaml: localization). PROPOSED for the robot.
DEFAULT_PARAMS = {'scan_age_limit_s': 0.5, 'pose_age_limit_s': 0.5, 'match_tolerance_m': 0.15,
                  'score_valid_min': 0.70, 'score_lost_max': 0.40, 'valid_consecutive': 5,
                  'recover_consecutive': 10, 'lost_consecutive': 5, 'degraded_timeout_s': 3.0,
                  'jump_threshold_m': 0.5, 'jump_threshold_rad': 0.35}


def confirm_label(t, scored, now, max_wait):
    """State for the pose at time t: the state after the first scored scan with stamp >= t.

    A pose is never vouched for ahead of the sensor (a kidnap between scans would otherwise
    be labelled VALID). Returns None while waiting, 'UNCONFIRMED' if no scan confirms it in time.
    """
    for stamp, state in scored:
        if stamp >= t - 1e-6:
            return state
    return 'UNCONFIRMED' if now - t > max_wait else None


def scan_endpoints(ranges, angle_min, angle_increment, range_min, range_max, sensor_xy=(0.0, 0.0), sensor_yaw=0.0):
    """LaserScan endpoints (N,2) in the frame where the scan origin is at ``sensor_xy``/``sensor_yaw``."""
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    ok = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    a = angles[ok] + sensor_yaw
    return np.column_stack((sensor_xy[0] + ranges[ok] * np.cos(a), sensor_xy[1] + ranges[ok] * np.sin(a)))
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
        from .map_store import load_map_yaml
        grid = load_map_yaml(path)
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
        self.last_rx_stamp = None
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

    def note_scan(self, stamp):
        """A scan with ``stamp`` arrived (it may be scored later, once its TF is available)."""
        if self.last_rx_stamp is None or stamp > self.last_rx_stamp:
            self.last_rx_stamp = stamp

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
        stamps = [t for t in (self.last_scan_stamp, self.last_rx_stamp) if t is not None]
        if not stamps:
            if self.state != 'INITIALIZING':
                self._set('STALE', 'No scan received', now)
            return self.state
        last_scan = max(stamps)
        if now - last_scan > self.p['scan_age_limit_s']:
            self._set('STALE', f'No scan for {now - last_scan:.2f} s (> {self.p["scan_age_limit_s"]} s)', now)
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
