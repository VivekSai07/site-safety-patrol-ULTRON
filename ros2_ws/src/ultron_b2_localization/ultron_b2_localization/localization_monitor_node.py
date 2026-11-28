"""LocalizationState publisher for the B2 stack: ``state/localization`` (ultron_b2_interfaces/LocalizationState).

Port of the validated simulation monitor (src/ultron/autonomy/localization_monitor.py) with the
same rules (``monitor_core``): match score = fraction of 2D scan endpoints, placed with the
``map -> base_link`` TF at the scan stamp, within ``match_tolerance_m`` of an occupied map cell.
VALID only after consecutive matching scans; STALE when scans or the transform stop; pose jumps
and kidnaps drop to DEGRADED/LOST. Ground truth is never read.

Map source:
- ``map_info`` (localization): the saved map announced on ``map/info`` by map_manager (file,
  hash-verified there). Until one arrives the state stays INITIALIZING ("no map loaded").
- ``live_grid`` (mapping): the live ``map`` OccupancyGrid from slam_toolbox. Scans are then
  scored against a map built from the same scans: a consistency check, not a map validation.
"""
import json
import math
import time

import numpy as np

from .monitor_core import DEFAULT_PARAMS, LocalizationMonitor, MapMatcher, PARAM_KEYS, confirm_label, scan_endpoints
from .replay_clock_guard import ReplayClockGuard

# Minimum scan age before scoring. slam_toolbox future-dates map->odom (transform_timeout) and processes scans
# at most every minimum_time_interval, so the limiting link is odom->base_link at the scan stamp: a scan whose
# map->base_link is not yet in the TF buffer (odometry latency under load) waits up to SCORE_MAX_WAIT_S before it
# is scored as 'No map->base_link transform'. Integration: was a fixed 0.15 s, which used ~0.15-0.25 s of the
# 0.5 s pose-age budget of the motion gate on every scan. Age limits and the gate are unchanged.
SCORE_DELAY_S = 0.05
SCORE_MAX_WAIT_S = 0.4


def grid_occupied_xy(info, data, occupied_min=65):
    """OccupancyGrid -> occupied cell centres (map frame; unrotated origin)."""
    grid = np.asarray(data, dtype=np.int16).reshape(info.height, info.width)
    rows, cols = np.nonzero(grid >= occupied_min)
    res = info.resolution
    o = info.origin.position
    return np.column_stack((o.x + (cols + 0.5) * res, o.y + (rows + 0.5) * res))


class LocalizationMonitorNode:
    def __init__(self, node):
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener
        from ultron_b2_interfaces.msg import LocalizationState, MapInfo
        self.node = node
        p = lambda name, default: node.declare_parameter(name, default).value
        self.params = {k: p(k, v) for k, v in DEFAULT_PARAMS.items()}
        assert set(self.params) == set(PARAM_KEYS)
        self.map_source = p('map_source', 'map_info')
        if self.map_source not in ('map_info', 'live_grid'):
            raise SystemExit(f'map_source must be map_info or live_grid, not {self.map_source!r}')
        self.map_frame, self.base_frame = p('map_frame', 'map'), p('base_frame', 'base_link')
        self.source = p('source', 'SLAM_TOOLBOX_LOCALIZATION')
        self.live_map_id = p('live_map_id', 'live-mapping-session')
        self.live_update_s = p('live_map_update_s', 2.0)
        self.heartbeat_s = p('heartbeat_s', 0.5)
        self.clock_id = 'ros_sim_time' if node.get_parameter('use_sim_time').value else 'ros_time'
        replay_watchdog = p('replay_clock_watchdog', False)
        if replay_watchdog and self.clock_id != 'ros_sim_time':
            raise ValueError('replay clock watchdog requires use_sim_time')
        self.clock_guard = ReplayClockGuard(p('replay_clock_timeout_s', 0.5),
                                            p('replay_clock_jump_s', 0.5)) if replay_watchdog else None
        self.clock_invalidated = bool(self.clock_guard)
        self.clock_watchdog_last_publish = -math.inf
        self.last_observed_ros = None
        self.static_transforms = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.LocalizationState = LocalizationState
        self.pub = node.create_publisher(LocalizationState, p('state_topic', 'state/localization'), latched)
        node.create_subscription(LaserScan, p('scan_topic', 'sensors/scan'), self.pending_scans_append,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        if self.map_source == 'map_info':
            node.create_subscription(MapInfo, p('map_info_topic', 'map/info'), self._on_map_info, latched)
        else:
            node.create_subscription(OccupancyGrid, p('map_topic', 'map'), self._on_grid, latched)
        self.pending_scans, self.scored = [], []
        self.matcher, self.monitor = None, None
        self.map_key = None
        self.last_grid_t = None
        self.no_map_reason = 'No map loaded (waiting for map/info)' if self.map_source == 'map_info' \
            else 'Waiting for the live map'
        self.sensor_poses = {}
        self.last_heartbeat = -math.inf
        self.transitions = []
        node.create_timer(0.05, self._tick)
        if self.clock_guard:
            from rclpy.clock import Clock, ClockType
            from rosgraph_msgs.msg import Clock as ClockMsg
            from tf2_msgs.msg import TFMessage
            clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(ClockMsg, '/clock', self._on_clock, clock_qos)
            node.create_subscription(TFMessage, '/tf_static', self._on_static, static_qos)
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            node.create_timer(0.1, self._watchdog_tick, clock=self.steady_clock)
        node.get_logger().info(f'localization_monitor: map_source={self.map_source}, params={json.dumps(self.params)}')

    def pending_scans_append(self, msg):
        if self.clock_guard and self.clock_guard.stalled(time.monotonic()):
            return
        self.pending_scans.append(msg)

    def _on_static(self, msg):
        for transform in msg.transforms:
            self.static_transforms[transform.child_frame_id] = transform

    def _invalidate_clock(self, reason, clear_tf=False):
        self.clock_invalidated = True
        self.pending_scans.clear()
        self.scored.clear()
        self.last_grid_t = None
        if clear_tf:
            self.buffer.clear()
            for transform in self.static_transforms.values():
                self.buffer.set_transform_static(transform, 'replay_static_cache')
        if self.monitor is not None:
            self.monitor.invalidate_clock(self._now(), reason)
        self._publish(self._now())
        self.clock_watchdog_last_publish = time.monotonic()

    def _on_clock(self, msg):
        wall = time.monotonic()
        if self.clock_guard.stalled(wall) and not self.clock_invalidated:
            self._invalidate_clock('Replay clock stopped advancing')
        stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        event = self.clock_guard.observe(stamp, wall)
        if event == 'jump':
            self._invalidate_clock('Replay clock jumped; awaiting fresh scans and transforms', clear_tf=True)
        elif event == 'progress':
            self.clock_invalidated = False

    def _watchdog_tick(self):
        wall = time.monotonic()
        if not self.clock_guard.stalled(wall):
            return
        if not self.clock_invalidated:
            self._invalidate_clock('Replay clock stopped advancing')
        elif wall - self.clock_watchdog_last_publish >= self.heartbeat_s:
            self._publish(self._now())
            self.clock_watchdog_last_publish = wall

    # ---------------------------------------------------------------- map
    def _install(self, matcher, map_id, version):
        key = (map_id, int(version))
        if key != self.map_key or self.monitor is None:
            if self.monitor is not None:
                self.transitions += self.monitor.transitions
            self.monitor = LocalizationMonitor(self.params, map_id, version)
            if self.clock_guard and self.clock_guard.stalled(time.monotonic()):
                self.monitor.invalidate_clock(self._now(), 'Waiting for advancing replay clock')
            self.scored = []
            self.map_key = key
            self.node.get_logger().info(f'localization_monitor: scoring against {map_id} v{version}')
        self.matcher = matcher

    def _on_map_info(self, info):
        if not info.yaml_path:
            self.node.get_logger().error('map/info without yaml_path; ignoring')
            return
        try:
            matcher = MapMatcher.from_map_yaml(info.yaml_path, self.params['match_tolerance_m'])
        except Exception as exc:
            self.node.get_logger().error(f'cannot load {info.yaml_path}: {exc}')
            return
        self._install(matcher, info.map_id, info.map_version)

    def _on_grid(self, grid):
        now = self._now()
        if self.last_grid_t is not None and now - self.last_grid_t < self.live_update_s:
            return
        if abs(grid.info.origin.orientation.z) > 1e-6:
            self.node.get_logger().warn('rotated map origin not supported', throttle_duration_sec=10.0)
            return
        xy = grid_occupied_xy(grid.info, grid.data)
        if not len(xy):
            return
        self.last_grid_t = now
        self._install(MapMatcher(xy, grid.info.resolution, self.params['match_tolerance_m']), self.live_map_id, 0)

    # ---------------------------------------------------------------- TF
    def _now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _lookup(self, stamp):
        from rclpy.time import Time
        try:
            tf = self.buffer.lookup_transform(self.map_frame, self.base_frame, Time.from_msg(stamp))
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def _sensor(self, frame):
        if frame not in self.sensor_poses:
            from rclpy.time import Time
            tf = self.buffer.lookup_transform(self.base_frame, frame, Time())
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.sensor_poses[frame] = ((tf.transform.translation.x, tf.transform.translation.y), yaw)
        return self.sensor_poses[frame]

    # ---------------------------------------------------------------- output
    def _publish(self, now):
        from ultron_b2_contracts import localization_state_to_msg
        if self.monitor is None:
            d = {'schema': 'ultron.localization_state/v1', 'stamp': now, 'evaluated_at': now, 'map_id': '',
                 'map_version': 0, 'state': 'INITIALIZING',
                 'reason': ('Waiting for advancing replay clock' if self.clock_guard and self.clock_invalidated
                            else self.no_map_reason),
                 'position': [0.0, 0.0, 0.0], 'orientation_xyzw': [0.0, 0.0, 0.0, 1.0], 'match_score': None,
                 'covariance_xy_yaw': None,
                 'covariance_source': 'UNAVAILABLE: backend does not provide one; use state and match_score',
                 'age_s': 0.0, 'age_limit_s': self.params['pose_age_limit_s']}
        else:
            d = self.monitor.snapshot(now).to_dict()
        d.update({'clock_id': self.clock_id, 'frame_id': self.map_frame, 'child_frame_id': self.base_frame,
                  'source': self.source, 'ground_truth_used': False})
        self.pub.publish(localization_state_to_msg(d))
        self.last_heartbeat = now

    def _tick(self):
        now = self._now()
        if self.clock_guard and self.last_observed_ros is not None and now < self.last_observed_ros:
            self.clock_guard.progressing = False
            self.clock_guard.last_ns = None
            self._invalidate_clock('Replay clock jumped; awaiting fresh scans and transforms', clear_tf=True)
        self.last_observed_ros = now
        if self.clock_guard and self.clock_guard.stalled(time.monotonic()):
            return
        if now <= 0:
            return
        if self.monitor is None:
            self.pending_scans = self.pending_scans[-5:]
            if now - self.last_heartbeat >= self.heartbeat_s:
                self._publish(now)
            return
        pending = sorted(self.pending_scans, key=lambda m: (m.header.stamp.sec, m.header.stamp.nanosec))
        self.pending_scans = []
        age_limit = min(self.params['scan_age_limit_s'], self.params['pose_age_limit_s'])
        pending = [m for m in pending
                   if self.monitor.accepts_scan(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
                   and -self.params['scan_age_limit_s'] <= now - (
                       m.header.stamp.sec + m.header.stamp.nanosec * 1e-9) <= age_limit]
        for msg in pending:
            self.monitor.note_scan(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        published = False
        for i, msg in enumerate(pending):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if not self.monitor.accepts_scan(t):
                continue
            if now - t < SCORE_DELAY_S:
                self.pending_scans = pending[i:]
                break
            try:
                sensor_xy, sensor_yaw = self._sensor(msg.header.frame_id)
            except Exception as exc:
                self.node.get_logger().warn(f'no {self.base_frame}->{msg.header.frame_id}: {exc}',
                                            throttle_duration_sec=5.0)
                continue
            pose = self._lookup(msg.header.stamp)
            if pose is None and now - t < SCORE_MAX_WAIT_S:
                self.pending_scans = pending[i:]  # keep scan order; retry on the next tick
                break
            endpoints = scan_endpoints(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                                       sensor_xy, sensor_yaw)
            self.monitor.update_scan(t, pose, endpoints, self.matcher)
            self.scored.append((t, self.monitor.state))
            self._publish(now)
            published = True
        before = self.monitor.state
        self.monitor.tick(now)
        if self.monitor.state != before or (not published and now - self.last_heartbeat >= self.heartbeat_s):
            self._publish(now)
        self.scored = [x for x in self.scored if x[0] >= now - 5.0]

    def close(self):
        if self.monitor is not None:
            self.transitions += self.monitor.transitions
        self.node.get_logger().info('localization_monitor transitions: ' + json.dumps(self.transitions[-50:]))


def main(argv=None):
    from .node_runtime import run
    run('localization_monitor', LocalizationMonitorNode, argv)


__all__ = ['LocalizationMonitorNode', 'confirm_label', 'grid_occupied_xy', 'main']
