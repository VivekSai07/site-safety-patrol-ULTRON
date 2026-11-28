"""Publish ULTRON's LocalizationState for an isolated simulation or recording replay.

Inputs: 2D scan, TF (map->odom from slam_toolbox, odom->base_link), the saved map image and
its map_meta.json (map ID/version). Ground truth is never read. Output: JSON
LocalizationState on the configured topic, plus logs for offline evaluation.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from .localization_state import LocalizationMonitor, MapMatcher, TransformHistory
from .live_grid import TOPIC, LiveMapMatcher, grid_is_fresh, grid_stamp_ns

SCORE_DELAY_S = 0.15  # let the SLAM backend apply a scan before scoring the pose at its stamp


def confirm_label(t, scored, now, max_wait):
    """State for the pose at time t: the state after the first scored scan with stamp >= t.

    A pose is never vouched for ahead of the sensor (a kidnap between scans would otherwise
    be labelled VALID). Returns None while waiting, 'UNCONFIRMED' if no scan confirms it in time.
    """
    for stamp, state in scored:
        if stamp >= t - 1e-6:
            return state
    return 'UNCONFIRMED' if now - t > max_wait else None


def scan_endpoints_base(msg, mount_xyz, mount_yaw):
    ranges = np.asarray(msg.ranges, dtype=float)
    angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
    ok = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
    a = angles[ok] + mount_yaw
    return np.column_stack((mount_xyz[0] + ranges[ok] * np.cos(a), mount_xyz[1] + ranges[ok] * np.sin(a)))


class MonitorNode:
    def __init__(self, config, map_dir, out_dir):
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener
        meta = json.loads((Path(map_dir) / 'map_meta.json').read_text())
        self.clock_id = meta.get('clock_id', 'ros_sim_time')
        self.config = config
        self.monitor = LocalizationMonitor(config.localization, meta['map_id'], meta['map_version'])
        self.live_map = meta.get('live_map', False)
        self.grid_stamp_ns = 0
        self.matcher = None if self.live_map else MapMatcher.from_map_yaml(
            Path(map_dir) / 'map.yaml', config.localization['match_tolerance_m'])
        self.history = TransformHistory()
        self.node = Node('ultron_localization_monitor', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.String = String
        self.pub = self.node.create_publisher(String, config.topics['localization_state'], latched)
        self.node.create_subscription(LaserScan, config.topics['scan'], lambda m: self.pending_scans.append(m), qos)
        self.node.create_subscription(Odometry, config.topics['odom'], lambda m: self.pending_odom.append(m.header.stamp), qos)
        if self.live_map:
            self.node.create_subscription(OccupancyGrid, TOPIC, self._on_grid, latched)
        self.pending_scans, self.pending_odom = [], []
        self.scored = []          # (scan stamp, state after scoring), increasing stamps
        self.mount = config.lidar.mount_xyz_m
        self.mount_yaw = config.lidar.mount_rpy_rad[2]
        out_dir = Path(out_dir)
        self.states_log = (out_dir / 'states.jsonl').open('w')
        self.estimate_log = (out_dir / 'estimate.csv').open('w')
        self.estimate_log.write('t,x,y,yaw,state\n')
        self.node.create_timer(0.05, self._tick)

    def _now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_grid(self, msg):
        stamp = grid_stamp_ns(msg)
        if stamp <= self.grid_stamp_ns:
            return
        try:
            self.matcher = LiveMapMatcher(msg, self.config.localization['match_tolerance_m'])
        except ValueError:
            self.matcher = None
        self.grid_stamp_ns = stamp

    def _lookup(self, stamp):
        from rclpy.time import Time
        f = self.config.frames
        try:
            tf = self.buffer.lookup_transform(f['map'], f['base'], Time.from_msg(stamp))
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def _publish(self, now):
        state = self.monitor.snapshot(now)
        state.clock_id = self.clock_id
        self.pub.publish(self.String(data=json.dumps(state.to_dict())))
        return state

    def _tick(self):
        now = self._now()
        if now <= 0:
            return
        ready, waiting = [], []
        for m in self.pending_scans:
            age = now - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            (ready if age >= SCORE_DELAY_S else waiting).append(m)
        self.pending_scans = waiting
        for msg in ready:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            grid_ready = self.matcher is not None and (not self.live_map or grid_is_fresh(self.grid_stamp_ns, now))
            pose = self._lookup(msg.header.stamp) if grid_ready else None
            self.monitor.update_scan(t, pose, scan_endpoints_base(msg, self.mount, self.mount_yaw), self.matcher)
            self.scored.append((t, self.monitor.state))
            state = self._publish(now)
            self.states_log.write(json.dumps({'stamp': round(t, 3), 'evaluated_at': round(now, 3), 'state': state.state,
                                              'reason': state.reason, 'score': state.match_score,
                                              'pose': list(pose) if pose else None}) + '\n')
        before = self.monitor.state
        self.monitor.tick(now)
        if self.monitor.state != before:
            state = self._publish(now)
            self.states_log.write(json.dumps({'stamp': None, 'evaluated_at': round(now, 3), 'state': state.state,
                                              'reason': state.reason, 'score': None, 'pose': None}) + '\n')
        keep = []
        max_wait = self.config.localization['scan_age_limit_s'] + SCORE_DELAY_S
        for stamp in self.pending_odom:
            t = stamp.sec + stamp.nanosec * 1e-9
            label = confirm_label(t, self.scored, now, max_wait) if now - t >= SCORE_DELAY_S else None
            if label is None:
                keep.append(stamp)
                continue
            pose = self._lookup(stamp)
            if pose is not None:
                self.estimate_log.write(f'{t:.3f},{pose[0]:.5f},{pose[1]:.5f},{pose[2]:.6f},{label}\n')
                try:
                    self.history.add(t, pose, label)
                except ValueError:
                    pass
        self.pending_odom = keep
        oldest = min((s.sec + s.nanosec * 1e-9 for s in self.pending_odom), default=now)
        self.scored = [x for x in self.scored if x[0] >= oldest - 1.0]

    def close(self):
        for t in self.monitor.transitions:
            self.states_log.write(json.dumps({'transition': t}) + '\n')
        self.states_log.close()
        self.estimate_log.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Publish LocalizationState for a loaded map')
    parser.add_argument('--map', type=Path, required=True, help='Mapping run folder (map.yaml, map_meta.json)')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--recorded-b2', action='store_true', help='Score the isolated B2 recording replay')
    args = parser.parse_args(argv)
    import signal
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from .config import load_sim_slam_config
    from .sim_bridge import check_isolation
    config = load_sim_slam_config()
    check_isolation(config)
    if args.recorded_b2:
        from dataclasses import replace
        from .real_bag import ODOM, SCAN

        # The derived scan origin is vertically above the artificial floor-level base.
        lidar = replace(config.lidar, mount_xyz_m=(0.0, 0.0, 0.0), mount_rpy_rad=(0.0, 0.0, 0.0))
        topics = dict(config.topics, scan=SCAN, odom=ODOM, localization_state='/ultron/real/localization_state')
        config = replace(config, lidar=lidar, topics=topics)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.append(True))
    node = MonitorNode(config, args.map, args.out)
    try:
        while not stopping:
            rclpy.spin_once(node.node, timeout_sec=0.05)
    finally:
        node.close()
        print('localization_monitor transitions: ' + json.dumps(node.monitor.transitions), flush=True)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
