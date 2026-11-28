#!/usr/bin/env python3
"""Evaluation recorder for bag replay / sim runs (dev tool, not installed, not part of the runtime stack).

Records, until SIGINT/SIGTERM:
  odom.csv        state/odom  (stamp, x, y, z, yaw, roll, pitch) - the odometry backend output
  slam.csv        TF map->base_link at each odometry stamp (slam_toolbox estimate), when available
  localization.csv state/localization (stamp, evaluated_at, state, score, reason)
  latency.csv     wall-clock delay from cloud arrival to state/odom with the same stamp
  resources.csv   CPU % (of one core) and RSS MB per process group, sampled at 1 Hz
and writes summary.json. Ground truth is never published to the stack by this tool.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import statistics
import struct
import threading
import time

import psutil
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from ultron_b2_interfaces.msg import LocalizationState

GROUPS = {
    'mola': ('mola-cli',),
    'kiss': ('kiss_icp_odometry',),
    'slam_toolbox': ('async_slam_toolbox_node', 'localization_slam_toolbox_node', 'sync_slam_toolbox_node'),
    'monitor': ('localization_monitor',),
    'map_manager': ('map_manager',),
    'scan_conversion': ('cloud_to_scan',),
    'sim_bridge': ('eval_sim_bridge',),
}


def _yaw_rp(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return yaw, roll, pitch


class Sampler(threading.Thread):
    """Per-group CPU (% of one core) and RSS of stack processes, matched by command line."""

    def __init__(self, period=1.0):
        super().__init__(daemon=True)
        self.period, self.rows, self.stop = period, [], threading.Event()
        self.procs = {}

    def _find(self):
        me = os.getpid()
        for p in psutil.process_iter(['pid', 'cmdline']):
            if p.pid == me or p.pid in self.procs:
                continue
            cmd = ' '.join(p.info['cmdline'] or [])
            if cmd.startswith(('ros2 launch', '/usr/bin/python3 /opt/ros/humble/bin/ros2')):
                continue
            for group, keys in GROUPS.items():
                if any(k in cmd for k in keys):
                    try:
                        p.cpu_percent(None)
                        self.procs[p.pid] = (group, p)
                    except psutil.Error:
                        pass
                    break

    def run(self):
        while not self.stop.wait(self.period):
            self._find()
            totals = {}
            for pid, (group, p) in list(self.procs.items()):
                try:
                    cpu, rss = p.cpu_percent(None), p.memory_info().rss / 1e6
                except psutil.Error:
                    self.procs.pop(pid, None)
                    continue
                c, r = totals.get(group, (0.0, 0.0))
                totals[group] = (c + cpu, r + rss)
            t = time.time()
            for group, (c, r) in totals.items():
                self.rows.append((t, group, c, r))


class Recorder(Node):
    def __init__(self, args):
        super().__init__('eval_recorder', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        ns = args.namespace.rstrip('/')
        self.map_frame, self.base_frame = args.map_frame, args.base_frame
        self.cloud_wall, self.latency, self.odom, self.slam, self.loc = {}, [], [], [], []
        self.pending, self.clouds_without_odom = [], 0
        self.tf = Buffer(cache_time=Duration(seconds=120))
        self.tf_listener = TransformListener(self.tf, self)
        rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, f'{ns}/{args.cloud_topic}', self._cloud, rel, raw=True)
        self.create_subscription(Odometry, f'{ns}/state/odom', self._odom, rel)
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scans = 0
        self.create_subscription(LaserScan, f'{ns}/sensors/scan', self._scan,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(LocalizationState, f'{ns}/state/localization', self._loc, latched)
        self.create_timer(0.1, self._resolve)

    def _cloud(self, raw):
        sec, nsec = struct.unpack_from('<iI', raw, 4)            # CDR encapsulation + std_msgs/Header stamp
        self.cloud_wall[sec + nsec * 1e-9] = time.time()

    def _scan(self, _msg):
        self.scans += 1

    def _odom(self, msg):
        now = time.time()
        s = msg.header.stamp
        t = s.sec + s.nanosec * 1e-9
        # MOLA stamps odometry at mid-scan (cloud stamp + ~50 ms); KISS uses the cloud stamp. Match the latest
        # cloud whose stamp is within (t - 0.1 s, t].
        key = max((k for k in self.cloud_wall if t - 0.1 < k <= t + 1e-6), default=None)
        sent = self.cloud_wall.pop(key) if key is not None else None
        for k in [k for k in self.cloud_wall if k < t - 1.0]:
            self.cloud_wall.pop(k)
            self.clouds_without_odom += 1
        if sent is not None:
            self.latency.append((t, now - sent))
        p = msg.pose.pose.position
        yaw, roll, pitch = _yaw_rp(msg.pose.pose.orientation)
        self.odom.append((t, p.x, p.y, p.z, yaw, roll, pitch))
        self.pending.append((now, s))

    def _resolve(self):
        now, keep = time.time(), []
        for wall, stamp in self.pending:
            try:
                tr = self.tf.lookup_transform(self.map_frame, self.base_frame, Time.from_msg(stamp))
            except Exception:  # noqa: BLE001 - map->odom may not exist (yet)
                if now - wall < 2.0:
                    keep.append((wall, stamp))
                continue
            v = tr.transform.translation
            yaw, _, _ = _yaw_rp(tr.transform.rotation)
            self.slam.append((stamp.sec + stamp.nanosec * 1e-9, v.x, v.y, yaw))
        self.pending = keep

    def _loc(self, msg):
        s, e = msg.header.stamp, msg.evaluated_at
        self.loc.append((s.sec + s.nanosec * 1e-9, e.sec + e.nanosec * 1e-9, msg.state,
                         msg.match_score if msg.has_match_score else float('nan'), msg.reason))


def _write(path, header, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def _pct(values, q):
    v = sorted(values)
    return v[min(len(v) - 1, int(round(q / 100 * (len(v) - 1))))]


def summarize(rec, sampler, out, wall_s):
    s = {'wall_s': round(wall_s, 1), 'odom_msgs': len(rec.odom), 'scans': rec.scans, 'clouds_without_odom': rec.clouds_without_odom + len(rec.cloud_wall)}
    if rec.odom:
        a, b = rec.odom[0], rec.odom[-1]
        path = sum(math.hypot(q[1] - p[1], q[2] - p[2]) for p, q in zip(rec.odom, rec.odom[1:]))
        s['odom'] = {'duration_s': round(b[0] - a[0], 2), 'path_length_m': round(path, 3),
                     'start_end_gap_m': round(math.hypot(b[1] - a[1], b[2] - a[2]), 3),
                     'start_end_yaw_deg': round(math.degrees(math.remainder(b[4] - a[4], 2 * math.pi)), 2),
                     'max_abs_z_m': round(max(abs(r[3]) for r in rec.odom), 4),
                     'max_abs_roll_pitch_deg': round(math.degrees(max(max(abs(r[5]), abs(r[6])) for r in rec.odom)), 3)}
    if rec.slam:
        a, b = rec.slam[0], rec.slam[-1]
        s['slam'] = {'poses': len(rec.slam), 'start_end_gap_m': round(math.hypot(b[1] - a[1], b[2] - a[2]), 3),
                     'start_end_yaw_deg': round(math.degrees(math.remainder(b[3] - a[3], 2 * math.pi)), 2)}
    if rec.latency:
        lat = [x[1] * 1000 for x in rec.latency]
        s['latency_ms'] = {'n': len(lat), 'median': round(statistics.median(lat), 1),
                           'p95': round(_pct(lat, 95), 1), 'max': round(max(lat), 1)}
    if rec.loc:
        states = [r[2] for r in rec.loc]
        s['localization'] = {'messages': len(states), **{k: states.count(k) for k in sorted(set(states))},
                             'valid_fraction': round(states.count('VALID') / len(states), 3),
                             'last_state': states[-1], 'last_reason': rec.loc[-1][4]}
    groups = {}
    for _, g, c, r in sampler.rows:
        groups.setdefault(g, ([], []))
        groups[g][0].append(c)
        groups[g][1].append(r)
    s['resources'] = {g: {'cpu_pct_mean': round(statistics.mean(c), 1), 'cpu_pct_p95': round(_pct(c, 95), 1),
                          'rss_mb_max': round(max(r), 1), 'samples': len(c)} for g, (c, r) in groups.items()}
    s['host'] = {'cpu_count': psutil.cpu_count(), 'load_avg_1m_at_end': round(os.getloadavg()[0], 2)}
    (out / 'summary.json').write_text(json.dumps(s, indent=2) + '\n')
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--namespace', default='/ultron/replay')
    ap.add_argument('--cloud-topic', default='sensors/lidar/points_filtered')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='base_link')
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    rec, sampler = Recorder(args), Sampler()
    sampler.start()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    t0 = time.time()
    try:
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(rec, timeout_sec=0.05)
    finally:
        sampler.stop.set()
        _write(out / 'odom.csv', ['t', 'x', 'y', 'z', 'yaw', 'roll', 'pitch'], rec.odom)
        _write(out / 'slam.csv', ['t', 'x', 'y', 'yaw'], rec.slam)
        _write(out / 'localization.csv', ['stamp', 'evaluated_at', 'state', 'score', 'reason'], rec.loc)
        _write(out / 'latency.csv', ['stamp', 'latency_s'], rec.latency)
        _write(out / 'resources.csv', ['wall', 'group', 'cpu_pct', 'rss_mb'], sampler.rows)
        print(json.dumps(summarize(rec, sampler, out, time.time() - t0), indent=2), flush=True)
        rec.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
