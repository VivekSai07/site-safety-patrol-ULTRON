"""Measure scan -> map->odom update latency and sample process CPU/RAM. EVALUATION ONLY.

slam_toolbox stamps ``map -> odom`` with (latest scan stamp + transform_timeout) and updates
it on every received scan (measured in a recorded bag). The first time a new stamp appears,
latency = clock now - scan stamp. This is scan-to-transform-update latency, not proof that a
full scan match ran (slam_toolbox only adds scans after minimum travel).
"""
import argparse
import os
from pathlib import Path
import threading
import time

import yaml

CLK_TCK = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')


class ResourceSampler(threading.Thread):
    """Samples CPU% and RSS of whole process groups (a tool and the children it spawns)."""

    def __init__(self, groups, period=0.5):
        super().__init__(daemon=True)
        self.groups = dict(groups)       # name -> pgid
        self.period = period
        self.stats = {name: {'cpu_samples': [], 'rss_mb_max': 0.0} for name in self.groups}
        self._halt = threading.Event()  # not _stop: that name is a Thread method
        self._lock = threading.Lock()
        self.errors = []
        self._last = {}

    def add(self, name, pgid):
        with self._lock:  # the sampling thread iterates over groups
            self.groups[name] = pgid
            self.stats.setdefault(name, {'cpu_samples': [], 'rss_mb_max': 0.0})

    def _read(self):
        with self._lock:
            groups = dict(self.groups)
        totals = {name: [0, 0] for name in groups}
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            try:
                stat = Path(f'/proc/{entry}/stat').read_text()
            except OSError:
                continue
            fields = stat[stat.rindex(')') + 2:].split()
            pgrp = int(fields[2])
            for name, pgid in groups.items():
                if pgrp == pgid:
                    totals[name][0] += int(fields[11]) + int(fields[12])   # utime + stime ticks
                    totals[name][1] += int(fields[21]) * PAGE                # rss
        return totals

    def run(self):
        while not self._halt.is_set():
            try:
                now, totals = time.monotonic(), self._read()
            except Exception as exc:  # never die silently: record and keep sampling
                self.errors.append(repr(exc))
                self._halt.wait(self.period)
                continue
            for name, (ticks, rss) in totals.items():
                if name in self._last and ticks:
                    t0, k0 = self._last[name]
                    self.stats[name]['cpu_samples'].append(100.0 * (ticks - k0) / CLK_TCK / (now - t0))
                if ticks:
                    self._last[name] = (now, ticks)
                    self.stats[name]['rss_mb_max'] = max(self.stats[name]['rss_mb_max'], rss / 1e6)
            self._halt.wait(self.period)

    def stop(self):
        self._halt.set()
        self.join(timeout=2)
        out = {}
        for name, s in self.stats.items():
            cpu = sorted(s['cpu_samples'])
            out[name] = {'cpu_percent_mean': round(sum(cpu) / len(cpu), 1) if cpu else None,
                         'cpu_percent_p95': round(cpu[int(0.95 * (len(cpu) - 1))], 1) if cpu else None,
                         'rss_mb_max': round(s['rss_mb_max'], 1), 'samples': len(cpu)}
        if self.errors:
            out['_sampler_errors'] = self.errors[:5]
        return out


def main(argv=None):
    parser = argparse.ArgumentParser(description='Log scan -> map->odom update latency (simulation)')
    parser.add_argument('out', type=Path)
    parser.add_argument('--slam-params', type=Path, required=True)
    args = parser.parse_args(argv)
    import signal
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from tf2_msgs.msg import TFMessage
    from .config import load_sim_slam_config
    from .sim_bridge import check_isolation
    check_isolation(load_sim_slam_config())
    timeout = float(yaml.safe_load(args.slam_params.read_text())['slam_toolbox']['ros__parameters']['transform_timeout'])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.append(True))
    node = Node('ultron_latency_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    seen = set()
    handle = args.out.open('w')
    handle.write('scan_stamp,seen_at,latency_s\n')

    def on_tf(msg):
        now = node.get_clock().now().nanoseconds * 1e-9
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                stamp = round(t.header.stamp.sec + t.header.stamp.nanosec * 1e-9 - timeout, 3)
                if stamp not in seen and now > 0:
                    seen.add(stamp)
                    handle.write(f'{stamp:.3f},{now:.3f},{now - stamp:.3f}\n')

    node.create_subscription(TFMessage, '/tf', on_tf, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
    try:
        while not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        handle.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
