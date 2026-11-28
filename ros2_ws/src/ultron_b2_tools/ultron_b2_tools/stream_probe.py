"""stream_probe: measure topic rates, sizes, point counts, LaserScan ranges and process CPU over a window.

    ros2 run ultron_b2_tools stream_probe --duration 30 --namespace /ultron/replay \
        --topics sensors/lidar/points sensors/lidar/points_filtered sensors/scan sensors/imu/data \
        --proc cloud_filter pointcloud_to_laserscan --json out.json

Subscribes RELIABLE when every publisher offers reliable, else best-effort. A best-effort reader
loses most ~1 MB fragmented samples (clouds, images) on a host with a small net.core.rmem_max,
because lost fragments are never retransmitted; a reliable reader recovers them (measured on the
real-bag replay, VALIDATION.md Agent B). Durability is volatile (compatible with any offer). Rates are reported from
receive wall time and from header stamps. CPU is read from /proc (utime+stime delta over the window,
percent of one core) for processes whose command line contains each --proc pattern.
"""
import argparse
import json
import os
import sys
import time
from collections import defaultdict

import struct

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

CLK_TCK = os.sysconf('SC_CLK_TCK') if hasattr(os, 'sysconf') else 100


def _stats(values):
    a = np.asarray(values, dtype=float)
    if a.size == 0:
        return None
    return {'n': int(a.size), 'min': float(a.min()), 'median': float(np.median(a)), 'mean': float(a.mean()),
            'max': float(a.max())}


def _rate(times):
    if len(times) < 2:
        return None
    span = times[-1] - times[0]
    return (len(times) - 1) / span if span > 0 else None


# Deserialized in Python; everything else (images, metadata) is taken raw to keep the probe cheap.
DECODED_TYPES = ('sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/Imu')


def _raw_header(raw):
    """(stamp_s, frame_id) from serialized CDR whose message starts with std_msgs/Header, else None."""
    try:
        little = raw[1] == 1
        fmt = '<' if little else '>'
        sec, nsec, n = struct.unpack_from(fmt + 'iII', raw, 4)
        frame = raw[16:16 + n - 1].decode(errors='replace') if 0 < n < 512 else ''
        return sec + nsec * 1e-9, frame
    except (struct.error, IndexError):
        return None


def _proc_ticks(patterns):
    """{pattern: [(pid, ticks)]} for processes whose cmdline contains the pattern."""
    found = defaultdict(list)
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
            with open(f'/proc/{pid}/stat') as f:
                fields = f.read().rsplit(')', 1)[1].split()
            with open(f'/proc/{pid}/status') as f:
                rss_kb = next((int(line.split()[1]) for line in f if line.startswith('VmRSS:')), 0)
        except OSError:
            continue
        for p in patterns:
            if p in cmd and 'stream_probe' not in cmd:
                found[p].append((int(pid), int(fields[11]) + int(fields[12]), rss_kb))
    return found


class Probe:
    def __init__(self, node, topics):
        self.node = node
        self.data = defaultdict(lambda: {'recv': [], 'stamp': [], 'bytes': [], 'points': [], 'finite_ranges': [],
                                         'range_median': [], 'range_max': [], 'frame_id': set(), 'type': None})
        self.subs = []
        self.pending = list(topics)

    def try_subscribe(self):
        known = dict(self.node.get_topic_names_and_types())
        for topic in list(self.pending):
            if topic in known:
                msg_type = known[topic][0]
                raw = msg_type not in DECODED_TYPES
                pubs = self.node.get_publishers_info_by_topic(topic)
                if not pubs:
                    continue
                reliable = all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in pubs)
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE) if reliable \
                    else qos_profile_sensor_data
                self.data[topic]['type'] = msg_type
                self.data[topic]['reliability'] = 'reliable' if reliable else 'best_effort'
                self.subs.append(self.node.create_subscription(
                    get_message(msg_type), topic, lambda m, t=topic, r=raw: self.on_msg(t, m, r),
                    qos, raw=raw))
                self.pending.remove(topic)

    def on_msg(self, topic, msg, raw=False):
        d = self.data[topic]
        d['recv'].append(time.monotonic())
        if raw:
            d['bytes'].append(len(msg))
            header = _raw_header(msg) if d['type'] not in ('realsense2_camera_msgs/msg/Extrinsics',) else None
            if header is not None:
                d['stamp'].append(header[0])
                d['frame_id'].add(header[1])
            return
        header = getattr(msg, 'header', None)
        if header is not None:
            d['stamp'].append(header.stamp.sec + header.stamp.nanosec * 1e-9)
            d['frame_id'].add(header.frame_id)
        if hasattr(msg, 'point_step'):
            d['points'].append(msg.width * msg.height)
            d['bytes'].append(len(msg.data))
        elif hasattr(msg, 'ranges'):
            r = np.asarray(msg.ranges, dtype=float)
            fin = r[np.isfinite(r)]
            d['finite_ranges'].append(int(fin.size))
            if fin.size:
                d['range_median'].append(float(np.median(fin)))
                d['range_max'].append(float(fin.max()))
            d.setdefault('scan_meta', {'n_beams': int(r.size), 'range_min': msg.range_min,
                                       'range_max': msg.range_max, 'angle_min': msg.angle_min,
                                       'angle_max': msg.angle_max})

    def report(self):
        out = {}
        for topic, d in self.data.items():
            entry = {'type': d['type'], 'reliability': d.get('reliability'), 'count': len(d['recv']), 'rate_recv_hz': _rate(d['recv']),
                     'rate_stamp_hz': _rate(sorted(d['stamp'])), 'frame_id': sorted(d['frame_id'])}
            if d['points']:
                entry['points'] = _stats(d['points'])
            if d['bytes']:
                entry['bytes'] = _stats(d['bytes'])
            if d['finite_ranges']:
                entry['finite_beams'] = _stats(d['finite_ranges'])
                entry['range_median_m'] = _stats(d['range_median'])
                entry['range_max_m'] = _stats(d['range_max'])
                entry['scan_meta'] = d.get('scan_meta')
            out[topic] = entry
        for topic in self.pending:
            out[topic] = {'type': None, 'count': 0, 'note': 'topic never appeared'}
        return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topics', nargs='+', required=True, help='topic names (relative ones get --namespace)')
    ap.add_argument('--namespace', default='')
    ap.add_argument('--duration', type=float, default=20.0, help='measurement window (wall seconds)')
    ap.add_argument('--wait', type=float, default=30.0, help='max wall seconds to wait for the first message')
    ap.add_argument('--proc', nargs='*', default=[], help='cmdline substrings of processes to measure CPU for')
    ap.add_argument('--json', default='', help='write the report here as JSON')
    args = ap.parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        print('stream_probe: refusing ROS_DOMAIN_ID 0/unset', file=sys.stderr)
        return 2
    ns = '/' + args.namespace.strip('/') if args.namespace.strip('/') else ''
    topics = [t if t.startswith('/') else f'{ns}/{t}' for t in args.topics]

    rclpy.init()
    node = rclpy.create_node('stream_probe')
    probe = Probe(node, topics)
    t_wait = time.monotonic() + args.wait
    while time.monotonic() < t_wait and not any(d['recv'] for d in probe.data.values()):
        probe.try_subscribe()
        rclpy.spin_once(node, timeout_sec=0.1)
    for d in probe.data.values():
        for key in ('recv', 'stamp', 'bytes', 'points', 'finite_ranges', 'range_median', 'range_max'):
            d[key].clear()
    ticks0, w0 = _proc_ticks(args.proc), time.monotonic()
    t_end = w0 + args.duration
    while time.monotonic() < t_end:
        probe.try_subscribe()
        rclpy.spin_once(node, timeout_sec=0.05)
    ticks1, w1 = _proc_ticks(args.proc), time.monotonic()
    report = {'window_s': w1 - w0, 'topics': probe.report(), 'cpu': {}}
    for p in args.proc:
        before = {pid: t for pid, t, _ in ticks0.get(p, [])}
        procs = [(pid, t, rss) for pid, t, rss in ticks1.get(p, []) if pid in before]
        cpu = sum(t - before[pid] for pid, t, _ in procs) / CLK_TCK / (w1 - w0) * 100.0
        report['cpu'][p] = {'pids': [pid for pid, _, _ in procs], 'cpu_percent_one_core': round(cpu, 1),
                            'rss_mb': round(sum(rss for _, _, rss in procs) / 1024.0, 1)}
    node.destroy_node()
    rclpy.try_shutdown()
    text = json.dumps(report, indent=2, default=list)
    print(text)
    if args.json:
        with open(args.json, 'w') as f:
            f.write(text + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
