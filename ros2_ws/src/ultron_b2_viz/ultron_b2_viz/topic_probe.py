"""topic_probe: measure per-topic rate and bandwidth (serialized bytes) on the current domain.

    ros2 run ultron_b2_viz topic_probe --pattern '/ultron/viz/.*' --duration 20 --out runs/viz/bw.json

Like ``ros2 topic bw`` for many topics at once; subscribes raw (no deserialization). Types are
discovered from the graph. Output JSON: per-topic hz, bytes/s, mean message size, totals (Mbit/s).
"""
import argparse
import json
import os
from pathlib import Path
import re
import time


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--pattern', default='.*', help='regex (full match) of topics to measure')
    p.add_argument('--exclude', default=r'/rosout|/parameter_events', help='regex of topics to skip')
    p.add_argument('--duration', type=float, default=10.0)
    p.add_argument('--discovery', type=float, default=3.0, help='seconds to wait for graph discovery')
    p.add_argument('--best-effort', action='store_true', help='subscribe best effort (default reliable)')
    p.add_argument('--out', type=Path)
    p.add_argument('--label', default='')
    return p


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    args = build_parser().parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise SystemExit('topic_probe: refusing ROS_DOMAIN_ID 0 (set the sim/replay/viz domain)')
    rclpy.init()
    node = Node('topic_probe')
    end = time.monotonic() + args.discovery
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    inc, exc = re.compile(args.pattern), re.compile(args.exclude) if args.exclude else None
    stats = {}
    for name, types in sorted(node.get_topic_names_and_types()):
        if not inc.fullmatch(name) or (exc and exc.fullmatch(name)):
            continue
        durability = DurabilityPolicy.VOLATILE
        pubs = node.get_publishers_info_by_topic(name)
        if pubs and all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in pubs):
            durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos = QoSProfile(depth=10, durability=durability,
                         reliability=ReliabilityPolicy.BEST_EFFORT if args.best_effort else ReliabilityPolicy.RELIABLE)
        st = stats[name] = {'type': types[0], 'msgs': 0, 'bytes': 0, 'first': None, 'last': None}

        def cb(data, st=st):
            now = time.monotonic()
            st['msgs'] += 1
            st['bytes'] += len(data)
            st['first'] = st['first'] or now
            st['last'] = now
        node.create_subscription(get_message(types[0]), name, cb, qos, raw=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    elapsed = time.monotonic() - t0
    report = {'schema': 'ultron.topic_probe/v1', 'label': args.label, 'domain': os.environ.get('ROS_DOMAIN_ID'),
              'duration_s': round(elapsed, 2), 'topics': {}}
    total = 0
    for name, st in stats.items():
        hz = (st['msgs'] - 1) / (st['last'] - st['first']) if st['msgs'] > 1 and st['last'] > st['first'] else 0.0
        bps = st['bytes'] / elapsed
        total += bps
        report['topics'][name] = {'type': st['type'], 'msgs': st['msgs'], 'hz': round(hz, 2),
                                  'bytes_per_s': round(bps), 'mean_msg_bytes': round(st['bytes'] / st['msgs']) if st['msgs'] else 0}
    report['total_bytes_per_s'] = round(total)
    report['total_mbit_s'] = round(total * 8 / 1e6, 3)
    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + '\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
