#!/usr/bin/env python3
"""Measure scan arrival versus availability of timestamped odom->scan TF, without sensor data."""
import argparse
import csv
import json
import signal
import statistics
import time
from pathlib import Path


def main():
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('replay_tf_timing')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    rows, pending = [], []
    stats = {'scans': 0, 'available_at_arrival': 0, 'scans_unready_at_next_scan': 0,
             'max_pending': 0, 'unresolved': 0}
    steady = Clock(clock_type=ClockType.STEADY_TIME)

    def resolve():
        now = time.monotonic()
        keep = []
        for row in pending:
            if buffer.can_transform('odom', row['frame'], row['stamp']):
                row['wait_ms'] = (now - row['arrival']) * 1000
            elif now - row['arrival'] < 2.0:
                keep.append(row)
            else:
                row['wait_ms'] = None
        pending[:] = keep

    def scan(msg):
        resolve()
        for row in pending:
            if not row['unready_at_next']:
                row['unready_at_next'] = True
                stats['scans_unready_at_next_scan'] += 1
        stamp = Time.from_msg(msg.header.stamp)
        now = time.monotonic()
        ready = buffer.can_transform('odom', msg.header.frame_id, stamp)
        row = {'stamp_ns': stamp.nanoseconds, 'stamp': stamp, 'frame': msg.header.frame_id,
               'arrival': now, 'wait_ms': 0.0 if ready else None, 'unready_at_next': False}
        rows.append(row)
        stats['scans'] += 1
        stats['available_at_arrival'] += int(ready)
        if not ready:
            pending.append(row)
            stats['max_pending'] = max(stats['max_pending'], len(pending))

    node.create_subscription(LaserScan, '/ultron/replay/sensors/scan', scan,
                             QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_timer(0.01, resolve, clock=steady)
    stop = False

    def finish(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)
    try:
        while not stop and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        resolve()
        waits = sorted(row['wait_ms'] for row in rows if row['wait_ms'] is not None)
        stats['unresolved'] = len(rows) - len(waits)
        if waits:
            stats['wait_ms_median'] = round(statistics.median(waits), 1)
            stats['wait_ms_p95'] = round(waits[round(0.95 * (len(waits) - 1))], 1)
            stats['wait_ms_max'] = round(max(waits), 1)
        out = Path(args.out)
        (out / 'tf_timing.json').write_text(json.dumps(stats, indent=2) + '\n')
        with (out / 'tf_timing.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(('stamp_ns', 'frame', 'wait_ms'))
            writer.writerows((r['stamp_ns'], r['frame'], r['wait_ms']) for r in rows)
        del listener
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
