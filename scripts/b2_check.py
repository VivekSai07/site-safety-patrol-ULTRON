#!/usr/bin/env python3
"""Preflight: measure which sensor topics are live and exit non-zero if a required one is not.

    python3 scripts/b2_check.py                 # camera, lidar and IMU required
    python3 scripts/b2_check.py --camera-only   # only the camera required

Read-only: it subscribes for a few seconds and publishes nothing. Needs ROS sourced.
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ultron.preflight import CAMERA_ONLY, REQUIRED, all_topics, evaluate, table


def measure(seconds, discovery=2.0):
    rclpy.init()
    node = Node("ultron_preflight")
    try:
        deadline = time.monotonic() + discovery
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        types = dict(node.get_topic_names_and_types())
        counts, missing_types, subscriptions = {}, set(), []
        try:
            from rosidl_runtime_py.utilities import get_message
        except ImportError:
            get_message = None
        for topic in all_topics():
            if topic not in types:
                continue
            try:
                message = get_message(types[topic][0])
            except Exception:
                missing_types.add(topic)
                continue
            counts[topic] = 0

            def count(_, topic=topic):
                counts[topic] += 1
            subscriptions.append(node.create_subscription(message, topic, count, qos_profile_sensor_data, raw=True))
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.monotonic() - start
        return {t: n / elapsed for t, n in counts.items()}, missing_types, types
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--camera-only", action="store_true", help="require only the camera")
    args = parser.parse_args()
    rates, missing_types, types = measure(args.seconds)
    ok, rows = evaluate(rates, CAMERA_ONLY if args.camera_only else REQUIRED, missing_types)
    print(f"{len(types)} topics visible on this ROS graph; rates over {args.seconds:.0f} s:")
    print(table(rows))
    if len(types) <= 3:
        print("\nAlmost nothing is visible: drivers not running, or another ROS_DOMAIN_ID / RMW / "
              "ROS_LOCALHOST_ONLY than the drivers use. Run: scripts/b2.sh doctor")
    print("\nREADY" if ok else "\nNOT READY: a required sensor is missing (see 'needs:' above)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
