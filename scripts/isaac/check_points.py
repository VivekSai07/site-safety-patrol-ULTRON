"""Receive PointCloud2 from the Isaac smoke test (run inside scripts/sim-slam.sh exec). Evaluation helper.

Prints one line per message (wall time, points, ranges) and saves the newest cloud to
runs/isaac/last_cloud.npy as it arrives. Stops after --count messages or --seconds.
"""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

parser = argparse.ArgumentParser()
parser.add_argument('--count', type=int, default=30)
parser.add_argument('--seconds', type=float, default=300.0)
parser.add_argument('--save', default='runs/isaac/last_cloud.npy')
args = parser.parse_args()

rclpy.init()
node = Node('ultron_isaac_listener')
log = []


def on_cloud(msg):
    names = [f.name for f in msg.fields]
    offsets = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = np.stack([raw[:, offsets[k]:offsets[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], axis=1)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    rng = np.hypot(xyz[:, 0], xyz[:, 1]) if len(xyz) else np.zeros(1)
    azimuth = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])) if len(xyz) else np.zeros(1)
    entry = {'t': round(time.time(), 3), 'frame': msg.header.frame_id, 'points': int(len(xyz)), 'fields': names,
             'range_min': round(float(rng.min()), 3), 'range_max': round(float(rng.max()), 3),
             'azimuth_span_deg': round(float(azimuth.max() - azimuth.min()), 1)}
    log.append(entry)
    np.save(args.save, xyz)
    print(json.dumps(entry), flush=True)


node.create_subscription(PointCloud2, '/ultron/sim/isaac/points', on_cloud, qos_profile_sensor_data)
end = time.time() + args.seconds
try:
    while time.time() < end and len(log) < args.count and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
finally:
    span = log[-1]['t'] - log[0]['t'] if len(log) > 1 else 0.0
    print(json.dumps({'messages': len(log), 'rate_hz_wall': round((len(log) - 1) / span, 2) if span else None}), flush=True)
    if rclpy.ok():
        rclpy.shutdown()
