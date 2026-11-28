"""Save one simulated head-camera color frame; never commands motion."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image as PillowImage

TOPIC = '/ultron/sim/sensors/cam_head/color/image_raw'
FRAME = 'cam_head_color_optical_frame'


def rgb_array(msg):
    if msg.encoding != 'rgb8' or msg.header.frame_id != FRAME:
        raise ValueError('expected rgb8 from the simulated head color optical frame')
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if width <= 0 or height <= 0 or step < width * 3 or len(msg.data) != step * height:
        raise ValueError('invalid image dimensions or buffer length')
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(height, step)
    return rows[:, :width * 3].reshape(height, width, 3).copy()


def save_snapshot(msg, output):
    rgb = rgb_array(msg)
    output = Path(output)
    output.mkdir(exist_ok=False)
    image_path = output / 'color.png'
    with image_path.open('xb') as stream:
        PillowImage.fromarray(rgb, 'RGB').save(stream, format='PNG')
    metadata = {
        'schema': 'ultron.sim_snapshot/v1',
        'provenance': 'SIM_TOPIC_UNVERIFIED',
        'topic': TOPIC,
        'frame_id': msg.header.frame_id,
        'stamp_sec': msg.header.stamp.sec,
        'stamp_nanosec': msg.header.stamp.nanosec,
        'width': msg.width,
        'height': msg.height,
        'source_encoding': msg.encoding,
        'settled': 'NOT_ASSESSED',
        'sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
    }
    with (output / 'metadata.json').open('x', encoding='utf-8') as stream:
        json.dump(metadata, stream, indent=2)
        stream.write('\n')
    return metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='new capture directory; parent must exist')
    parser.add_argument('--timeout', type=float, default=15.0, help='maximum wait in wall seconds')
    args = parser.parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') != '71' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        print('sim_snapshot requires simulation domain 71 and ROS_LOCALHOST_ONLY=1', file=sys.stderr)
        return 2
    if not 0 < args.timeout <= 120:
        print('timeout must be in (0, 120] seconds', file=sys.stderr)
        return 2
    if Path(args.output).exists():
        print('output directory already exists; refusing to overwrite', file=sys.stderr)
        return 2

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node('sim_snapshot')
    frames = []
    node.create_subscription(Image, TOPIC, lambda msg: frames.append(msg),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    try:
        deadline = time.monotonic() + args.timeout
        while not frames and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if not frames:
            print('no simulated color frame received', file=sys.stderr)
            return 1
        try:
            metadata = save_snapshot(frames[0], args.output)
        except (ValueError, OSError) as exc:
            print(f'capture failed: {exc}', file=sys.stderr)
            return 1
        print(json.dumps({'output': str(args.output), **metadata}, indent=2))
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
