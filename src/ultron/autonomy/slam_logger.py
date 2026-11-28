"""Record the SLAM-estimated pose ``map -> base_link`` at each odometry timestamp.

Reads only TF and odometry stamps (never ground truth). Each odometry stamp is looked up
once the map transform for that time is available; stamps that never resolve are counted
as unresolved, not interpolated. Output CSV: t, x, y, yaw in the map frame.
"""
import argparse
import math
from pathlib import Path


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SlamLogger:
    def __init__(self, out_csv, frames, odom_topic, max_wait=1.0):
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from nav_msgs.msg import Odometry
        from tf2_ros import Buffer, TransformListener
        self.node = Node('ultron_slam_logger', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.frames = frames
        self.pending = []
        self.max_wait = max_wait
        self.resolved = self.unresolved = 0
        self.handle = Path(out_csv).open('w')
        self.handle.write('t,x,y,yaw\n')
        self.node.create_subscription(Odometry, odom_topic, self._on_odom,
                                      QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_timer(0.05, self._flush)

    def _on_odom(self, msg):
        self.pending.append(msg.header.stamp)

    def _flush(self, final=False):
        from rclpy.time import Time
        keep = []
        newest = max((s.sec + s.nanosec * 1e-9 for s in self.pending), default=0.0)
        for stamp in self.pending:
            t = stamp.sec + stamp.nanosec * 1e-9
            try:
                tf = self.buffer.lookup_transform(self.frames['map'], self.frames['base'], Time.from_msg(stamp))
            except Exception:
                if final or newest - t > self.max_wait:
                    self.unresolved += 1
                else:
                    keep.append(stamp)
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            self.handle.write(f'{t:.3f},{p.x:.5f},{p.y:.5f},{yaw_of(q):.6f}\n')
            self.resolved += 1
        self.pending = keep

    def close(self):
        self._flush(final=True)
        self.handle.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Log SLAM map->base_link at odometry stamps')
    parser.add_argument('out', type=Path)
    parser.add_argument('--odom-topic', help='Override the odometry topic for offline replay')
    args = parser.parse_args(argv)
    import signal
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from .config import load_sim_slam_config
    from .sim_bridge import check_isolation
    config = load_sim_slam_config()
    check_isolation(config)
    # Own SIGINT/SIGTERM handling: stop between callbacks instead of inside a take().
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.append(True))
    logger = SlamLogger(args.out, config.frames, args.odom_topic or config.topics['odom'])
    try:
        while not stopping:
            rclpy.spin_once(logger.node, timeout_sec=0.1)
    finally:
        logger.close()
        print(f'slam_logger: resolved {logger.resolved}, unresolved {logger.unresolved}', flush=True)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
