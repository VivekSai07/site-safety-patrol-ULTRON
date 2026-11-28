"""Relay recorded TF during bag replay, dropping transforms produced by a previous SLAM run.

A recorded /tf contains the original slam_toolbox ``map -> odom``. Replaying it unfiltered
would hand the new SLAM instance the old answer, so only non-map transforms are relayed.
"""
import argparse


def main(argv=None):
    parser = argparse.ArgumentParser(description='Relay replayed TF without map transforms')
    parser.add_argument('--input', default='/ultron/sim/replay/tf')
    parser.add_argument('--drop-parent', default='map')
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
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.append(True))
    node = Node('ultron_tf_replay_filter', parameter_overrides=[Parameter('use_sim_time', value=True)])
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    out = node.create_publisher(TFMessage, '/tf', qos)
    counts = {'relayed': 0, 'dropped': 0}

    def relay(msg):
        keep = [t for t in msg.transforms if t.header.frame_id != args.drop_parent]
        counts['dropped'] += len(msg.transforms) - len(keep)
        if keep:
            counts['relayed'] += len(keep)
            out.publish(TFMessage(transforms=keep))

    node.create_subscription(TFMessage, args.input, relay, qos)
    try:
        while not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        print(f'tf_filter: {counts}', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
