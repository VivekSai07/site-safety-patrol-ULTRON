#!/usr/bin/env python3
"""Sample replay graph and TF authority diagnostics without recording sensor data."""
import argparse
import json
import re
import signal
import time
from pathlib import Path


def forbidden(name):
    return bool(re.search(r'(^|/)(cmd_vel[^/]*|api|control)(/|$)', name))


def diagnostic_level(value):
    return int.from_bytes(value, 'little') if isinstance(value, bytes) else int(value)


def main():
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('replay_graph_audit')
    result = {'samples': 0, 'forbidden_topics': [], 'forbidden_services': [],
              'tf_diagnostics': None, 'tf_diagnostic_messages': 0, 'seen_topics': []}
    topics, bad_topics, bad_services = set(), set(), set()

    def diagnostic(msg):
        for status in msg.status:
            if status.name == 'ultron/tf_authority_check':
                result['tf_diagnostic_messages'] += 1
                result['tf_diagnostics'] = {
                    'level': diagnostic_level(status.level), 'message': status.message,
                    'frames': {v.key: v.value for v in status.values}}

    node.create_subscription(DiagnosticArray, '/ultron/replay/diagnostics', diagnostic, 100)
    stop = False

    def finish(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)
    next_sample = 0.0
    try:
        while not stop and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_sample:
                names = {name for name, _ in node.get_topic_names_and_types()}
                services = {name for name, _ in node.get_service_names_and_types()}
                topics.update(names)
                bad_topics.update(filter(forbidden, names))
                bad_services.update(filter(forbidden, services))
                result['samples'] += 1
                next_sample = time.monotonic() + 0.5
    finally:
        result['seen_topics'] = sorted(topics)
        result['forbidden_topics'] = sorted(bad_topics)
        result['forbidden_services'] = sorted(bad_services)
        Path(args.out).write_text(json.dumps(result, indent=2) + '\n')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
