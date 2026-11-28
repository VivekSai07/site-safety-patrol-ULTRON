"""mock_sport_server: stands in for the B2 sport service in tests. NEVER a robot.

Subscribes /api/sport/request, records every request (monotonic time, id, api_id, parameter) to a
JSONL file (``log_path``), replies on /api/sport/response with ``reply_code`` (non-zero simulates a
robot-side error) and optionally publishes /sportmodestate. Refuses ROS_DOMAIN_ID 0 / non-localhost.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node


class MockSportServer(Node):
    def __init__(self):
        super().__init__('mock_sport_server')
        self.declare_parameter('log_path', '')
        self.declare_parameter('reply_code', 0)
        self.declare_parameter('publish_state', True)
        from unitree_api.msg import Request, Response
        self.Response = Response
        path = str(self.get_parameter('log_path').value)
        self.log = open(path, 'a', buffering=1) if path else None
        self.count = 0
        self.resp_pub = self.create_publisher(Response, '/api/sport/response', 10)
        self.create_subscription(Request, '/api/sport/request', self._on_request, 10)
        if self.get_parameter('publish_state').value:
            from unitree_go.msg import SportModeState
            self.State = SportModeState
            self.state_pub = self.create_publisher(SportModeState, '/sportmodestate', 10)
            self.create_timer(0.1, lambda: self.state_pub.publish(self.State()))
        self.get_logger().info('mock_sport_server ready (records /api/sport/request)')

    def _on_request(self, msg):
        self.count += 1
        rec = {'t': time.monotonic(), 'id': int(msg.header.identity.id), 'api_id': int(msg.header.identity.api_id),
               'parameter': msg.parameter}
        if self.log:
            self.log.write(json.dumps(rec) + '\n')
        r = self.Response()
        r.header.identity.id = msg.header.identity.id
        r.header.identity.api_id = msg.header.identity.api_id
        r.header.status.code = int(self.get_parameter('reply_code').value)
        r.data = '{}'
        self.resp_pub.publish(r)


def main(args=None):
    if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        print('[mock_sport_server] REFUSED: needs a non-zero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1', file=sys.stderr)
        sys.exit(2)
    rclpy.init(args=args)
    node = MockSportServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.log:
            node.log.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
