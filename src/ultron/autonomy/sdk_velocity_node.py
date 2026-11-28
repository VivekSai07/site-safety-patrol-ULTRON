"""Consume motion-gate output with an injected B2 SportClient-compatible object.

The launcher uses a recording stand-in in the isolated simulation. The node never
initializes an SDK or selects a network interface.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import time

from ..motion import Command

PERIOD_S = 0.05
WATCHDOG_S = 0.25
MAX_LINEAR_M_S = 0.5
MAX_YAW_RAD_S = 0.8


class SdkVelocity:
    """Bound the motion gate's planar commands and stop on loss of its heartbeat."""

    def __init__(self, client, *, clock=time.monotonic):
        self.client = client
        self.clock = clock
        self.last_command = None
        self.stopped = False
        self.fault = None

    def stop(self, *, force=False):
        if self.stopped and not force:
            return
        try:
            if self.client.StopMove() != 0:
                raise RuntimeError('SDK StopMove returned an error')
            self.stopped = True
        except Exception as error:
            self.stopped = False
            self.fault = str(error)

    def receive(self, linear, angular):
        now = self.clock()
        self.tick(now)
        vx, vy, vz = linear
        roll, pitch, yaw = angular
        if (self.fault or not all(map(math.isfinite, (*linear, *angular)))
                or any((vy, vz, roll, pitch)) or not (vx or yaw)):
            self.stop()
            return
        vx = max(-MAX_LINEAR_M_S, min(MAX_LINEAR_M_S, vx))
        yaw = max(-MAX_YAW_RAD_S, min(MAX_YAW_RAD_S, yaw))
        command = Command(vx, 0.0, yaw, PERIOD_S)
        self.stopped = False
        try:
            command.validate()
            if self.client.Move(command.vx, command.vy, command.vyaw) != 0:
                raise RuntimeError('SDK Move returned an error')
            self.last_command = now
        except Exception as error:
            self.fault = str(error)
            self.stop()

    def tick(self, now=None):
        now = self.clock() if now is None else now
        if self.fault or self.last_command is None or now - self.last_command >= WATCHDOG_S:
            self.stop()


class SdkVelocityNode:
    def __init__(self, config, client):
        from geometry_msgs.msg import Twist
        from rclpy.clock import Clock, ClockType
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        self.node = Node('ultron_sdk_velocity')
        self.velocity = SdkVelocity(client)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(Twist, config.topics['cmd_vel'], self._receive, qos)
        self.node.create_timer(PERIOD_S, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _receive(self, msg):
        linear, angular = msg.linear, msg.angular
        self.velocity.receive((linear.x, linear.y, linear.z), (angular.x, angular.y, angular.z))

    def _tick(self):
        self.velocity.tick()
        if self.velocity.fault:
            self.node.get_logger().error(self.velocity.fault, throttle_duration_sec=2.0)

    def close(self):
        self.velocity.stop(force=True)
        if not self.velocity.stopped:
            self.node.get_logger().error(f'Shutdown stop failed: {self.velocity.fault}')
        self.node.destroy_node()


class RecordingSportClient:
    def __init__(self, handle):
        self.handle = handle

    def _record(self, method, args):
        self.handle.write(json.dumps({'time_monotonic': time.monotonic(), 'method': method, 'args': args}) + '\n')
        self.handle.flush()
        return 0

    def Move(self, vx, vy, yaw):
        return self._record('Move', [vx, vy, yaw])

    def StopMove(self):
        return self._record('StopMove', [])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stand-in-log', type=Path, required=True, help='Enable the recording stand-in and write a new log')
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from .config import load_sim_slam_config
    from .sim_bridge import check_isolation

    config = load_sim_slam_config()
    check_isolation(config)
    stopping = False

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    with args.stand_in_log.open('x') as handle:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        for signum in (signal.SIGINT, signal.SIGTERM):
            signal.signal(signum, request_stop)
        wrapper = SdkVelocityNode(config, RecordingSportClient(handle))
        try:
            while rclpy.ok() and not stopping:
                rclpy.spin_once(wrapper.node, timeout_sec=PERIOD_S)
        finally:
            wrapper.close()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
