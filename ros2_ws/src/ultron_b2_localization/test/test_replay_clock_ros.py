"""Opt-in ROS-clock transport check; runs only in an isolated replay domain."""
import os
import time
from types import SimpleNamespace

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('ULTRON_REPLAY_CLOCK_TEST') != '1',
                                reason='opt-in isolated ROS clock test')


def test_stopped_and_jumping_clock_never_keeps_valid_state():
    assert os.environ.get('ROS_DOMAIN_ID') == '72'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from ultron_b2_interfaces.msg import LocalizationState
    from ultron_b2_localization.localization_monitor_node import LocalizationMonitorNode
    from ultron_b2_localization.monitor_core import DEFAULT_PARAMS, LocalizationMonitor

    rclpy.init()
    executor = SingleThreadedExecutor()
    node = Node('clock_test_monitor', namespace='/ultron/replay', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True),
        Parameter('replay_clock_watchdog', Parameter.Type.BOOL, True)])
    peer = Node('clock_test_peer')
    impl = LocalizationMonitorNode(node)
    publisher = peer.create_publisher(Clock, '/clock',
                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    states = []
    peer.create_subscription(LocalizationState, '/ultron/replay/state/localization',
                             lambda msg: states.append((msg.state, msg.reason)),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    executor.add_node(node)
    executor.add_node(peer)

    def spin_for(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)

    def advance(stamp_ns):
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(stamp_ns, 10**9)
        publisher.publish(msg)
        spin_for(0.08)

    try:
        spin_for(0.7)
        assert states and states[-1][0] == 'INITIALIZING'
        assert 'advancing replay clock' in states[-1][1]
        advance(10_000_000_000)
        advance(10_100_000_000)
        impl.monitor = LocalizationMonitor(DEFAULT_PARAMS, 'fixture-map', 1)
        matcher = SimpleNamespace(score=lambda *_: 1.0)
        endpoints = np.zeros((1, 2))
        for _ in range(DEFAULT_PARAMS['valid_consecutive']):
            impl.monitor.update_scan(10.1, (0, 0, 0), endpoints, matcher)
        impl._publish(10.1)
        spin_for(0.1)
        assert states[-1][0] == 'VALID'
        spin_for(0.8)
        assert states[-1] == ('STALE', 'Replay clock stopped advancing')
        assert impl.monitor.last_pose_stamp is None
        advance(10_200_000_000)
        for _ in range(DEFAULT_PARAMS['recover_consecutive'] - 1):
            impl.monitor.update_scan(10.2, (0, 0, 0), endpoints, matcher)
        assert impl.monitor.state == 'DEGRADED'
        impl.monitor.update_scan(10.2, (0, 0, 0), endpoints, matcher)
        assert impl.monitor.state == 'VALID'
        impl._publish(10.2)
        spin_for(0.05)
        advance(9_000_000_000)
        assert impl.monitor.state == 'STALE'
        assert 'clock jumped' in impl.monitor.reason
        advance(9_100_000_000)
        advance(9_200_000_000)
        advance(9_300_000_000)
        assert not impl.clock_guard.stalled(time.monotonic()), (
            impl.clock_guard.last_ns, impl.clock_guard.progressing, impl.last_observed_ros, impl._now())
        for _ in range(DEFAULT_PARAMS['recover_consecutive']):
            impl.monitor.update_scan(9.3, (0, 0, 0), endpoints, matcher)
        assert impl.monitor.state == 'VALID'
        advance(11_000_000_000)
        assert impl.monitor.state == 'STALE' and 'clock jumped' in impl.monitor.reason
    finally:
        executor.remove_node(node)
        executor.remove_node(peer)
        node.destroy_node()
        peer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
