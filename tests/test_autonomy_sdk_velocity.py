import os
import time
import unittest

from ultron.autonomy.sdk_velocity_node import SdkVelocity, WATCHDOG_S


class SportClient:
    def __init__(self):
        self.calls = []
        self.move_error = 0
        self.stop_error = 0

    def Move(self, x, y, yaw):
        self.calls.append(('Move', x, y, yaw))
        return self.move_error

    def StopMove(self):
        self.calls.append(('StopMove',))
        return self.stop_error


class SdkVelocityTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.client = SportClient()
        self.wrapper = SdkVelocity(self.client, clock=lambda: self.now)

    def move(self):
        self.wrapper.receive((2.0, 0.0, 0.0), (0.0, 0.0, -3.0))

    def test_speed_limits_and_watchdog(self):
        self.move()
        self.assertEqual(self.client.calls, [('StopMove',), ('Move', 0.5, 0.0, -0.8)])
        self.now += WATCHDOG_S - 0.01
        self.wrapper.tick()
        self.assertEqual(self.client.calls[-1][0], 'Move')
        self.now += 0.02
        self.wrapper.tick()
        self.wrapper.tick()
        self.assertEqual(self.client.calls.count(('StopMove',)), 2)
        self.move()
        self.assertEqual(self.client.calls[-1][0], 'Move')

    def test_zero_nonfinite_and_unsupported_axes_stop(self):
        for velocity in ((0.0, 0.0, 0.0), (float('nan'), 0.0, 0.0), (0.1, 0.1, 0.0)):
            with self.subTest(velocity=velocity):
                self.move()
                self.wrapper.receive(velocity, (0.0, 0.0, 0.0))
                self.assertEqual(self.client.calls[-1], ('StopMove',))

    def test_fault_stops_retries_stop_and_blocks_further_motion(self):
        self.move()
        self.client.move_error = self.client.stop_error = -1
        self.move()
        self.assertIsNotNone(self.wrapper.fault)
        self.assertFalse(self.wrapper.stopped)
        self.client.stop_error = 0
        self.wrapper.tick()
        self.assertTrue(self.wrapper.stopped)
        calls = len(self.client.calls)
        self.move()
        self.assertEqual(len(self.client.calls), calls)
        self.wrapper.stop(force=True)
        self.assertEqual(self.client.calls[-1], ('StopMove',))
        self.assertEqual(len(self.client.calls), calls + 1)


@unittest.skipUnless(os.environ.get('ULTRON_ROS_TESTS') == '1', 'enable the isolated ROS tests')
class SdkVelocityRosTest(unittest.TestCase):
    def test_only_gate_output_drives_sdk_and_silence_stops(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        from ultron.autonomy.config import load_sim_slam_config
        from ultron.autonomy.sdk_velocity_node import SdkVelocityNode
        from ultron.autonomy.sim_bridge import check_isolation

        config = load_sim_slam_config()
        check_isolation(config)
        rclpy.init()
        client = SportClient()
        sdk = SdkVelocityNode(config, client)
        origin = Node('ultron_test_velocity_origin')
        executor = SingleThreadedExecutor()
        executor.add_node(sdk.node)
        executor.add_node(origin)

        def wait_for(check, timeout=2.0):
            deadline = time.monotonic() + timeout
            while not check() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
            return check()

        try:
            gated = origin.create_publisher(Twist, config.topics['cmd_vel'], 1)
            raw = origin.create_publisher(Twist, config.topics['nav_cmd_raw'], 1)
            command = Twist()
            command.linear.x = 0.3
            self.assertTrue(wait_for(lambda: gated.get_subscription_count() == 1))
            raw.publish(command)
            self.assertFalse(wait_for(lambda: any(c[0] == 'Move' for c in client.calls), 0.1))
            gated.publish(command)
            self.assertTrue(wait_for(lambda: client.calls[-1] == ('Move', 0.3, 0.0, 0.0)))
            self.assertTrue(wait_for(lambda: client.calls[-1] == ('StopMove',)))
            count = len(client.calls)
        finally:
            executor.shutdown()
            sdk.close()
            origin.destroy_node()
            rclpy.try_shutdown()
        self.assertEqual(len(client.calls), count + 1)
        self.assertEqual(client.calls[-1], ('StopMove',))
