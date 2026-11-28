"""Phase 2: live ROS 2 bridge checks (opt-in; needs ROS 2 Humble and .venv-ros).

Run: ULTRON_ROS_TESTS=1 scripts/sim-slam.sh exec .venv-ros/bin/python -m unittest tests.test_autonomy_ros_bridge -v
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
ENABLED = os.environ.get('ULTRON_ROS_TESTS') == '1'
ALLOWED_INFRA = {'/clock', '/tf', '/tf_static', '/rosout', '/parameter_events'}


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


@unittest.skipUnless(ENABLED, 'set ULTRON_ROS_TESTS=1 and run inside scripts/sim-slam.sh exec')
class BridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        cls.rclpy = rclpy
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise unittest.SkipTest('not inside the isolated simulation environment')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        cls.rclpy.shutdown()

    def start_bridge(self, *args):
        self.tmp = tempfile.TemporaryDirectory()
        out = Path(self.tmp.name) / 'run'
        proc = subprocess.Popen([sys.executable, '-m', 'ultron.autonomy.sim_bridge', '--out', str(out), *args],
                                cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.addCleanup(self.tmp.cleanup)
        self.addCleanup(proc.stdout.close)
        self.addCleanup(lambda: proc.poll() is None and (proc.kill(), proc.wait()))
        return proc, out

    def make_node(self, name):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        node = Node(name)
        self.addCleanup(node.destroy_node)
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        static = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        return node, qos, static

    def test_streams_frames_and_isolation(self):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan, PointCloud2
        from tf2_msgs.msg import TFMessage
        proc, out = self.start_bridge('--route', str(ROOT / 'config/autonomy/routes/hall_survey_loop.yaml'),
                                      '--rate', '2', '--duration', '16')
        node, qos, static = self.make_node('ultron_test_streams')
        got = {k: [] for k in ('clock', 'scan', 'points', 'odom', 'truth', 'tf', 'tf_static')}
        node.create_subscription(Clock, '/clock', lambda m: got['clock'].append(m.clock.sec + m.clock.nanosec * 1e-9), qos)
        node.create_subscription(LaserScan, '/ultron/sim/scan', lambda m: got['scan'].append(m), qos)
        node.create_subscription(PointCloud2, '/ultron/sim/lidar/points', lambda m: got['points'].append(m), qos)
        node.create_subscription(Odometry, '/ultron/sim/odom', lambda m: got['odom'].append(m), qos)
        node.create_subscription(PoseStamped, '/ultron/sim/ground_truth/pose', lambda m: got['truth'].append(m), qos)
        node.create_subscription(TFMessage, '/tf', lambda m: got['tf'].extend(m.transforms), qos)
        node.create_subscription(TFMessage, '/tf_static', lambda m: got['tf_static'].extend(m.transforms), static)
        topics = set()
        deadline = time.monotonic() + 60
        while proc.poll() is None and time.monotonic() < deadline:
            self.rclpy.spin_once(node, timeout_sec=0.05)
            if len(got['scan']) == 20:
                topics = {name for name, _ in node.get_topic_names_and_types()}
        for _ in range(40):
            self.rclpy.spin_once(node, timeout_sec=0.05)
        self.assertEqual(proc.wait(timeout=10), 0, proc.stdout.read())

        # Only simulation-namespaced topics plus ROS infrastructure exist.
        self.assertTrue(topics, 'topic list was not sampled')
        foreign = {t for t in topics if t not in ALLOWED_INFRA and not t.startswith('/ultron/sim/')}
        self.assertFalse(foreign, f'unexpected topics: {foreign}')
        self.assertNotIn('/cmd_vel', topics)

        # Rates in simulated time (configuration: 50 Hz odom/clock, 10 Hz lidar), within +-10 %.
        def rate(stamps):
            return (len(stamps) - 1) / (stamps[-1] - stamps[0])
        scan_stamps = [stamp(m) for m in got['scan']]
        point_stamps = [stamp(m) for m in got['points']]
        odom_stamps = [stamp(m) for m in got['odom']]
        self.assertAlmostEqual(rate(got['clock']), 50, delta=5)
        self.assertAlmostEqual(rate(odom_stamps), 50, delta=5)
        self.assertAlmostEqual(rate(scan_stamps), 10, delta=1)
        self.assertAlmostEqual(rate(point_stamps), 10, delta=1)
        for series in (got['clock'], scan_stamps, point_stamps, odom_stamps):
            self.assertTrue(all(b > a for a, b in zip(series, series[1:])), 'stamps must strictly increase')
            self.assertGreater(series[0], 1.0, 'stamps must not start at zero')
        common = set(scan_stamps) & set(point_stamps)
        self.assertGreaterEqual(len(common), min(len(scan_stamps), len(point_stamps)) - 1,
                                'scan and cloud of one sweep share a stamp')

        # TF tree: exactly odom -> base_link dynamic and base_link -> lidar_link static.
        dynamic = {(t.header.frame_id, t.child_frame_id) for t in got['tf']}
        fixed = {(t.header.frame_id, t.child_frame_id) for t in got['tf_static']}
        self.assertEqual(dynamic, {('odom', 'base_link')})
        self.assertEqual(fixed, {('base_link', 'lidar_link')})
        frames = {f for pair in dynamic | fixed for f in pair}
        self.assertNotIn('ultron_sim_world', frames, 'ground truth must never be on TF')
        self.assertNotIn('map', frames, 'no map frame before SLAM runs')
        self.assertTrue(all(m.header.frame_id == 'ultron_sim_world' for m in got['truth']))

        # Message content.
        scan = got['scan'][-1]
        self.assertEqual(scan.header.frame_id, 'lidar_link')
        finite = sum(r == r and r != float('inf') for r in scan.ranges) / len(scan.ranges)
        self.assertGreater(finite, 0.9, 'hall walls should return in nearly every open-sector bin')
        cloud = got['points'][-1]
        self.assertEqual(cloud.point_step, 24)
        self.assertEqual([f.name for f in cloud.fields], ['x', 'y', 'z', 'intensity', 'ring', 'time'])
        self.assertGreater(cloud.width, 20000)
        meta = json.loads((out / 'run_meta.json').read_text())
        self.assertIn('EVALUATION_ONLY', meta['ground_truth'])
        summary = json.loads((out / 'bridge_summary.json').read_text())
        self.assertEqual(summary['collisions'], 0)
        # Odometry drifts away from truth (otherwise SLAM would have nothing to correct).
        truth_csv = (out / 'ground_truth.csv').read_text().splitlines()[1:]
        self.assertGreater(len(truth_csv), 700)

    def test_cmd_vel_timeout_and_collision_stop(self):
        from geometry_msgs.msg import PoseStamped, Twist
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        proc, out = self.start_bridge('--rate', '1', '--duration', '30')
        node, qos, static = self.make_node('ultron_test_cmd')
        pub = node.create_publisher(Twist, '/ultron/sim/cmd_vel', qos)
        now = {'t': None}
        truth, status = [], []
        node.create_subscription(Clock, '/clock', lambda m: now.update(t=m.clock.sec + m.clock.nanosec * 1e-9), qos)
        node.create_subscription(PoseStamped, '/ultron/sim/ground_truth/pose',
                                 lambda m: truth.append((stamp(m), m.pose.position.x, m.pose.position.y)), qos)
        node.create_subscription(String, '/ultron/sim/bridge_status', lambda m: status.append(json.loads(m.data)), static)

        def drain():
            # spin_once handles one callback per call; drain the backlog so now['t'] is current.
            for _ in range(500):
                before = (now['t'], len(truth), len(status))
                self.rclpy.spin_once(node, timeout_sec=0.0)
                if (now['t'], len(truth), len(status)) == before:
                    break

        def drive(vx, until):
            while proc.poll() is None and (now['t'] is None or now['t'] < until):
                msg = Twist()
                msg.linear.x = vx
                pub.publish(msg)
                self.rclpy.spin_once(node, timeout_sec=0.02)
                drain()
            return now['t']

        while now['t'] is None:
            self.rclpy.spin_once(node, timeout_sec=0.05)
        start = now['t']
        last_cmd = drive(0.3, start + 2.0)          # 2 s forward, then silence
        while now['t'] < last_cmd + 2.0:
            self.rclpy.spin_once(node, timeout_sec=0.02)
            drain()
        moving = [t for (t, x, y), (t0, x0, y0) in zip(truth[1:], truth) if abs(y - y0) > 1e-6]
        self.assertTrue(moving, 'robot should have moved')
        stopped_after = moving[-1] - last_cmd
        print(f'\n  last motion {stopped_after:.3f} s (sim) after last command; timeout 0.5 s')
        self.assertLessEqual(stopped_after, 0.5 + 0.1, f'base kept moving {stopped_after:.2f} s after last command')
        y_after_timeout = truth[-1][2]
        # Drive straight towards the cart (x 0.75-1.65, y >= 5.85): the footprint check must stop it.
        drive(0.6, now['t'] + 12.0)
        until = now['t'] + 1.5
        while now['t'] < until and proc.poll() is None:
            self.rclpy.spin_once(node, timeout_sec=0.02)
            drain()
        final_y = truth[-1][2]
        self.assertGreater(final_y, y_after_timeout + 1.0)
        self.assertLess(final_y, 5.85 - 0.55 + 0.02, 'footprint must not enter the cart')
        self.assertTrue(any(s['collisions'] >= 1 for s in status), status[-1:] )
        self.assertTrue(any(s['command_timeouts'] >= 1 for s in status), status[-1:])
        proc.terminate()
        proc.wait(timeout=10)


if __name__ == '__main__':
    unittest.main()
