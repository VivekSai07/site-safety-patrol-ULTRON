"""ROS 2 bridge for the ULTRON lidar hall simulation. SIMULATION ONLY.

Publishes simulated time, a Helios-32-profile PointCloud2, a 2D LaserScan of the unblocked
sector, drifting odometry with TF ``odom -> base_link``, static ``base_link -> lidar_link``
and an evaluation-only ground-truth pose (never on TF). Accepts velocity only on the
simulation command topic. No hardware topic, transport or SDK is used.

Run through ``scripts/sim-slam.sh bridge`` (sets the isolated ROS domain).
"""
import os

# Must precede numpy import: OpenBLAS busy-wait threads slow MuJoCo ray casting.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import argparse  # noqa: E402
import array  # noqa: E402
from collections import deque  # noqa: E402
import dataclasses  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
from pathlib import Path  # noqa: E402
import time  # noqa: E402

import numpy as np  # noqa: E402

from .config import CONFIG_DIR, load_lidar_profile, load_route, load_scenario, load_sim_slam_config  # noqa: E402
from .driver import KinematicDriver, route_segments  # noqa: E402
from .hall_scene import HallSim  # noqa: E402
from .lidar import SimLidar  # noqa: E402
from .odometry import OdometrySimulator  # noqa: E402
from .scan2d import geometry_from, project  # noqa: E402

DEFAULT_START = (1.6, 1.5, math.pi / 2)
CLOCK_START_S = 10.0          # avoid t=0, which ROS tools treat as "unset"
GROUND_TRUTH_FRAME = 'ultron_sim_world'
CLOUD_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                        'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                        'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24})


def check_isolation(config):
    """Refuse to start outside the configured isolated ROS domain."""
    domain = os.environ.get('ROS_DOMAIN_ID')
    local = os.environ.get('ROS_LOCALHOST_ONLY')
    if domain != str(config.ros['domain_id']) or local != '1':
        raise SystemExit(f'Refusing to start: need ROS_DOMAIN_ID={config.ros["domain_id"]} and '
                         f'ROS_LOCALHOST_ONLY=1 (got {domain!r}, {local!r}). Use scripts/sim-slam.sh.')


def _quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class SimBridge:
    def __init__(self, config, route=None, rate=1.0, out=None, scenario=None, people='static',
                 distortion=False, odom_scale=1.0, lidar_profile_path=None, publish_static_tf=True,
                 publish_odom_tf=True):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan, PointCloud2, PointField
        from std_msgs.msg import String
        from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

        self.msg = dict(PoseStamped=PoseStamped, TransformStamped=TransformStamped, Odometry=Odometry,
                        Clock=Clock, LaserScan=LaserScan, PointCloud2=PointCloud2, PointField=PointField,
                        String=String)
        self.rclpy = rclpy
        self.config, self.rate, self.out = config, float(rate), out
        if not 0 < self.rate <= 10:
            raise ValueError('rate must be in (0, 10]')
        lidar_profile = config.lidar
        self.people, self.distortion, self.odom_scale = people, bool(distortion), float(odom_scale)
        # Wrappers whose robot_state_publisher owns base_link -> lidar_link, or whose odom->base_link
        # comes from another estimator, disable these so each TF edge keeps a single authority.
        self.publish_static_tf, self.publish_odom_tf = bool(publish_static_tf), bool(publish_odom_tf)
        self.lidar_profile_path = lidar_profile_path
        self.sim = HallSim(lidar_profile.mount_xyz_m, lidar_profile.mount_rpy_rad, people=people)
        if route and scenario:
            raise ValueError('Use either a route or a scenario')
        start = route.start_pose if route else scenario.start_pose if scenario else DEFAULT_START
        self.sim.place(*start)
        self.lidar = SimLidar(self.sim, lidar_profile, config.self_filter, seed=config.odometry['seed'] + 1)
        first = self.lidar.scan(0.0)
        self.geometry = geometry_from(config, first.blind_columns, first.azimuth_deg)
        radius = config.driver['footprint_radius_m']
        self.driver = KinematicDriver(config.driver, start, lambda x, y: self.sim.footprint_free(x, y, radius))
        self.route = route
        if route:
            self.driver.load_route(route_segments(start, route.waypoints, route.final_yaw))
        self.scenario = scenario
        self.steps = list(scenario.steps) if scenario else []
        self.wait_until = None
        self.blackout_until = float('-inf')
        self.scans_suppressed = 0
        self.points_dropped_by_fault = 0
        if not 0 <= self.odom_scale <= 20:
            raise ValueError('odom_scale must be in [0, 20]')
        scaled = dict(config.odometry)
        for key in ('linear_scale_error', 'yaw_rate_bias_rad_s', 'linear_noise_std', 'angular_noise_std'):
            scaled[key] *= self.odom_scale
        self.odom = OdometrySimulator(scaled)
        self.period = 1.0 / lidar_profile.scan_rate_hz
        self.history = deque(maxlen=int(4 * self.period * config.odometry['rate_hz']) + 4)
        self.dropout_until, self.dropout_p = float('-inf'), 0.0
        self.fault_rng = np.random.default_rng(config.odometry['seed'] + 2)
        self.start = start
        self.dt = 1.0 / config.odometry['rate_hz']
        ratio = 1.0 / (lidar_profile.scan_rate_hz * self.dt)
        if abs(ratio - round(ratio)) > 1e-6 or round(ratio) < 1:
            raise ValueError(f'Lidar rate {lidar_profile.scan_rate_hz} Hz is not an integer divisor of the '
                             f'{1 / self.dt:.0f} Hz control rate; use --control-hz')
        self.scan_every = round(ratio)
        self.t = CLOCK_START_S
        self.ignored_commands = 0

        self.node = Node('ultron_sim_bridge')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        topics = config.topics
        self.pub = {
            'clock': self.node.create_publisher(Clock, topics['clock'], qos),
            'points': self.node.create_publisher(PointCloud2, topics['points'], qos),
            'scan': self.node.create_publisher(LaserScan, topics['scan'], qos),
            'odom': self.node.create_publisher(Odometry, topics['odom'], qos),
            'ground_truth': self.node.create_publisher(PoseStamped, topics['ground_truth'], qos),
            'status': self.node.create_publisher(String, topics['status'], latched),
        }
        # Latest command wins: a queue would apply stale velocities late.
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(Twist, topics['cmd_vel'], self._on_cmd, latest)
        # Simulation test hook (obstacles, lidar faults) for navigation tests; never a robot interface.
        self.node.create_subscription(String, topics['sim_control'], self._on_control, qos)
        self.tf = TransformBroadcaster(self.node)
        self.static_tf = StaticTransformBroadcaster(self.node) if self.publish_static_tf else None
        if self.static_tf:
            self._publish_static()
        self.fields = [PointField(name=n, offset=CLOUD_DTYPE.fields[n][1], count=1,
                                  datatype=PointField.UINT16 if n == 'ring' else PointField.FLOAT32)
                       for n in CLOUD_DTYPE.names]
        self.logs = None
        if out:
            out.mkdir(parents=True, exist_ok=False)
            self.logs = {'ground_truth': (out / 'ground_truth.csv').open('w'),
                         'odometry': (out / 'odometry.csv').open('w')}
            self.logs['events'] = (out / 'events.jsonl').open('w')
            self.logs['ground_truth'].write('t,x,y,yaw\n')
            self.logs['odometry'].write('t,x,y,yaw\n')
            self._write_meta()

    # --- helpers -----------------------------------------------------------------------
    def _stamp(self, t=None):
        from builtin_interfaces.msg import Time
        t = self.t if t is None else t
        sec = int(math.floor(t))
        return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)

    def _transform(self, parent, child, xyz, yaw, stamp):
        tf = self.msg['TransformStamped']()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, parent, child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, xyz)
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = _quat(yaw)
        return tf

    def _publish_static(self):
        f = self.config.frames
        self.static_tf.sendTransform(self._transform(f['base'], f['lidar'], self.config.lidar.mount_xyz_m,
                                                     self.config.lidar.mount_rpy_rad[2], self._stamp()))

    def _write_meta(self):
        g = self.geometry
        meta = {
            'schema': 'ultron.sim_run/v1', 'provenance': 'SIMULATION_KINEMATIC_BASE_RAYCAST_LIDAR',
            'scene': 'ultron_lidar_hall_v1', 'start_pose_world': list(self.start),
            'odom_origin': 'odom frame origin = start pose; map frame defined by the SLAM backend',
            'config_sha256': self.config.sha256, 'lidar_profile_sha256': self.config.lidar.sha256,
            'lidar_unconfirmed_parameters': self.config.lidar.unconfirmed,
            'route': None if not self.route else {'name': self.route.name, 'sha256': self.route.sha256},
            'scenario': None if not self.scenario else {'name': self.scenario.name, 'sha256': self.scenario.sha256},
            'rate': self.rate, 'clock_start_s': CLOCK_START_S, 'noise_seed': self.config.odometry['seed'],
            'people': self.people, 'motion_distortion': self.distortion, 'odom_error_scale': self.odom_scale,
            'lidar_profile': {'name': self.config.lidar.name, 'path': self.lidar_profile_path,
                              'scan_rate_hz': self.config.lidar.scan_rate_hz,
                              'horizontal_resolution_deg': self.config.lidar.horizontal_resolution_deg},
            'scan_stamp': 'sweep start (points carry time offsets)' if self.distortion else 'single pose at stamp',
            'scan_geometry': {'angle_min': g.angle_min, 'angle_max': g.angle_max,
                              'angle_increment': g.angle_increment, 'bins': g.bins,
                              'z_band_base_link': [g.z_min, g.z_max]},
            'ground_truth': 'EVALUATION_ONLY: ground_truth.csv / topic; never on TF or estimator input',
            'tf_published': {'base_to_lidar_static': self.publish_static_tf, 'odom_to_base': self.publish_odom_tf},
        }
        (self.out / 'run_meta.json').write_text(json.dumps(meta, indent=2))

    def _on_control(self, msg):
        try:
            cmd = json.loads(msg.data)
            if 'obstacle' in cmd:
                o = cmd['obstacle']
                self.sim.place_test_obstacle(int(o['id']), o.get('x'), o.get('y'))
                self._event('TEST_OBSTACLE', **o)
            elif cmd.get('fault') == 'lidar_blackout':
                self.blackout_until = self.t + float(cmd['duration_s'])
                self._event('FAULT_START', fault='lidar_blackout', duration_s=float(cmd['duration_s']), probability=1.0)
            else:
                raise ValueError(f'unknown sim control {cmd}')
        except (ValueError, KeyError, TypeError) as exc:
            self.node.get_logger().error(f'Rejected sim control: {exc}')

    def _on_cmd(self, msg):
        if self.driver.route_active:
            self.ignored_commands += 1
            return
        if msg.linear.y or msg.linear.z or msg.angular.x or msg.angular.y:
            self.node.get_logger().warn('Only linear.x and angular.z are simulated; other axes ignored',
                                        throttle_duration_sec=5.0)
        try:
            self.driver.set_command(msg.linear.x, msg.angular.z, self.t)
        except ValueError:
            self.ignored_commands += 1

    # --- scenario ----------------------------------------------------------------------
    def _event(self, kind, **payload):
        record = {'t': round(self.t, 3), 'event': kind, **payload}
        self.node.get_logger().info(json.dumps(record))
        if self.logs:
            self.logs['events'].write(json.dumps(record) + '\n')
            self.logs['events'].flush()

    def _advance_scenario(self):
        """Start the next step once the previous route/wait has finished."""
        while self.steps and not self.driver.route_active and (self.wait_until is None or self.t >= self.wait_until):
            self.wait_until = None
            step = self.steps.pop(0)
            s = self.driver.state
            if step[0] == 'route':
                self.driver.load_route(route_segments((s.x, s.y, s.yaw), step[1], step[2]))
                self._event('ROUTE_START', waypoints=[list(w) for w in step[1]])
                return
            if step[0] == 'wait':
                self.wait_until = self.t + step[1]
                self._event('WAIT', seconds=step[1])
                return
            if step[0] == 'teleport':
                before = (s.x, s.y, s.yaw)
                self.driver.teleport(*step[1])
                self._event('TELEPORT', true_before=list(before), true_after=list(step[1]),
                            note='odometry not informed (kidnapped robot)')
            elif step[0] == 'fault':
                if step[1] == 'lidar_blackout':
                    self.blackout_until = self.t + step[2]
                else:
                    self.dropout_until, self.dropout_p = self.t + step[2], step[3]
                self._event('FAULT_START', fault=step[1], duration_s=step[2], probability=step[3])

    def _true_pose_at(self, t):
        """Interpolated TRUE base pose (simulator internal; used only to cast distorted sweeps)."""
        h = self.history
        if t <= h[0][0]:
            return h[0][1:]
        for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(h, list(h)[1:]):
            if t0 <= t <= t1:
                f = (t - t0) / (t1 - t0)
                return x0 + f * (x1 - x0), y0 + f * (y1 - y0), a0 + f * math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        return h[-1][1:]

    def _drop_points(self, scan):
        keep = self.fault_rng.random(len(scan.points)) >= self.dropout_p
        self.points_dropped_by_fault += int((~keep).sum())
        return dataclasses.replace(scan, points=scan.points[keep], ring=scan.ring[keep], time=scan.time[keep],
                                   intensity=scan.intensity[keep], eval_geom_id=scan.eval_geom_id[keep])

    @property
    def scenario_done(self):
        return (self.scenario is not None and not self.steps and not self.driver.route_active
                and (self.wait_until is None or self.t >= self.wait_until))

    # --- publishing --------------------------------------------------------------------
    def _publish_cloud(self, scan, stamp):
        cloud = np.zeros(len(scan.points), dtype=CLOUD_DTYPE)
        cloud['x'], cloud['y'], cloud['z'] = scan.points[:, 0], scan.points[:, 1], scan.points[:, 2]
        cloud['intensity'], cloud['ring'], cloud['time'] = scan.intensity, scan.ring, scan.time
        msg = self.msg['PointCloud2']()
        msg.header.stamp, msg.header.frame_id = stamp, self.config.frames['lidar']
        msg.height, msg.width = 1, len(cloud)
        msg.fields, msg.is_bigendian, msg.is_dense = self.fields, False, True
        msg.point_step, msg.row_step = CLOUD_DTYPE.itemsize, CLOUD_DTYPE.itemsize * len(cloud)
        # array.array skips rclpy's per-byte Python validation (78 M checks per 10 s of scans).
        data = array.array('B')
        data.frombytes(cloud.tobytes())
        msg.data = data
        self.pub['points'].publish(msg)

    def _publish_scan(self, scan, stamp):
        g = self.geometry
        msg = self.msg['LaserScan']()
        msg.header.stamp, msg.header.frame_id = stamp, self.config.frames['lidar']
        msg.angle_min, msg.angle_max, msg.angle_increment = g.angle_min, g.angle_max, g.angle_increment
        # With distortion, bins are ordered in sweep direction from the stamped sweep start.
        msg.time_increment = (self.period * g.angle_increment / (2 * math.pi)) if self.distortion else 0.0
        msg.scan_time = 1.0 / self.config.lidar.scan_rate_hz
        msg.range_min, msg.range_max = g.range_min, g.range_max
        ranges = array.array('f')
        ranges.frombytes(project(scan.points, g).astype('<f4').tobytes())
        msg.ranges = ranges
        self.pub['scan'].publish(msg)

    def _publish_state(self, stamp):
        f = self.config.frames
        x, y, yaw = self.odom.x, self.odom.y, self.odom.yaw
        if self.publish_odom_tf:
            self.tf.sendTransform(self._transform(f['odom'], f['base'], (x, y, 0.0), yaw, stamp))
        odom = self.msg['Odometry']()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, f['odom'], f['base']
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        q = _quat(yaw)
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        cov = [0.0] * 36
        cov[0] = cov[7] = max(self.odom.var_xy, 1e-6)
        cov[35] = max(self.odom.var_yaw, 1e-6)
        cov[14] = cov[21] = cov[28] = 1e6  # z, roll, pitch not estimated
        odom.pose.covariance = cov
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.odom.vx, self.odom.wz
        self.pub['odom'].publish(odom)
        gx, gy, gyaw = self.sim.pose
        truth = self.msg['PoseStamped']()
        truth.header.stamp, truth.header.frame_id = stamp, GROUND_TRUTH_FRAME
        truth.pose.position.x, truth.pose.position.y = gx, gy
        (truth.pose.orientation.x, truth.pose.orientation.y,
         truth.pose.orientation.z, truth.pose.orientation.w) = _quat(gyaw)
        self.pub['ground_truth'].publish(truth)
        if self.logs:
            self.logs['ground_truth'].write(f'{self.t:.3f},{gx:.5f},{gy:.5f},{gyaw:.6f}\n')
            self.logs['odometry'].write(f'{self.t:.3f},{x:.5f},{y:.5f},{yaw:.6f}\n')

    def status(self, achieved_rtf):
        s = self.driver.state
        return {'sim_time_s': round(self.t, 3), 'driver_event': s.last_event, 'collisions': s.collisions,
                'command_timeouts': s.timeouts, 'route_active': self.driver.route_active,
                'ignored_commands': self.ignored_commands, 'requested_rate': self.rate,
                'scans_suppressed_by_fault': self.scans_suppressed,
                'points_dropped_by_fault': self.points_dropped_by_fault,
                'achieved_rate': round(achieved_rtf, 3), 'odom_distance_m': round(self.odom.distance, 3),
                'provenance': 'SIMULATION_ONLY'}

    # --- main loop ---------------------------------------------------------------------
    def run(self, duration=None, hold_after_route=3.0):
        clock = self.msg['Clock']()
        wall0, t0, k = time.monotonic(), self.t, 0
        route_done_at = None
        last_status = -1.0
        try:
            while self.rclpy.ok():
                for _ in range(5):  # spin_once runs at most one callback per call
                    self.rclpy.spin_once(self.node, timeout_sec=0.0)
                self.t = t0 + (k + 1) * self.dt
                k += 1
                stamp = self._stamp()
                clock.clock = stamp
                self.pub['clock'].publish(clock)
                if self.scenario:
                    self._advance_scenario()
                state = self.driver.step(self.t, self.dt)
                self.sim.place(state.x, state.y, state.yaw)
                self.history.append((self.t, state.x, state.y, state.yaw))
                self.sim.update_people(self.t)
                self.odom.step(state.vx, state.wz, self.dt)
                self._publish_state(stamp)
                if k % self.scan_every == 0:
                    if self.t < self.blackout_until:
                        self.scans_suppressed += 1  # injected fault: the sensor publishes nothing
                    else:
                        if self.scans_suppressed and self.t - self.blackout_until < self.dt * self.scan_every:
                            self._event('FAULT_END', fault='lidar_blackout', scans_suppressed=self.scans_suppressed)
                        if self.distortion:
                            start = self.t - self.period
                            scan = self.lidar.scan(start, pose_at=lambda off: self._true_pose_at(start + off))
                            scan_stamp = self._stamp(start)
                        else:
                            scan, scan_stamp = self.lidar.scan(self.t), stamp
                        if self.t < self.dropout_until:
                            scan = self._drop_points(scan)
                        self._publish_cloud(scan, scan_stamp)
                        self._publish_scan(scan, scan_stamp)
                elapsed = time.monotonic() - wall0
                rtf = (self.t - t0) / elapsed if elapsed > 0 else 0.0
                if self.t - last_status >= 1.0:
                    msg = self.msg['String']()
                    msg.data = json.dumps(self.status(rtf))
                    self.pub['status'].publish(msg)
                    last_status = self.t
                if ((self.route and not self.driver.route_active) or self.scenario_done) and route_done_at is None:
                    route_done_at = self.t
                if route_done_at is not None and self.t - route_done_at >= hold_after_route:
                    break
                if duration is not None and self.t - t0 >= duration:
                    break
                ahead = wall0 + (self.t - t0) / self.rate - time.monotonic()
                if ahead > 0:
                    time.sleep(ahead)
        finally:
            elapsed = time.monotonic() - wall0
            summary = self.status((self.t - t0) / elapsed if elapsed > 0 else 0.0)
            if self.out:
                for handle in self.logs.values():
                    handle.close()
                (self.out / 'bridge_summary.json').write_text(json.dumps(summary, indent=2))
            self.lidar.close()
            self.node.get_logger().info('Bridge stopped: ' + json.dumps(summary))
        return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description='ULTRON lidar hall ROS 2 simulation bridge (simulation only)')
    parser.add_argument('--config', type=Path, default=CONFIG_DIR / 'sim_slam.yaml')
    parser.add_argument('--route', type=Path, help='Scripted route YAML; omit for cmd_vel control')
    parser.add_argument('--scenario', type=Path, help='Scenario YAML (route/wait/teleport/fault steps)')
    parser.add_argument('--rate', type=float, default=1.0, help='Simulation speed relative to wall time')
    parser.add_argument('--duration', type=float, help='Stop after this many simulated seconds')
    parser.add_argument('--out', type=Path, help='New folder for run metadata and evaluation logs')
    parser.add_argument('--seed', type=int, help='Override odometry/lidar noise seed (recorded in run_meta)')
    parser.add_argument('--people', choices=['static', 'walking', 'none'], default='static')
    parser.add_argument('--motion-distortion', action='store_true', help='Cast each sweep slice from its own pose')
    parser.add_argument('--odom-scale', type=float, default=1.0, help='Multiply all odometry error terms')
    parser.add_argument('--lidar-profile', type=Path, help='Alternative lidar profile YAML')
    parser.add_argument('--control-hz', type=float, help='Override control/odometry rate (default from config)')
    parser.add_argument('--no-static-tf', action='store_true',
                        help='Do not publish base_link -> lidar_link (robot_state_publisher owns it)')
    parser.add_argument('--no-odom-tf', action='store_true',
                        help='Do not publish odom -> base_link (another estimator owns it)')
    args = parser.parse_args(argv)
    config = load_sim_slam_config(args.config)
    if args.control_hz:
        config = dataclasses.replace(config, odometry=dict(config.odometry, rate_hz=args.control_hz))
    if args.lidar_profile:
        config = dataclasses.replace(config, lidar=load_lidar_profile(args.lidar_profile))
    if args.seed is not None:
        config = dataclasses.replace(config, odometry=dict(config.odometry, seed=args.seed))
    check_isolation(config)
    route = load_route(args.route) if args.route else None
    scenario = load_scenario(args.scenario) if args.scenario else None
    import rclpy
    rclpy.init()
    try:
        bridge = SimBridge(config, route, args.rate, args.out, scenario, args.people, args.motion_distortion,
                           args.odom_scale, str(args.lidar_profile) if args.lidar_profile else None,
                           publish_static_tf=not args.no_static_tf, publish_odom_tf=not args.no_odom_tf)
        bridge.run(args.duration)
        bridge.node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
