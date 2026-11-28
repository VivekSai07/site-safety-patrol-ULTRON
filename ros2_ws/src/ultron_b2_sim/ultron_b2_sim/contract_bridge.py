"""ROS 2 contract wrapper around ``ultron.autonomy.sim_bridge`` (Agent F). SIMULATION ONLY.

Runs the validated MuJoCo lidar-hall bridge under the stack namespace (``/ultron/sim``) with the
contract topic names of docs/b2_ros_stack/PLAN.md §3.2 and adds:

* ``sensors/imu/data``  kinematic synthetic IMU (gyro z, planar accel + gravity, no orientation)
* ``sensors/cam_head/{color/image_raw,color/camera_info,depth/image_rect_raw,depth/camera_info}``
  MuJoCo RGB-D render (primitive scene; not sensor data)
* ``sim/ground_truth/base_pose`` / ``sim/ground_truth/ladders``  EVALUATION ONLY, never on TF
* ``sim/oracle/cam_head/ladder_labels``  ORACLE segmentation for the labelled ``oracle_sim`` backend
* ``sim/scenario`` (``ultron_b2_interfaces/srv/SimScenario``) obstacle / blackout / dropout /
  kidnap / ladder move-hide-show hooks

Velocity input is the gated ``cmd_vel`` only (``motion_gate`` is its sole publisher in the stack).
``base_link -> sensor`` TF belongs to robot_state_publisher, so the bridge's own static
``base_link -> lidar_link`` is disabled; ``odom -> base_link`` is published only when
``--odom-source sim``. Kinematic base motion, not validated walking.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('MUJOCO_GL', 'egl')
os.environ.setdefault('LP_NUM_THREADS', '4')  # llvmpipe threads; shared hosts

import argparse  # noqa: E402
import array  # noqa: E402
import dataclasses  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
from pathlib import Path  # noqa: E402
import queue  # noqa: E402
import sys  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from .paths import ensure_ultron_importable  # noqa: E402

PROVENANCE = 'SIMULATION_KINEMATIC_BASE_MUJOCO_RENDER'


def contract_topics(base_topics, odom_source='sim', scan_on_contract=True):
    """Relative contract names for the bridge's topic keys (namespace comes from the launch)."""
    topics = dict(base_topics)
    topics.update({
        'clock': '/clock',
        'points': 'sensors/lidar/points',
        'scan': 'sensors/scan' if scan_on_contract else 'sim/scan',
        # Simulated leg odometry is the contract odometry only when it is the selected authority.
        'odom': 'state/odom' if odom_source == 'sim' else 'sim/leg_odom',
        'ground_truth': 'sim/ground_truth/base_pose',
        'cmd_vel': 'cmd_vel',
        'status': 'sim/bridge_status',
        'sim_control': 'sim/sim_control',
    })
    return topics


EXTRA_TOPICS = {
    'imu': 'sensors/imu/data',
    'color': 'sensors/cam_head/color/image_raw',
    'color_info': 'sensors/cam_head/color/camera_info',
    'depth': 'sensors/cam_head/depth/image_rect_raw',
    'depth_info': 'sensors/cam_head/depth/camera_info',
    'ladder_labels': 'sim/oracle/cam_head/ladder_labels',
    'ladders': 'sim/ground_truth/ladders',
    'scenario': 'sim/scenario',
}


def load_calibration(path):
    data = yaml.safe_load(Path(path).read_text())
    return data['frames']


def make_bridge_class(SimBridge):
    """Subclass the existing bridge (imported lazily so ``ultron`` resolves from the repo)."""

    class ContractSimBridge(SimBridge):
        def __init__(self, config, *, odom_source, camera_cfg, calibration, imu=True, camera=True,
                     oracle_segmentation=True, hidden_ladders=(), camera_threaded=True, **kwargs):
            super().__init__(config, publish_static_tf=False, publish_odom_tf=(odom_source == 'sim'), **kwargs)
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import CameraInfo, Image, Imu
            from std_msgs.msg import String
            from ultron_b2_interfaces.srv import SimScenario

            from .scenario import GROUND_TRUTH_FRAME, LadderHooks, TRUTH_SCHEMA

            self.msg.update(CameraInfo=CameraInfo, Image=Image, Imu=Imu)
            self.SimScenario = SimScenario
            self.truth_schema, self.truth_frame = TRUTH_SCHEMA, GROUND_TRUTH_FRAME
            self.odom_source = odom_source
            node = self.node
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.frames_cal = calibration
            self.hooks = LadderHooks(self.sim)
            for name in hidden_ladders:
                self.hooks.hide(name)
            self.pub['ladders'] = node.create_publisher(String, EXTRA_TOPICS['ladders'], latched)
            node.create_service(SimScenario, EXTRA_TOPICS['scenario'], self._on_scenario)
            self._k = 0
            self._last_truth_t = -math.inf
            self._truth_version = -1
            self.imu_enabled = bool(imu)
            if self.imu_enabled:
                self.pub['imu'] = node.create_publisher(Imu, EXTRA_TOPICS['imu'], QoSProfile(
                    depth=50, reliability=ReliabilityPolicy.RELIABLE))
                self._imu_rng = np.random.default_rng(config.odometry['seed'] + 3)
                self._prev_vx = 0.0
            self.camera = None
            self.cam_stats = {'frames': 0, 'skipped_busy': 0, 'render_ms_sum': 0.0, 'render_ms_max': 0.0}
            if camera:
                self._setup_camera(camera_cfg, oracle_segmentation, camera_threaded, qos)

        # --- camera --------------------------------------------------------------------
        def _setup_camera(self, cfg, oracle_segmentation, threaded, qos):
            from .camera import HeadCamera, stream_from_config
            rate = float(cfg['rate_hz'])
            every = (1.0 / rate) / self.dt
            if abs(every - round(every)) > 1e-6 or round(every) < 1:
                raise ValueError(f'camera rate {rate} Hz must divide the {1 / self.dt:.0f} Hz control rate')
            self.cam_every = round(every)
            self.cam_phase = self.cam_every // 2   # interleave with lidar sweeps (k % scan_every == 0)
            mount = self.frames_cal['cam_head_link']
            if mount.get('parent') != 'base_link':
                raise ValueError('cam_head_link must be a child of base_link')
            color, depth = stream_from_config(cfg['color']), stream_from_config(cfg['depth'])
            seg = bool(oracle_segmentation and cfg.get('oracle_segmentation', {}).get('enabled', True))

            def factory():
                return HeadCamera(self.sim, mount['xyz'], mount['rpy'], color, depth,
                                  min_depth_m=cfg['depth']['min_m'], max_depth_m=cfg['depth']['max_m'],
                                  segmentation=seg)

            node = self.node
            Image, CameraInfo = self.msg['Image'], self.msg['CameraInfo']
            self.pub['color'] = node.create_publisher(Image, EXTRA_TOPICS['color'], qos)
            self.pub['color_info'] = node.create_publisher(CameraInfo, EXTRA_TOPICS['color_info'], qos)
            self.pub['depth'] = node.create_publisher(Image, EXTRA_TOPICS['depth'], qos)
            self.pub['depth_info'] = node.create_publisher(CameraInfo, EXTRA_TOPICS['depth_info'], qos)
            if seg:
                self.pub['ladder_labels'] = node.create_publisher(Image, EXTRA_TOPICS['ladder_labels'], qos)
            self.cam_info = {'color': self._camera_info(color), 'depth': self._camera_info(depth)}
            self.cam_specs = {'color': color, 'depth': depth}
            self.cam_threaded = threaded
            if threaded:
                # EGL contexts are bound to the thread that creates them: create and render in the worker.
                self._cam_jobs = queue.Queue(maxsize=1)
                self._cam_busy = threading.Event()
                ready = threading.Event()
                errors = []

                def worker():
                    try:
                        self.camera = factory()
                    except Exception as exc:  # noqa: BLE001 - reported to the main thread
                        errors.append(exc)
                        return
                    finally:
                        ready.set()
                    while True:
                        stamp = self._cam_jobs.get()
                        if stamp is None:
                            break
                        try:
                            self._render_and_publish(stamp)
                        except Exception as exc:  # noqa: BLE001
                            self.node.get_logger().error(f'camera render failed: {exc}')
                        finally:
                            self._cam_busy.clear()
                    self.camera.close()

                self._cam_thread = threading.Thread(target=worker, name='head_camera', daemon=True)
                self._cam_thread.start()
                ready.wait()
                if errors:
                    raise errors[0]
            else:
                self.camera = factory()

        def _camera_info(self, spec):
            k = spec.intrinsics()
            info = self.msg['CameraInfo']()
            info.header.frame_id = spec.frame_id
            info.width, info.height = spec.width, spec.height
            info.distortion_model = 'plumb_bob'
            info.d = [0.0] * 5
            info.k = [k['fx'], 0.0, k['cx'], 0.0, k['fy'], k['cy'], 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [k['fx'], 0.0, k['cx'], 0.0, 0.0, k['fy'], k['cy'], 0.0, 0.0, 0.0, 1.0, 0.0]
            return info

        def _image(self, arr, encoding, frame_id, stamp, pixel_bytes):
            msg = self.msg['Image']()
            msg.header.stamp, msg.header.frame_id = stamp, frame_id
            msg.height, msg.width = arr.shape[0], arr.shape[1]
            msg.encoding, msg.is_bigendian, msg.step = encoding, 0, arr.shape[1] * pixel_bytes
            data = array.array('B')
            data.frombytes(np.ascontiguousarray(arr).tobytes())
            msg.data = data
            return msg

        def _render_and_publish(self, stamp):
            t0 = time.monotonic()
            rgb, depth_mm, labels = self.camera.render_prepared()
            ms = (time.monotonic() - t0) * 1000.0
            color, depth = self.cam_specs['color'], self.cam_specs['depth']
            for key, spec in (('color', color), ('depth', depth)):
                self.cam_info[key].header.stamp = stamp
            self.pub['color'].publish(self._image(rgb, 'rgb8', color.frame_id, stamp, 3))
            self.pub['color_info'].publish(self.cam_info['color'])
            self.pub['depth'].publish(self._image(depth_mm, '16UC1', depth.frame_id, stamp, 2))
            self.pub['depth_info'].publish(self.cam_info['depth'])
            if labels is not None and 'ladder_labels' in self.pub:
                self.pub['ladder_labels'].publish(self._image(labels, '16UC1', color.frame_id, stamp, 2))
            st = self.cam_stats
            st['frames'] += 1
            st['render_ms_sum'] += ms
            st['render_ms_max'] = max(st['render_ms_max'], ms)

        def _camera_tick(self, stamp):
            if self.camera is None:
                return
            if self.cam_threaded:
                if self._cam_busy.is_set():
                    self.cam_stats['skipped_busy'] += 1
                    return
                self.camera.prepare()       # reads MjData in this (simulation) thread
                self._cam_busy.set()
                self._cam_jobs.put(stamp)
            else:
                self.camera.prepare()
                self._render_and_publish(stamp)

        # --- IMU -----------------------------------------------------------------------
        def _publish_imu(self, stamp):
            s = self.driver.state
            rng = self._imu_rng
            ax = (s.vx - self._prev_vx) / self.dt
            self._prev_vx = s.vx
            msg = self.msg['Imu']()
            msg.header.stamp, msg.header.frame_id = stamp, 'imu_link'
            msg.orientation_covariance[0] = -1.0          # orientation not provided
            gyro_std, acc_std = 0.005, 0.02               # PROPOSED synthetic noise
            msg.angular_velocity.x = float(rng.normal(0, gyro_std))
            msg.angular_velocity.y = float(rng.normal(0, gyro_std))
            msg.angular_velocity.z = float(s.wz + rng.normal(0, gyro_std))
            msg.linear_acceleration.x = float(ax + rng.normal(0, acc_std))
            msg.linear_acceleration.y = float(s.vx * s.wz + rng.normal(0, acc_std))
            msg.linear_acceleration.z = float(9.80665 + rng.normal(0, acc_std))
            for cov, std in ((msg.angular_velocity_covariance, gyro_std), (msg.linear_acceleration_covariance, acc_std)):
                cov[0] = cov[4] = cov[8] = std * std
            self.pub['imu'].publish(msg)

        # --- ground truth (evaluation only) ---------------------------------------------
        def _publish_truth(self, force=False):
            if not force and self.hooks.version == self._truth_version and self.t - self._last_truth_t < 1.0:
                return
            labels = self.camera.ladder_labels if self.camera is not None else None
            payload = {'schema': self.truth_schema, 'provenance': 'SIMULATION_GROUND_TRUTH_EVALUATION_ONLY',
                       'note': 'Never estimator input; never on TF. Oracle use must be labelled ORACLE.',
                       'frame_id': self.truth_frame, 'sim_time_s': round(self.t, 3),
                       'odom_origin_world': list(self.start), 'version': self.hooks.version,
                       'ladders': self.hooks.truth(labels)}
            msg = self.msg['String']()
            msg.data = json.dumps(payload)
            self.pub['ladders'].publish(msg)
            self._truth_version, self._last_truth_t = self.hooks.version, self.t

        # --- scenario hooks --------------------------------------------------------------
        def _on_scenario(self, request, response):
            response.schema_version = self.SimScenario.Request.SCHEMA_VERSION
            cmd, target, v = request.command, request.target, list(request.values)
            try:
                if request.schema_version and request.schema_version != self.SimScenario.Request.SCHEMA_VERSION:
                    raise ValueError(f'schema_version {request.schema_version!r} unsupported')
                if not all(math.isfinite(x) for x in v):
                    raise ValueError('nonfinite values')
                if cmd == 'obstacle':
                    if len(v) not in (0, 2):
                        raise ValueError('obstacle needs values [x, y] or [] to park')
                    x, y = (v + [None, None])[:2]
                    self.sim.place_test_obstacle(int(target or 0), x, y)
                    event = dict(kind='TEST_OBSTACLE', id=int(target or 0), x=x, y=y)
                elif cmd == 'lidar_blackout':
                    self.blackout_until = self.t + float(v[0])
                    event = dict(kind='FAULT_START', fault='lidar_blackout', duration_s=float(v[0]), probability=1.0)
                elif cmd == 'lidar_dropout':
                    if not 0.0 <= v[1] <= 1.0:
                        raise ValueError('probability must be in [0, 1]')
                    self.dropout_until, self.dropout_p = self.t + float(v[0]), float(v[1])
                    event = dict(kind='FAULT_START', fault='lidar_dropout', duration_s=float(v[0]), probability=v[1])
                elif cmd == 'kidnap':
                    s = self.driver.state
                    before = [s.x, s.y, s.yaw]
                    self.driver.teleport(*v[:3])
                    self.sim.place(*v[:3])
                    event = dict(kind='TELEPORT', true_before=before, true_after=v[:3],
                                 note='odometry not informed (kidnapped robot)')
                elif cmd == 'ladder_move':
                    self.hooks.move(target, *v[:3])
                    event = dict(kind='LADDER_MOVE', ladder=target, pose=v[:3])
                elif cmd == 'ladder_hide':
                    self.hooks.hide(target)
                    event = dict(kind='LADDER_HIDE', ladder=target)
                elif cmd == 'ladder_show':
                    self.hooks.show(target)
                    event = dict(kind='LADDER_SHOW', ladder=target)
                else:
                    raise ValueError(f'unknown command {cmd!r}')
            except (ValueError, IndexError, TypeError) as exc:
                response.accepted, response.message = False, str(exc)
                self.node.get_logger().error(f'Rejected sim scenario {cmd!r}: {exc}')
                return response
            kind = event.pop('kind')
            self._event(kind, **event)
            if kind.startswith('LADDER'):
                self._publish_truth(force=True)
            response.accepted, response.message = True, 'ok'
            response.event_json = json.dumps({'t': round(self.t, 3), 'event': kind, **event})
            return response

        # --- per-step hook ---------------------------------------------------------------
        def _publish_state(self, stamp):
            super()._publish_state(stamp)
            self._k += 1
            if self.imu_enabled:
                self._publish_imu(stamp)
            if self.camera is not None and self._k % self.cam_every == self.cam_phase:
                self._camera_tick(stamp)
            self._publish_truth()

        def status(self, achieved_rtf):
            out = super().status(achieved_rtf)
            st = self.cam_stats
            out['camera'] = {'frames': st['frames'], 'skipped_busy': st['skipped_busy'],
                             'render_ms_mean': round(st['render_ms_sum'] / st['frames'], 2) if st['frames'] else None,
                             'render_ms_max': round(st['render_ms_max'], 2)}
            out['odom_source'] = self.odom_source
            out['provenance'] = PROVENANCE
            return out

        def shutdown(self):
            if self.camera is not None and getattr(self, 'cam_threaded', False):
                try:
                    self._cam_jobs.put(None, timeout=1.0)
                    self._cam_thread.join(timeout=3.0)
                except queue.Full:
                    pass
            elif self.camera is not None:
                self.camera.close()

    return ContractSimBridge


def build_parser():
    p = argparse.ArgumentParser(description='ULTRON sim bridge on the B2 ROS topic contract (simulation only)')
    p.add_argument('--odom-source', default='sim', choices=['sim', 'mola', 'kiss'],
                   help='sim: publish state/odom + odom->base_link; otherwise sim/leg_odom and no odom TF')
    p.add_argument('--calibration', type=Path, required=True, help='ultron_b2_description calibration/sim.yaml')
    p.add_argument('--camera-config', type=Path, required=True, help='config/head_camera_sim.yaml')
    p.add_argument('--config', type=Path, help='sim_slam.yaml (default: repo config/autonomy/sim_slam.yaml)')
    p.add_argument('--route', type=Path)
    p.add_argument('--scenario', type=Path)
    p.add_argument('--rate', type=float, default=1.0)
    p.add_argument('--duration', type=float)
    p.add_argument('--out', type=Path)
    p.add_argument('--seed', type=int)
    p.add_argument('--people', choices=['static', 'walking', 'none'], default='static')
    p.add_argument('--motion-distortion', action='store_true')
    p.add_argument('--odom-scale', type=float, default=1.0)
    p.add_argument('--no-camera', action='store_true')
    p.add_argument('--no-imu', action='store_true')
    p.add_argument('--no-oracle-segmentation', action='store_true')
    p.add_argument('--camera-inline', action='store_true', help='render in the simulation thread')
    p.add_argument('--scan-topic', choices=['contract', 'sim'], default='contract',
                   help='contract: publish sensors/scan; sim: sim/scan (when preprocess owns sensors/scan)')
    p.add_argument('--hidden-ladders', default='', help='comma-separated ladder names hidden at start')
    return p


def _clean_args(argv):
    from rclpy.utilities import remove_ros_args
    return [a for a in remove_ros_args(args=argv)[1:] if a]


def main(argv=None):
    argv = sys.argv if argv is None else argv
    args = build_parser().parse_args(_clean_args(argv))
    ensure_ultron_importable()
    from ultron.autonomy.config import load_route, load_scenario, load_sim_slam_config, CONFIG_DIR
    from ultron.autonomy.sim_bridge import SimBridge, check_isolation

    config = load_sim_slam_config(args.config or CONFIG_DIR / 'sim_slam.yaml')
    if args.seed is not None:
        config = dataclasses.replace(config, odometry=dict(config.odometry, seed=args.seed))
    config = dataclasses.replace(config, topics=contract_topics(config.topics, args.odom_source,
                                                                args.scan_topic == 'contract'))
    check_isolation(config)
    camera_cfg = yaml.safe_load(args.camera_config.read_text())
    calibration = load_calibration(args.calibration)
    hidden = [n.strip() for n in args.hidden_ladders.split(',') if n.strip()]
    import rclpy
    rclpy.init(args=argv)
    bridge = None
    try:
        cls = make_bridge_class(SimBridge)
        bridge = cls(config, odom_source=args.odom_source, camera_cfg=camera_cfg, calibration=calibration,
                     imu=not args.no_imu, camera=not args.no_camera,
                     oracle_segmentation=not args.no_oracle_segmentation, hidden_ladders=hidden,
                     camera_threaded=not args.camera_inline,
                     route=load_route(args.route) if args.route else None, rate=args.rate, out=args.out,
                     scenario=load_scenario(args.scenario) if args.scenario else None, people=args.people,
                     distortion=args.motion_distortion, odom_scale=args.odom_scale)
        bridge.run(args.duration)
    except KeyboardInterrupt:
        pass
    finally:
        if bridge is not None:
            bridge.shutdown()
            bridge.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
