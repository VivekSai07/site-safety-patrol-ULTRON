"""Nav2 bring-up and failure cases on saved or live SLAM maps. SIMULATION ONLY.

Starts slam_toolbox (localization), the LocalizationState monitor, Nav2 (map_server, planner,
controller, behaviors, BT navigator + lifecycle manager), the motion gate and the bridge in
command mode, then runs test cases through the Nav2 adapter. The case runner reads ground
truth only to score cases; it never feeds Nav2, the gate or localization.

Usage: python -m ultron.autonomy.run_navigation OUT --map MAPPING_RUN [--cases c1,c2,...] [--rate R]
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid

from .config import CONFIG_DIR, load_sim_slam_config
from .eval_slam import compose
from .live_grid import TOPIC as LIVE_MAP_TOPIC
from .navigation_contract import make_goal
from .rtabmap_sim import LINEAR_SPEED_M_S as RTABMAP_SPEED
from .rtabmap_sim import CLOUD_MAP as RTABMAP_CLOUD, ODOM as RTABMAP_ODOM, PARAMS as RTABMAP_PARAMS
from .run_localization import LOC_PARAMS, inverse
from .run_mapping import SLAM_PARAMS, ensure_no_running_stack, sha256, spawn, stop, terminate_cleanly
from .sim_bridge import check_isolation

NAV2_PARAMS = CONFIG_DIR / 'nav2_params.yaml'
NAV_MAP_TOPIC = '/ultron/sim/nav/map'
NAV2_NODES = ['map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
START_WORLD = (1.6, 1.5, math.pi / 2)
# Thresholds fixed before evaluation (docs/SAHIL_NAV_PLAN.md).
ACCEPT = {'arrive_xy_m': 0.25, 'arrive_yaw_rad': 0.35, 'stop_within_s': 1.2, 'still_m': 0.02,
          'gate_after_operator_s': 0.10, 'gate_after_blackout_s': 0.80, 'unreachable_timeout_s': 60.0}
ALL_CASES = ['reach_near', 'reach_far', 'reject_occupied', 'reject_outside', 'unreachable', 'new_obstacle',
             'cancel', 'operator_inhibit', 'localization_loss']


class Runner:
    def __init__(self, config, map_dir, out, *, test_operator=True):
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Bool, String
        from .nav2_adapter import Nav2Adapter
        self.rclpy, self.config, self.out = rclpy, config, out
        self.node = Node('ultron_nav_case_runner', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.nav = Nav2Adapter(self.node, config, map_dir)
        meta = json.loads((Path(map_dir) / 'map_meta.json').read_text())
        self.origin = tuple(meta['sim_run']['start_pose_world'])       # map frame origin in the world
        self.rtabmap_3d = meta.get('slam_backend') == 'rtabmap_3d'
        self.cloud_map = None
        if self.rtabmap_3d:
            cloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.node.create_subscription(PointCloud2, RTABMAP_CLOUD, self._on_cloud_map, cloud_qos)
        self.Bool, self.String = Bool, String
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.operator_pub = self.node.create_publisher(Bool, config.topics['operator_enable'], qos)
        self.control_pub = self.node.create_publisher(String, config.topics['sim_control'], qos)
        self.operator = True
        self.truth = []   # (t, x, y, yaw) EVALUATION ONLY
        self.node.create_subscription(PoseStamped, config.topics['ground_truth'], self._on_truth, qos)
        if test_operator:
            self.node.create_timer(0.2, self._publish_operator)
        self.node.create_timer(0.05, self.nav.tick)
        self.results = []

    def _on_cloud_map(self, msg):
        if msg.header.frame_id == 'map' and msg.width and msg.height:
            self.cloud_map = msg

    def _on_truth(self, msg):
        q = msg.pose.orientation
        self.truth.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.pose.position.x,
                           msg.pose.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)))

    def _publish_operator(self):
        self.operator_pub.publish(self.Bool(data=self.operator))

    def now(self):
        return self.nav.now()

    def spin(self, seconds=None, until=None, wall_limit=180.0):
        """Spin until sim time advanced by ``seconds`` or ``until()`` is true (wall-clock bounded)."""
        start, wall = self.now(), time.monotonic()
        while time.monotonic() - wall < wall_limit:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            if until is not None and until():
                return True
            if seconds is not None and self.now() - start >= seconds:
                return True
        return False

    def control(self, **cmd):
        self.control_pub.publish(self.String(data=json.dumps(cmd)))

    def to_map(self, world):
        return compose(inverse(self.origin), world)

    def to_world(self, pose):
        return compose(self.origin, pose)

    def goal(self, world_xy, yaw_world=0.0, timeout=None, reason='test'):
        x, y, yaw = self.to_map((world_xy[0], world_xy[1], yaw_world))
        return make_goal(x, y, yaw, mission_id='NAV-TEST', map_id=self.nav.map_id, map_version=self.nav.map_version,
                         params=self.config.navigation, reason=reason, now=self.now(), timeout_s=timeout)

    def true_pose(self):
        return self.truth[-1][1:] if self.truth else None

    def moved_after(self, t0, t1=None):
        """Largest true displacement between samples in [t0, t1]."""
        window = [s for s in self.truth if s[0] >= t0 and (t1 is None or s[0] <= t1)]
        if len(window) < 2:
            return 0.0
        x0, y0 = window[0][1:3]
        return max(math.hypot(x - x0, y - y0) for _, x, y, _ in window)

    def last_motion_time(self, after):
        """Last true sample after ``after`` that moved OR turned (rotation in place counts as motion)."""
        moving = [b[0] for a, b in zip(self.truth, self.truth[1:])
                  if b[0] >= after and (math.hypot(b[1] - a[1], b[2] - a[2]) > 1e-4
                                        or abs(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))) > 1e-4)]
        return moving[-1] if moving else after

    def run_goal(self, goal, wall_limit=240.0):
        result = self.nav.submit(goal)
        if result is None:
            self.spin(until=lambda: self.nav.done, wall_limit=wall_limit)
        return self.nav.tracker.result

    def wait_for_valid(self):
        ok = self.spin(until=lambda: self.nav._pose() is not None, wall_limit=60)
        if not ok:
            raise RuntimeError('Localization never became VALID')

    # --- cases -------------------------------------------------------------------------
    def case(self, name, fn):
        print(f'nav case {name} ...', flush=True)
        try:
            self.wait_for_valid()   # a case must not inherit localization loss from the previous one
        except RuntimeError as exc:
            print(f'  starting without VALID localization: {exc}', flush=True)
        t0 = self.now()
        try:
            record = fn()
        except Exception as exc:  # a failed case is recorded, the run continues
            record = {'checks': {'ran': False}, 'error': repr(exc)}
        record.update(case=name, started=round(t0, 3), ended=round(self.now(), 3),
                      events=[e for e in self.nav.tracker.events if e['t'] >= t0 - 1e-6])
        record['verdict'] = 'PASS' if record['checks'] and all(record['checks'].values()) else 'FAIL'
        self.results.append(record)
        print(f'  -> {record["verdict"]} {record["checks"]}', flush=True)
        self.spin(1.0)

    def arrive(self, world_xy, yaw_world):
        goal = self.goal(world_xy, yaw_world)
        result = self.run_goal(goal)
        tx, ty, tyaw = self.true_pose()
        err_xy = math.hypot(tx - world_xy[0], ty - world_xy[1])
        err_yaw = abs(math.atan2(math.sin(tyaw - yaw_world), math.cos(tyaw - yaw_world)))
        return {'status': result.status, 'reason': result.reason, 'capture_ready': result.capture_ready,
                'true_error_xy_m': round(err_xy, 3), 'true_error_yaw_rad': round(err_yaw, 3),
                'checks': {'reached': result.status == 'REACHED', 'capture_ready': result.capture_ready,
                           'xy_within': err_xy <= ACCEPT['arrive_xy_m'], 'yaw_within': err_yaw <= ACCEPT['arrive_yaw_rad']}}

    def rejected(self, world_xy, expect):
        t0 = self.now()
        result = self.run_goal(self.goal(world_xy))
        self.spin(2.0)
        moved = self.moved_after(t0)
        return {'status': result.status, 'reason': result.reason, 'moved_m': round(moved, 4),
                'checks': {'rejected': result.status == 'REJECTED', 'reason_matches': expect in result.reason,
                           'no_motion': moved <= ACCEPT['still_m']}}

    def unreachable(self):
        target = (7.5, 4.5)
        self.control(obstacle={'id': 0, 'x': target[0], 'y': target[1]})
        self.spin(1.0)
        t0 = self.now()
        result = self.run_goal(self.goal(target, timeout=ACCEPT['unreachable_timeout_s']))
        t_end = self.now()
        elapsed = t_end - t0
        self.spin(3.0)
        stop_after = self.last_motion_time(t_end) - t_end   # same stop budget as the other cases
        self.control(obstacle={'id': 0, 'x': None, 'y': None})
        return {'status': result.status, 'reason': result.reason, 'elapsed_s': round(elapsed, 2),
                'stop_after_s': round(stop_after, 3),
                'checks': {'failed_or_timeout': result.status in ('FAILED', 'TIMEOUT'),
                           'bounded': elapsed <= ACCEPT['unreachable_timeout_s'] + 5.0,
                           'stopped_in_time': stop_after <= ACCEPT['stop_within_s']}}

    def new_obstacle(self):
        collisions_before = self.collisions()
        goal = self.goal((2.5, 5.5), math.pi)
        self.nav.submit(goal)
        start = self.true_pose()
        self.spin(until=lambda: self.true_pose() and math.dist(self.true_pose()[:2], start[:2]) > 1.0, wall_limit=60)
        x, y, yaw = self.true_pose()
        spot = (x + 1.6 * math.cos(yaw), y + 1.6 * math.sin(yaw))
        self.control(obstacle={'id': 1, 'x': spot[0], 'y': spot[1]})
        self.spin(until=lambda: self.nav.done, wall_limit=240)
        result = self.nav.tracker.result
        self.control(obstacle={'id': 1, 'x': None, 'y': None})
        collisions = self.collisions() - collisions_before
        return {'status': result.status, 'reason': result.reason, 'obstacle_at_world': [round(v, 2) for v in spot],
                'collision_stops': collisions,
                'checks': {'replanned_or_stopped': result.status in ('REACHED', 'FAILED', 'TIMEOUT'),
                           'no_collision_stop': collisions == 0,
                           'no_false_arrival': result.status != 'REACHED' or
                           math.dist(self.true_pose()[:2], (2.5, 5.5)) <= ACCEPT['arrive_xy_m']}}

    def stop_case(self, trigger):
        goal = self.goal((9.6, 7.8), 0.0)
        self.nav.submit(goal)
        self.spin(until=lambda: self.nav.tracker.status == 'ACTIVE', wall_limit=30)
        self.spin(3.0)
        t_trigger = self.now()
        gate_before = len(self.gate_log)
        trigger()
        self.spin(until=lambda: self.nav.tracker.status in ('CANCELLED', 'TIMEOUT', 'FAILED', 'REACHED'), wall_limit=30)
        self.spin(3.0)
        result = self.nav.tracker.result
        stopped_at = self.last_motion_time(t_trigger)
        inhibit = next((g for g in self.gate_log[gate_before:] if g.get('inhibited')), None)
        return result, t_trigger, stopped_at, inhibit

    def collisions(self):
        return self.nav.bridge_status.get('collisions', 0)

    def cancel(self):
        result, t, stopped, _ = self.stop_case(lambda: self.nav.cancel('USER_CANCEL'))
        return {'status': result.status, 'stop_after_s': round(stopped - t, 3),
                'checks': {'cancelled': result.status == 'CANCELLED', 'stopped_in_time': stopped - t <= ACCEPT['stop_within_s']}}

    def operator_inhibit(self):
        def disable():
            self.operator = False
            self._publish_operator()
        result, t, stopped, inhibit = self.stop_case(disable)
        self.operator = True
        self.spin(1.0)
        delay = inhibit['t'] - t if inhibit else None
        return {'status': result.status, 'gate_reason': inhibit and inhibit['reason'], 'gate_delay_s': delay and round(delay, 3),
                'stop_after_s': round(stopped - t, 3),
                'checks': {'cancelled': result.status == 'CANCELLED',
                           'gate_inhibited_fast': delay is not None and delay <= ACCEPT['gate_after_operator_s'],
                           'stopped_in_time': stopped - t <= ACCEPT['stop_within_s']}}

    def localization_loss(self):
        result, t, stopped, inhibit = self.stop_case(lambda: self.control(fault='lidar_blackout', duration_s=3.0))
        delay = inhibit['t'] - t if inhibit else None
        moved_while_invalid = self.moved_after(t + ACCEPT['gate_after_blackout_s'] + ACCEPT['stop_within_s'], t + 3.0)
        return {'status': result.status, 'gate_reason': inhibit and inhibit['reason'], 'gate_delay_s': delay and round(delay, 3),
                'stop_after_s': round(stopped - t, 3), 'moved_while_invalid_m': round(moved_while_invalid, 4),
                'checks': {'cancelled': result.status == 'CANCELLED',
                           'gate_inhibited_in_time': delay is not None and delay <= ACCEPT['gate_after_blackout_s'],
                           'no_motion_while_invalid': moved_while_invalid <= ACCEPT['still_m']}}

    def dashboard(self, url):
        from ..dashboard.ros_link import Dashboard

        dashboard = Dashboard(url, timeout=2.0)
        dashboard._send('POST', '/api/missions', {'mission_id': 'NAV-DASHBOARD', 'site_id': 'sim'})

        def map_and_pose_ready():
            live = dashboard.live()
            grid = live.get('grid') or {}
            return (grid.get('map_id') == self.nav.map_id and grid.get('map_version') == self.nav.map_version
                    and live.get('pose') is not None)

        if not self.spin(until=map_and_pose_ready, wall_limit=20):
            raise RuntimeError('Dashboard did not receive a corresponding saved map and valid pose')

        terminal = {'SUCCEEDED', 'REJECTED', 'FAILED', 'CANCELED'}

        def send(world, **fields):
            x, y, yaw = self.to_map(world)
            return dashboard._send('POST', '/api/goal', dict(x=x, y=y, yaw=yaw) | fields)

        def wait(request, statuses):
            # Keep pumping ROS between HTTP polls so the test operator heartbeat continues.
            next_poll = 0.0
            result = None

            def done():
                nonlocal next_poll, result
                if time.monotonic() < next_poll:
                    return False
                next_poll = time.monotonic() + 0.2
                result = dashboard._send('GET', '/api/goal')
                return result['request_id'] == request['request_id'] and result['status'] in statuses

            if not self.spin(until=done, wall_limit=110.0):
                raise RuntimeError(f'Dashboard goal {request["seq"]} timed out: {result}')
            return result

        if self.nav.live_map:
            return self.dashboard_live(dashboard, send, wait, terminal)

        t0 = self.now()
        target = (2.5, 5.5, 0.0)
        self.operator = False
        self.spin(0.5)
        disabled = wait(send(target), terminal)
        self.operator = True
        self.spin(0.5)
        mismatched = wait(send(target, map_id=self.nav.map_id, map_version=self.nav.map_version + 1), terminal)
        occupied = wait(send((4.0, 4.0, 0.0)), terminal)
        outside = wait(send((14.0, 5.0, 0.0)), terminal)
        self.control(fault='lidar_blackout', duration_s=3.0)
        if not self.spin(until=lambda: self.nav.localization and self.nav.localization.state == 'STALE', wall_limit=10):
            raise RuntimeError('The localization monitor did not report the injected blackout')
        stale = wait(send(target), terminal)
        self.wait_for_valid()
        no_motion = self.moved_after(t0) <= ACCEPT['still_m']
        result = wait(send(target), terminal)
        tx, ty, tyaw = self.true_pose()
        error = math.hypot(tx - target[0], ty - target[1])
        if result['status'] != 'SUCCEEDED':
            return {'checks': {'reached': False}, 'goal': result, 'true_error_xy_m': round(error, 3)}

        moving = send((9.6, 4.5, -math.pi / 2))
        active = wait(moving, terminal | {'ACCEPTED'})
        replacement = wait(send(target), terminal)
        self.spin(0.5)
        bridge_log = (self.out / 'dashboard_bridge.log').read_text()
        sdk_path = self.out / 'sdk_stand_in.jsonl'
        sdk_calls = [json.loads(line) for line in sdk_path.read_text().splitlines()] if sdk_path.exists() else []
        return {'goals': {'disabled': disabled, 'wrong_map': mismatched, 'occupied': occupied,
                          'outside': outside, 'stale_localization': stale, 'reached': result,
                          'replacement': replacement},
                'true_error_xy_m': round(error, 3), 'true_error_yaw_rad': round(abs(tyaw), 3),
                'sdk_move_count': sum(row['method'] == 'Move' for row in sdk_calls),
                'checks': {'map_and_pose_published': True,
                           'disabled': disabled['status'] == 'REJECTED' and 'OPERATOR' in disabled['detail'],
                           'wrong_map': mismatched['status'] == 'REJECTED' and 'MAP_MISMATCH' in mismatched['detail'],
                           'occupied': occupied['status'] == 'REJECTED' and 'OCCUPIED' in occupied['detail'],
                           'outside': outside['status'] == 'REJECTED' and 'OUTSIDE_MAP' in outside['detail'],
                           'stale_localization': stale['status'] == 'REJECTED' and 'LOCALIZATION' in stale['detail'],
                           'no_motion_after_rejections': no_motion,
                           'reached': error <= ACCEPT['arrive_xy_m'] and abs(tyaw) <= ACCEPT['arrive_yaw_rad'],
                           'replacement': active['status'] == 'ACCEPTED' and replacement['status'] == 'SUCCEEDED'
                           and f'goal {moving["seq"]}: CANCELED' in bridge_log,
                           'sdk_move_and_stop': not sdk_path.exists() or (any(
                               row['method'] == 'Move' for row in sdk_calls) and sdk_calls[-1]['method'] == 'StopMove')}}

    def dashboard_live(self, dashboard, send, wait, terminal):
        candidates = [(1.6, 3.0), (1.6, 2.7), (1.6, 2.5), (2.6, 1.5), (1.9, 2.5),
                      (1.4, 2.6), (2.0, 2.6)]
        clearance = self.config.navigation['target_clearance_m']
        target = next(((*xy, math.pi / 2) for xy in candidates
                       if self.nav.footprint.check(*self.to_map((*xy, 0.0))[:2], clearance)[0]), None)
        if target is None:
            raise RuntimeError('No test goal has known clearance on the initial live grid')
        initial = dashboard.live()['grid']
        subscriptions = self.node.get_subscriptions_info_by_topic(LIVE_MAP_TOPIC)
        costmap_uses_live_grid = any(info.node_name == 'global_costmap' for info in subscriptions)
        t0 = self.now()
        future_stamp = initial['grid_stamp_ns'] + 60_000_000_000
        future = wait(send(target, grid_stamp_ns=future_stamp), terminal)
        stale = wait(send(target, grid_stamp_ns=1), terminal)
        outside = wait(send((50.0, 50.0, 0.0)), terminal)
        outside_motion = self.moved_after(t0)

        # Submit against the grid displayed at click time after a newer grid has arrived.
        displayed = dashboard.live()['grid']
        if not self.spin(until=lambda: self.nav.grid_stamp_ns > displayed['grid_stamp_ns'], wall_limit=10.0):
            raise RuntimeError('The live grid never updated')
        forwarded_from = self.nav.grid_stamp_ns
        request = send(target, map_id=displayed['map_id'], map_version=displayed['map_version'],
                       grid_stamp_ns=displayed['grid_stamp_ns'])
        result = wait(request, terminal)
        tx, ty, tyaw = self.true_pose()
        error = math.dist((tx, ty), target[:2])
        yaw_error = abs(math.remainder(tyaw - target[2], math.tau))
        after = dashboard.live()['grid']
        record = {
            'future_grid': future, 'stale_grid': stale, 'outside': outside, 'goal': result, 'target_world': target,
            'displayed_grid_stamp_ns': displayed['grid_stamp_ns'],
            'grid_stamp_before_submission_ns': forwarded_from, 'grid_stamp_after_goal_ns': after['grid_stamp_ns'],
            'true_error_xy_m': round(error, 3), 'true_error_yaw_rad': round(yaw_error, 3),
            'checks': {
                'live_grid_and_valid_pose': initial['source'] == 'live_navigation',
                'nav2_costmap_subscribes_to_live_grid': costmap_uses_live_grid,
                'future_revision_rejected': future['status'] == 'REJECTED' and 'REVISION_UNAVAILABLE' in future['detail'],
                'stale_revision_rejected': stale['status'] == 'REJECTED' and 'REVISION_STALE' in stale['detail'],
                'outside_rejected_without_motion': outside['status'] == 'REJECTED'
                and 'OUTSIDE_MAP' in outside['detail'] and outside_motion <= ACCEPT['still_m'],
                'reached_on_newer_grid': result['status'] == 'SUCCEEDED' and error <= ACCEPT['arrive_xy_m']
                and yaw_error <= ACCEPT['arrive_yaw_rad'],
                'mapping_continued_during_goal': after['grid_stamp_ns'] > forwarded_from,
            },
        }
        if self.rtabmap_3d:
            self.check_3d_sensor_loss(record, send, wait, terminal)
            self.record_3d_evidence(record)
        return record

    def check_3d_sensor_loss(self, record, send, wait, terminal):
        nearby = [(1.6, 4.3), (1.6, 3.9), (1.6, 2.35), (2.5, 3.0), (1.3, 3.8)]
        clearance = self.config.navigation['target_clearance_m']
        current = self.true_pose()[:2]
        target = next(((*xy, math.pi / 2) for xy in nearby if math.dist(xy, current) >= 0.7
                       and self.nav.footprint.check(*self.to_map((*xy, 0.0))[:2], clearance)[0]), None)
        if target is None:
            record['checks']['sensor_loss_cancels_active_goal'] = False
            record['sensor_loss'] = {'reason': 'No second test goal has known clearance on the live grid'}
            return
        request = send(target)
        dispatched = wait(request, terminal | {'ACCEPTED'})
        from_stamp = self.now()
        moving = dispatched['status'] == 'ACCEPTED' and self.spin(
            until=lambda: self.moved_after(from_stamp) >= 0.15, wall_limit=20.0)
        if not moving:
            record['checks']['sensor_loss_cancels_active_goal'] = False
            record['sensor_loss'] = {'goal': dispatched, 'reason': 'Second goal did not start moving'}
            return
        gate_before, injected_at = len(self.gate_log), self.now()
        self.control(fault='lidar_blackout', duration_s=3.0)
        canceled = wait(request, terminal)
        self.spin(2.0)
        inhibit = next((g for g in self.gate_log[gate_before:]
                        if g.get('inhibited') and 'LOCALIZATION' in (g.get('reason') or '')), None)
        stop_after = self.last_motion_time(injected_at) - injected_at
        record['sensor_loss'] = {'goal': canceled, 'inhibit': inhibit, 'injected_at': injected_at,
                                 'stop_after_s': round(stop_after, 3)}
        record['checks']['sensor_loss_cancels_active_goal'] = canceled['status'] == 'CANCELED' and inhibit is not None
        record['checks']['sensor_loss_stop_in_budget'] = stop_after <= ACCEPT['stop_within_s']

    def record_3d_evidence(self, record):
        import numpy as np
        from .real_bag import cloud_xyz

        def publishers(topic):
            return sorted({info.node_name for info in self.node.get_publishers_info_by_topic(topic)})

        sources = {topic: publishers(topic) for topic in (LIVE_MAP_TOPIC, RTABMAP_ODOM, RTABMAP_CLOUD)}
        record['publishers'] = sources
        xyz = cloud_xyz(self.cloud_map) if self.cloud_map is not None else np.empty((0, 3))
        height_span = float(np.ptp(xyz[:, 2])) if len(xyz) else 0.0
        record.update(cloud_points=len(xyz), cloud_height_span_m=height_span)
        record['checks'].update(
            rtabmap_is_live_grid_source=sources[LIVE_MAP_TOPIC] == ['rtabmap'],
            icp_is_navigation_odometry_source=sources[RTABMAP_ODOM] == ['icp_odometry'],
            nonplanar_map_cloud=sources[RTABMAP_CLOUD] == ['rtabmap'] and len(xyz) > 100 and height_span > 0.5,
        )
        if len(xyz):
            header = ('ply\nformat binary_little_endian 1.0\n'
                      f'element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
            with (self.out / 'cloud_map.ply').open('wb') as handle:
                handle.write(header.encode('ascii'))
                handle.write(xyz.astype('<f4').tobytes())


def main(argv=None):
    parser = argparse.ArgumentParser(description='Nav2 bring-up and failure cases in simulation')
    parser.add_argument('out', type=Path)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--map', type=Path)
    source.add_argument('--live-map', action='store_true')
    source.add_argument('--live-map-3d', action='store_true', help='Use online RTAB-Map LiDAR SLAM and its 2D projection')
    parser.add_argument('--cases')
    parser.add_argument('--rate', type=float, help='Simulation speed (default 1.0, or 0.6 for live 3D mapping)')
    parser.add_argument('--dashboard-url', help='Local dashboard URL for dashboard goals')
    parser.add_argument('--serve-dashboard', action='store_true', help='Wait for interactive dashboard goals')
    parser.add_argument('--sdk-stand-in', action='store_true', help='Log motion gate output as stand-in SDK calls')
    args = parser.parse_args(argv)
    args.live_map = args.live_map or args.live_map_3d
    if args.rate is None:
        args.rate = 0.6 if args.live_map_3d else 1.0
    if args.live_map_3d:
        os.environ.setdefault('OMP_NUM_THREADS', '4')
    default_cases = 'dashboard' if args.live_map else ','.join(ALL_CASES)
    requested_cases = default_cases if args.cases is None else args.cases
    cases = [] if args.serve_dashboard else [c for c in requested_cases.split(',') if c]
    available = [*ALL_CASES, 'dashboard']
    unknown = set(cases) - set(available)
    if unknown:
        parser.error(f'Unknown cases {sorted(unknown)}; choose from {available}')
    if (args.serve_dashboard or 'dashboard' in cases) and not args.dashboard_url:
        parser.error('Dashboard mode requires --dashboard-url')
    if args.live_map and set(cases) - {'dashboard'}:
        parser.error('Live mapping supports the dashboard navigation case')
    if args.dashboard_url:
        try:
            from ultron_nav_interfaces.action import NavigateDashboard  # noqa: F401
        except ImportError:
            parser.error('Build and source ultron_nav_interfaces as described in docs/dashboard_nav_goal.md')
    config = load_sim_slam_config()
    check_isolation(config)
    terminate_cleanly()
    ensure_no_running_stack()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    map_dir = out if args.live_map else args.map.resolve()
    if args.live_map:
        meta = {'map_id': f'ultron-live-{uuid.uuid4().hex[:12]}', 'map_version': 1, 'live_map': True,
                'frame_id': 'map', 'provenance': 'ONLINE_SLAM_SIMULATION',
                'slam_backend': 'rtabmap_3d' if args.live_map_3d else 'slam_toolbox_2d',
                'sim_run': {'start_pose_world': START_WORLD}}
        (out / 'map_meta.json').write_text(json.dumps(meta, indent=2) + '\n')
    py = sys.executable
    procs = {}
    ros = lambda *a: ['ros2', 'run', *a]
    remap_cmd = ['-r', f'cmd_vel:={config.topics["nav_cmd_raw"]}']
    odom_topic = RTABMAP_ODOM if args.live_map_3d else config.topics['odom']
    remap_odom = ['-r', f'odom:={odom_topic}']
    try:
        if args.live_map_3d:
            remappings = ['--ros-args', '--params-file', str(RTABMAP_PARAMS), *remap_odom,
                          '-r', f'scan_cloud:={config.topics["points"]}']
            procs['icp'] = spawn(ros('rtabmap_odom', 'icp_odometry') + remappings, out / 'icp_odometry.log')
            procs['slam'] = spawn(ros('rtabmap_slam', 'rtabmap') + remappings + [
                '-p', f'database_path:={out / "rtabmap.db"}',
                '-r', f'grid_map:={LIVE_MAP_TOPIC}', '-r', f'cloud_map:={RTABMAP_CLOUD}'], out / 'rtabmap.log')
        else:
            slam_node = 'async_slam_toolbox_node' if args.live_map else 'localization_slam_toolbox_node'
            slam_args = ['--ros-args', '--params-file', str(SLAM_PARAMS if args.live_map else LOC_PARAMS),
                         '-r', '__node:=slam_toolbox']
            if not args.live_map:
                slam_args += ['-p', f'map_file_name:={map_dir / "map"}', '-p', 'map_start_pose:=[0.0,0.0,0.0]']
            procs['slam'] = spawn(ros('slam_toolbox', slam_node) + slam_args, out / 'slam_toolbox.log')
        procs['monitor'] = spawn([py, '-m', 'ultron.autonomy.localization_monitor', '--map', str(map_dir), '--out', str(out)],
                                 out / 'monitor.log')
        procs['gate'] = spawn([py, '-m', 'ultron.autonomy.motion_gate_node', '--map', str(map_dir),
                               '--log', str(out / 'gate.json')], out / 'gate.log')
        lifecycle_nodes = NAV2_NODES[1:] if args.live_map else NAV2_NODES
        if not args.live_map:
            procs['map_server'] = spawn(ros('nav2_map_server', 'map_server') + [
                '--ros-args', '-p', 'use_sim_time:=true', '-p', f'yaml_filename:={map_dir / "map.yaml"}',
                '-p', 'topic_name:=' + NAV_MAP_TOPIC, '-r', 'map:=' + NAV_MAP_TOPIC], out / 'map_server.log')
        for name in ('planner_server', 'controller_server', 'behavior_server', 'bt_navigator'):
            package = {'planner_server': 'nav2_planner', 'controller_server': 'nav2_controller',
                       'behavior_server': 'nav2_behaviors', 'bt_navigator': 'nav2_bt_navigator'}[name]
            extra = (remap_cmd if name in ('controller_server', 'behavior_server') else []) + remap_odom
            if args.live_map and name == 'planner_server':
                extra += ['-r', f'{NAV_MAP_TOPIC}:={LIVE_MAP_TOPIC}']
            if args.live_map_3d and name == 'controller_server':
                extra += ['-p', f'FollowPath.desired_linear_vel:={RTABMAP_SPEED}']
            procs[name] = spawn(ros(package, name) + ['--ros-args', '--params-file', str(NAV2_PARAMS), *extra],
                                out / f'{name}.log')
        procs['lifecycle'] = spawn(ros('nav2_lifecycle_manager', 'lifecycle_manager') + [
            '--ros-args', '-r', '__node:=lifecycle_manager_navigation', '-p', 'use_sim_time:=true', '-p', 'autostart:=true',
            '-p', 'node_names:=' + json.dumps(lifecycle_nodes).replace('"', "'")], out / 'lifecycle.log')
        time.sleep(5.0)
        bridge_args = [py, '-m', 'ultron.autonomy.sim_bridge', '--rate', str(args.rate), '--duration', '3000',
                       '--out', str(out / 'sim')]
        if args.live_map_3d:
            bridge_args.append('--external-odom-tf')
        procs['bridge'] = spawn(bridge_args, out / 'bridge.log')
        if args.sdk_stand_in:
            procs['sdk'] = spawn([py, '-m', 'ultron.autonomy.sdk_velocity_node',
                                  '--stand-in-log', str(out / 'sdk_stand_in.jsonl')], out / 'sdk.log')
        if args.dashboard_url:
            procs['dashboard_goal'] = spawn([py, '-m', 'ultron.autonomy.dashboard_goal_node',
                                             '--map', str(map_dir)], out / 'dashboard_goal.log')
            script = Path(__file__).resolve().parents[3] / 'scripts' / 'dashboard_bridge.py'
            procs['dashboard_bridge'] = spawn([py, str(script), '--map', str(map_dir), '--url', args.dashboard_url],
                                             out / 'dashboard_bridge.log')
        for name, proc in procs.items():
            if proc.poll() is not None:
                raise RuntimeError(f'{name} exited early; see {out}')
        import rclpy
        rclpy.init()
        try:
            runner = Runner(config, map_dir, out, test_operator=not args.serve_dashboard)
            _attach_status(runner)
            print('navigation: waiting for VALID localization and Nav2 ...', flush=True)
            runner.wait_for_valid()
            if not runner.nav.client.wait_for_server(timeout_sec=60):
                raise RuntimeError('Nav2 NavigateToPose server not available')
            runner.spin(2.0)
            if args.serve_dashboard:
                print('Dashboard simulation ready. Operator heartbeat is required for goal dispatch.', flush=True)
                while all(proc.poll() is None for proc in procs.values()):
                    runner.spin(1.0, wall_limit=2.0)
                raise RuntimeError('A simulation process exited; see the run logs')
            plan = {'reach_near': lambda: runner.arrive((2.5, 5.5), 0.0),
                    'reach_far': lambda: runner.arrive((9.6, 4.5), -math.pi / 2),
                    'reject_occupied': lambda: runner.rejected((4.0, 4.0), 'OCCUPIED'),
                    'reject_outside': lambda: runner.rejected((14.0, 5.0), 'OUTSIDE_MAP'),
                    'unreachable': runner.unreachable, 'new_obstacle': runner.new_obstacle,
                    'cancel': runner.cancel, 'operator_inhibit': runner.operator_inhibit,
                    'localization_loss': runner.localization_loss,
                    'dashboard': lambda: runner.dashboard(args.dashboard_url)}
            for name in cases:
                runner.case(name, plan[name])
            summary = {'schema': 'ultron.nav_cases/v1', 'map_id': runner.nav.map_id, 'map_version': runner.nav.map_version,
                       'accept': ACCEPT, 'nav2_params_sha256': sha256(NAV2_PARAMS),
                       'cases': runner.results, 'gate_log': runner.gate_log,
                       'verdict': 'PASS' if all(r['verdict'] == 'PASS' for r in runner.results) else 'FAIL'}
            if args.live_map_3d:
                summary['rtabmap'] = {'params_sha256': sha256(RTABMAP_PARAMS), 'simulation_rate': args.rate,
                                     'controller_desired_linear_m_s': RTABMAP_SPEED}
            (out / 'nav_cases.json').write_text(json.dumps(summary, indent=2, default=str))
            print(json.dumps({r['case']: r['verdict'] for r in runner.results}, indent=1))
            print('navigation verdict:', summary['verdict'])
            if args.live_map and summary['verdict'] != 'PASS':
                raise SystemExit(1)
        finally:
            rclpy.shutdown()
    finally:
        for name in ('dashboard_bridge', 'dashboard_goal', 'sdk', 'bridge', 'lifecycle', 'bt_navigator',
                     'behavior_server', 'controller_server', 'planner_server', 'map_server', 'gate', 'monitor',
                     'slam', 'icp'):
            if name in procs:
                stop(procs[name])


def _attach_status(runner):
    """Gate status log and bridge status (collisions) for scoring."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    runner.gate_log = []
    runner.nav.bridge_status = {}

    def on_gate(msg):
        runner.gate_log.append(json.loads(msg.data))

    def on_bridge(msg):
        runner.nav.bridge_status = json.loads(msg.data)

    runner.node.create_subscription(runner.String, runner.config.topics['gate_status'], on_gate, latched)
    runner.node.create_subscription(runner.String, runner.config.topics['status'], on_bridge, latched)


if __name__ == '__main__':
    main()
