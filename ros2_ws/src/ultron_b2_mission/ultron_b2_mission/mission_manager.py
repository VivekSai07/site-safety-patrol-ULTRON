"""mission_manager node: RunMission server around MissionCore. Never publishes velocities.

Motion only through Agent D's ``nav/navigate_to_view`` (NavigateToView). HOLD = cancel the active goal and
send no new one (the mission has no velocity output; the gate then zeroes on COMMAND_STALE).
``ops/hold`` is the single OPERATOR-owned twist_mux lock (integration decision): by default
(``assert_hold_lock: false``) the mission only reads it as an external hold and never writes it, so a
mission release can never clear an operator's hold. ``assert_hold_lock: true`` restores E's original
behaviour (true at ``hold_publish_hz``, one false on release) for single-writer test setups only.

Inputs: ``state/localization`` (LocalizationState: VALID, map id/version, pose in map),
``map`` (OccupancyGrid, transient local), ``perception/ladder_candidates`` (latched),
``ops/operator_enable`` (OperatorEnable heartbeat), ``ops/hold`` (external holds).
Outputs: ``mission/status``, ``mission/events``, ``mission/coverage``, ``mission/frontiers``,
``ops/hold`` (only with ``assert_hold_lock``). Services: ``mission/hold`` (SetBool), ``mission/resume``/``mission/abort`` (Trigger),
``mission/set_patrol_zone`` (SetPatrolZone). Capture via ``mission/capture_views`` (CaptureViews).
"""
import json
import math
import os
import pathlib
import sys
import threading
import time

from .coverage_reporter import compute_coverage
from .frontier_explorer import find_frontiers
from .grid import GridMap
from .mission_core import TERMINAL, Inputs, MissionConfig, MissionCore
from .zone import PatrolZone, load_route, load_zone

VALID_MODES = ('PATROL', 'EXPLORE', 'SURVEY_ROUTE')


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(argv=None):
    import rclpy
    from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Point32, Polygon, PoseArray, PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool, Trigger
    from ultron_b2_interfaces.action import CaptureViews, NavigateToView, RunMission
    from ultron_b2_interfaces.msg import (CoverageReport, LadderCandidateArray, LocalizationState, MissionEvent,
                                          MissionStatus, OperatorEnable)
    from ultron_b2_interfaces.srv import SetPatrolZone

    from ultron_b2_mission.memory_bridge import MemoryBridge

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status_names = {GoalStatus.STATUS_SUCCEEDED: 'REACHED', GoalStatus.STATUS_CANCELED: 'CANCELLED',
                    GoalStatus.STATUS_ABORTED: 'FAILED'}

    def pose_stamped(frame, stamp, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id, ps.header.stamp = frame, stamp
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return ps

    class MissionManager(Node):
        def __init__(self):
            super().__init__('mission_manager')
            p = self.declare_parameter
            self.params = {
                'zone_file': p('zone_file', '').value, 'route_file': p('route_file', '').value,
                'map_origin_in_world': list(p('map_origin_in_world', [0.0, 0.0, 0.0]).value),
                'default_mode': p('default_mode', 'EXPLORE').value,
                'max_duration_s': float(p('max_duration_s', 900.0).value),
                'detect_dwell_s': float(p('detect_dwell_s', 2.0).value),
                'settle_dwell_s': float(p('settle_dwell_s', 0.5).value),
                'views_per_ladder': int(p('views_per_ladder', 2).value),
                'hold_timeout_s': float(p('hold_timeout_s', 120.0).value),
                'loc_loss_timeout_s': float(p('loc_loss_timeout_s', 30.0).value),
                'operator_enable_timeout_s': float(p('operator_enable_timeout_s', 1.0).value),
                'localization_timeout_s': float(p('localization_timeout_s', 1.0).value),
                'require_operator_enable': bool(p('require_operator_enable', True).value),
                'nav_timeout_s': float(p('nav_timeout_s', 90.0).value),
                'capture_timeout_s': float(p('capture_timeout_s', 20.0).value),
                'clearance_m': float(p('clearance_m', 0.45).value),
            }
            self.use_candidate_yaw = bool(p('use_candidate_yaw', False).value)
            self.memory_db = p('memory_db', '').value
            self.run_root = p('run_dir', os.path.expanduser('~/.ultron/missions')).value
            self.map_frame = p('map_frame', 'map').value
            hold_topic = p('hold_topic', 'ops/hold').value
            self.assert_hold_lock = bool(p('assert_hold_lock', False).value)
            self.hold_period = 1.0 / max(0.5, float(p('hold_publish_hz', 5.0).value))
            tick_hz = float(p('tick_hz', 10.0).value)

            self.lock = threading.RLock()
            self.cb = ReentrantCallbackGroup()
            self.zone = None
            self.zone_map = None
            self.route = []
            self.grid = None
            self.inputs = Inputs()
            self.core = None
            self.memory = None
            self.coverage = None
            self.nav_handles, self.cap_handles = {}, {}
            self.cancel_requested = set()
            self.asserting_hold = False
            self.last_hold_pub = 0.0
            self.event_seq = 0
            self._load_static_config()

            self.hold_pub = self.create_publisher(Bool, hold_topic, 10)
            self.status_pub = self.create_publisher(MissionStatus, 'mission/status', 10)
            self.event_pub = self.create_publisher(MissionEvent, 'mission/events', 50)
            self.coverage_pub = self.create_publisher(CoverageReport, 'mission/coverage', latched)
            self.frontier_pub = self.create_publisher(PoseArray, 'mission/frontiers', 10)
            self.create_subscription(LocalizationState, 'state/localization', self._on_loc, 10)
            self.create_subscription(OccupancyGrid, 'map', self._on_map, latched)
            self.create_subscription(LadderCandidateArray, 'perception/ladder_candidates', self._on_cands, latched)
            self.create_subscription(OperatorEnable, 'ops/operator_enable', self._on_operator, 10)
            self.create_subscription(Bool, hold_topic, self._on_hold_topic, 10)
            self.nav = ActionClient(self, NavigateToView, 'nav/navigate_to_view', callback_group=self.cb)
            self.capture = ActionClient(self, CaptureViews, 'mission/capture_views', callback_group=self.cb)
            self.create_service(SetBool, 'mission/hold', self._srv_hold, callback_group=self.cb)
            self.create_service(Trigger, 'mission/resume', self._srv_resume, callback_group=self.cb)
            self.create_service(Trigger, 'mission/abort', self._srv_abort, callback_group=self.cb)
            self.create_service(SetPatrolZone, 'mission/set_patrol_zone', self._srv_zone, callback_group=self.cb)
            self.server = ActionServer(self, RunMission, 'mission/run', execute_callback=self._execute,
                                       goal_callback=self._goal_cb, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                       callback_group=self.cb)
            self.create_timer(1.0 / tick_hz, self._tick, callback_group=self.cb)
            self.create_timer(1.0, self._slow, callback_group=self.cb)

        # ------------------------------------------------------------ config
        def _load_static_config(self):
            if self.params['zone_file']:
                try:
                    self.zone = load_zone(self.params['zone_file'])
                except Exception as exc:
                    self.get_logger().error(f'zone_file unusable: {exc}')
            if self.params['route_file']:
                try:
                    self.route = load_route(self.params['route_file'], self.params['map_origin_in_world'])
                except Exception as exc:
                    self.get_logger().error(f'route_file unusable: {exc}')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        # ------------------------------------------------------------ inputs
        def _on_loc(self, msg):
            with self.lock:
                i = self.inputs
                stale = msg.age_limit_s > 0 and msg.age_s > msg.age_limit_s
                i.loc_state = 'STALE' if (msg.state == 'VALID' and stale) else msg.state
                i.loc_map = (msg.map_id, int(msg.map_version))
                i.loc_stamp = self.now()
                if msg.header.frame_id and msg.header.frame_id != self.map_frame:
                    i.loc_state = 'WRONG_FRAME'
                i.robot = (msg.pose.position.x, msg.pose.position.y, yaw_from_quat(msg.pose.orientation))

        def _on_map(self, msg):
            try:
                grid = GridMap.from_msg(msg)
            except Exception as exc:
                self.get_logger().error(f'map rejected: {exc}')
                return
            with self.lock:
                self.grid = grid if grid.frame_id == self.map_frame else None
                self.inputs.grid = self.grid

        def _on_cands(self, msg):
            cands = []
            for c in msg.candidates:
                cands.append({'candidate_id': c.candidate_id, 'asset_id': c.asset_id,
                              'position': [c.position.x, c.position.y, c.position.z],
                              'yaw': c.yaw if self.use_candidate_yaw else None, 'ladder_type': c.ladder_type,
                              'state': c.association_state, 'from_oracle': c.from_oracle,
                              'map': (c.map_id, int(c.map_version))})
            with self.lock:
                self.inputs.candidates = cands

        def _on_operator(self, msg):
            with self.lock:
                self.inputs.operator_enabled = bool(msg.enabled)
                self.inputs.operator_stamp = self.now()

        def _on_hold_topic(self, msg):
            with self.lock:
                if self.asserting_hold and msg.data:
                    return  # indistinguishable from our own lock echo
                self.inputs.external_hold = bool(msg.data)

        # ------------------------------------------------------------ services
        def _srv_hold(self, req, res):
            with self.lock:
                if self.core is None:
                    res.success, res.message = False, 'no active mission'
                elif req.data:
                    self.core.request_hold()
                    self._tick_locked()
                    res.success, res.message = True, 'hold requested'
                else:
                    res.success, res.message = self.core.resume()
                    self._flush()
            return res

        def _srv_resume(self, req, res):
            with self.lock:
                if self.core is None:
                    res.success, res.message = False, 'no active mission'
                else:
                    res.success, res.message = self.core.resume()
                    self._flush()
            return res

        def _srv_abort(self, req, res):
            with self.lock:
                if self.core is None or self.core.state in TERMINAL:
                    res.success, res.message = False, 'no active mission'
                else:
                    self.core.abort('Operator abort via mission/abort')
                    self._flush()
                    res.success, res.message = True, 'aborted'
            return res

        def _srv_zone(self, req, res):
            with self.lock:
                if self.core is not None and self.core.state not in TERMINAL:
                    res.success, res.message = False, 'mission active; zone cannot change mid-mission'
                    return res
                try:
                    keep = [[(pt.x, pt.y) for pt in k.points] for k in req.keepout]
                    base = None
                    if req.base_pose.header.frame_id:
                        if req.base_pose.header.frame_id != self.map_frame:
                            raise ValueError('base_pose must be in the map frame')
                        bp = req.base_pose.pose
                        base = (bp.position.x, bp.position.y, yaw_from_quat(bp.orientation))
                    self.zone = PatrolZone(req.zone_id or 'zone', [(pt.x, pt.y) for pt in req.boundary.points],
                                           keep, base)
                    self.zone_map = (req.map_id, int(req.map_version))
                    res.success, res.message = True, f'zone {self.zone.zone_id} set ({self.zone.area_m2():.1f} m2)'
                except Exception as exc:
                    res.success, res.message = False, str(exc)
            res.schema_version = 'ultron.set_patrol_zone/v1'
            return res

        # ------------------------------------------------------------ RunMission
        def _goal_cb(self, goal):
            with self.lock:
                if self.core is not None and self.core.state not in TERMINAL:
                    self.get_logger().warning('RunMission rejected: mission already active')
                    return GoalResponse.REJECT
                if self.zone is None:
                    self.get_logger().warning('RunMission rejected: no patrol zone')
                    return GoalResponse.REJECT
                if self.zone_map and goal.map_id and self.zone_map != (goal.map_id, int(goal.map_version)):
                    self.get_logger().warning('RunMission rejected: patrol zone set for another map')
                    return GoalResponse.REJECT
                if goal.mode and goal.mode not in VALID_MODES:
                    self.get_logger().warning(f'RunMission rejected: mode {goal.mode!r}')
                    return GoalResponse.REJECT
            return GoalResponse.ACCEPT

        def _execute(self, goal_handle):
            g = goal_handle.request
            with self.lock:
                cfg = MissionConfig(**{k: v for k, v in self.params.items()
                                       if k in MissionConfig.__dataclass_fields__})
                cfg.mode = g.mode or self.params['default_mode']
                if g.max_duration_s > 0:
                    cfg.max_duration_s = float(g.max_duration_s)
                cfg.view.robot_radius_m = cfg.clearance_m
                map_id = g.map_id or (self.inputs.loc_map[0] if self.inputs.loc_map else '')
                map_version = g.map_version if g.map_id else (self.inputs.loc_map[1] if self.inputs.loc_map else 0)
                if self.memory is not None:
                    self.memory.close()
                run_dir = ''
                self.memory = None
                try:
                    stamp = time.strftime('%Y%m%dT%H%M%S')
                    run_dir = str(pathlib.Path(self.run_root) / f'{stamp}_{g.mission_id or "mission"}') \
                        if self.run_root else ''
                    self.memory = MemoryBridge(self.memory_db, run_dir)
                    mission_id = self.memory.start(f'{map_id}@{int(map_version)}', g.mission_id, self.zone)
                except Exception as exc:
                    self.get_logger().error(f'memory bridge unavailable ({exc}); mission runs without persistence')
                    self.memory = None
                    mission_id = g.mission_id or 'MISSION'
                route = self.route
                self.core = MissionCore(cfg, self.zone, map_id=map_id, map_version=map_version, route=route,
                                        memory=self.memory, mission_id=mission_id)
                self.core.coverage_complete = lambda: bool(self.coverage and self.coverage.complete)
                self.core.start(self.now(), self.inputs)
                self._flush()
                self.run_dir = run_dir
            self.get_logger().info(f'mission {mission_id} started mode={cfg.mode} map={map_id}@{map_version}')
            while rclpy.ok():
                with self.lock:
                    core = self.core
                    if goal_handle.is_cancel_requested and core.state not in TERMINAL:
                        core.abort('RunMission cancelled by client')
                        self._flush()
                    if core.state in TERMINAL:
                        break
                    goal_handle.publish_feedback(RunMission.Feedback(status=self._status_msg()))
                time.sleep(0.2)
            with self.lock:
                self._update_coverage()
                status, reason = core.result or ('ABORTED', 'shutdown')
                summary = core.summary()
                summary['coverage'] = self.coverage.__dict__ if self.coverage else None
                summary_ref = ''
                if self.memory is not None:
                    summary['not_inspected_this_mission'] = self.memory.finish(
                        {'status': status, 'reason': reason, 'ladders_captured': summary['ladders_captured'],
                         'ladders_unreachable': summary['ladders_unreachable']})
                if self.run_dir:
                    path = pathlib.Path(self.run_dir) / 'mission_summary.json'
                    path.write_text(json.dumps(summary, indent=2, default=str))
                    summary_ref = str(path)
                result = RunMission.Result(schema_version='ultron.run_mission_result/v1', status=status,
                                           reason=reason, coverage=self._coverage_msg(), summary_ref=summary_ref)
            if status == 'ABORTED' and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif status == 'ABORTED':
                goal_handle.abort()
            else:
                goal_handle.succeed()
            self.get_logger().info(f'mission {core.mission_id} ended: {status} ({reason})')
            return result

        # ------------------------------------------------------------ periodic
        def _tick(self):
            with self.lock:
                self._tick_locked()
                if self.asserting_hold and time.monotonic() - self.last_hold_pub >= self.hold_period:
                    self._publish_hold(True)

        def _tick_locked(self):
            if self.core is None:
                return
            # Never treat candidates from another map scope as targets.
            mid = (self.core.map_id, self.core.map_version)
            inp = self.inputs
            snapshot = Inputs(grid=inp.grid, robot=inp.robot,
                              candidates=[c for c in inp.candidates if c['map'] == mid],
                              loc_state=inp.loc_state, loc_map=inp.loc_map, loc_stamp=inp.loc_stamp,
                              operator_enabled=inp.operator_enabled, operator_stamp=inp.operator_stamp,
                              external_hold=inp.external_hold)
            self.core.tick(self.now(), snapshot)
            self._flush()

        def _slow(self):
            with self.lock:
                if self.core is not None and self.core.state not in TERMINAL:
                    self._update_coverage()
                    self.coverage_pub.publish(self._coverage_msg())
                self.status_pub.publish(self._status_msg())

        def _update_coverage(self):
            core, grid, robot = self.core, self.inputs.grid, self.inputs.robot
            if core is None or grid is None or robot is None or self.zone is None:
                return
            reach = grid.reachable_from(robot[:2], core.cfg.clearance_m)
            s = core.summary()
            self.coverage = compute_coverage(grid, self.zone, reach, ladders_pending=s['ladders_pending'],
                                             ladders_unreachable=sorted(s['ladders_unreachable']),
                                             ladders_missing=s['ladders_missing'])
            fr, _ = find_frontiers(grid, self.zone, robot[:2], clearance_m=core.cfg.clearance_m, reach=reach,
                                   blacklist=core.frontier_blacklist)
            pa = PoseArray()
            pa.header.frame_id, pa.header.stamp = self.map_frame, self.get_clock().now().to_msg()
            for f in fr:
                pa.poses.append(pose_stamped(self.map_frame, pa.header.stamp, *f.goal).pose)
            self.frontier_pub.publish(pa)

        # ------------------------------------------------------------ command execution
        def _flush(self):
            core = self.core
            if core is None:
                return
            # Commands may synchronously produce more commands (e.g. server unavailable => next goal).
            for _ in range(100):
                cmds = core.drain()
                if not cmds:
                    break
                for kind, d in cmds:
                    getattr(self, '_cmd_' + kind.lower())(d)
            for ev in core.drain_events():
                self.event_seq += 1
                self.event_pub.publish(MissionEvent(
                    schema_version='ultron.mission_event/v1', mission_id=core.mission_id,
                    event_id=f'{core.mission_id}-E{self.event_seq:05}', event_type=ev['event_type'],
                    from_state=ev['from_state'], to_state=ev['to_state'], reason=ev['reason'],
                    asset_id=ev['asset_id'] or '', goal_id=ev['goal_id'] or '',
                    payload_json=json.dumps(ev['payload'], default=str)))

        def _publish_hold(self, on):
            self.hold_pub.publish(Bool(data=bool(on)))
            self.last_hold_pub = time.monotonic()

        def _cmd_hold(self, d):
            if not self.assert_hold_lock:
                return  # ops/hold is operator-owned; the mission HOLD is the goal cancel only
            self.asserting_hold = bool(d['on'])
            self._publish_hold(d['on'])  # release: a single false (see CONTRACT_CHANGES on lock ownership)

        def _cmd_nav(self, d):
            gid = d['goal_id']
            if not self.nav.server_is_ready():
                self.core.on_nav_result(gid, 'FAILED', 'NAV_SERVER_UNAVAILABLE: nav/navigate_to_view')
                return
            goal = NavigateToView.Goal(schema_version='ultron.navigate_to_view/v1', goal_id=gid,
                                       mission_id=d['mission_id'], map_id=d['map_id'],
                                       map_version=int(d['map_version']),
                                       target=pose_stamped(self.map_frame, self.get_clock().now().to_msg(),
                                                           d['x'], d['y'], d['yaw']),
                                       xy_tolerance_m=d['xy_tolerance_m'], yaw_tolerance_rad=d['yaw_tolerance_rad'],
                                       timeout_s=d['timeout_s'], reason=d['reason'], asset_id=d['asset_id'])
            fut = self.nav.send_goal_async(goal)
            fut.add_done_callback(lambda f, gid=gid: self._nav_accepted(gid, f))

        def _nav_accepted(self, gid, fut):
            with self.lock:
                try:
                    handle = fut.result()
                except Exception as exc:
                    self.core.on_nav_result(gid, 'FAILED', f'send failed: {exc}')
                    self._flush()
                    return
                if not handle.accepted:
                    self.core.on_nav_result(gid, 'REJECTED', 'goal rejected by nav server')
                    self._flush()
                    return
                self.nav_handles[gid] = handle
                if gid in self.cancel_requested:
                    handle.cancel_goal_async()
                handle.get_result_async().add_done_callback(lambda f, gid=gid: self._nav_done(gid, f))

        def _nav_done(self, gid, fut):
            with self.lock:
                self.nav_handles.pop(gid, None)
                try:
                    wrapped = fut.result()
                    r = wrapped.result
                    status = r.status or status_names.get(wrapped.status, 'FAILED')
                    self.core.on_nav_result(gid, status, r.reason, settled=bool(r.settled))
                except Exception as exc:
                    self.core.on_nav_result(gid, 'FAILED', f'result error: {exc}')
                self._flush()

        def _cmd_cancel_nav(self, d):
            gid = d['goal_id']
            handle = self.nav_handles.get(gid)
            if handle is not None:
                handle.cancel_goal_async()
            else:
                self.cancel_requested.add(gid)

        def _cmd_capture(self, d):
            cid = d['capture_id']
            if not self.capture.server_is_ready():
                self.core.on_capture_result(cid, 'FAILED', 'CAPTURE_SERVER_UNAVAILABLE: mission/capture_views', 0, [])
                return
            x, y, yaw = d['view']
            goal = CaptureViews.Goal(schema_version='ultron.capture_views/v1', capture_id=cid,
                                     mission_id=d['mission_id'], asset_id=d['asset_id'], map_id=d['map_id'],
                                     map_version=int(d['map_version']),
                                     view_poses=[pose_stamped(self.map_frame, self.get_clock().now().to_msg(),
                                                              x, y, yaw)],
                                     modalities=list(d['modalities']))
            fut = self.capture.send_goal_async(goal)
            fut.add_done_callback(lambda f, cid=cid: self._cap_accepted(cid, f))

        def _cap_accepted(self, cid, fut):
            with self.lock:
                try:
                    handle = fut.result()
                except Exception as exc:
                    self.core.on_capture_result(cid, 'FAILED', f'send failed: {exc}', 0, [])
                    self._flush()
                    return
                if not handle.accepted:
                    self.core.on_capture_result(cid, 'FAILED', 'capture rejected', 0, [])
                    self._flush()
                    return
                self.cap_handles[cid] = handle
                handle.get_result_async().add_done_callback(lambda f, cid=cid: self._cap_done(cid, f))

        def _cap_done(self, cid, fut):
            with self.lock:
                self.cap_handles.pop(cid, None)
                try:
                    r = fut.result().result
                    self.core.on_capture_result(cid, r.status or 'FAILED', r.reason, r.views_captured,
                                                list(r.evidence_refs))
                except Exception as exc:
                    self.core.on_capture_result(cid, 'FAILED', f'result error: {exc}', 0, [])
                self._flush()

        def _cmd_cancel_capture(self, d):
            handle = self.cap_handles.get(d['capture_id'])
            if handle is not None:
                handle.cancel_goal_async()

        # ------------------------------------------------------------ messages
        def _status_msg(self):
            m = MissionStatus(schema_version=MissionStatus.SCHEMA_VERSION)
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.map_frame
            core = self.core
            if core is None:
                m.state = 'IDLE'
                return m
            s = core.summary()
            m.mission_id, m.state, m.supervisor_action, m.reason = core.mission_id, core.state, core.action, core.reason
            m.active_asset_id = core.target['key'] if core.target else ''
            m.active_goal_id = core.active_goal['goal_id'] if core.active_goal else ''
            m.map_id, m.map_version = core.map_id, core.map_version
            m.hold = core.state in ('HOLD', 'AWAIT_OPERATOR')
            m.ladders_known, m.ladders_captured = s['ladders_known'], len(s['ladders_captured'])
            return m

        def _coverage_msg(self):
            m = CoverageReport(schema_version='ultron.coverage_report/v1')
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.map_frame
            if self.core is not None:
                m.mission_id, m.map_id, m.map_version = self.core.mission_id, self.core.map_id, self.core.map_version
            if self.zone is not None:
                m.zone_id = self.zone.zone_id
            c = self.coverage
            if c is None:
                m.reason = 'Coverage not computed (no map/pose)'
                return m
            m.zone_area_m2, m.observed_area_m2 = c.zone_area_m2, c.observed_area_m2
            m.unknown_area_m2, m.unreachable_area_m2 = c.unknown_area_m2, c.unreachable_area_m2
            m.coverage_fraction, m.gap_ids, m.complete, m.reason = c.coverage_fraction, c.gap_ids, c.complete, c.reason
            m.gap_polygons = [Polygon(points=[Point32(x=float(x), y=float(y)) for x, y in poly])
                              for poly in c.gap_polygons]
            return m

    rclpy.init(args=argv)
    node = MissionManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with node.lock:
            if node.memory is not None:
                node.memory.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
