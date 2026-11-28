"""End-to-end mission run against a MOCK NavigateToView server (no Nav2, no simulator).

Runs the real ``mission_manager`` and ``capture_coordinator`` executables as subprocesses and a
mock world in this process: latched OccupancyGrid, LocalizationState (VALID, pose teleported on
REACHED), OperatorEnable heartbeat, latched ORACLE ladder candidates, synthetic camera frames and
a mock ``nav/navigate_to_view`` server. Checks: ladders captured, evidence written, registry and
events.jsonl updated, HOLD on operator-enable loss cancels the active goal and never writes the operator-owned ops/hold,
resume continues to the end, no velocity publisher on the mission nodes. This is not a navigation
or perception result; the mock teleports the robot.
"""
import json
import math
import os
import pathlib
import shutil
import signal
import sqlite3
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

import numpy as np  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
NAV_DELAY_S = 0.4
NAV_CANCEL_DELAY_S = 2.0


@pytest.fixture
def artifacts():
    path = HERE / '.artifacts' / 'e2e_mock'
    shutil.rmtree(path, ignore_errors=True)
    path.mkdir(parents=True)
    yield path
    shutil.rmtree(path, ignore_errors=True)
    try:
        path.parent.rmdir()
    except OSError:
        pass


class MockWorld(Node):
    def __init__(self, asset_ids):
        super().__init__('mock_world_e2e')
        from geometry_msgs.msg import Twist  # noqa: F401
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import Image
        from std_msgs.msg import Bool
        from ultron_b2_interfaces.action import NavigateToView, RunMission
        from ultron_b2_interfaces.msg import (LadderCandidate, LadderCandidateArray, LocalizationState,
                                              MissionEvent, MissionStatus, OperatorEnable)
        self.msgs = dict(OccupancyGrid=OccupancyGrid, Image=Image, LocalizationState=LocalizationState,
                         OperatorEnable=OperatorEnable, LadderCandidate=LadderCandidate,
                         LadderCandidateArray=LadderCandidateArray, NavigateToView=NavigateToView)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.robot = [1.0, 1.0, 0.0]
        self.asset_ids = asset_ids
        self.operator_enabled = True
        self.goals, self.cancels, self.holds, self.states = [], [], [], []
        self.nav_busy = False
        self.busy_rejections = 0
        self.stop_acknowledged = True
        self.events = []
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', latched)
        self.loc_pub = self.create_publisher(LocalizationState, 'state/localization', 10)
        self.op_pub = self.create_publisher(OperatorEnable, 'ops/operator_enable', 10)
        self.cand_pub = self.create_publisher(LadderCandidateArray, 'perception/ladder_candidates', latched)
        self.color_pub = self.create_publisher(Image, 'sensors/cam_head/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'sensors/cam_head/depth/image_rect_raw', 10)
        self.create_subscription(Bool, 'ops/hold', lambda m: self.holds.append((time.monotonic(), m.data)), 10)
        self.create_subscription(MissionStatus, 'mission/status', lambda m: self.states.append(m.state), 10)
        self.create_subscription(MissionEvent, 'mission/events', self.events.append, 50)
        self.nav = ActionServer(self, NavigateToView, 'nav/navigate_to_view', execute_callback=self._nav_exec,
                                goal_callback=self._nav_goal,
                                cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.run_client = ActionClient(self, RunMission, 'mission/run', callback_group=cb)
        self.create_timer(0.1, self._publish, callback_group=cb)
        self._publish_static()

    def _nav_goal(self, goal):
        from ultron_b2_contracts import ContractError, msg_to_navigation_goal
        try:
            msg_to_navigation_goal(goal)
        except ContractError:
            return GoalResponse.REJECT
        with self.lock:
            if self.nav_busy:
                self.busy_rejections += 1
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _publish_static(self):
        OccupancyGrid = self.msgs['OccupancyGrid']
        res, w, h = 0.1, 100, 80
        data = np.zeros((h, w), dtype=np.int8)
        data[0, :] = data[-1, :] = 100
        data[:, 0] = data[:, -1] = 100
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.info.resolution, g.info.width, g.info.height = res, w, h
        g.info.origin.orientation.w = 1.0
        g.data = data.reshape(-1).tolist()
        self.map_pub.publish(g)
        C, A = self.msgs['LadderCandidate'], self.msgs['LadderCandidateArray']
        arr = A(schema_version=A.SCHEMA_VERSION)
        arr.header.frame_id = 'map'
        for cid, (x, y, t) in {'LAD-901': (5.0, 4.0, 'TRESTLE'),
                               'LAD-902': (7.5, 2.5, 'LAYHER_A_FRAME_STEPLADDER')}.items():
            c = C(schema_version=C.SCHEMA_VERSION, candidate_id=cid, asset_id=self.asset_ids[cid],
                  map_id='hall', map_version=1,
                  ladder_type=t, association_state='NEW', from_oracle=True)
            c.position.x, c.position.y = x, y
            arr.candidates.append(c)
        self.cand_pub.publish(arr)

    def _publish(self):
        L, O, Image = self.msgs['LocalizationState'], self.msgs['OperatorEnable'], self.msgs['Image']
        now = self.get_clock().now().to_msg()
        with self.lock:
            x, y, yaw = self.robot
            enabled = self.operator_enabled
        m = L(schema_version='ultron.localization_state/v1', map_id='hall', map_version=1, state='VALID',
              age_s=0.0, age_limit_s=1.0)
        m.header.frame_id, m.header.stamp = 'map', now
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.loc_pub.publish(m)
        op = O(schema_version='ultron.operator_enable/v1', enabled=enabled, operator_id='test')
        op.header.stamp = now
        self.op_pub.publish(op)
        for pub, enc, arr in ((self.color_pub, 'rgb8', np.zeros((8, 8, 3), np.uint8)),
                              (self.depth_pub, '16UC1', np.full((8, 8), 2000, np.uint16))):
            img = Image(encoding=enc, height=8, width=8, is_bigendian=0, step=int(arr.strides[0]))
            img.header.stamp, img.header.frame_id = now, 'cam_head_color_optical_frame'
            img.data = arr.tobytes()
            pub.publish(img)

    def _nav_exec(self, gh):
        NavigateToView = self.msgs['NavigateToView']
        g = gh.request
        with self.lock:
            self.nav_busy = True
            self.goals.append({'goal_id': g.goal_id, 'reason': g.reason, 'map_id': g.map_id,
                               'map_version': g.map_version, 'frame': g.target.header.frame_id,
                               'x': g.target.pose.position.x, 'y': g.target.pose.position.y})
        t_end = time.monotonic() + NAV_DELAY_S
        while time.monotonic() < t_end:
            if gh.is_cancel_requested:
                with self.lock:
                    self.cancels.append(g.goal_id)
                time.sleep(NAV_CANCEL_DELAY_S)
                with self.lock:
                    self.nav_busy = False
                gh.canceled()
                return NavigateToView.Result(status='CANCELLED', reason='cancel requested',
                                             stop_acknowledged=self.stop_acknowledged)
            time.sleep(0.02)
        q = g.target.pose.orientation
        with self.lock:
            self.robot = [g.target.pose.position.x, g.target.pose.position.y,
                          math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)]
            self.nav_busy = False
        gh.succeed()
        return NavigateToView.Result(status='REACHED', reason='mock teleport', settled=True,
                                     stop_acknowledged=True)


def _spawn(module, params, env):
    args = [sys.executable, '-m', module, '--ros-args']
    for k, v in params.items():
        args += ['-p', f'{k}:={v}']
    return subprocess.Popen(args, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def _stop(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
    return proc.stdout.read() if proc.stdout else ''


@pytest.mark.parametrize('stop_acknowledged', [True, False])
def test_mission_end_to_end_against_mock_nav_server(artifacts, stop_acknowledged):
    from ultron_b2_interfaces.action import RunMission
    from ultron_b2_perception.registry_store import open_registry
    zone = artifacts / 'zone.yaml'
    zone.write_text('zone_id: TEST\nframe: map\nboundary: [[0,0],[10,0],[10,8],[0,8]]\nbase_pose: [1.0, 1.0, 0.0]\n')
    registry = open_registry(artifacts / 'registry.sqlite')
    asset_ids = {
        'LAD-901': registry.associate((5.0, 4.0, 0.0), 'TRESTLE', 'hall@1')[0],
        'LAD-902': registry.associate((7.5, 2.5, 0.0), 'LAYHER_A_FRAME_STEPLADDER', 'hall@1')[0],
    }
    registry.close()
    env = dict(os.environ)
    mission = _spawn('ultron_b2_mission.mission_manager', {
        'zone_file': str(zone), 'memory_db': str(artifacts / 'registry.sqlite'), 'run_dir': str(artifacts / 'runs'),
        'detect_dwell_s': 0.2, 'settle_dwell_s': 0.2, 'hold_timeout_s': 60.0}, env)
    capture = _spawn('ultron_b2_mission.capture_coordinator', {'evidence_dir': str(artifacts / 'evidence')}, env)
    rclpy.init()
    world = MockWorld(asset_ids)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(world)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    logs = ''
    try:
        assert world.run_client.wait_for_server(timeout_sec=30.0), 'mission/run not available'
        time.sleep(1.0)  # latched map/candidates delivered
        goal = RunMission.Goal(schema_version='ultron.run_mission/v1', mission_id='E2E', zone_id='TEST',
                               map_id='hall', map_version=1, mode='EXPLORE', max_duration_s=120.0)
        send = world.run_client.send_goal_async(goal)
        deadline = time.monotonic() + 15
        while not send.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        handle = send.result()
        assert handle.accepted
        result_fut = handle.get_result_async()

        # Wait for the first INSPECT_VIEW goal, then drop the operator enable mid-goal => HOLD.
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and not any(g['reason'] == 'INSPECT_VIEW' for g in world.goals):
            time.sleep(0.02)
        assert world.goals, 'no NavigateToView goal received'
        with world.lock:
            world.stop_acknowledged = stop_acknowledged
            world.operator_enabled = False
            n_goals = len(world.goals)
            held_goal = world.goals[-1]['goal_id']
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and held_goal not in world.cancels:
            time.sleep(0.02)
        with world.lock:
            assert held_goal in world.cancels, 'active goal was not cancelled on HOLD'
            assert len(world.goals) == n_goals, 'goal sent while operator enable was lost'
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and 'HOLD' not in world.states:
            time.sleep(0.02)
        assert 'HOLD' in world.states
        assert not world.holds, 'mission wrote the operator-owned ops/hold lock (assert_hold_lock defaults false)'
        with world.lock:
            world.operator_enabled = True
        time.sleep(0.5)
        with world.lock:
            assert world.nav_busy, 'mock cancellation completed before overlap was exercised'
            assert len(world.goals) == n_goals and world.busy_rejections == 0, \
                'replacement goal sent before cancellation completed'
        if not stop_acknowledged:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and 'AWAIT_OPERATOR' not in world.states:
                time.sleep(0.05)
            assert 'AWAIT_OPERATOR' in world.states, 'unconfirmed stop did not fail closed'
            with world.lock:
                assert len(world.goals) == n_goals and world.busy_rejections == 0
            handle.cancel_goal_async()
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and not result_fut.done():
                time.sleep(0.05)
            assert result_fut.done(), 'mission did not exit after cancellation'
            return

        deadline = time.monotonic() + 90
        while not result_fut.done() and time.monotonic() < deadline:
            time.sleep(0.1)
        assert result_fut.done(), 'mission did not finish'
        res = result_fut.result().result
        assert res.status in ('DONE', 'DONE_WITH_GAPS'), (res.status, res.reason)
        assert world.busy_rejections == 0
        summary = json.loads(pathlib.Path(res.summary_ref).read_text())
        assert len(summary['ladders_captured']) == 2, summary
        assert summary['oracle_detections_used'] is True
        assert world.goals[-1]['reason'].startswith('RETURN_BASE')
        assert all(g['map_id'] == 'hall' and g['map_version'] == 1 and g['frame'] == 'map' for g in world.goals)
        assert res.coverage.zone_area_m2 > 0 and res.coverage.observed_area_m2 > 0
        assert world.events and all(e.header.stamp.sec > 0 for e in world.events)

        evidence = list((artifacts / 'evidence').rglob('*.npy'))
        assert len(evidence) >= 4 * 2  # 2 ladders x 2 views x (color, depth)
        run_dirs = list((artifacts / 'runs').iterdir())
        events = [json.loads(line) for line in (run_dirs[0] / 'events.jsonl').read_text().splitlines()]
        types = [e['type'] for e in events]
        assert types[0] == 'MISSION_START' and types[-1] == 'MISSION_FINISHED'
        assert 'CAPTURE_COMMITTED' in types and 'ASSESS_REQUESTED' in types
        db = sqlite3.connect(str(artifacts / 'registry.sqlite'))
        rows = db.execute('SELECT payload FROM inspections').fetchall()
        assert db.execute('SELECT count(*) FROM missions').fetchone()[0] == 1
        db.close()
        assert len(rows) == 2 and all(json.loads(r[0])['verdict'] == 'PENDING_ASSESSMENT' for r in rows)
        commits = [e for e in events if e['type'] == 'CAPTURE_COMMITTED']
        assert all(e['verdict'] == 'PENDING_ASSESSMENT' and e['human_review_required'] for e in commits)
        assert all(e['detection_source'] == 'ORACLE' for e in commits)

        # Mission nodes never publish velocities.
        for name, ns in world.get_node_names_and_namespaces():
            if name in ('mission_manager', 'capture_coordinator'):
                pubs = world.get_publisher_names_and_types_by_node(name, ns)
                assert not [t for _, types_ in pubs for t in types_ if 'Twist' in t], pubs
    finally:
        ex.shutdown()
        world.destroy_node()
        rclpy.shutdown()
        logs += _stop(mission) + _stop(capture)
        print(logs[-6000:])
