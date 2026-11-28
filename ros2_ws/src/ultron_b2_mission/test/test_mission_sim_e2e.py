"""FULL-SIM mission e2e (UNRUN until Agents C/D/F land). Skipped unless ULTRON_E2E_SIM=1.

Starts ``ros2 launch ultron_b2_bringup sim.launch.py mode:=mission`` (domain 71, /ultron/sim),
sends the operator-enable heartbeat, sets nothing else, and runs a RunMission EXPLORE goal in the
hall. Expected with C (localization + map), D (nav/navigate_to_view, twist_mux ops/hold) and F
(sim with RGB-D head camera, ground_truth/pose for the ORACLE detector):
* result DONE or DONE_WITH_GAPS with a reason; coverage reported (observed > 0);
* at least one ladder captured, every capture from ORACLE detections (labelled ORACLE);
* a HOLD injected while an active view goal is moving cancels that goal and the mission resumes;
* the gated ``cmd_vel`` is zero while the HOLD is asserted (after a short stop allowance);
* the simulation bridge reports no collision-stop events through mission completion;
* the mission returns to base (RETURN_BASE goal sent) before it ends;
* the mission nodes publish no velocity topics.
Optional env: ULTRON_E2E_LAUNCH_ARGS (extra launch args, e.g. ``map:=<dir> odom_source:=mola viz:=foxglove``),
ULTRON_E2E_OUT (new directory for launch log, events and result; default: no artifacts, log discarded),
ULTRON_E2E_TIMEOUT_S (wall-clock timeout for view and mission result; default 1200).
Simulation results are not hardware evidence and oracle detections are not real perception.
"""
import json
import os
import pathlib
import shlex
import signal
import subprocess
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('ULTRON_E2E_SIM') != '1',
                                reason='UNRUN: full-sim e2e needs C/D/F stacks; set ULTRON_E2E_SIM=1')

NS = '/ultron/sim'
HOLD_STOP_ALLOWANCE_S = 1.2   # = sim_nav_cases accept stop_within_s
HOLD_MAX_SAMPLE_GAP_S = 0.5  # proposed observation continuity limit, not a hardware threshold


def test_full_sim_mission_hall():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, String
    from ultron_b2_interfaces.action import RunMission
    from ultron_b2_interfaces.msg import MissionEvent, MissionStatus, OperatorEnable

    env = dict(os.environ, ROS_DOMAIN_ID='71', ROS_LOCALHOST_ONLY='1')
    out = pathlib.Path(os.environ['ULTRON_E2E_OUT']) if os.environ.get('ULTRON_E2E_OUT') else None
    if out:
        out.mkdir(parents=True, exist_ok=False)
    cmd = ['ros2', 'launch', 'ultron_b2_bringup', 'sim.launch.py', 'mode:=mission'] + \
        shlex.split(os.environ.get('ULTRON_E2E_LAUNCH_ARGS', ''))
    log = open(out / 'launch.log', 'w') if out else subprocess.DEVNULL   # never an unread PIPE (blocks launch)
    launch = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    rclpy.init()
    node = Node('mission_sim_e2e', namespace=NS)
    events = []
    node.create_subscription(MissionEvent, 'mission/events', lambda m: events.append((time.monotonic(), m)), 50)
    states = []
    node.create_subscription(MissionStatus, 'mission/status', lambda m: states.append(m.state), 10)
    op_pub = node.create_publisher(OperatorEnable, 'ops/operator_enable', 10)
    hold_pub = node.create_publisher(Bool, 'ops/hold', 10)
    cmd_vel = []
    node.create_subscription(Twist, 'cmd_vel', lambda m: cmd_vel.append((
        time.monotonic(), m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z)), 50)
    bridge_status = []
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, 'sim/bridge_status',
                             lambda m: bridge_status.append(json.loads(m.data)), latched)
    seq = [0]

    def heartbeat():
        seq[0] += 1
        m = OperatorEnable(schema_version='ultron.operator_enable/v1', enabled=True, operator_id='e2e',
                           sequence=seq[0])
        m.header.stamp = node.get_clock().now().to_msg()
        op_pub.publish(m)
    node.create_timer(0.1, heartbeat)
    client = ActionClient(node, RunMission, 'mission/run')
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    res = None
    held = []
    moving_sample = None
    missing_server_since = None

    def check_server():
        nonlocal missing_server_since
        if client.server_is_ready():
            missing_server_since = None
        elif missing_server_since is None:
            missing_server_since = time.monotonic()
        else:
            assert time.monotonic() - missing_server_since < 5.0, 'mission action server disappeared'

    try:
        assert client.wait_for_server(timeout_sec=120.0), 'mission/run never came up'
        time.sleep(20.0)  # localization VALID + map + perception warm-up
        pubs = node.get_publishers_info_by_topic(f'{NS}/cmd_vel')
        assert [(p.node_namespace, p.node_name) for p in pubs] == [(NS, 'motion_gate')], pubs
        goal = RunMission.Goal(schema_version='ultron.run_mission/v1', mission_id='SIM-E2E', zone_id='HALL-SIM-1',
                               mode='EXPLORE', max_duration_s=900.0)
        send = client.send_goal_async(goal)
        deadline = time.monotonic() + 30
        while not send.done() and time.monotonic() < deadline:
            time.sleep(0.1)
        assert send.done(), 'mission goal response timed out'
        assert send.result().accepted
        result = send.result().get_result_async()
        # Inject HOLD only while a view goal is active and the gated output is moving.
        timeout = float(os.environ.get('ULTRON_E2E_TIMEOUT_S', '1200'))
        deadline = time.monotonic() + timeout
        view_id = None
        while time.monotonic() < deadline and not result.done() and 'AWAIT_OPERATOR' not in states:
            check_server()
            snapshot = list(events)
            views = [(t, e) for t, e in snapshot if e.event_type == 'GOAL_SENT' and e.reason == 'INSPECT_VIEW']
            if views:
                view_time, view = views[-1]
                finished = any(e.goal_id == view.goal_id and e.event_type in ('GOAL_RESULT', 'GOAL_CANCELLED')
                               for _, e in snapshot)
                now = time.monotonic()
                moving = [c for c in cmd_vel if view_time <= c[0] and now - c[0] <= 0.2
                          and any(abs(v) > 1e-4 for v in c[1:])]
                if moving and not finished:
                    view_id, moving_sample = view.goal_id, moving[-1]
                    break
            time.sleep(0.02)
        assert view_id, f'no moving active INSPECT_VIEW goal; last state={states[-1:]}, events saved to {out}'
        hold_t0 = time.monotonic()
        for _ in range(30):
            hold_pub.publish(Bool(data=True))
            time.sleep(0.1)
        hold_t1 = time.monotonic()
        hold_pub.publish(Bool(data=False))
        held = [c for c in cmd_vel if hold_t0 + HOLD_STOP_ALLOWANCE_S <= c[0] <= hold_t1]
        deadline = time.monotonic() + timeout
        while not result.done() and 'AWAIT_OPERATOR' not in states and time.monotonic() < deadline:
            check_server()
            time.sleep(1.0)
        assert result.done(), f'mission result timed out or awaits operator; last state={states[-1:]}'
        res = result.result().result
        assert res.status in ('DONE', 'DONE_WITH_GAPS'), (res.status, res.reason)
        assert res.coverage.observed_area_m2 > 0
        summary = json.loads(pathlib.Path(res.summary_ref).read_text())
        assert summary['ladders_captured'], summary
        assert summary['oracle_detections_used'] is True
        assessed_oracle = {e.asset_id for _, e in events if e.event_type == 'ASSESS_REQUESTED'
                           and json.loads(e.payload_json).get('from_oracle') is True}
        assert set(summary['ladders_captured']) <= assessed_oracle, (summary, assessed_oracle)
        assert any(hold_t0 <= t <= hold_t1 and e.event_type == 'GOAL_CANCELLED'
                   and e.goal_id == view_id for t, e in events), \
            'HOLD did not cancel the selected view goal'
        assert any(t >= hold_t1 and e.event_type == 'RESUMED' for t, e in events), 'mission did not resume'
        assert held, 'no cmd_vel samples while HOLD was asserted'
        assert held[0][0] <= hold_t0 + HOLD_STOP_ALLOWANCE_S + HOLD_MAX_SAMPLE_GAP_S
        assert held[-1][0] >= hold_t1 - HOLD_MAX_SAMPLE_GAP_S
        assert all(b[0] - a[0] <= HOLD_MAX_SAMPLE_GAP_S for a, b in zip(held, held[1:])), \
            'cmd_vel observation gap during HOLD'
        assert all(all(v == 0.0 for v in sample[1:]) for sample in held), ('motion during HOLD', held)
        deadline = time.monotonic() + 5
        end_stamps = []
        while time.monotonic() < deadline and not end_stamps:
            end_stamps = [e.header.stamp.sec + e.header.stamp.nanosec * 1e-9
                          for _, e in events if e.event_type == 'MISSION_END']
            if not end_stamps:
                time.sleep(0.1)
        assert end_stamps, 'missing MISSION_END event'
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not any(
                s.get('sim_time_s', -1) >= end_stamps[-1] for s in bridge_status):
            time.sleep(0.1)
        assert bridge_status and bridge_status[-1].get('sim_time_s', -1) >= end_stamps[-1], \
            'no bridge status through mission completion'
        assert all(type(s.get('collisions')) is int and s['collisions'] == 0 for s in bridge_status), \
            ('simulation collision stops', bridge_status[-1])
        reached = {e.goal_id for _, e in events if e.event_type == 'GOAL_RESULT'
                   and json.loads(e.payload_json).get('status') == 'REACHED'
                   and json.loads(e.payload_json).get('settled') is True}
        assert any(e.event_type == 'GOAL_SENT' and json.loads(e.payload_json).get('kind') == 'EXPLORE'
                   and e.goal_id in reached for _, e in events), 'no settled exploration goal reached'
        assert any(e.event_type == 'GOAL_SENT' and 'RETURN_BASE' in e.reason and e.goal_id in reached
                   for _, e in events), 'no settled RETURN_BASE goal reached'
        for name, ns in node.get_node_names_and_namespaces():
            if ns == NS and name in ('mission_manager', 'capture_coordinator', 'ladder_detector',
                                     'ladder_localizer', 'ladder_tracker'):
                pubs = node.get_publisher_names_and_types_by_node(name, ns)
                assert not [t for _, ts in pubs for t in ts if 'Twist' in t], (name, pubs)
    finally:
        ex.shutdown()
        if out:
            (out / 'events.jsonl').write_text(''.join(json.dumps({
                'receipt_monotonic': t, 'stamp': e.header.stamp.sec + e.header.stamp.nanosec * 1e-9,
                'type': e.event_type, 'from': e.from_state, 'to': e.to_state, 'reason': e.reason,
                'goal_id': e.goal_id, 'payload': e.payload_json})
                + '\n' for t, e in events))
            (out / 'result.json').write_text(json.dumps({
                'status': res.status if res else None, 'reason': res.reason if res else None,
                'summary_ref': res.summary_ref if res else None,
                'observed_area_m2': res.coverage.observed_area_m2 if res else None,
                'pre_hold_cmd_vel': moving_sample[1:] if moving_sample else None,
                'hold_cmd_vel_samples': len(held),
                'bridge_status': bridge_status[-1] if bridge_status else None,
                'hold_cmd_vel_max_abs': max((abs(v) for sample in held for v in sample[1:]), default=None)},
                indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait(timeout=10)
        if out:
            log.close()
