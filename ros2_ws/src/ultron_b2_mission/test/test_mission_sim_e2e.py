"""FULL-SIM mission e2e (UNRUN until Agents C/D/F land). Skipped unless ULTRON_E2E_SIM=1.

Starts ``ros2 launch ultron_b2_bringup sim.launch.py mode:=mission`` (domain 71, /ultron/sim),
sends the operator-enable heartbeat, sets nothing else, and runs a RunMission EXPLORE goal in the
hall. Expected with C (localization + map), D (nav/navigate_to_view, twist_mux ops/hold) and F
(sim with RGB-D head camera, ground_truth/pose for the ORACLE detector):
* result DONE or DONE_WITH_GAPS with a reason; coverage reported (observed > 0);
* at least one ladder captured, every capture from ORACLE detections (labelled ORACLE);
* a HOLD injected mid-mission via ops/hold cancels the active goal and the mission resumes;
* the mission nodes publish no velocity topics.
Simulation results are not hardware evidence and oracle detections are not real perception.
"""
import json
import os
import pathlib
import signal
import subprocess
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('ULTRON_E2E_SIM') != '1',
                                reason='UNRUN: full-sim e2e needs C/D/F stacks; set ULTRON_E2E_SIM=1')

NS = '/ultron/sim'


def test_full_sim_mission_hall():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool
    from ultron_b2_interfaces.action import RunMission
    from ultron_b2_interfaces.msg import MissionEvent, OperatorEnable

    env = dict(os.environ, ROS_DOMAIN_ID='71', ROS_LOCALHOST_ONLY='1')
    launch = subprocess.Popen(['ros2', 'launch', 'ultron_b2_bringup', 'sim.launch.py', 'mode:=mission'],
                              env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                              start_new_session=True)
    rclpy.init()
    node = Node('mission_sim_e2e', namespace=NS)
    events = []
    node.create_subscription(MissionEvent, 'mission/events', events.append, 50)
    op_pub = node.create_publisher(OperatorEnable, 'ops/operator_enable', 10)
    hold_pub = node.create_publisher(Bool, 'ops/hold', 10)
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
    try:
        assert client.wait_for_server(timeout_sec=120.0), 'mission/run never came up'
        time.sleep(20.0)  # localization VALID + map + perception warm-up
        goal = RunMission.Goal(schema_version='ultron.run_mission/v1', mission_id='SIM-E2E', zone_id='HALL-SIM-1',
                               mode='EXPLORE', max_duration_s=900.0)
        send = client.send_goal_async(goal)
        while not send.done():
            time.sleep(0.1)
        assert send.result().accepted
        result = send.result().get_result_async()
        # Inject a hold once the first view goal is active, then release.
        deadline = time.monotonic() + 600
        while time.monotonic() < deadline and not any(
                e.event_type == 'GOAL_SENT' and 'INSPECT_VIEW' in e.payload_json for e in events):
            time.sleep(0.5)
        mark = len(events)
        for _ in range(20):
            hold_pub.publish(Bool(data=True))
            time.sleep(0.1)
        hold_pub.publish(Bool(data=False))
        deadline = time.monotonic() + 1200
        while not result.done() and time.monotonic() < deadline:
            time.sleep(1.0)
        assert result.done(), 'mission did not finish within 20 min'
        res = result.result().result
        assert res.status in ('DONE', 'DONE_WITH_GAPS'), (res.status, res.reason)
        assert res.coverage.observed_area_m2 > 0
        summary = json.loads(pathlib.Path(res.summary_ref).read_text())
        assert summary['ladders_captured'], summary
        assert summary['oracle_detections_used'] is True
        assert any(e.event_type == 'GOAL_CANCELLED' for e in events[mark:]), 'HOLD did not cancel the active goal'
        for name, ns in node.get_node_names_and_namespaces():
            if ns == NS and name in ('mission_manager', 'capture_coordinator', 'ladder_detector',
                                     'ladder_localizer', 'ladder_tracker'):
                pubs = node.get_publisher_names_and_types_by_node(name, ns)
                assert not [t for _, ts in pubs for t in ts if 'Twist' in t], (name, pubs)
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
