#!/usr/bin/env python3
"""Walk docs/EXPLORATION_CONTRACT.md against a running dashboard, the real bridge, fake_nav2.py and
fake_explore.py (started by explore_check.sh). Localhost only. Prints one line per check; exit 1 if any failed."""
import json
import os
import signal
import subprocess
import threading
import time
import urllib.error
import urllib.request

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

URL = 'http://127.0.0.1:8000'
LOG = '/tmp/explore-check'
FAILED = []


def call(method, path, body=None):
    request = urllib.request.Request(URL + path, method=method, data=None if body is None else json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            text = response.read()
            return response.status, json.loads(text) if text else None
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read() or b'null')


def check(name, ok, info=''):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {info if not ok else ''}".rstrip(), flush=True)
    if not ok:
        FAILED.append(name)


def wait(fn, timeout=15.0, period=0.2):
    end = time.monotonic() + timeout
    while True:
        value = fn()
        if value or time.monotonic() > end:
            return value
        time.sleep(period)


def exploration():
    return call('GET', '/api/exploration')[1]


def wait_state(state, timeout=15.0):
    wait(lambda: exploration()['state'] == state, timeout)
    return exploration()


def goal():
    return call('GET', '/api/goal')[1]


def wait_goal(statuses, timeout=10.0):
    wait(lambda: goal()['status'] in statuses, timeout)
    return goal()


def ask(action, **body):
    return call('POST', '/api/exploration', {'action': action, **body})


def set_goal(by='operator', x=1.0, y=1.0):
    return call('POST', '/api/goal', {'x': x, 'y': y, 'by': by})


class Watch(Node):
    def __init__(self):
        super().__init__('explore_check')
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.latest, self.states = {}, []
        for topic, kind in (('/map_fenced', OccupancyGrid), ('/keepout_mask', OccupancyGrid),
                            ('/ultron/object_obstacles', PointCloud2)):
            self.create_subscription(kind, topic, lambda msg, t=topic: self.latest.__setitem__(t, msg), latched)
        self.create_subscription(String, '/fake_explore/state', lambda msg: self.states.append(msg.data), latched)


def start_bridge():
    params = ['-p', 'explore_command:=python3 /repo/scripts/b2_standin/fake_explore.py', '-p', 'nav_wait_s:=3.0',
              '-p', 'explore_ready_wait_s:=20.0']
    log = open(f'{LOG}/bridge.log', 'a')
    return subprocess.Popen(['python3', '/repo/scripts/dashboard_bridge.py', '--ros-args', *params], stdout=log, stderr=log)


def explorer_pids():
    # anchored: the bridge's own command line names fake_explore.py in its explore_command parameter
    out = subprocess.run(['pgrep', '-f', '^python3 /repo/scripts/b2_standin/fake_explore'], capture_output=True,
                         text=True).stdout.split()
    return [int(p) for p in out]


def main():
    rclpy.init()
    node = Watch()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    bridge = start_bridge()
    check('bridge uploads the map', wait(lambda: call('GET', '/api/map')[1], 20))

    # fence -> /map_fenced, /keepout_mask (map: 60 x 40 cells of 0.1 m from (-1, -1))
    status, _ = call('PUT', '/api/fence', {'frame_id': 'map', 'polygon': [[0, 0], [3, 0], [3, 2], [0, 2]]})
    check('fence accepted', status == 200, status)

    def fenced():
        m, k = node.latest.get('/map_fenced'), node.latest.get('/keepout_mask')
        if not (m and k):
            return None
        inside, outside = 15 * 60 + 15, 5 * 60 + 5  # cells at (0.55, 0.55) and (-0.45, -0.45)
        return m.data[inside] == 0 and m.data[outside] == 100 and k.data[inside] == 0 and k.data[outside] == 100
    check('/map_fenced and /keepout_mask follow the fence', wait(fenced, 10))

    # rung 2: manual goal with Nav2 there
    status, g = set_goal()
    check('operator goal with no exploration', status == 201, (status, g))
    check('operator goal reaches Nav2 and succeeds', wait_goal({'SUCCEEDED'})['status'] == 'SUCCEEDED', goal())

    # rung 1: explore
    status, ex = ask('start')
    check('start -> STARTING', status == 201 and ex['state'] == 'STARTING', ex)
    ex = wait_state('EXPLORING', 25)
    check('bridge confirms EXPLORING once the explorer has its costmap', ex['state'] == 'EXPLORING', ex)
    check('explorer read the fenced map', wait(lambda: any('occupied' in s and not s.startswith('costmap 60x40, 0 ') for s in node.states), 5),
          node.states)
    status, g = set_goal()
    check('operator goal while exploring -> 409 exploring', (status, g['detail']) == (409, 'exploring'), (status, g))

    status, ex = ask('pause', by='pipeline', lease_s=20, reason='views of LAD-002')
    check('pipeline pause -> PAUSING with a lease', status == 201 and ex['state'] == 'PAUSING' and ex['lease_until'], ex)
    ex = wait_state('PAUSED')
    check('bridge confirms PAUSED', ex['state'] == 'PAUSED' and ex['by'] == 'pipeline', ex)
    check('explorer heard explore/resume=false', wait(lambda: node.states[-1:] == ['stopped'], 5), node.states[-3:])
    status, g = set_goal('pipeline', 2.0, 1.0)
    check('pipeline goal while paused accepted', status == 201, (status, g))
    check('pipeline goal ACCEPTED by Nav2', wait_goal({'ACCEPTED', 'SUCCEEDED'})['status'] in ('ACCEPTED', 'SUCCEEDED'), goal())
    check('renew keeps the pause', ask('renew', by='pipeline', lease_s=20)[0] == 201)

    status, g = set_goal('pipeline', 2.5, 1.0)  # make sure one runs when the pause ends
    status, ex = ask('resume', by='pipeline')
    check('pipeline resume -> RESUMING', status == 201 and ex['state'] == 'RESUMING', ex)
    check('the pipeline goal is cancelled when its pause ends', goal()['status'] == 'CANCELED', goal())
    check('bridge confirms EXPLORING after resume', wait_state('EXPLORING')['state'] == 'EXPLORING')
    check('explorer heard explore/resume=true', wait(lambda: 'resumed' in node.states, 5), node.states[-3:])

    # operator takes over a pipeline pause
    ask('pause', by='pipeline', lease_s=5)
    wait_state('PAUSED')
    set_goal('pipeline', 2.0, 0.5)
    status, g = set_goal('operator', 1.0, 0.5)
    ex = exploration()
    check('operator goal takes over a pipeline pause', status == 201 and ex['by'] == 'operator' and ex['lease_until'] is None, ex)
    status, g = set_goal('pipeline', 2.0, 0.5)
    check('pipeline goal during an operator goal -> 409', (status, g['detail']) == (409, 'operator goal underway'), (status, g))
    time.sleep(6)
    check('no lease after the takeover: still PAUSED', exploration()['state'] == 'PAUSED', exploration())
    check('operator goal succeeds', wait_goal({'SUCCEEDED'})['status'] == 'SUCCEEDED', goal())
    ask('resume')
    check('operator resume -> EXPLORING', wait_state('EXPLORING')['state'] == 'EXPLORING')

    # lease runs out: the pipeline "died"
    ask('pause', by='pipeline', lease_s=3)
    wait_state('PAUSED')
    set_goal('pipeline', 2.0, 1.5)
    wait_goal({'ACCEPTED'})
    ex = wait(lambda: exploration()['state'] in ('RESUMING', 'EXPLORING') and exploration(), 8) or exploration()
    check('lease ran out: the dashboard resumed by itself', ex['state'] in ('RESUMING', 'EXPLORING') and
          (ex['by'] == 'dashboard' or ex['detail'] == ''), ex)
    check('its pipeline goal was cancelled', goal()['status'] == 'CANCELED' and 'lease' in goal()['detail'], goal())
    check('bridge confirms EXPLORING after the lease', wait_state('EXPLORING')['state'] == 'EXPLORING')

    # obstacles -> PointCloud2
    call('PUT', '/api/obstacles', {'frame_id': 'map', 'cell': 0.1, 'cells': [[1.05, 1.05], [1.15, 1.05], [1.25, 1.05]]})
    cloud = wait(lambda: (lambda c: c if c and c.width == 6 else None)(node.latest.get('/ultron/object_obstacles')), 5)
    check('obstacles republished as a 6-point cloud in map', bool(cloud) and cloud.header.frame_id == 'map',
          node.latest.get('/ultron/object_obstacles'))

    # finish
    status, ex = ask('finish')
    check('finish -> FINISHING', status == 201 and ex['state'] == 'FINISHING', ex)
    ex = wait_state('MAPPED')
    check('bridge confirms MAPPED', ex['state'] == 'MAPPED', ex)
    check('explore node stopped', wait(lambda: not explorer_pids(), 8), explorer_pids())

    # explorer crashes
    ask('start')
    wait_state('EXPLORING', 25)
    for pid in explorer_pids():
        os.kill(pid, signal.SIGKILL)
    ex = wait_state('FAILED', 10)
    check('explorer crash -> FAILED with detail', ex['state'] == 'FAILED' and 'exited' in ex['detail'], ex)
    status, g = set_goal()
    check('goals work after the explorer failed (rung 2)', status == 201 and wait_goal({'SUCCEEDED'})['status'] == 'SUCCEEDED', goal())

    # bridge restart while exploring: explorer dies with it, nothing is replayed, page says FAILED
    ask('start')
    wait_state('EXPLORING', 25)
    bridge.kill()  # SIGKILL: no clean shutdown
    bridge.wait()
    check('explorer dies with a killed bridge', wait(lambda: not explorer_pids(), 8), explorer_pids())
    seq_goal = goal()['seq']
    bridge = start_bridge()
    ex = wait_state('FAILED', 15)
    check('restarted bridge reports FAILED, launches nothing', ex['state'] == 'FAILED' and not explorer_pids(), ex)
    time.sleep(2)
    check('no old goal replayed', goal()['seq'] == seq_goal and goal()['status'] == 'SUCCEEDED', goal())

    try:  # built from stub_explore_lite_msgs/ when explore_check.sh runs with STATUS=1
        import explore_lite_msgs  # noqa: F401
        have_status = True
    except ImportError:
        have_status = False
    print(f"explore/status path: {'on (explore_lite_msgs)' if have_status else 'off (no explore_lite_msgs)'}", flush=True)
    if have_status:
        # nothing left to explore: explore_lite says complete and returning in one burst; the page must say MAPPED
        call('PUT', '/api/fence', {'frame_id': 'map', 'polygon': [[50, 50], [51, 50], [51, 51]]})  # off the map
        time.sleep(3)
        ask('start')
        ex = wait_state('MAPPED', 30)
        check('no frontiers: MAPPED by the explorer, from explore/status', (ex['state'], ex['by'], ex['detail']) ==
              ('MAPPED', 'explorer', 'no frontiers left'), ex)
        check('a goal is accepted after the explorer finished', set_goal()[0] == 201)
        wait_goal({'SUCCEEDED'})

    # Nav2 absent
    os.kill(int(open(f'{LOG}/nav2.pid').read()), signal.SIGINT)
    time.sleep(2)
    status, g = set_goal()
    g = wait_goal({'FAILED', 'SUCCEEDED'}, 12)
    check('Nav2 absent -> FAILED navigation not available', (g['status'], g['detail']) == ('FAILED', 'navigation not available'), g)

    bridge.send_signal(signal.SIGINT)
    bridge.wait(10)
    print(f'{len(FAILED)} failed: {FAILED}' if FAILED else 'all checks passed', flush=True)
    return 1 if FAILED else 0


if __name__ == '__main__':
    raise SystemExit(main())
