# Sending navigation goals from the dashboard

The dashboard can send a goal from its map or an HTTP call.
`scripts/dashboard_bridge.py` forwards it to a Nav2 `NavigateToPose` action
server. Goals are off by default, but **the bridge has no independent
simulation-only guard**: any reachable matching server, including one attached
to real hardware, could receive a goal. The dashboard endpoints have no user
authentication, including on localhost.

Do not run the bridge or goal examples unless a responsible operator has
independently verified an isolated simulation with no route to hardware.
Localhost, a ROS domain or a namespace alone do not prove isolation. This
document authorizes no hardware connection or motion.

```text
browser / curl ──HTTP──▶ dashboard ──live stream──▶ dashboard_bridge.py ──▶ nav2 navigate_to_pose
               ◀─status─           ◀── status, /map, TF pose ──
```

## 1. Run it next to your Nav2 + SLAM sim

The dashboard is present in main. Only after independently verifying the
simulation isolation above, install the extras with
`pip install -e '.[dashboard]'` and use a simulated Nav2/SLAM environment.
The following are examples, not evidence that the present host is isolated:

```bash
# terminal 1: the dashboard, with goals turned on (they are off by default)
PYTHONPATH=src .venv/bin/python -m ultron.dashboard --goals        # open http://127.0.0.1:8000

# terminal 2: the bridge, in a shell with ROS sourced (Humble or Jazzy)
python3 scripts/dashboard_bridge.py --ros-args -p base_frame:=base_link
```

Do **not** prefix the bridge with `PYTHONPATH=src`. That replaces ROS's `PYTHONPATH`, so `rclpy` is not found. The script finds `src/` itself.

The robot only shows on the map while a mission is running. The pose API is per mission. To start one by hand:

```bash
curl -X POST http://127.0.0.1:8000/api/missions -H 'Content-Type: application/json' -d '{"mission_id":"NAV-01","site_id":"sim"}'
```

| Bridge parameter | Default | Purpose |
| --- | --- | --- |
| `dashboard_url` | `http://127.0.0.1:8000` | Where the dashboard runs |
| `map_topic` | `/map` | OccupancyGrid from SLAM or map_server |
| `map_frame` / `base_frame` | `map` / `base_link` | TF used for the robot pose; set `base_frame` to your robot's frame |
| `action_name` | `navigate_to_pose` | nav2 action |
| `pose_rate_hz` / `map_min_period_s` | `5` / `2` | Upload rates |
| `nav_wait_s` / `nav_accept_timeout_s` | `3` / `5` | A goal fails with `navigation not available` instead of hanging: no Nav2 found in 3 s, or no answer in 5 s |
| `explore_command` | `ros2 launch explore_lite explore.launch.py use_sim_time:=false` | Started on Start exploration, stopped on Finish. **Reads the unfenced `/map`**; see docs/EXPLORATION_CONTRACT.md for one that reads `/map_fenced` |
| `explore_resume_topic` / `explore_status_topic` | `explore/resume` / `explore/status` | explore_lite's topics |
| `fenced_map_topic` / `keepout_mask_topic` / `obstacles_topic` | `/map_fenced` / `/keepout_mask` / `/ultron/object_obstacles` | Fence and object obstacles out to ROS |

`--lan` exposes the unauthenticated dashboard on all interfaces. Do not use it
on a shared network or with sensitive data; a private network alone is not an
authentication or hardware-isolation boundary.

## 2. The API

Positions are in metres in the `map` frame. `yaw` is in radians, counter-clockwise from +x (0 faces +x, 1.57 faces +y).

| Call | Purpose |
| --- | --- |
| `POST /api/goal` `{"x": 3.0, "y": 1.0, "yaw": 1.57}` | Send a goal. A new goal replaces and cancels the running one. Returns `201` and the goal with its `seq`. Returns `403` if the dashboard runs without `--goals`. |
| `GET /api/goal` | The current goal and its `status`. |
| `GET /api/stream` | Live events (Server-Sent Events). Each `goal` event carries the new status. |
| `GET /api/map` / `GET /api/map.png` | Occupancy map metadata and image (from the bridge). |
| `POST /api/exploration` `{"action": "start"}` | Exploration: `start`, `pause`, `renew`, `resume`, `finish` (see docs/EXPLORATION_CONTRACT.md). |
| `GET/PUT /api/fence`, `GET/PUT /api/obstacles` | Where the robot may go; objects Nav2 may not see. |

A goal also takes `"by": "pipeline"` (default `operator`). Every goal is refused with 409 `exploring` while
the explorer drives, and `pausing` while it stops; pause it first. A pipeline goal is refused with
`operator goal underway` while an operator goal runs.

Goal `status` moves through these values:

- `SENT`: the dashboard has the goal.
- `ACCEPTED`: Nav2 took it.
- One final value:
  - `SUCCEEDED`: the action server reported success, not independent proof of physical arrival.
  - `FAILED`: Nav2 rejected or aborted the goal, or no Nav2 is running. `detail` says why.
  - `CANCELED`: a newer goal replaced this one.

### curl

```bash
H=http://127.0.0.1:8000
curl -X POST $H/api/goal -H 'Content-Type: application/json' -d '{"x": 3.0, "y": 1.0, "yaw": 1.57}'
curl $H/api/goal                  # poll the status
curl -N $H/api/stream             # or watch it live (Ctrl-C to stop)
```

### Python (standard library only)

```python
import json, time, urllib.request

HOST = 'http://127.0.0.1:8000'

def send_goal(x, y, yaw=0.0):
    req = urllib.request.Request(f'{HOST}/api/goal', method='POST', headers={'Content-Type': 'application/json'},
                                 data=json.dumps({'x': x, 'y': y, 'yaw': yaw}).encode())
    return json.load(urllib.request.urlopen(req, timeout=5))

def wait(seq, timeout=120):
    end = time.time() + timeout
    while time.time() < end:
        goal = json.load(urllib.request.urlopen(f'{HOST}/api/goal', timeout=5))
        if goal['seq'] != seq:
            return 'REPLACED'
        if goal['status'] in ('SUCCEEDED', 'FAILED', 'CANCELED'):
            return goal['status'], goal['detail']
        time.sleep(0.5)
    return 'TIMEOUT'

goal = send_goal(3.0, 1.0, 1.57)
print(wait(goal['seq']))
```

## What has been checked

- Unit tests: `tests/test_dashboard.py` (goal and map endpoints) and `tests/test_ros_link.py` (map conversion, stream parsing).
- A teammate reported a ROS 2 Jazzy test against a stand-in
  `navigate_to_pose` server: map and pose reached the dashboard, success and
  cancellation were observed, and no server yielded failure. The exact
  network isolation and reproduction were not independently verified here.
- `scripts/b2_standin/explore_check.sh`: the bridge against `fake_nav2.py` (stand-in NavigateToPose
  server and `/map`) and `fake_explore.py` (stand-in explorer) in a Jazzy container with the network off:
  manual goals, start / pause / pipeline goal / resume / finish, operator takeover, lease expiry, fence and
  obstacle topics, explorer crash, bridge restart, Nav2 absent; with and without `explore_lite_msgs`.
- **Not yet checked:** real Nav2 with SLAM, real explore_lite, ROS Humble, and the B2's real base frame name.
