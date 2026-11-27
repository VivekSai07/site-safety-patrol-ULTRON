# Sending navigation goals from the dashboard

This works like RViz's 2D Goal Pose, but on the live dashboard. You set a goal on the dashboard map or with one HTTP call. `scripts/dashboard_bridge.py` turns it into a nav2 `NavigateToPose` goal and shows the result on the map. **Simulation only:** anyone who can open the dashboard with goals turned on can move the robot.

```text
browser / curl ──HTTP──▶ dashboard ──live stream──▶ dashboard_bridge.py ──▶ nav2 navigate_to_pose
               ◀─status─           ◀── status, /map, TF pose ──
```

## 1. Run it next to your Nav2 + SLAM sim

Check out the `feature/dashboard` branch. Install the extras once, with `pip install -e .[dashboard]` or into the repo's `.venv`. Then start your sim with Nav2 and SLAM (for example slam_toolbox publishing `/map`) and run:

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

Other people need `--lan` (dashboard on all network interfaces) to open it. Use that only on a private network: goals have no password.

## 2. The API

Positions are in metres in the `map` frame. `yaw` is in radians, counter-clockwise from +x (0 faces +x, 1.57 faces +y).

| Call | Purpose |
| --- | --- |
| `POST /api/goal` `{"x": 3.0, "y": 1.0, "yaw": 1.57}` | Send a goal. A new goal replaces and cancels the running one. Returns `201` and the goal with its `seq`. Returns `403` if the dashboard runs without `--goals`. |
| `GET /api/goal` | The current goal and its `status`. |
| `GET /api/stream` | Live events (Server-Sent Events). Each `goal` event carries the new status. |
| `GET /api/map` / `GET /api/map.png` | Occupancy map metadata and image (from the bridge). |

Goal `status` moves through these values:

- `SENT`: the dashboard has the goal.
- `ACCEPTED`: Nav2 took it.
- One final value:
  - `SUCCEEDED`: the robot reached the goal.
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
- End to end on ROS 2 Jazzy against a stand-in `navigate_to_pose` server, on an isolated ROS domain. Map and pose reached the dashboard. A single goal went `SENT` → `ACCEPTED` → `SUCCEEDED`. A goal replaced mid-run was `CANCELED`. With no server running, the goal was `FAILED`.
- **Not yet checked:** real Nav2 with SLAM, ROS Humble, and the B2's real base frame name.
