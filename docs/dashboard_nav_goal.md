# Sending navigation goals to the dashboard

This works like RViz's 2D Goal Pose, but on the live dashboard. You set a goal on the dashboard map or with one HTTP call. `scripts/dashboard_bridge.py` turns it into a nav2 `NavigateToPose` goal and reports the result back. **Simulation only:** anyone who can open the dashboard with goals turned on can move the robot.

```
teammate laptop ──HTTP──▶ dashboard (robot laptop) ──live stream──▶ dashboard_bridge.py ──▶ nav2 navigate_to_pose
                ◀─status─                          ◀──── status, /map, TF pose ─────────
```

## 1. On the robot laptop (runs the sim and Nav2)

```bash
# terminal 1: the dashboard, with goals turned on (they are off by default)
PYTHONPATH=src .venv/bin/python -m ultron.dashboard --goals            # this laptop only
PYTHONPATH=src .venv/bin/python -m ultron.dashboard --goals --lan      # also reachable from other laptops

# terminal 2: the bridge (ROS sourced; do NOT prefix PYTHONPATH=src, it hides rclpy)
python3 scripts/dashboard_bridge.py --ros-args -p base_frame:=base_link
```

The bridge parameters are `dashboard_url`, `map_topic`, `map_frame`, `base_frame`, `action_name`, `pose_rate_hz` and `map_min_period_s`. They are listed in `adapters/ros2/README.md`.

## 2. Connecting from a teammate's laptop

There are two options. Pick based on the network you are on.

**A. SSH tunnel (recommended on campus or event Wi-Fi).** The dashboard stays on `127.0.0.1` with no `--lan`, so nobody else on the network can reach it. The robot laptop needs an SSH server once: `sudo apt install openssh-server`. Then, on the teammate's laptop:

```bash
ssh -N -L 8000:127.0.0.1:8000 <user>@<robot-laptop-ip>
# leave it running; the teammate now uses http://127.0.0.1:8000 as if it were local
```

**B. `--lan` (only on a private network, such as a phone hotspot or your own router).** Start the dashboard with `--goals --lan`. The teammate uses `http://<robot-laptop-ip>:8000` directly. To find the IP, run `ip -4 -br addr` on the robot laptop and use the Wi-Fi interface's address. If the address is not private (192.168.x.x, 10.x.x.x or 172.16–31.x.x), other people on that network can probably send goals too, so use option A.

Check the connection: `curl http://<host>:8000/api/goal` prints `null` or the current goal.

## 3. The API

`<host>` is `127.0.0.1:8000` with the tunnel, or `<robot-laptop-ip>:8000` with `--lan`. Positions are in metres in the `map` frame. `yaw` is in radians, counter-clockwise from +x (0 faces +x, 1.57 faces +y).

| Call | Purpose |
|---|---|
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
- A goal posted to the robot laptop's Wi-Fi address with `--lan` returned `201`. That call came from the same laptop.
- **Not yet checked:** real Nav2, the Humble Docker images, the B2's real base frame name, and a call from a second laptop through the tunnel or `--lan`.
