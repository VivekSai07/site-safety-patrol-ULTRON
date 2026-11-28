# Dashboard Site map refresh rate

How often the Site map on the dashboard redraws, and what to change to make it faster.

## Three limits in a row

The map passes through three stages before it reaches the browser. Each one limits the rate,
and the slowest one wins.

| # | Stage | Setting | Current value | Effect |
|---|---|---|---|---|
| 1 | slam_toolbox (builds `/map`) | `map_update_interval` in `config/autonomy/slam_toolbox_mapping.yaml` and `config/autonomy/slam_toolbox_localization.yaml` | `1.0` s | SLAM publishes at most one new map per second (1 Hz). |
| 2 | Dashboard bridge (`scripts/dashboard_bridge.py`) | ROS parameter `map_min_period_s` | `2.0` s | The bridge uploads at most one map every 2 s (0.5 Hz). |
| 3 | Dashboard bridge | Fixed timer `self.create_timer(0.5, self.flush_map)` | 0.5 s | Hard limit of 2 Hz. It is in the code and has no parameter. |

The browser has no limit of its own. `static/live.js` reloads `/api/map.png` whenever the map
sequence number changes.

With the current values the dashboard map updates **every 2 s** (limit 2). A 1 Hz update
matches `/map` itself (limit 1), as seen in RViz. The real robot's stack may also run its own
slam_toolbox config instead of ours. slam_toolbox's package default is 5 s.

## Recommended: smooth robot, map at 1 Hz

What usually looks choppy is the robot marker, not the map. The robot's position is sent
separately at `pose_rate_hz` (default 5). Raising that is cheap:

```bash
python3 scripts/dashboard_bridge.py --ros-args -p pose_rate_hz:=10.0 -p map_min_period_s:=1.0
```

- `pose_rate_hz:=10.0`: the robot marker moves at 10 Hz.
- `map_min_period_s:=1.0`: the map follows SLAM's 1 Hz instead of lagging at 0.5 Hz.

No code change is needed. `scripts/dashboard.sh live` starts the bridge with only
`-p dashboard_url:=...`, so add the two `-p` arguments to that line in `scripts/dashboard.sh`
when using the launcher.

## Full 10 Hz map (not recommended)

All three limits have to change:

1. **SLAM**: set `map_update_interval: 0.1` in the slam_toolbox yaml that is actually running.
2. **Bridge parameter**: `--ros-args -p map_min_period_s:=0.1`.
3. **Bridge code**: change the flush timer in `scripts/dashboard_bridge.py` from `0.5` to `0.05`,
   or derive it from `map_min_period_s`.

Cost: on every update slam_toolbox rebuilds the full occupancy grid, the bridge re-encodes it as
a PNG and POSTs it, and the browser reloads the image. All three get slower as the map grows.
The map barely changes within 0.1 s, so this mostly spends CPU on the robot computer for no
visible gain.

## Status

The values above were read from the code on `main` (f6e6a79). None of the rate changes were run
or measured on the robot.
