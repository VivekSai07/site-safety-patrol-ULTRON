# Exploration, goals and the fence: who drives, and where it may go

Shared contract between the dashboard (`ultron.dashboard`), its ROS bridge
(`scripts/dashboard_bridge.py`), the explorer (m-explore-ros2, package `explore_lite`) and the
pipeline (`python -m ultron.pipeline online`). Status: **agreed on branch `pipeline/online`, not yet
run with Nav2 or on the robot.**

## Fallback ladder: the demo must work at every rung

| Rung | Who moves the robot | What the pipeline does |
|---|---|---|
| 1. Explore | `explore_lite` drives Nav2 through frontiers. The pipeline pauses it to collect views of an object, then resumes it. | Survey, discovery, view goals during a pause |
| 2. Manual goals | Operator clicks goals on the dashboard map, which go to Nav2 | Survey; view goals when no operator goal is underway |
| 3. Teleop | Operator drives with RC or WASD | Survey; `focus` events tell the operator where to stand |

If any rung fails (explorer not installed, Nav2 down, bridge gone), the next one still works, with no
restart of the pipeline. Every failure is a status on the page, never a crash.

## One owner of Nav2 at a time

Priority: **operator > pipeline > explorer**.

- A goal carries `by`: `"operator"` (the default, from the page) or `"pipeline"`.
- An operator goal always wins. It replaces a running pipeline goal, as goals replace each other
  today.
- A pipeline goal while an operator goal is `SENT` or `ACCEPTED` is refused: **409**, detail
  `operator goal underway`. The pipeline then stands down until that goal ends, plus a grace
  period (default 10 s). It does not count this as the view being unreachable.
- While exploring (not paused), every goal is refused: **409**, detail `exploring`. The pipeline
  treats this as *busy*, not *unreachable*, and asks for a pause instead (below).

## Pause and resume: a lease

`POST /api/exploration` takes `{"action": ..., "by": "operator"|"pipeline", "lease_s": float, "reason": str}`.

| Action | From state | Dashboard sets | Bridge does, then confirms |
|---|---|---|---|
| `start` | IDLE, MAPPED, FAILED | STARTING | launches the explore node → EXPLORING |
| `pause` | EXPLORING | PAUSING | publishes `explore/resume=false` (explorer cancels its goal) → PAUSED |
| `renew` | PAUSING, PAUSED (same holder) | extends the lease | nothing |
| `resume` | PAUSING, PAUSED | RESUMING | publishes `explore/resume=true` → EXPLORING |
| `finish` | STARTING, EXPLORING, PAUSING, PAUSED, RESUMING | FINISHING | resume=false, stops the node → MAPPED |

- A pipeline pause has a lease, `lease_s` (default 60 s, maximum 180 s). The pipeline renews it
  while it collects views. If it is not renewed (the pipeline crashed or hung), the **dashboard
  itself** requests `resume` when it runs out. An operator pause has no lease.
- Goals are accepted in PAUSED; in PAUSING they are refused with 409 `pausing`. The pipeline waits
  for PAUSED.
- When a pipeline pause ends, any pipeline goal still running is cancelled first.
- `GET /api/exploration` returns `{seq, state, detail, by, since, lease_until}`.
- Pipeline policy (`online.py`):
  - Pause only when the explorer is EXPLORING, an object needs a view the explorer did not deliver,
    and the goal gate has a valid goal for it.
  - Nearby objects that still collect are batched into the same pause.
  - Each object causes at most one pause per patrol, and each pause has a hard cap (default 90 s).
  - Resume happens on every exit path: done, cap, error, Ctrl-C, end of stream.

## The fence: where the robot may go and where objects may be

An operator-set polygon in the `map` frame, the room's walls drawn slightly inside the glass.
Lidar sees through windows, so SLAM marks the space beyond them as free, and frontier exploration
would chase it. Nothing outside the fence is ever a goal, a frontier or an object.

- `GET /api/fence` and `PUT /api/fence` with `{"frame_id": "map", "polygon": [[x, y], ...]}`, at
  least 3 points. The pipeline also takes `--fence FILE` with the same JSON, for runs without a
  dashboard.
- **Bridge:** publishes `/map_fenced` (`nav_msgs/OccupancyGrid`), the SLAM map with every cell
  outside the fence set to 100 (occupied). The explorer reads it (`costmap_topic: /map_fenced`),
  so it sees no frontier outside. It also publishes `/keepout_mask`, the fence as an occupancy
  mask (100 outside, 0 inside), for Nav2's KeepoutFilter if the navigation stack loads one.
- **Pipeline goal gate** (`pipeline/gate.py`): a goal is sent only if all of these hold:
  - it lies inside the fence, by at least the robot's clearance;
  - it is known free in the latest dashboard map, with that clearance;
  - it is clear of every known object's footprint, inflated by clearance (ladders on the floor
    included).

  A rejected goal makes the pipeline try another side; it is never sent.
- **Pipeline objects:** a sighting whose position falls outside the fence is dropped before it
  becomes an object (reflections, things seen through glass).

## Not covered by this contract: obstacles Nav2 cannot see

A ladder lying on the floor may be below the lidar's view, so Nav2's costmap can plan straight over
it. The goal gate stops only the pipeline's own goals, not Nav2's path between them and not the
explorer's goals. The navigation stack needs a depth-camera or 3D obstacle layer that marks low
objects (min obstacle height of a few centimetres). The pipeline has no ROS. It sends the footprints of known
objects to the dashboard, `PUT /api/obstacles {"frame_id": "map", "cell": 0.1, "cells": [[x, y], ...]}`
(cell centres, replaced on every PUT). The bridge republishes them as `/ultron/object_obstacles`
(`sensor_msgs/PointCloud2`, map frame, points at z = 0.05 and 0.3 m) for an obstacle layer to
subscribe to. **Owner: navigation stack (Sahil); not done.**
