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

## As built: dashboard and bridge (deviations and details)

Implemented in `ultron.dashboard` (app, models, live page) and `scripts/dashboard_bridge.py`, with the
ROS-free half in `ultron/dashboard/ros_link.py`. Checked by unit tests and by
`scripts/b2_standin/explore_check.sh` (real bridge against a fake explorer and a fake Nav2, ROS 2 Jazzy,
network off, both with and without a stub `explore_lite_msgs`); **not yet with explore_lite, Nav2, Humble
or the robot.** Where the contract above was silent or could not be
followed exactly:

- **`lease_until`** (and `since`) are ISO 8601 UTC strings to the second, e.g. `2026-11-28T14:03:10+00:00`;
  `lease_until` is `null` when there is no lease (operator pause, or not paused). `lease_s` above 180 is
  cut to 180, not refused; `lease_s <= 0` is a 422.
- **`by` in the exploration state** is who asked for the current step (`operator` or `pipeline`), and also
  `dashboard` (a lease ran out) or `explorer` (bridge: no frontiers left). The bridge keeps `by` when it
  confirms PAUSED, so a pause stays its holder's.
- **Holder rules.** `renew` only by the holder, else 409 `paused by <holder>`. Operator > pipeline: an
  operator `pause` during a pipeline PAUSING/PAUSED, or an operator goal during a pipeline PAUSED, takes the
  pause over (same `seq`, lease removed, detail `operator took over`). After that a pipeline `resume` or
  `renew` gets 409 `paused by operator`: **the pipeline should read this as "the pause is no longer mine"
  and stand down, not as an error.** A pipeline `pause` while the operator holds one is 409.
- **Which goal is cancelled.** `resume`, a lease running out and `finish` mark the running goal CANCELED
  (whoever owns it) before the new state is published, so the bridge stops it first. With the takeover rule
  a pipeline pause can only have a pipeline goal running, as the contract says; an operator goal is
  cancelled only by the operator's own `resume` or by `finish`. Reason for `finish`: explore_lite calls
  `async_cancel_all_goals()` in `stop()`, and its destructor calls `stop()`, so stopping the node cancels
  every goal on the Nav2 action server anyway. A late `ACCEPTED` for a cancelled goal is refused (409).
- **FINISHING** refuses goals with 409 `exploring`, like STARTING and RESUMING.
- **Fence extras:** `DELETE /api/fence` clears it; `python -m ultron.dashboard --fence FILE` loads one at
  start-up (the dashboard keeps it in memory only, so pass it again after a restart). `/api/live` carries
  it and the live page draws it. The fence's `frame_id` must equal the SLAM map's frame; otherwise the
  bridge warns and publishes the map unfenced. `scripts/dashboard.sh live` passes `EXPLORE_COMMAND` from the
  environment to the bridge. With no fence, `/map_fenced` is the map unchanged and
  `/keepout_mask` all 0, so an explorer reading `/map_fenced` still works.
- **The explorer reads `/map_fenced`.** The bridge's default `explore_command` is
  `ros2 run explore_lite explore --ros-args -p use_sim_time:=false -p costmap_topic:=/map_fenced -p costmap_updates_topic:=/map_fenced_updates -p return_to_init:=false`
  (`costmap_updates_topic` set to an unused topic, so no partial update of the raw map lands on the
  fenced one). The package's own launch file would read the unfenced `map` (its `params.yaml`); a
  team params file must keep `costmap_topic: /map_fenced`.
- **explore_lite facts the bridge relies on** (read from robo-friends/m-explore-ros2 `main` on 2026-11-28
  through a summarising web fetch, not a line-by-line read; still assumed: the status publisher's
  reliability and depth, `wait_for_action_server()`'s default timeout, whether slam_toolbox publishes
  `map_updates`):
  `explore/resume` is a relative `std_msgs/Bool` subscription, depth 10; false calls `stop()` (cancels all
  Nav2 goals, publishes `exploration_paused`), true calls `resume()` (publishes `exploration_in_progress`,
  replans). `explore/status` is `explore_lite_msgs/ExploreStatus` (`status` string), transient-local,
  published without any `#ifdef`, so a built explore_lite always has the msgs package; only the bridge's
  machine may lack it. The costmap subscription uses default (volatile) QoS and the constructor blocks
  until the first costmap arrives, so the bridge repeats `/map_fenced` every 2 s (`fenced_period_s`).
  The node subscribes to `explore/resume` after its costmap arrived, then waits for the Nav2 action server
  with no timeout, then publishes `exploration_started`. Action name `navigate_to_pose`.
- **Confirmations.** Start: EXPLORING once the process has lived 1 s and either `explore/status` said
  `exploration_started` / `in_progress` / `complete` (with `explore_lite_msgs`), or the node listens on
  `explore/resume` and the Nav2 action server is up (always accepted, so a status topic the bridge cannot
  hear does not fail a working explorer); else FAILED after `explore_ready_wait_s` (30 s). Pause / Resume:
  PAUSED / EXPLORING once a listener heard `explore/resume` (the matching status is awaited up to 3 s).
  Statuses are kept as a log, not only the latest, because explore_lite publishes `exploration_complete`
  and `returning_to_origin` in the same call.
  Whenever the bridge reports FAILED it has stopped the explorer, so FAILED never leaves it driving.
  `exploration_complete` gives MAPPED with `by: explorer`; the node is left running (with
  `return_to_init` it drives back) and the next Start replaces it.
- **Bridge restarts.** Neither old goals nor old exploration requests are replayed: a page that shows
  exploration while no explorer runs gets FAILED (`press Start again`). The explorer runs in its own
  process group under `setpriv --pdeathsig TERM`, so it dies with the bridge even when the bridge is killed.
- **Obstacles** are repeated at 1 Hz with a fresh stamp (an obstacle layer drops stale observations).
- Pipeline side, as read on `pipeline/online` at `a002ce5`: `navigate.py` sends `"by": "pipeline"`, reads a
  409 naming the operator as *preempted* and any other 409 as *busy*, and a FAILED goal whose detail has
  `not available` as *no navigation*; the bridge reports exactly `navigation not available` both when no
  Nav2 is found in 3 s and when a goal gets no answer in 5 s.
