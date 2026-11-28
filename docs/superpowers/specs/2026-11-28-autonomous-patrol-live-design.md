# Autonomous patrol + live dashboard — design

Date: 2026-11-28 · Owner: Vivek · Branch: `feature/autonomous-patrol-live` (from main)

## Goal

1. Remove the scripted patrol route: goals are selected autonomously by a
   coverage planner over the zone grid, with goal-validation semantics aligned
   to Sahil's `navigation_contract` (`ultron.navigation_goal/v1`) so the same
   mission logic later drops onto his Nav2 stack.
2. End-to-end live stack test: the site manager watches the running simulation
   on Aravind's dashboard — pose on the map, live camera, event feed including
   PPE violations, photos and review-pending ladder records.

## Honest limits (unchanged)

- Base motion stays kinematic (planned A* paths, teleport integration); real
  locomotion/Nav2 is Sahil's organizer-sim lane. Autonomy added here is **goal
  selection**, not locomotion.
- Safety stations are visited at site-plan locations (fixed installations on a
  real site plan); the route to them is autonomous. Ladders are discovered
  dynamically. Ladder damage assessment is not wired into this test.
- Nav2/slam_toolbox are not installed on this machine; nothing here claims to
  run them.

## Components

### A. `CoveragePlanner` (`src/ultron/patrol_planner.py`)

- Free-cell lattice at `grid.resolution` over `grid.bounds` (cells passing
  `grid.free`). Cells within `sense_range` (default 3.0 m) of any visited pose
  are marked observed (`observe(pose)` called at every capture).
- `next_goal(pose)`: greedy next-best-view — among sampled unseen cells (≤25
  candidates), score = newly-observable-cells / (1 + A* path length); returns
  the best reachable free goal, or `None` once `coverage() >=
  coverage_target` (default 0.92) or `max_goals` (default 30) issued.
- Goals carry `ultron.navigation_goal/v1`-shaped records (goal_id, x, y, yaw,
  tolerances, timeout, reason) in `PATROL_GOAL_SELECTED` events; rejected /
  unreachable goals emit `REJECTED` status mirroring Sahil's statuses.

### B. Mission integration (`mission.py`)

- `run_demo(..., nav='scripted')` — `'auto'` replaces the fixed `route` list
  with planner-generated goals (same DISCOVER bursts, supervisor queue, safety
  sweep, return-to-base afterwards). Default `'scripted'` keeps all existing
  behavior and tests byte-identical. CLI: `--nav scripted|auto`.
- `MISSION_START` event payload gains `zone_bounds`, `keepouts`, `base` so a
  live consumer can build the dashboard site map before the run ends.
- Auto-mode summary reports `observed_coverage_pct` (planner estimate, with a
  note that it is grid-cell observation, not certified area coverage).

### C. Live tailer (`src/ultron/dashboard/live_tail.py`)

- Reuses `replay.Api`, `convert_record`, `depth_jpeg`. Polls the run dir and
  incrementally streams `telemetry.jsonl` (pose + front_rgb/front_depth
  frames) and `events.jsonl` (all events) to the dashboard as lines appear.
- Mission start is posted from the enriched `MISSION_START` event (site map
  included). `INSPECTION_COMMITTED` details, photos and `missions/end` are
  posted once `protocol.json` exists (end of run). Exits after the final
  posts or on idle timeout.
- Fail-silent toward the mission: it is a separate process; the sim never
  depends on it. CLI: `python3 -m ultron.dashboard.live_tail RUN --url URL`.

### D. End-to-end run

Install the dashboard extra into the demo python; start
`python3 -m ultron.dashboard` as a long-lived service; start the tailer on the
output dir; run the model-backed autonomous patrol:

    HF_HUB_DISABLE_IMPLICIT_TOKEN=1 PYTHONPATH=src MUJOCO_GL=osmesa python3 -m ultron demo \
      --out runs/live-e2e --patrols 1 --safety --nav auto \
      --detector yolo --weights weights/yoloe-11s-seg.pt \
      --inspector qwen --qwen Qwen/Qwen3-VL-2B-Instruct --allow-download

## Error handling

- Planner never yields an occupied/outside/unreachable goal; exhaustion or
  coverage target ends the patrol cleanly (supervisor return-to-base follows).
- Tailer: HTTP failures log and retry next poll; duplicate mission start (409)
  tolerated; missing files → keep polling until idle timeout.
- Dashboard down → sim unaffected; tailer exits with a clear message.

## Testing

- Unit: planner on a small synthetic grid (goals free+reachable, coverage
  monotone, termination); tailer drain-step with a fake Api (posts recorded,
  no network).
- Gated render test: `nav='auto'` single patrol emits `PATROL_GOAL_SELECTED`,
  reaches coverage, returns to base.
- Live E2E validated manually against the running dashboard (`/api/live`).
