# Agent E — Mission, bounded exploration and ladder search

Read [FLEET_COMMON.md](FLEET_COMMON.md) and [PLAN.md](PLAN.md) first.
Branch: `om/b2-ros-stack-e`. Depends on A and D (sim `NavigateToView`).
Start on contracts + unit tests against a fake action server immediately.

## Kickoff prompt

> You are Agent E for the ULTRON B2 ROS 2 stack. Follow AGENTS.md, CLAUDE.md,
> FLEET_COMMON.md, PLAN.md and AGENT_E.md. Build the mission layer: bounded
> frontier exploration, ladder detection/localization/tracking, view-pose
> planning, capture coordination, persistent map-scoped memory and honest
> coverage reporting, wrapping the existing supervisor. Oracle detections in
> sim must be labelled ORACLE. Arrival is never an inspection result.

## Deliverables

1. **`ultron_b2_perception`**
   - `ladder_detector` node, backend param: `oracle_sim` (uses sim
     segmentation; every message tagged `source: ORACLE_SIM`), `yolo`,
     `owlv2` (open-vocabulary "ladder", "step ladder", "A-frame ladder").
     Weights/paths come from the perception owner (Om); without weights the
     node reports UNAVAILABLE, never falls back to oracle in non-sim modes.
     Input `sensors/cam_head/color/image_raw`; output `LadderDetectionArray`.
   - `ladder_localizer`: depth (aligned) + intrinsics + TF **at image stamp**
     (`tf2` lookup with timeout; stale/extrapolated ⇒ drop + count) → 3D
     position + covariance in `map`; requires `state/localization` VALID.
   - `ladder_tracker`: map-scoped association (port logic from
     `src/ultron/memory.py` / `observations.py`), keeps ambiguity, publishes
     `LadderCandidateArray` (id, map_id/version, position, covariance,
     type proposal, first/last seen, n_obs, status NEW/CONFIRMED/AMBIGUOUS/MISSING).
2. **`ultron_b2_mission`**
   - `frontier_explorer`: frontiers on `/map` within PatrolZone polygon,
     min frontier size and info-gain PROPOSED; goals via `NavigateToView`;
     blacklist failed frontiers; stop conditions (no reachable frontiers,
     time budget, operator). Reference: m-explore-ros2 (BSD) — reuse or port.
     Fallback: configured survey route (`config/autonomy/routes`).
   - `view_pose_planner`: port `src/ultron/viewpoints.py` (4-pose ring per
     ladder type + gates: inside map/zone, not occupied/unknown, clearance,
     line of sight to target, camera FOV/height); returns ranked feasible poses.
   - `capture_coordinator`: after `settled=true`, request capture (existing
     label-first/inspection pipeline hooks), wait for ack/fail with timeout.
   - `mission_manager`: state machine PATROL→EXPLORE→DETECT→APPROACH→SETTLE→
     CAPTURE→ASSESS(hand-off, async)→COMMIT→next | RETURN_BASE, with HOLD and
     AWAIT_OPERATOR reachable from every state. Wraps `src/ultron/supervisor.py`
     actions (`INSPECT, SURVEY, RETURN_BASE, RETURN_BASE_WITH_GAPS,
     AWAIT_OPERATOR`). Records base pose at start. `RunMission` action.
   - `memory_bridge`: SQLite via `src/ultron/memory.py` (append-only,
     map-scoped; schema change ⇒ new DB or explicit migration).
   - `coverage_reporter`: `CoverageReport` (explored/free/unknown/unreachable
     areas in zone, ladders known/inspected/pending/unreachable/missing,
     return reason). Waypoint completion ≠ coverage.
   - Mission events to the existing dashboard (`src/ultron/dashboard`) via its
     current ingestion path (coordinate; do not change dashboard schema).

## Acceptance (sim, VALIDATION.md → Agent E)

- Hall with 7 ladders (`hall_scene.py`): bounded explore discovers ladders
  (oracle, labelled), plans views, settles, captures, resumes, returns to
  base ≤ 0.25 m (PROPOSED); coverage report with remaining unknown area.
- Two patrols on the same map: stationary ladder keeps ID; added ladder ⇒ NEW;
  two close ladders ⇒ AMBIGUOUS (not merged); removed ladder ⇒ MISSING.
- HOLD / operator inhibit / localization loss in each state ⇒ no motion,
  mission resumes or ends with reason.
- Unreachable view poses ⇒ ladder marked unreachable, not inspected.
- Stale TF on image ⇒ detection dropped (unit test).

## Do not

Treat oracle output as perception, change verdict policy, or infer complete
coverage from waypoints/object counts.
