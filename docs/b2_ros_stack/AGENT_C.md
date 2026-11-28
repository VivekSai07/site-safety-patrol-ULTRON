# Agent C — State estimation, mapping and localization (MOLA-LO + slam_toolbox)

Read [FLEET_COMMON.md](FLEET_COMMON.md) and [PLAN.md](PLAN.md) first.
Branch: `om/b2-ros-stack-c`. Depends on A (skeleton) and B (real replay).
Start with sim (`odom_source:=sim`, existing bridge) while B finishes.

## Kickoff prompt

> You are Agent C for the ULTRON B2 ROS 2 stack. Follow AGENTS.md, CLAUDE.md,
> FLEET_COMMON.md, PLAN.md and AGENT_C.md. Port the validated slam_toolbox
> mapping/localization and LocalizationState monitor into ament packages, add
> MOLA-LO and KISS-ICP lidar odometry as interchangeable `odom→base_link`
> sources, add a map manager, and evaluate in sim (with ground truth) and on
> the real bag (without). Thresholds are fixed before runs. No hardware.

## Design

- Exactly one `odom→base_link` publisher, chosen by `odom_source`:
  - `sim`: existing drifting sim odometry (baseline, reproduces Phase 3).
  - `mola`: `mola_lidar_odometry` (Humble binary preferred; **GPLv3** —
    separate process only, never import/link it in our code; record exact
    version). Input `sensors/lidar/points_filtered` (+ optional
    `sensors/imu/data`). Configure output frames (`odom`, `base_link`,
    `lidar_link`), publish `state/odom` (Odometry) and TF; disable MOLA's own
    `map` frame publication so slam_toolbox stays the `map→odom` authority.
    Save MOLA's 3D map (`.mm`) via its service on demand.
  - `kiss`: `kiss_icp` ROS 2 node with the same output frames/topics (MIT).
  - `b2`: adapter `/sportmodestate` (unitree_go/SportModeState) → Odometry +
    TF for live fallback; tested against a recorded/mock message only.
- `slam_toolbox`: port `config/autonomy/slam_toolbox_{mapping,localization}.yaml`
  into `ultron_b2_localization/config/{sim,real}/`; input `sensors/scan`.
- Port `localization_monitor` (`src/ultron/autonomy/localization_monitor.py`)
  as a ROS node publishing `state/localization` (interfaces msg) — keep all
  rules incl. "later-scan confirmation"; import the existing module rather than
  copy logic.
- `map_manager` node: services `SaveMap`/`LoadMap`; writes `map.pgm/yaml`
  (with the `free_thresh` 0.196 fix from Phase 3), slam_toolbox posegraph,
  MOLA 3D map if present, and `map_meta.json` (map_id, version, config hashes,
  file hashes, odom_source, data provenance, `ground_truth_used_by_estimator:false`).
  Version increments on every save; LoadMap publishes `MapInfo`.
- Decide and document `base_footprint` (gravity-aligned) vs `base_link` for
  the scan target frame; coordinate with B and D.

## Evaluation (thresholds PROPOSED, fix before running)

| Case | Data | Metric |
|---|---|---|
| Sim mapping, `odom_source` ∈ {sim, mola, kiss} | existing hall survey, seeds 7/21/99 | ATE RMSE ≤ 0.10 m, max ≤ 0.25 m, yaw ≤ 2°, wall recall ≥ 0.85 (reuse `eval_slam.py`) |
| Sim odometry only (no slam_toolbox) | same | drift % of path for mola/kiss vs sim odom |
| Sim localization + blackout + kidnap | Phase 4 scenarios | zero false VALID; STALE ≤ 0.75 s; recovery ≤ 5 s |
| Real bag mapping, mola and kiss | B's replay | no ground truth: report start/end loop gap (if route closes), map crispness (wall thickness), drift visual, CPU/RAM/latency (dev PC; Orin UNRUN until available) |
| Real bag localization on saved map | replay twice | VALID fraction, match score distribution, no jumps > 0.5 m |

Use the existing `latency_probe.py` ideas for scan→TF latency. Store runs in
`runs/b2_stack/c/…` (ignored).

## Acceptance (VALIDATION.md → Agent C)

- Sim parity with `odom_source:=sim` (M1); mola and kiss runs reported (pass
  or fail honestly).
- Real-bag map produced and reloaded; LocalizationState sane on replay (M2).
- `tf_authority_check` clean in every mode.
- Recommendation: default `odom_source` for live, with evidence; GPL note.

## Do not

Use ground truth as estimator input; let MOLA publish `map→odom`; report real
accuracy numbers without a reference.
