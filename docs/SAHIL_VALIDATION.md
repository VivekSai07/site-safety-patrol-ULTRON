# Sahil autonomy validation record

Actual results only. Phases and acceptance criteria are defined in [SAHIL_SLAM_PLAN.md](SAHIL_SLAM_PLAN.md). Results are simulation-scoped and are not hardware acceptance.

## Phase 0 — Environment and configuration scaffold

**Date:** 2026-11-27 · **Machine:** Ubuntu 22.04, ROS 2 Humble (binary), RTX 3070 Laptop, 16 cores, 15 GB RAM · **Backend:** none (no SLAM yet)

| Check | Result | Evidence |
|---|---|---|
| `.venv-ros` created from `/usr/bin/python3` 3.10.12 with system site-packages via `scripts/setup-ros-venv.sh` (uv; `python3-venv` is not installed on this machine) | PASS | Prints `mujoco 3.3.7`, `numpy 1.26.4`, `yaml 6.0.4`, `rclpy` from `/opt/ros/humble` |
| Existing fast suite (system Python) | PASS | 59 tests, 4 rendered skipped |
| Rendered suite in `.venv-ros` (`MUJOCO_GL=egl ULTRON_RENDER_TESTS=1`) | PASS | 52 existing tests, none skipped, 32 s |
| Config loader rejects unknown/missing keys, invalid sources/numbers, non-sim topics, disabled isolation | PASS | `tests/test_autonomy_config.py`, 7 tests |

Open inputs carried forward: Helios-32 variant and channel table, lidar mounting pose (all `source: assumed` in `config/autonomy/lidar_helios32.yaml`).

## Phase 1 — Simulated Helios-32 lidar and test hall (no ROS)

**Date:** 2026-11-27 · **Environment:** `.venv-ros` (Python 3.10.12, mujoco 3.3.7, numpy 1.26.4) · **Code:** `src/ultron/autonomy/{hall_scene,lidar}.py`

Profile under test: 32 channels uniformly spaced over −55…+15° (placeholder; variant unconfirmed), 0.2° × 360° (57,600 rays), 10 Hz, σ = 15 mm range noise, 0.5 % dropout, mount `[0.342, 0, 0.18]` m in `base_link` (lidar 0.83 m above floor). All `assumed`.

| Check | Result | Evidence |
|---|---|---|
| Beam pattern matches profile (elevations, ring indices, per-point time within one sweep, finite float32) | PASS | `test_beam_pattern_matches_profile` |
| Noise-free ranges on known planes: east wall face x = 12.0 m and floor z = 0 within 1e-4 m | PASS | `test_exact_ranges_on_known_planes` |
| Noise statistics: residual std within 20 % of σ, unbiased | PASS | `test_noise_statistics` |
| Seeded scans reproducible | PASS | `test_seeded_scans_are_reproducible` |
| Robot self-occlusion: rear sector (\|az\| > 170°) fully blocked by compute box, front (\|az\| < 60°) open; no returns inside the self-filter crop box. Blind fraction 0.30 (≈108° of azimuth) | PASS | `test_robot_self_occlusion_and_filter` |
| Batch cast agrees with exact single-ray `mj_ray` (sampled every 23rd ray over 4 poses) | PASS (0 mismatches) | `test_batch_cast_agrees_with_exact_single_rays` |
| Scan time ≤ 50 ms median (4 threads) | PASS: 38.4 ms median, 43.1 ms p95 (other runs 28–31 ms) | `ULTRON_PERF_TESTS=1 … test_scan_time_budget` |
| Hall contains 7 ladders of 4 kinds; lidar housing excluded from casting | PASS | `test_hall_contents` |
| Full suites | PASS | System Python: 67 tests, 12 skipped (no MuJoCo). `.venv-ros` rendered: 67 tests, 1 skipped (opt-in timing) |

Findings fixed during the phase (both measured, mujoco 3.3.7):
1. `mj_multiRay` missed geometry whose nearest face point is not a bounding-box corner as seen from the sensor: 12 % of sampled rays disagreed with `mj_ray`, mostly low beams passing through long walls and missing the robot's own compute box. Fix: scene boxes tiled to ≤ 0.3 × 0.3 m footprint; robot occlusion computed once with exact `mj_ray` (robot is rigid relative to the lidar in this simulation). Result: 0 / 38,400 sampled mismatches.
2. NumPy `@` (OpenBLAS) busy-wait threads slowed the following ray cast ~2.5×. Fix: BLAS-free rotation in the scan path. Launch scripts should also set `OPENBLAS_NUM_THREADS=1`.

Local review images (not committed): `runs/phase1/scan_start.png`, `scan_center.png`, `view_hall.png`, `view_ladders.png`.

Limitations: intensity is a constant placeholder; no motion distortion yet (whole sweep cast from one pose); channel angles uniform placeholder; ladder/hall dimensions approximate; static people only.

## Phase 2 — ROS 2 simulation bridge

**Date:** 2026-11-27 · **Environment:** `.venv-ros` + ROS 2 Humble, `ROS_DOMAIN_ID=71`, `ROS_LOCALHOST_ONLY=1` · **Code:** `src/ultron/autonomy/{sim_bridge,driver,odometry,scan2d}.py`, `scripts/sim-slam.sh`

Interfaces (all simulation-only): `/clock` (sim time from 10 s), `/ultron/sim/lidar/points` (PointCloud2 x,y,z,intensity,ring,time; 24 B/pt; `lidar_link`), `/ultron/sim/scan` (LaserScan, `lidar_link`, open sector −123.6°…+123.4°, 0.5°, 495 bins, base_link height band −0.45…0.60 m), `/ultron/sim/odom` + TF `odom -> base_link` (50 Hz), static TF `base_link -> lidar_link`, `/ultron/sim/ground_truth/pose` (frame `ultron_sim_world`, never on TF), `/ultron/sim/bridge_status` (JSON), input `/ultron/sim/cmd_vel` (latest-only queue).

| Check | Result | Evidence |
|---|---|---|
| Rates in sim time within ±10 %: clock/odom 50 Hz, scan/points 10 Hz | PASS | `test_streams_frames_and_isolation` (2× speed, scripted route) |
| Stamps strictly increasing, non-zero; scan and cloud of a sweep share a stamp | PASS | same |
| TF tree exactly `odom -> base_link` + static `base_link -> lidar_link`; no `map`; ground-truth frame absent from TF | PASS | same |
| Only `/ultron/sim/*`, `/clock`, `/tf`, `/tf_static`, `/rosout`, `/parameter_events` exist; no `/cmd_vel` | PASS | same |
| Bridge refuses to start outside domain 71 / localhost-only | PASS | `test_bridge_refuses_without_isolated_domain` |
| Command timeout stops the base: last motion 0.480 s (sim) after the last command, timeout 0.5 s | PASS | `test_cmd_vel_timeout_and_collision_stop` |
| Footprint check stops the base before the cart when driven into it; collision counted | PASS | same |
| Driver: acceleration and speed limits, nonfinite rejection, collision cancels route; route reproduces every waypoint within 2 cm and returns to start | PASS | `tests/test_autonomy_bridge_logic.py` (11 tests) |
| Scan projection: nearest return per bin, height band, blind rear sector excluded (not published as free space) | PASS | same |
| Scripted survey (`hall_survey_loop`, 100.3 s sim, 31.7 m) at requested 2×: achieved 1.98×, 0 collisions | PASS | `runs/phase2/survey_r2/` (local) |
| rosbag2 recording of all bridge topics: 1,003 scans / 1,003 clouds / 5,013 odom for 100.3 s (no drops), 763 MiB | PASS | `runs/phase2/bag_survey` (local, not committed) |
| Odometry drifts from truth (SLAM has something to correct): final 2.02 m / 22.1°, RMSE 1.25 m over 31.7 m | Measured | from `ground_truth.csv` vs `odometry.csv` |
| RViz shows cloud, scan, odom and TF (`scripts/sim-slam.sh rviz`) | UNRUN | config provided; display not checked by the agent |
| Full suites | PASS | System Python 80 tests (14 skipped: MuJoCo/ROS); `.venv-ros` rendered 80 tests (3 skipped: timing/ROS opt-in); ROS live 2 tests |

Findings fixed during the phase:
1. rclpy validated PointCloud2 `data` byte-by-byte in Python (75 % of loop time; 0.35× real time). Passing `array.array` skips it: unthrottled bridge now 2.2× real time.
2. A queued `cmd_vel` subscription applied stale commands late (base moved 0.86 s after the last command). Latest-only queue (depth 1) plus servicing several callbacks per step: 0.48 s.

Limitations: kinematic base (no walking dynamics, flat floor, no body roll/pitch); odometry error model is synthetic (yaw bias 0.004 rad/s dominates drift); no IMU topic; no motion distortion (`time_increment = 0`); bags are large (≈7.6 MiB/s) and stay local.

## Phase 3 — Online 2D mapping with slam_toolbox

**Date:** 2026-11-28 · **Backend:** slam_toolbox 2.6.10 (Humble binary), online async, `config/autonomy/slam_toolbox_mapping.yaml` (package defaults; changes commented) · **Code:** `src/ultron/autonomy/{run_mapping,slam_logger,eval_slam,tf_filter}.py` · **Command:** `scripts/sim-slam.sh map OUT [--rate R] [--record] [--seed N] [--replay RUN]`

Estimator inputs: `/ultron/sim/scan`, `/ultron/sim/odom`, TF `odom -> base_link`, `base_link -> lidar_link`. Ground truth is used only by `eval_slam.py` (`map_meta.json`: `ground_truth_used_by_estimator: false`). Trajectory error is **start-anchored** (map origin = start pose; no best-fit alignment). Map reference = noise-free lidar hits from the true trajectory inside the scan band/open sector. Thresholds fixed in the plan before evaluation.

| Run (`hall_survey_loop`, 31.7 m, 1.5×) | ATE RMSE / max | Yaw RMSE | Odometry-only RMSE / final | Precision | Wall recall | All-surface recall | Verdict |
|---|---|---|---|---|---|---|---|
| seed 7 (acceptance, `runs/phase3/accept_seed7`) | 0.048 / 0.097 m | 0.85° | 1.25 / 2.02 m | 0.996 | 0.964 | 0.913 | PASS |
| seed 21 | 0.053 / 0.140 m | 0.87° | 1.24 / 2.10 m | 1.000 | 0.956 | — | PASS |
| seed 99 | 0.039 / 0.079 m | 0.54° | 1.27 / 2.14 m | 1.000 | 0.957 | — | PASS |
| Thresholds | ≤ 0.10 / ≤ 0.25 m | ≤ 2° | ≥ 2× SLAM RMSE | reported | ≥ 0.85 | reported | |

| Check | Result | Evidence |
|---|---|---|
| Map saved (`map.pgm/.yaml`) and pose graph serialized (`map.posegraph/.data`); `map_meta.json` with map ID/version, config hashes, file hashes | PASS | `runs/phase3/accept_seed7/map_meta.json` (`ultron-hall-be73b8cbf8bf`, v1) |
| Replay of the recorded bag into a fresh slam_toolbox (old `map -> odom` filtered out: 3,439 dropped, 5,014 relayed) | PASS: identical occupancy grid (0 differing cells); 4,853 / 5,001 poses identical, rest ≤ 1.6 cm | `runs/phase3/replay_r1` |
| Saved map loads in `nav2_map_server` with unknown space preserved | PASS after fix: −1: 3,259, 0: 44,138, 100: 2,787 (matches the image) | manual map_server check |
| Evaluation maths: start anchoring exact, offsets not hidden, map precision/recall, trinary loading | PASS | `tests/test_autonomy_slam_eval.py` (5 tests) |
| End-to-end mapping run meets acceptance (opt-in, 2×) | PASS: RMSE 0.048 m, wall recall 0.964 | `ULTRON_SLAM_TESTS=1 … MappingRunTest` |
| Full suites | PASS | System Python 86 (15 skipped); `.venv-ros` rendered 86 (4 skipped); ROS/SLAM opt-in 8 |

Finding fixed: `map_saver` (via slam_toolbox) writes `free_thresh: 0.25`, so `nav2_map_server` loaded every unknown cell (pixel 205 = 0.196) as **free** — 3,259 cells in this map. `run_mapping` now rewrites it to 0.196 (recorded in `map_meta.json`), and `load_map` reports `unknown_reads_as_free`.

Review images (local): `runs/phase3/*/overlay.png` — SLAM path on truth, odometry drifting ~2 m, occupied cells on reference surfaces; ladders appear as small rail clusters.

Limitations: one scripted route in one synthetic hall; kinematic motion on a flat floor; no dynamic obstacles; wall recall excludes surfaces never observed in the scan band; replay determinism shown for one bag; map ID/version are per-run metadata (versioning on map updates is Phase 4).

## Phase 4 — Map reload, LocalizationState and failure gates

**Date:** 2026-11-28 · **Backend:** slam_toolbox 2.6.10 `localization_slam_toolbox_node`, `config/autonomy/slam_toolbox_localization.yaml` (mapping config + the package's localization differences) · **Map:** `runs/phase3/accept_seed7` (`ultron-hall-be73b8cbf8bf`, v1) · **Code:** `src/ultron/autonomy/{localization_state,localization_monitor,run_localization,eval_localization}.py`, scenario steps in `sim_bridge.py` · **Command:** `scripts/sim-slam.sh localize OUT --map MAP_RUN [--scenario FILE] [--offset DX DY DYAW_DEG]`

**LocalizationState** (`ultron.localization_state/v1`, JSON on `/ultron/sim/localization_state`): `stamp` (scan time the pose refers to), `evaluated_at`, `clock_id`, `frame_id`/`child_frame_id` (`map`/`base_link`), `map_id`, `map_version`, `position` (m), `orientation_xyzw`, `state` ∈ {INITIALIZING, VALID, DEGRADED, LOST, STALE}, `reason`, `match_score`, `covariance_xy_yaw` (null: slam_toolbox provides none; `covariance_source` says so), `age_s`, `age_limit_s`, `source`, `ground_truth_used: false`. Consumers gate with `accept_pose(state, map_id, map_version, now)` (rejects map mismatch, non-VALID, stale, future stamps). Observations use `TransformHistory.transform_at(t)` (interpolates; rejects extrapolation, too-old, gaps > 0.1 s, non-VALID).

Validity rules (`config/autonomy/sim_slam.yaml: localization`, fixed before evaluation): match score = fraction of scan endpoints within 0.15 m of an occupied map cell; VALID needs 5 consecutive scans ≥ 0.70 (10 to leave LOST/STALE); < 0.70 → DEGRADED; ≤ 0.40 for 5 scans or DEGRADED > 3 s → LOST; no scan / transform for 0.5 s → STALE; pose jump > 0.5 m or 0.35 rad → DEGRADED. **A pose at time t is labelled VALID only after a scan taken at or after t has been scored VALID** (added after the first kidnap run, below).

Thresholds (evaluation): no VALID sample with true error > 0.25 m (all scenarios); first VALID ≤ 10 s; while VALID RMSE ≤ 0.10 m, max ≤ 0.25 m, yaw RMSE ≤ 2°; blackout → STALE ≤ 0.75 s and VALID again ≤ 5 s after scans resume; kidnap leaves VALID ≤ 2 s.

| Scenario (1.5×) | Verdict | Results |
|---|---|---|
| Reload, initial guess 0.5 m / 15° off (`runs/phase4/final_reload_offset`) | PASS | slam_toolbox corrected the guess on its first scan; first VALID 0.40 s; while VALID RMSE 0.040 m, max 0.093 m, yaw 0.73°; false VALID 0 |
| Reload, exact guess (`reload_zero`) | PASS | RMSE 0.039 m, max 0.088 m |
| Lidar blackout 3 s while moving (`final_blackout`) | PASS | VALID → STALE 0.48 s after the last scan; VALID again 1.08 s after scans resumed; false VALID 0 |
| Kidnap: true pose teleported 3.78 m, odometry not told (`kidnap_v2`) | PASS | VALID → DEGRADED 0.08 s after teleport (score 0.07) → LOST; slam_toolbox relocalized, VALID again at +8.2 s, final error 0.04 m; false VALID 0 |
| Kidnap, first implementation (`kidnap`) | **FAIL** (kept for the record) | 2 false VALID samples (error 3.78 m): poses between the teleport and the next scan inherited the previous scan's VALID. Fixed by the later-scan confirmation rule |
| Stress (beyond plan): guess 3 m off (`stress_offset3m`) | FAIL on convergence time only | LOST while wrong (error up to 3.0 m), first VALID at 15.2 s (> 10 s), **false VALID 0**, then RMSE 0.043 m |
| Opt-in end-to-end: fresh map at 2×, then kidnap on it | PASS | kidnap detected 0.08 s, VALID RMSE 0.043 m, false VALID 0 (`LocalizationRunTest`); with bridge + mapping opt-in tests: 23 tests OK |
| Full suites | PASS | System Python 101 (16 skipped: MuJoCo/ROS); `.venv-ros` rendered 101 (5 skipped: opt-in) |
| Unit tests: transitions, data arrival never validates, stale/recovery, jump, gate, interpolation/rejections, later-scan confirmation, scenario/threshold validation | PASS | `tests/test_autonomy_localization.py` (14) |

Findings fixed during the phase:
1. False VALID during a kidnap (above): later-scan confirmation rule.
2. Orchestrators killed by SIGTERM (`timeout`) left slam_toolbox and the monitor running (own process groups). SIGTERM now raises KeyboardInterrupt so cleanup runs; verified no orphans after `timeout 15`.
3. `map_start_pose` is applied by slam_toolbox only when the first scan is processed (identity `map -> odom` before); confirmed with a 3 m offset (parameter `[3.0, 0.0, 0.0]`, pose jumped to x = 2.88). Samples before the first scored scan are INITIALIZING.

Limitations: match score measures agreement with the map, not accuracy; a repetitive environment could match confidently at a wrong pose. No covariance from the backend. Single hall and route family; static scene; one map. Consumers see VALID ~0.25 s after a pose's time (scan period + scoring delay). Map versioning beyond v1 and Nav2 integration are next.

## Phase 5 — Robustness and realism

**Date:** 2026-11-28 · **Code:** time-sliced lidar (`lidar.py`), walking people (`hall_scene.py`), bridge options `--people`, `--motion-distortion`, `--odom-scale`, `--lidar-profile`, `--control-hz`, scenario fault `lidar_dropout`, `latency_probe.py` (scan→TF latency, CPU/RAM sampler) · **Matrix:** `runs/phase5/matrix.sh` (local). Thresholds unchanged from Phases 3/4; nothing retuned.

Conditions: *walking* = three people walking 1 m/s (replacing the static ones; map reference excludes people); *odom 3×* = all odometry error terms ×3 (final drift 4.1 m RMSE); *distortion* = each 36° slice of a sweep cast from its own true pose, stamp = sweep start, no deskewing; *20 Hz* = 0.4° columns at 20 Hz (100 Hz control); *dropout* = 60 % of points dropped for 6 s while moving.

Mapping (`hall_survey_loop`, 1.5×, 20 Hz at 1×):

| Run | Verdict | ATE RMSE / max | Yaw RMSE | Odom-only RMSE | Precision | Wall recall | Scan→TF latency median/p95 |
|---|---|---|---|---|---|---|---|
| Baseline (regression after lidar changes) | PASS | 0.069 / 0.160 m | 1.15° | 1.25 m | 0.981 | 0.951 | 0.08 / 0.10 s |
| Walking people | PASS | 0.055 / 0.137 m | 0.98° | 1.25 m | 0.985 | 0.959 | 0.06 / 0.10 s |
| Odometry error 3× | PASS | 0.059 / 0.151 m | 1.71° | 4.10 m | 0.999 | 0.956 | 0.08 / 0.10 s |
| Motion distortion | PASS | 0.049 / 0.091 m | 0.67° | 1.25 m | 1.000 | 0.955 | 0.18 / 0.20 s (stamp = sweep start) |
| 20 Hz lidar | PASS | 0.051 / 0.128 m | 0.84° | 1.28 m | 0.995 | 0.968 | 0.05 / 0.05 s |
| Combined: walking + distortion + odom 3× | PASS | 0.068 / 0.137 m | 1.63° | 4.10 m | 0.971 | 0.955 | 0.18 / 0.20 s |

Localization on the Phase 3 map (1.5×; initial guess 0.5 m / 15° off unless noted):

| Run | Verdict | First VALID | VALID fraction | False VALID | While VALID RMSE / max | Notes |
|---|---|---|---|---|---|---|
| Baseline (regression) | PASS | 0.4 s | 1.00 | 0 | 0.040 / 0.086 m | |
| Walking people | PASS | 0.4 s | 1.00 | 0 | 0.041 / 0.094 m | median match score 0.97 |
| Odometry error 3× | PASS | 0.4 s | 0.99 | 0 | 0.042 / 0.089 m | |
| Motion distortion | PASS | 0.4 s | 0.99 | 0 | 0.048 / 0.085 m | |
| Dropout 60 % for 6 s (exact guess) | PASS | 0.4 s | 1.00 | 0 | 0.041 / 0.091 m | VALID throughout dropout, max error 0.027 m |
| Combined kidnap: walking + distortion + odom 3× | PASS | 0.4 s | 0.81 | 0 | 0.055 / 0.120 m | kidnap detected 0.38 s after teleport (0.08 s without distortion); error up to 3.79 m only while not VALID |

Resources at real time (1.0×, 16-core laptop; % of one core; simulator bridge is not deployed on the robot):

| Process | Mapping | Localization |
|---|---|---|
| slam_toolbox | 8.4 % mean, 17.3 % p95, 85 MB | 26.2 % mean, 83.8 % p95, 92 MB |
| ULTRON logger / localization monitor | 14.0 %, 65 MB | 16.8 %, 70 MB |
| Simulator bridge (MuJoCo + lidar) | 176 %, 110 MB | 177 %, 110 MB |
| Scan → `map->odom` update latency | median 0.06 s, p95 0.08 s, max 0.10 s | same |

Latency is sim-time from a scan's stamp to the first `map->odom` carrying it (slam_toolbox updates it for every received scan, measured in a bag); it is not full scan-matching time. Jetson Orin figures are not measured.

Findings fixed during the phase:
1. **Concurrent `mj_multiRay` on one `MjData` is unsafe** (mujoco 3.3.7): 0–31 rays per distorted sweep differed run to run. Same-origin sweeps had shown 0/76,800 mismatches against exact rays, so Phases 1–4 results were not visibly affected, but the race was real. Each worker now owns an `MjData` synced before every scan; regression test asserts threaded == single-threaded bit for bit. Baselines re-run after the fix: PASS (rows above).
2. Resource sampler named its stop event `_stop`, overriding a `threading.Thread` method, and crashed on join; the first matrix attempt therefore left slam_toolbox/logger processes running, and **later runs overlapped with several slam_toolbox instances** in one domain. That attempt is discarded (`runs/phase5/invalid_first_attempt/`). Fixes: rename, children stopped even if sampling fails, and a preflight that refuses to start while any simulation stack process runs.
3. The sampler thread died silently when process groups were added after start (dictionary changed during iteration); now locked, and errors are recorded instead of killing it.
4. The localization evaluator applied blackout checks (must go STALE) to the dropout fault; only blackouts are now checked that way (dropout must not produce false VALID, covered globally).
5. Process note: editing `sim_slam.yaml` while the matrix ran broke one run's final evaluation (20 Hz); it was re-evaluated afterwards from its saved outputs.

Deferred: long low-feature corridor (degenerate along-track geometry) — not representative of the ~10 × 10 m course; noted as an open risk for corridor-like venues.

Unit tests: `tests/test_autonomy_robustness.py` (7): per-slice pose casting exact, static pose function equals undistorted, threaded determinism, walkers on their segments, people modes, dropout scenario and 20 Hz profile, sampler/latency summary. Full suites: system Python 117 (23 skipped), `.venv-ros` rendered 117 (5 skipped), ROS/SLAM opt-in 23 OK.

## Nav N0–N2 — Nav2 navigation behind the motion gate

**Date:** 2026-11-28 · **Stack:** Nav2 1.1.20 (Humble binaries: map_server, planner_server/NavFn, controller_server/RotationShim + Regulated Pure Pursuit, behavior_server, bt_navigator, lifecycle_manager), `config/autonomy/nav2_params.yaml` (generated from the package defaults; overrides listed in its header), slam_toolbox localization on `runs/phase3/accept_seed7` (`ultron-hall-be73b8cbf8bf` v1), `localization_monitor`, `motion_gate_node` · **Code:** `src/ultron/autonomy/{navigation_contract,nav2_adapter,motion_gate_node,run_navigation}.py`; test hooks: bridge `/ultron/sim/sim_control` (test obstacles, lidar blackout) · **Command:** `scripts/sim-slam.sh nav OUT --map MAP_RUN [--cases …]` · **Plan:** `docs/SAHIL_NAV_PLAN.md`

Command path: Nav2 publishes only `/ultron/sim/nav/cmd_vel_raw`; the motion gate alone publishes `/ultron/sim/cmd_vel` and forwards only while LocalizationState is VALID and fresh for the loaded map, operator enable is true and re-asserted within 1.0 s, and the Nav2 command is younger than 0.3 s; otherwise it publishes zero. Missing inputs inhibit. The adapter cancels the active goal when the gate inhibits for operator/localization/map reasons.

Thresholds (fixed before the runs): arrival ≤ 0.25 m / 0.35 rad true error; stop within 1.2 s (0.3 s command age + 0.8 s braking from 0.4 m/s at 0.5 m/s² + 0.1 s); gate after operator disable ≤ 0.10 s; gate after blackout ≤ 0.80 s; unreachable goal bounded by its 60 s timeout; no motion while localization is not VALID (≤ 0.02 m).

Run `runs/nav/all2` (1.0×, one session, cases in order):

| Case | Verdict | Result |
|---|---|---|
| N1 reach near (2.5, 5.5) | PASS | REACHED → CAPTURE_READY after settle; true error 0.187 m / 0.003 rad |
| N1 reach far (9.6, 4.5) | PASS | REACHED → CAPTURE_READY; true error 0.197 m / 0.004 rad |
| Occupied target (ladder) | PASS | REJECTED before Nav2: `TARGET_FOOTPRINT_OCCUPIED: 105 occupied cells within 0.55 m`; 0.0 m motion |
| Target outside map | PASS | REJECTED: `TARGET_OUTSIDE_MAP`; 0.0 m motion |
| Unreachable (test obstacle placed on the target) | PASS | Nav2 kept retrying recoveries; adapter TIMEOUT at 60.04 s, backend confirmed cancellation; robot still 0.22 s later |
| New obstacle 1.6 m ahead while moving | PASS | replanned around it, REACHED; 0 collision stops |
| Cancellation | PASS | CANCELLED; last motion (translation or rotation) 0.68 s after cancel |
| Operator inhibit | PASS | gate `OPERATOR_DISABLED` 0.02 s after disable; goal CANCELLED; stopped 0.74 s |
| Localization loss (3 s lidar blackout) | PASS | gate `LOCALIZATION: STALE` 0.42 s after blackout; goal CANCELLED; stopped 1.2 s; 0.0 m motion while not VALID |
| Contract unit tests (status mapping, pre-checks, late results, timeout, settle, gate defaults, supervisor mapping) | PASS | `tests/test_autonomy_navigation.py` (9) |

First full run (`runs/nav/all1`) recorded 8/9: the unreachable case's "stopped" check sampled a window 0.5–2.0 s after cancellation (inside the 1.2 s braking budget the other cases use) and saw 0.027 m of recovery motion. The check was aligned to the same stop rule as the other cases. Review also found the motion check counted only translation, so a cancel during an in-place turn showed 0.0 s; rotation now counts as motion. Both runs' raw logs are kept locally.

Findings: the stack preflight first matched free text and flagged the agent's own shell; it now matches exact argv tokens and skips its own ancestors. Nav2 warnings during the run were expected for the failure cases (planner failures, RPP collision-ahead, controller patience exceeded).

Limitations: kinematic base; one map and hall; goals chosen by the test; Nav2's own recovery behaviours (spin/back-up) move the robot during failure handling; supervisor-driven mission (N3) not yet run; the gate is software in the same computer, not an independent watchdog or E-stop.

## Recorded B2 LiDAR replay

28 November 2026. ROS 2 Humble, slam_toolbox 2.6.10 and KISS-ICP 1.3.0. The source is the team recording `rosbag2_2026_11_28-00_48_42`. The local source ZIP has SHA-256 `00406d92a93c2961c01c97831d408c1358a4fae23b298baa3267c19ee968f20d`.

`ros2 bag info` reports 36.56 s, 364 PointCloud2 messages on `/scan_cloud`, IMU streams and RealSense frames. Recorded TF consists of `world -> imu_link` and the static head camera transforms. The recording has no base odometry, reference trajectory or LiDAR-to-base transform.

Run these commands from the repository root with the existing ROS Humble mapping dependencies and an extracted SQLite bag.

```bash
scripts/setup-ros-venv.sh
.venv-ros/bin/python -m pip install 'kiss-icp==1.3.0'
scripts/sim-slam.sh real-bag runs/b2_real/rosbag2_2026_11_28-00_48_42 runs/b2_real/slam_replay
```

Use a new output folder and stop any active local SLAM or Nav2 stack before replay. The launcher confines ROS to localhost. Only the derived 2D scans, odometry and TF are replayed. KISS-ICP estimates translation and rotation from the LiDAR clouds. The first ten clouds provide the floor estimate and define a replay base directly below the levelled LiDAR. The projection uses a 0.25–1.60 m height band with 0.5° rays. The derived messages and replay clock use the LiDAR acquisition timestamps.

The run exports the occupancy grid, serialized pose graph, versioned map metadata, odometry and mapping trajectories, and the reload monitor's scan-by-scan localization states. The reload process starts with the mapping origin as its pose guess and uses the same recording. This checks map persistence and scan alignment on familiar data. Endpoint separation measures the closure of each estimated trajectory, and pose error against ground truth is unavailable.

Run `runs/b2_real/slam_replay_02` used 1× playback.

| Check | Result |
|---|---|
| Recorded input | All 364 clouds processed in 36.30 s of LiDAR acquisition time (10.00 Hz). Estimated LiDAR height 0.722 m and tilt 0.80°. |
| Map save and serialization | Both services succeeded. The 5 cm occupancy map has 5,634 occupied cells. The mapping logger resolved 361 poses and missed the first three timestamps during startup. |
| Estimated path and endpoint separation | KISS-ICP estimated a 17.40 m path and 0.137 m endpoint separation. slam_toolbox estimated an 18.66 m path and 0.237 m endpoint separation. The earlier team KISS-ICP result was about 0.15 m. |
| Map reload | A fresh localization process deserialized the graph. Of 362 scored scans, 8 were INITIALIZING, 349 VALID and 5 DEGRADED. It reached VALID at 0.80 s, briefly degraded at 29.70 s, and recovered after 0.50 s. The last two scans arrived within the monitor's scoring delay of playback shutdown. |

The raw bag and run artifacts are local. This run provides no evidence for localization on a separate patrol, measured sensor calibration, or physical navigation.

## Dashboard goals and the SDK wrapper

28 November 2026. Run `runs/dashboard-nav-e2e-03` used ROS 2 Humble,
Nav2 1.1.20, slam_toolbox 2.6.10, the existing simulated hall map
`ultron-hall-be73b8cbf8bf` v1 and the kinematic MuJoCo bridge. Goals were
submitted through the dashboard's HTTP API. [Launch and reproduction commands](dashboard_nav_goal.md)

| Check | Observed result |
|---|---|
| Saved map and pose | The dashboard received the matching map ID and version with a valid localization pose. |
| Goal rejection | Disabled operator, wrong map version, occupied footprint, out-of-map target, and stale localization after an injected LiDAR blackout each returned `REJECTED` with a reason. Simulated displacement stayed within 2 cm during these checks. |
| Goal arrival | Nav2 reported success. Simulated ground truth was 0.182 m and 0.119 rad from the goal, within the existing 0.25 m and 0.35 rad thresholds. |
| Goal replacement | The first goal received backend cancellation acknowledgement before the replacement was dispatched. The replacement succeeded. |
| Operator inhibition during navigation | The gate inhibited output in 0.06 s, Nav2 confirmed cancellation, and kinematic motion stopped in 0.84 s. |
| Gated commands and shutdown | The recording SportClient received 345 `Move` calls and five `StopMove` calls across the two cases. The final call was the shutdown stop. |

The separate ROS test published commands on both velocity topics in the
simulation domain. Only the motion-gate output topic reached the stand-in.
After command silence, the wrapper's steady-clock watchdog issued `StopMove`.
The interactive launch also published the map and pose and rejected a goal
without an operator heartbeat. Its SDK wrapper was off by default.
Unit tests also exercised clamping, zero commands, unsupported axes,
nonfinite values and SDK error handling. The configured limits are simulation
values. The SDK package and physical B2 transport remain untested. The pinned
SportClient sends `Move` without waiting for a robot reply. [Unitree source](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/b2/sport/sport_client.py#L125)

## Live dashboard mapping

On 28 November 2026, `runs/live-map-replay-01` rebuilt a map from the
simulated hall recording at 2× playback. The HTTP probe captured four grid
updates with three distinct PNGs and 18 changing `MAPPING_ESTIMATE` poses
while mapping ran. It received HTTP 409 with `LIVE_MAP_PREVIEW` on a goal
request to the live grid. Map save and pose-graph serialization both succeeded.
The probe log and sampled PNGs are under `runs/live-map-dashboard-test/`.
[Launch instructions](dashboard_nav_goal.md#watch-the-map-while-it-is-being-built)

## 3D LiDAR SLAM on the recorded B2 scan

The offline path gives KISS-SLAM each full `/scan_cloud` point set at its
acquisition timestamp. It estimates 6-DoF poses in the initial LiDAR frame,
runs its submap pose graph, and writes a merged PLY, a graph and timestamped
trajectories. The comparison is a separate KISS-ICP-only run on the same input
and odometry settings. Both runs use LiDAR only and leave scan deskew disabled.
[KISS-SLAM project and paper](https://github.com/PRBonn/kiss-slam)

Install into a separate target so these versions stay out of the regular ROS
mapping environment. Run from the repository root with the extracted bag.

```bash
.venv-ros/bin/python -m pip install --target runs/kiss-slam-python \
  'kiss-slam==0.0.2' 'kiss-icp==1.3.0' 'map-closures==2.0.2' \
  'open3d==0.20.0' 'numpy==1.26.4'
PYTHONPATH="$PWD/runs/kiss-slam-python" PYTHONNOUSERSITE=1 \
  scripts/sim-slam.sh slam3d runs/b2_real/rosbag2_2026_11_28-00_48_42 runs/b2_real/slam3d
```

All three trials processed the 364 scans and produced five submaps. The
recording has no ground-truth trajectory. The values below are estimated
3D distances between the first and last poses.

| Trial | Merged 3D points | Accepted closure edges | KISS-SLAM endpoint separation | Odometry-only endpoint separation |
|---|---:|---|---:|---:|
| `slam3d_kiss_01` | 55,243 | 0 → 4 | 0.160 m | 0.152 m |
| `slam3d_kiss_02` | 55,420 | none | 0.166 m | 0.152 m |
| `slam3d_kiss_03` | 55,262 | none | 0.177 m | 0.152 m |

The first trial's backend overlap log was 1.195, outside the range of a
fraction. Treat the accepted edge as provisional. The script now records
bounded nearest-neighbor fractions for any accepted edge. The two repeats
had no accepted edge to score. This recording demonstrates 3D reconstruction
and full-pose export. It leaves reliable loop closure and any accuracy benefit
unverified.

The local run folders contain `map.ply`, `slam.csv`, `odometry.csv`,
`submaps.g2o`, `config.json` and `summary.json`. The optional
`slam3d_kiss_02/map_view.html` shows a cropped view around the route.

## Dashboard navigation on the live SLAM grid

On 28 November 2026, `runs/live-nav-05` started SLAM Toolbox in mapping mode
and Nav2 1.1.20 on ROS 2 Humble. The launch used an empty map. The ROS graph
confirmed that Nav2's global costmap subscribed to the live `/map`, the same
topic used by the dashboard and the goal checker.

| Check | Result |
|---|---|
| Dashboard goal on a changing map | The HTTP request specified the grid displayed at 13.5 simulated seconds. The checked action logged that it validated the target on the 14.5 s grid and forwarded it to Nav2. The dashboard had received the 17.5 s grid by the time the goal completed. |
| Goal outcome | Nav2 reported `SUCCEEDED`. The simulated pose was 0.172 m and 0.004 rad from the goal, within the existing 0.25 m and 0.35 rad limits. |
| Rejections | Future and stale grid revisions and an out-of-map target were refused. The simulator stayed within 2 cm of its initial position during the rejection cases. |
| Active target revalidation | A focused test verified that an occupied cell in the latest grid causes the adapter to request cancellation of an active goal. |
| Saved-map regression | `runs/live-nav-saved-regression` passed the existing dashboard navigation case. |

The first run stayed in `INITIALIZING` because the saved-map scan matcher
counted returns in unmapped space as mismatches. The live matcher requires at
least 55% of scan endpoints to be in mapped space or near a mapped obstacle.
It then applies the existing agreement threshold to that supported subset,
where returns in mapped free space count against agreement. This is a
simulation check on scan-to-map consistency. The run does not establish
independent pose accuracy, performance across a large loop closure, or
physical B2 operation.

[Launch instructions](dashboard_nav_goal.md#navigate-while-mapping)
