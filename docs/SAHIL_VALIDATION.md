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
