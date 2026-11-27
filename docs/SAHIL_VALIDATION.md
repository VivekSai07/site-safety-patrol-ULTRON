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
