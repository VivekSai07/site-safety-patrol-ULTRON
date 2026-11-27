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
