# Isaac Sim feasibility for the ULTRON SLAM/Nav2 stack

**Date:** 2026-11-28 · **Status:** feasibility check, not a port. MuJoCo remains the validated SLAM/Nav2 test bench (docs/SAHIL_VALIDATION.md).

## Installed here

| Item | Finding |
|---|---|
| Isaac Sim | 6.1.0 release candidate (`~/isaac-sim`), Python 3.12, headless works on the RTX 3070 8 GB |
| ROS 2 bridge | `isaacsim.ros2.bridge` with bundled Humble/Jazzy libraries; `ros_distro = system_default`. System rclpy (Python 3.10) cannot load, so Isaac uses its internal Humble rclpy |
| RTX lidar | New API `isaacsim.sensors.experimental.rtx.Lidar`. Vendor configs (HESAI, NVIDIA, Ouster, SICK, Slamtec, ZVISION) resolve to USD assets on NVIDIA's remote asset server. **No RoboSense/Helios profile.** A bare OmniLidar prim can be created locally; emitter patterns are a multi-instance USD schema (`omni:sensor:Core:emitterState:<name>:elevationDeg`, …) |
| Assets | Default asset root is NVIDIA's public S3; none installed locally. The smoke test uses no remote assets |
| Robots | Isaac Lab 3.0 references Unitree A1, Go1, Go2, G1, H1 — **no B2** asset or locomotion task. MJCF and URDF importers are installed, so the official B2 MJCF in this repo can be imported (mesh-free variant, as ULTRON already builds) |

## Smoke test (executed)

`scripts/isaac/run-smoke.sh --seconds 25` (Isaac's interpreter, `ROS_DOMAIN_ID=71`, `ROS_LOCALHOST_ONLY=1`, bundled Humble libraries) with `scripts/isaac/check_points.py` listening in `.venv-ros`:

| Check | Result |
|---|---|
| Headless start, primitive 8 × 6 m room, bare RTX lidar at 0.83 m, 10 Hz | PASS: 1,672 app frames in 25 s; GPU memory back to idle afterwards |
| PointCloud2 on `/ultron/sim/isaac/points` received by ULTRON's ROS environment | PASS: 50 messages, ~76,000 points each, frame `lidar_link`, ranges 1.5–4.3 m (consistent with the room) |
| Lidar pattern | Default bare prim has **128 emitters**, not Helios-32: a custom emitter pattern must be authored |
| Point fields | x, y, z only (no ring/time/intensity) |
| Message rate | ~3 Hz observed; the Python listener converted every point and is the likely bottleneck — rate **not measured** reliably |

## What carries over unchanged

Everything that consumes ROS topics: slam_toolbox configs, map save/serialize/reload, LocalizationState monitor and consumer gate, motion gate, Nav2 params and adapter, evaluation of trajectories (given ground-truth CSVs and run metadata in the same format).

## Work needed for parity with the MuJoCo bench (estimate)

| Item | Effort | Notes |
|---|---|---|
| Helios-32 emitter pattern as local USD/attributes from `config/autonomy/lidar_helios32.yaml` | ~0.5 day | still blocked on the confirmed variant/channel table |
| Hall scene in USD generated from `hall_scene.py` primitives | ~0.5 day | ladders as boxes/cylinders, same dimensions |
| B2 import (mesh-free MJCF) + kinematic base driven by the existing `driver.py` | ~0.5–1 day | physics walking needs a B2 policy that does not exist here |
| Isaac-side bridge: clock, odometry drift (`odometry.py`), TF, 2D scan (`scan2d.py`), ground truth, sim-control hooks | ~1 day | pure ULTRON modules can be imported by Isaac's Python 3.12 |
| Evaluation reference surfaces from Isaac (noise-free lidar pass) | ~0.5 day | |

Roughly 3–4 days for kinematic parity. Benefits: realistic RGB/depth for the camera detector, RTX lidar material effects, and a path to physics walking if a B2 policy becomes available. Recommendation: after hand-in, as a second simulator behind the same topic contract; not on the critical path.
