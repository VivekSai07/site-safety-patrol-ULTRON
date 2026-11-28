# B2 ROS 2 stack — validation record

Actual results only (PASS / FAIL / UNRUN with command, revision, config,
data provenance, artifact path). Simulation results are not hardware evidence.
Each agent edits only its own section. The M6 format follows
`docs/SIM_TO_HARDWARE.md`.

## Agent A — Foundation
Revision: branch `om/b2-ros-stack-a` (results below measured at `4801856`; docs commit follows).
Environment: x86_64 host without ROS; everything ROS ran in `ultron-b2-dev:humble`
(`ros2_ws/docker/Dockerfile.dev`, base `ros:humble-ros-base`) via `ros2_ws/scripts/dev.sh`
(uid-mapped, private bridge network, `ROS_LOCALHOST_ONLY=1`, domains 71–74, never 0).
No hardware, robot network, bag or live launch touched. Simulation/stub results only.

| Check | Command | Result |
|---|---|---|
| Dev image builds; required apt set installed | `docker build -f ros2_ws/docker/Dockerfile.dev -t ultron-b2-dev:humble ros2_ws/docker` | PASS (navigation2 1.1.20, slam_toolbox 2.6.10, foxglove_bridge 3.6.3, domain_bridge 0.5.0, twist_mux 4.3.0, rosbag2_storage_mcap 0.15.17) |
| Optional binaries (`/opt/ultron/apt_optional.txt`) | `docker run --rm ultron-b2-dev:humble cat /opt/ultron/apt_optional.txt` | mola 3.6.0, mola_lidar_odometry 3.6.0, mola_state_estimation 3.4.0, mp2p_icp 3.2.1, robot_localization 3.5.5, spatio_temporal_voxel_layer 2.3.4, realsense2_camera_msgs 4.59.2 INSTALLED; **ros-humble-kiss-icp, ros-humble-rslidar-msg UNAVAILABLE** (pip `kiss-icp==1.2.3` installed; rslidar_msg built from source) |
| Pinned sources import + unitree msgs build | `scripts/import_deps.sh && colcon build --symlink-install` | PASS: unitree_api, unitree_go, unitree_hg (unitree_ros2 `668d1ec`), rslidar_msg (`fe8a95c`). No B2-specific unitree msg package exists upstream; B2 uses `unitree_go`/`unitree_api` |
| Workspace build (16 packages) | `ros2_ws/scripts/dev.sh bash -lc "colcon build --symlink-install"` | PASS, 0 errors, no build stderr |
| `ci.sh` from a clean clone | `git clone -b om/b2-ros-stack-a … && ros2_ws/scripts/dev.sh scripts/ci.sh` | PASS (deps import, `rosdep check`, build, 94 ROS tests, repo suite) |
| colcon test (interfaces, description, bringup) | `colcon test --packages-select-regex '^ultron_b2_' && colcon test-result --verbose` | PASS 94 tests, 0 failures |
| Interface converter round trips | `ultron_b2_interfaces/test/test_json_roundtrip.py` | PASS 38 (LocalizationState ↔ `ultron.localization_state/v1`, NavigationGoal/Result ↔ NavigateToView, generic round trip of every msg/srv/action) |
| Description | `ultron_b2_description/test/test_description.py` | PASS 9 (frames, lidar mount `[0.342, 0, 0.18]`, real.yaml all `TO_VERIFY`, footprint) |
| Preflight unit tests | `ultron_b2_bringup/test/test_preflight.py` | PASS 26 (wrong domain → refuse; live + hardware_enable without/expired/invalid token → refuse; replay never launches control) |
| `sim.launch.py mode:=mapping` starts stubs under `/ultron/sim` | `test_sim_launch.py` (+ manual `ros2 node list`) | PASS: 11 nodes all under `/ultron/sim`, no `[ERROR]`, clean SIGINT shutdown |
| Replay excludes nav/control/mission | `test_sim_launch.py::test_replay_starts_no_control_nodes` | PASS |
| Wrong domain / namespace / hardware_enable / odom_source refused | `test_sim_launch.py` | PASS (nothing under `/ultron/` started) |
| `tf_authority_check`: two publishers of `odom→base_link` | `test_tf_authority.py` (domain 73) | PASS (exit 1, `MULTIPLE_PUBLISHERS`); single authority → exit 0; ground-truth frame, two parents, static/dynamic conflict → exit 1 |
| Compose sim smoke | `docker compose -f ros2_ws/docker/compose.sim.yaml up` (25 s) | PASS (preflight PASS, 9 stubs + RSP + tf check started) |
| Compose replay | `docker compose … compose.replay.yaml config` | config validated only; run UNRUN (no bag) |
| Repo fast suite | `PYTHONPATH=src python3 -m unittest discover -s tests` | PASS in container (411, skipped 22) and with the team `.venv` on host (411, skipped 22). Host system python3 lacks mujoco/fastapi → 2 import errors (environment only) |
| Orin image | `ros2_ws/docker/Dockerfile.orin` | **UNRUN** (arm64; `BASE_IMAGE` TO_VERIFY against the Orin JetPack) |
| GitHub workflow | `ros2_ws/ci/github-workflow.ros2_ws.yml` | UNRUN, inactive template pending user approval (AGENT_A.md: script only) |

Known limitations / notes:
- `ros2 launch` creates a DDS participant before the in-launch preflight refuses; use
  `ros2_ws/scripts/ultron_launch.sh` (env check first). Verified: domain 74 → exit 2 before launch.
- Cyclone on loopback has no multicast; `cyclonedds_localhost.xml` raises
  `MaxAutoParticipantIndex` to 120 (default 9 refused the 12th participant). Required for the stack.
- Graph tests are pytest subprocess tests rather than `launch_testing` test descriptions.
- Sim TF: `robot_state_publisher` owns `base_link → sensors`; legacy `sim_bridge` also publishes a
  static `base_link → lidar_link`, which `tf_authority_check` will flag until Agent F's wrapper
  disables one.

## Agent B — Ingestion, replay, preprocessing
_No results yet._

## Agent B (bag) — recovery and characterization of the real B2 recording

Revision: branch `om/b2-ros-stack-bag` (base `4488d20`). Data: local
`rosbag2_2026_11_28-00_48_42.zip` (1,987,480,517 B), extracted outside any
repository to `/home/azut25r/workspace/bags/rosbag2_2026_11_28-00_48_42/`; never
committed or uploaded. Reports: `/home/azut25r/workspace/bags/inspect/…` and
`/home/azut25r/workspace/bags/recovery_report.json` (local). Findings:
[BAG_FACTS.md](BAG_FACTS.md). Environment: Python 3.10 venv
`/home/azut25r/workspace/bags/.venv` with `pip install rosbags numpy kiss-icp`
(rosbags 0.11.11); sqlite3 3.37.2 CLI unpacked locally via `apt download sqlite3`
(no root, not installed system-wide). No ROS, no hardware, no network to robots.

| Check | Result | Command / evidence |
|---|---|---|
| Zip diagnosis | PASS — zip complete and valid (EOCD, zip64 extra, data descriptors); same size as completed browser download, no partial download | signature scan + `unzip -l` + `zipfile` |
| Extraction + salvage | PASS — CRC verified, `integrity_check` ok, 71,375/71,375 messages (100 % per topic) | `python3 ros2_ws/tools/recover_bag_zip.py <zip> --out /home/azut25r/workspace/bags --sqlite3-cli <local sqlite3>` |
| Page-scan fallback on the real 4.3 GB db3 | PASS — rebuilt DB has identical per-topic counts and identical sampled rows | `recover_by_page_scan()` vs original (local check) |
| Bag characterization | PASS — topics, TF, lidar, IMU, camera facts in BAG_FACTS.md | `.venv/bin/python ros2_ws/tools/bag_inspect.py <bag_dir> --out …/inspect/<name> --cloud-every 3 --image-every 10` (≈5 s) |
| Motion estimate | PASS (estimate, unvalidated) — ≈19.4 m KISS-ICP path, 365° yaw range | local ad-hoc KISS-ICP script (not committed) |
| Unit tests (synthetic bag, complete + truncated zip) | PASS (3 tests) | `PYTHONPATH=src /home/azut25r/workspace/bags/.venv/bin/python -m unittest tests.test_bag_tools -v` |
| Fast suite (system python, rosbags absent → bag tests skipped) | 411+3 tests: 2 errors, both **pre-existing on base** (`test_model_safety…fails_fast_before_model_load`, `test_patrol_dashboard_integration…`) | `PYTHONPATH=src python3 -m unittest discover -s tests` |
| `/scan_cloud` re-decode from `/rslidar_packets` | UNRUN — not viable for this bag (58 % of MSOP packets recorded) | — |
| Replay launch, driver configs, cloud filter, calibration yaml (rest of AGENT_B.md) | UNRUN — second half of Agent B | — |

Hand-back: works — `recover_bag_zip.py`, `bag_inspect.py` (standalone, no ROS).
Blocked/open — lidar/IMU/head-camera to body extrinsics (measure on robot,
owner: operator + B), depth scale and Helios variant (device check), B2
odometry topic (C). Next: Agent B second half (replay launch with remaps from
BAG_FACTS.md).

## Agent C — State estimation, mapping, localization
_No results yet._

## Agent D — Navigation, safety chain, control
_No results yet._

## Agent E — Mission, exploration, ladder search
_No results yet._

## Agent F — Sim parity, visualization, integration

### Part 1: sim wrapper, viz, recording (2026-11-28, branch `om/b2-ros-stack-f`)

Environment: dev container `ultron-b2-dev:humble` (unchanged; it already has mujoco 3.x, Pillow,
websockets, foxglove_bridge, domain_bridge, rosbag2 mcap and rviz2). Domain 71, loopback, no
hardware. All results are SIMULATION: kinematic base plus MuJoCo raycast/render. This is not
walking, real perception or hardware evidence. Artifacts are in `runs/agent_f/` (git-ignored).

| Check | Result |
|---|---|
| `colcon test` interfaces, description, sim, viz, bringup | 108 tests, 0 failures |
| Repo suite `PYTHONPATH=src python3 -m unittest discover -s tests` | 414 run, OK (22 skipped) |
| `sim.launch.py mode:=mapping viz:=both record:=minimal`: contract rates (`topic_probe`, 20 s) | `/clock` 50.2, `sensors/lidar/points` 10.05, `sensors/scan` 10.0, `sensors/imu/data` 50.0, `state/odom` 50.0, `sim/ground_truth/base_pose` 50.0, cam_head color/depth `image_raw`/`image_rect_raw` 10.05/10.0, both `camera_info` 10.0, `sim/oracle/cam_head/ladder_labels` 10.1 Hz |
| `tf_authority_check` (10 s, during the run above) | 8 frames, 0 violations. `base_link→sensor` frames come only from robot_state_publisher; sim_bridge static TF is disabled |
| Camera (MuJoCo EGL, 640×360) | 516 frames, 4 skipped while busy; render mean 52 ms, first-frame max 486 ms. Centre depth matched `mj_ray` within 1 mm |
| cmd_vel reaction (`test_sim_bridge_graph.py`) | 0.3 m/s Twist published **directly** on `/ultron/sim/cmd_vel` for 3 s: moved 0.84 m, twist vx 0.318, stopped after the 0.5 s timeout. Agent D's motion_gate is not merged, so the gated path is UNTESTED here |
| Scenario service `sim/scenario` | `ladder_hide` applied (ground truth updated); unknown command rejected |
| Foxglove whitelist (`foxglove_probe`, WebSocket, subscribed to all) | 15 channels advertised, all whitelisted (`viz/*`, scan, odom, map, diagnostics, status, `/tf`, `/tf_static`). No raw lidar, raw images, ground truth, oracle or cmd_vel. 0 services. 2.53 Mbit/s total |
| viz_throttle | points 6.99 MB/s → 0.20 MB/s (2 Hz, voxel 0.10 m); image 6.64 MB/s → 0.074 MB/s (4.4 Hz JPEG) |
| domain_bridge 71 → 42 (`topic_probe` in domain 42) | Only `/ultron/viz/*`, `/tf`, `/tf_static` (plus ROS-internal `/rosout`, `/parameter_events`). 2.59 Mbit/s total vs 172.8 Mbit/s in domain 71 |
| Record (mcap + zstd) | minimal: 1.2 MiB / 52 s, 7 topics. debug: 0.74 MiB / 23 s, 13 topics. full: 143 MiB / 23 s incl. raw lidar and images. `ros2 bag info` reads all three |

Findings:
- **Best-effort readers lose large samples on loopback.** Raw clouds (~0.78 MB) and images (~0.69 MB)
  were never delivered to a `SensorDataQoS` (best-effort) subscriber with the host's default
  `rmem_max` of 212 KB. RELIABLE readers receive them at full rate. `viz_throttle` now reads RELIABLE
  by default. **Other agents' sensor subscribers in sim (e.g. B's preprocess) should use RELIABLE
  depth-1, or the integration should raise the socket buffers.** Not fixed in the shared DDS profile.
- Stop launches with SIGINT. From a non-interactive script, background jobs ignore SIGINT unless you
  use `set -m`. A SIGTERM'd recorder leaves no `metadata.yaml`.

UNRUN: zenoh-bridge-ros2dds (not installed; configs generated only), Isaac launcher (stub), live
domain-0 bridge, laptop-side RViz display check (no display), Orin, and the full-system acceptance
(part 2).

## M6 — Full-system simulation acceptance
_Not assembled. Reviewer decision: pending (human)._
