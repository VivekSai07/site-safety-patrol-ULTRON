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

## Agent C — State estimation, mapping, localization
_No results yet._

## Agent D — Navigation, safety chain, control
_No results yet._

## Agent E — Mission, exploration, ladder search
_No results yet._

## Agent F — Sim parity, visualization, integration
_No results yet._

## M6 — Full-system simulation acceptance
_Not assembled. Reviewer decision: pending (human)._
