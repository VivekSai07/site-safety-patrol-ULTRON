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

Revision: branch `om/b2-ros-stack-b` (base `fb9f092`). Container `ultron-b2-dev:humble` via
`ros2_ws/scripts/dev.sh` (bridge network, `ROS_LOCALHOST_ONLY=1`, Cyclone localhost profile,
`--memory 6g --cpus 6`), WSL2 kernel 6.6, host `net.core.rmem_max` 212,992 B. Real bag
`/home/azut25r/workspace/bags/rosbag2_2026_11_28-00_48_42/` mounted **read-only** with
`ULTRON_BAG_DIR`; never copied or committed. Artifacts (local, ignored): `ros2_ws/runs/agent_b/`
(`replay6.*` = run reported below, `voxel_eval.json`). Replay script: `runs/agent_b/replay_probe.sh`
(launch + `stream_probe` 25 s + `tf2_echo`), i.e.

```bash
BAG=/home/azut25r/workspace/bags/rosbag2_2026_11_28-00_48_42
ULTRON_MODE=replay ULTRON_BAG_DIR=$BAG ULTRON_DEV_ARGS="--memory 6g --cpus 6" ros2_ws/scripts/dev.sh bash -lc \
  "source install/setup.bash && (scripts/ultron_launch.sh replay mode:=mapping bag:=$BAG &) && sleep 6 && \
   ros2 run ultron_b2_tools stream_probe --namespace /ultron/replay --duration 25 --topics sensors/lidar/points \
   sensors/lidar/points_filtered sensors/scan sensors/imu/data sensors/cam_head/color/image_raw \
   sensors/cam_head/depth/image_rect_raw sensors/cam_head/color/camera_info /clock \
   --proc cloud_filter pointcloud_to_laserscan 'ros2 bag play' --json runs/agent_b/<name>.json"
```

| Check | Result | Command / evidence |
|---|---|---|
| Build (`ultron_b2_{interfaces,description,preprocess,bringup,tools}`) | PASS | `colcon build --symlink-install --parallel-workers 4 --packages-up-to ultron_b2_bringup ultron_b2_tools` |
| colcon tests (preprocess 19 incl. node pub/sub, bringup replay table 5 + sim/replay launch 4 + tf authority 5, description 9+1, lint) | PASS — 80 tests, 0 failures | `colcon test --packages-select ultron_b2_preprocess ultron_b2_bringup ultron_b2_description ultron_b2_tools && colcon test-result --all` |
| Replay graph | PASS — `/rosbag2_player` (root ns) + `cloud_filter`, `pointcloud_to_laserscan`, `robot_state_publisher`, `tf_authority_check`, stubs (state_estimation, slam, viz, record); **no nav/control/mission**, no `cmd_vel` or `/api/*` topic | `ros2 node list`, `ros2 topic list` (`replay6.{nodes,topics}.txt`) |
| `sensors/lidar/points` | PASS — 10.04 Hz recv / 10.00 Hz stamp (bag 10.00), median 38,962 pts, ~1.0 MB, `lidar_link` | `replay6.json` |
| `sensors/lidar/points_filtered` | PASS — 10.0 Hz, median 8,154 pts (6,612–9,607), ~196 kB, `lidar_link`; voxel 0.10 m | `replay6.json` |
| `sensors/scan` | PASS — 10.0 Hz, `base_footprint`, 1,800 beams ±π, median 755 finite beams, median range 5.4 m (4.8–6.9), max ≤ 12 m | `replay6.json` |
| `sensors/imu/data` | PASS — 99.9 Hz recv / 99.8 Hz stamp (bag 99.81), `imu_link` | `replay6.json` |
| Camera topics | PASS — color image 29.5 Hz, depth 29.6 Hz, color camera_info 30.0 Hz (bag 29.99/29.82); frames `head_{color,depth}_optical_frame` | `replay6.json` |
| `/clock` | PASS — 99.7 Hz (`--clock 100`), nodes `use_sim_time:=true` | `replay6.json` |
| CPU / memory (dev PC, 1 core = 100 %) | `cloud_filter` 10.3 % (57–60 MB RSS; 3.9–5.0 ms/frame mean, ≤ 11 ms max from its stats log), `pointcloud_to_laserscan` 1.9 %, `ros2 bag play` 12–18 % (~400 MB RSS) | `replay6.json`, `replay5.launch.log` |
| TF authority | PASS — no `TF_AUTHORITY_VIOLATION`, launch stayed up; `tf2_echo base_footprint lidar_link` = (0.342, 0, 0.750); `base_link → head_color_optical_frame` resolves through `cam_head_link → head_link` (RSP) + bag `/tf_static` | `replay6.{launch.log,tf_echo.txt}` |
| Self box on the real bag (offline, all 364 scans, filter_core) | PASS — box removes median 71 pts/scan (0.18 %); near-body residue (range < 0.8 m, above floor) after filtering median 1, max 13 pts/scan at voxel 0.10 (< 0.04 %, < 1 % criterion) | `.venv/bin/python <session>/voxel_eval.py` → `runs/agent_b/voxel_eval.json` |
| Voxel choice (offline) | voxel 0: 35.9k pts; 0.05 m: 14.0k; **0.10 m: 8.1k (chosen, PROPOSED)**; filter core 4.5–5 ms/scan | `voxel_eval.json` |
| QoS finding | FAIL→fixed — with best-effort readers, ~all ~1 MB clouds and images from the reliable bag writer were lost (1 of 250 clouds delivered; IMU/camera_info fine). Lost UDP fragments are not retransmitted to best-effort readers and the host `rmem_max` (208 KiB, not settable per container, no root) overflows. Larger Cyclone fragments (`MaxMessageSize` 65500 B) did **not** help. Fix: `cloud_filter` subscribes reliable (`input_reliability` param), `stream_probe` matches the publisher's reliability → full rates above with the unchanged DDS profile | runs `replay1`–`replay6`, `runs/agent_b/qos_probe.py` |
| Repo suite (system python) | 414 tests: same 2 **pre-existing** errors as base (`test_model_safety…fails_fast_before_model_load`, `test_patrol_dashboard_integration…` — missing `reportlab`/model deps); bag tests skipped (no `rosbags`) | `PYTHONPATH=src python3 -m unittest discover -s tests` |
| Bag tools after move to `ultron_b2_tools` (shim at `ros2_ws/tools/bag_inspect.py`) | PASS (3) | `PYTHONPATH=src /home/azut25r/workspace/bags/.venv/bin/python -m unittest tests.test_bag_tools` |
| Foxglove/RViz screenshot of `sensors/scan` | UNRUN — no display here; numeric scan check above instead | — |
| Live drivers (`rslidar_helios32.yaml`, `xsens_mti.yaml`, `realsense_d457_head.yaml`, live branch of `drivers.launch.py`) | UNRUN — written only; driver packages not installed; no hardware connection; parameter names TO_VERIFY against the pinned driver versions | — |
| Deskew on real data | UNRUN — implemented and unit-tested (synthetic), off by default (downstream LO deskews from `time`) | — |
| Orin timing | UNRUN — no Orin | — |

Known caveats: lidar/IMU/head-camera to body extrinsics are TO_VERIFY (`lidar_link` z 0.10 m above
`base_link` from the 0.75 m floor estimate); the IMU 31 ms stamp lead is not compensated by default;
the replay scan band (0.45–1.00 m above `base_footprint`) assumes that floor estimate. Any consumer of
raw clouds/images in replay (Foxglove bridge, SLAM, perception) must subscribe **reliable** on this
host. These are replay results from one 36.6 s recording, not hardware validation.

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
| Replay launch, driver configs, cloud filter, calibration yaml (rest of AGENT_B.md) | see "Agent B — Ingestion, replay, preprocessing" above | — |

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
_No results yet._

## M6 — Full-system simulation acceptance
_Not assembled. Reviewer decision: pending (human)._
