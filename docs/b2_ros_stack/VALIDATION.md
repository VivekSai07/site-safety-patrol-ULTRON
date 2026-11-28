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

Branch `om/b2-ros-stack-c` (from fb9f092). Package `ultron_b2_localization` (see its README).
Bringup `state_estimation` and `slam` sub-launches are filled in. Every number below comes from
**simulation or bag replay on the dev PC** (20 cores, 15 GB, shared with other agents). None of
it is hardware, walking or real-perception validation. Thresholds are PROPOSED. Run artifacts and
maps live in ignored `runs/b2_stack/c/` and are not committed.

**Tests**
- `ultron_b2_localization`: 21 passed. Coverage:
  - map store: versioning, hashes, `free_thresh` fix, tamper refusal, directory refs;
  - monitor: VALID path, blackout/STALE and recovery, kidnap (jump) → DEGRADED → LOST, stale
    pose, parity with `src/ultron/autonomy/localization_state.py`;
  - odometry: KISS cloud parsing/deskew times, KISS-ICP tracking synthetic motion, B2 adapter on mocks (planarization, rejects
    non-finite data), planarizer.
- `ultron_b2_bringup`: 41 passed. Expected node sets were updated: slam_toolbox, `localization_monitor`
  and `map_manager` in sim/nav, plus `mola_lidar_odometry` and `mola_odom_planarizer` in replay.
- Repo suite (`PYTHONPATH=src python3 -m unittest discover -s tests`, host): 414 run, 74 skipped,
  2 errors (`reportlab` missing). This matches the base branch, so there is no regression.

**Simulation ATE vs ground truth** (`tools/eval_sim.sh`)
- Setup: the validated `sim_bridge` hall survey loop (100 s, 30.75 m), domain 71, slam_toolbox
  mapping.
- Estimates are compared with ground truth interpolated at each estimate stamp and anchored at the
  start pose. There is no best-fit alignment.
- The bridge's leg odometry and its TF are moved off the stack topics for mola/kiss, so the lidar
  odometry under test is the only `odom → base_link` authority.

| odom_source | seed | SLAM ATE RMSE / max (m) | SLAM yaw RMSE / max (°) | odometry-only ATE RMSE / max (m) | map precision / wall recall |
|---|---|---|---|---|---|
| sim (leg-odom model) | 7 | 0.069 / 0.160 | 1.15 / 2.3 | 1.25 / 2.02 | 0.98 / 0.95 |
| mola | 7 | **0.040** / 0.155 | 0.83 / 2.8 | 0.031 / 0.120 | 1.00 / 0.93 |
| mola | 21 | **0.043** / 0.166 | 0.80 / 2.6 | 0.030 / 0.120 | 1.00 / 0.98 |
| mola | 99 | **0.045** / 0.172 | 0.81 / 2.5 | 0.031 / 0.119 | 1.00 / 0.96 |
| kiss | 7 | **0.063** / 0.214 | 1.53 / 6.5 | 0.101 / 0.240 | 1.00 / 0.95 |
| kiss | 21 | **0.055** / 0.221 | 1.65 / 7.6 | 0.100 / 0.258 | 1.00 / 0.96 |
| kiss | 99 | **0.058** / 0.184 | 1.65 / 11.1 | 0.102 / 0.272 | 1.00 / 0.95 |

- All runs are within the PROPOSED limits: ATE RMSE ≤ 0.10 m, max ≤ 0.25 m, yaw RMSE ≤ 2°.
- KISS's yaw max (6.5–11°) is a transient and is not checked against any limit.
- Sim latency (cloud stamp → `state/odom`), median / p95: MOLA 16 / 107 ms, KISS 10 / 18 ms, both
  with the bridge sharing the CPU.

**Real bag replay** (`tools/eval_bag.sh`)
- Bag: `rosbag2_2026_11_28-00_48_42`, mounted read-only, domain 72, rate 1.0. Only `/scan_cloud`
  is played, remapped to `/ultron/replay/sensors/lidar/points_filtered`.
- Scans come from the eval converter `tools/cloud_to_scan.py` (364/364 delivered). The B2's
  `pointcloud_to_laserscan` (best effort) delivered only 0.3–79 % of the 1.5 MB clouds, and in one
  run slam_toolbox diverged as a result.
- There is no ground truth. The start/end gap only bounds loop error if the operator really
  returned to the start pose, which is unverified.

| run | odom path (m) | odom start/end gap (m) / yaw (°) | SLAM start/end gap (m) / yaw (°) | latency median / p95 / max (ms) | monitor |
|---|---|---|---|---|---|
| MOLA mapping 1 | 15.81 | 0.407 / 41.0 | 0.431 / 41.4 | 18.1 / 26.9 / 216 | live map: VALID 52 % |
| MOLA mapping 2 | 15.78 | 0.405 / 41.0 | 0.474 / 41.0 | 18.0 / 29.8 / 124 | live map: VALID 50 % |
| KISS mapping 1 | 18.05 | 0.406 / 41.3 | 0.382 / 42.9 | 17.9 / 32.4 / 123 | live map: VALID 37 % |
| KISS mapping 2 | 19.45 | 0.398 / 41.1 | 0.351 / 44.6 | 17.6 / 32.1 / 42 | live map: VALID 49 % |
| MOLA localization on saved `bag-mola` v2 | 15.78 | 0.405 / 41.0 | 0.468 / 41.3 | 15.5 / 22.3 / 120 | **VALID 75 %**, DEGRADED 82, no LOST/STALE |
| KISS localization on saved `bag-kiss` v2 | 17.88 | 0.390 / 41.1 | 0.339 / 44.1 | 18.7 / 35.3 / 124 | VALID 69 %, LOST 3 (last state LOST: DEGRADED > 3 s) |

- **Loop closure (bag).** The trajectory ends about 0.35–0.47 m and +41° from its start.
  - The yaw change matches the bag's IMU yaw change (−49° → −8°, BAG_FACTS), so this is probably
    the operator's real end pose, not drift.
  - The earlier unvalidated KISS estimate of "0.27 m from start" (BAG_FACTS) was not reproduced
    with the stack configuration.
  - Path length differs by estimator: MOLA 15.8 m (consistent across runs), KISS 17.9–19.5 m
    (varies run to run).
- **Save/reload.**
  - `map/save` wrote v1/v2 per map_id: `map.pgm`/`yaml`/`posegraph`/`data`, and for MOLA also
    `map3d.mm`/`.simplemap`. `map_meta.json` records the sha256 hashes.
  - The localization launch verified the hashes and reloaded the pose graph.
  - Bug found and fixed: the launch passed the version directory as `initial_map`, which crashed
    map_manager.
- **Mapping-mode monitor.** In mapping mode the monitor scores against the growing map. These
  numbers are a consistency check only and are not localization validity.
- **Resources on this PC.** The figures below are CPU % of one core (mean / p95) and RSS. They come
  from replaying the bag; sim figures were lower.
  - MOLA-LO: 59–78 % / 99–138 %, 215–224 MB. It drops about 0.8 % of frames ("worker busy").
  - KISS-ICP (Python wrapper): 123–139 % / 264–293 %, 113–115 MB. Processing per frame takes
    17–18 ms median and up to 40 ms.
  - slam_toolbox: 2–7 %, 47–62 MB.
  - monitor: about 6 %, 60 MB.
  - map_manager: about 4 %, 59 MB.
  - eval scan converter: about 8 %, 60 MB.

**Bugs found and fixed during evaluation**
- MOLA's `MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION` does not constrain the published ICP pose. On the
  bag, |z| reached 0.11 m and roll/pitch 2.6°.
  - MOLA's `/tf` and pose output are now remapped to private `mola/tf_raw` and `mola/odom_raw`.
  - `mola_odom_planarizer` is the `odom → base_link` authority.
- The monitor's scan QoS changed from RELIABLE to BEST_EFFORT, because it was incompatible with
  sensor-data publishers.

**Recommendation.** Make **MOLA-LO** the default `odom_source`.
- In sim it has lower ATE (0.040–0.045 m vs 0.055–0.063 m) and odometry-only ATE about 3× lower.
- On the bag its path length is more stable, it uses about half of KISS's CPU, it gives the best
  reload VALID fraction, and it can save a 3D map.
- KISS-ICP (MIT) stays the permissively licensed fallback, with identical outputs.

**UNRUN / open**
- B2 adapter against a real `/sportmodestate`. It is mock-tested only; the message variant and
  topic must be confirmed on the robot.
- Bringup end-to-end with Agent B's preprocessing (`sensors/scan` from B). B's scan must be
  reliable and complete; see the converter finding above.
- Bringup sim with Agent F's `ultron_b2_sim`. The eval harness used the bridge directly and
  without `robot_state_publisher`.
- IMU fusion.
- Kidnap and blackout on live ROS. These are covered by unit tests only.
- Bag playback at rates other than 1.0.
- Real-profile override of slam_toolbox `min_laser_range` (0.3 < scan range_min 1.0 m; it logs a
  harmless warning).


## Agent D — Navigation, safety chain, control

Branch `om/b2-ros-stack-d`. Dev image `ultron-b2-dev:humble`, localhost only, domains 71 (sim), 80/81 (graph/bridge
tests); never domain 0. **Simulation and mock only: no hardware was contacted and the bridge has only run against
`mock_sport_server`. None of this is walking, perception or hardware evidence.** Run artefacts (`runs/b2_stack/d/`)
are local and not committed.

Chain verified: Nav2 → `nav/cmd_vel_raw` → velocity_smoother → `control/in/autonomy`; teleop → `control/in/teleop`;
`ops/hold` lock → twist_mux → `control/cmd_vel_muxed` → collision_monitor → `control/cmd_vel_safe` → motion_gate →
`cmd_vel` → sim_driver (`sim/cmd_vel`) | b2_sport_bridge (mock).

### Unit, graph and launch tests

`colcon test --parallel-workers 4 --packages-select ultron_b2_bringup ultron_b2_navigation ultron_b2_control
ultron_b2_interfaces ultron_b2_description` → **155 tests, 0 failures**. A later
`pytest src/ultron_b2_control/test` run gave **36 passed**; it includes the operator-enable graph test added afterwards.
Repo fast suite `python3 -m unittest discover -s tests` → **414 tests OK (22 skipped)**, no regression.

| Acceptance item | Result | Evidence |
|---|---|---|
| motion_gate conditions: HOLD, operator heartbeat ≤ 1.0 s, command age ≤ 0.3 s, non-finite, over-limit → zero (not clamped), no map, localization VALID/fresh/map match | PASS | `ultron_b2_navigation/test/test_gate_logic.py` |
| teleop_requires_localization defaults true; relaxed only when false | PASS | unit test + graph `test_teleop_requires_localization_by_default` |
| Unknown or stale active source ⇒ strictest (autonomy) | PASS | `test_unknown_source_is_strictest`, `test_stale_source_is_strictest`, graph `test_stale_command_and_no_source_stop` |
| Single `cmd_vel` publisher (`motion_gate`); only sim_driver subscribes; no `/api/sport/request` publisher in the sim chain | PASS | `test_chain_graph.py::test_single_cmd_vel_publisher` |
| Mux priority teleop > autonomy; `control/active_source` consistent | PASS | `test_mux_teleop_over_autonomy`, `sport_logic` `test_select_source_priority_and_hold` |
| HOLD zeroes `cmd_vel` within one control period (0.05 s) | PASS | measured 1.1 ms (earlier runs 0.6–1.3 ms). twist_mux does not publish on lock; the gate ticks on HOLD itself |
| Teleop with the gate closed (operator disabled, or heartbeat stopped) ⇒ zero | PASS | `test_teleop_zero_without_operator_enable[False/None]` |
| Stale command ⇒ zero | PASS | `test_stale_command_and_no_source_stop` |
| collision_monitor stops **both** autonomy and teleop (obstacle 0.4 m ahead) | PASS | `test_collision_monitor_stops_every_source[autonomy/teleop]`; mux output unchanged, safe/`cmd_vel` zero |
| Bridge `hardware_enable=false` ⇒ silent, Arm refused | PASS | `test_bridge_mock.py::test_hardware_enable_false_is_silent` |
| Bridge disarmed ⇒ 0 requests; armed + stream ⇒ Move at rate | PASS | 45 Move at 19.6 Hz (20 Hz configured) |
| Stream stops ⇒ exactly one StopMove ≤ watchdog + 0.05 s | PASS | 1 StopMove, 307 ms after the last `cmd_vel` (limit 350 ms) |
| Clamps / acceleration limits | PASS | `test_sport_logic.py::test_clamp_and_accel` and the mock stream check |
| Non-finite ⇒ single StopMove + latched fault | PASS | `test_nonfinite_latches_fault_single_stop` |
| Wrong domain ⇒ refuse; `target:=robot` off domain 0 ⇒ refuse | PASS | exit code 2, no requests |
| API ids from pinned `unitree_ros2` @668d1ec (Move 1008, StopMove 1003, B2 sport client header) | PASS | read by hand from the pinned checkout (`deps.repos` @668d1ec5, `example/src/include/common/ros2_b2_sport_client.h`, cited in `sport_logic.py`); `test_api_ids_pinned` freezes the constants and the Move JSON |

### Simulation, full chain (N1/N2)

Runner: `sim_nav_cases` (NavigateToView client on `/ultron/sim/nav/navigate_to_view`). The thresholds are the
pre-registered `run_navigation.ACCEPT` values. Ground truth is used for scoring only. Map: `runs/b2_stack/d/map_seed7`
(map_id `ultron-hall-af1db9bbd8d9` v1). **TEMPORARY glue** until F/C deliver:
- the legacy `sim_bridge` with its cmd_vel input on `sim/cmd_vel`;
- slam_toolbox localization and `localization_monitor`;
- `sim_glue`, which relays scan → `sensors/scan`, odom → `state/odom`, and JSON → `state/localization` msg.

`odom_source:=sim` only. The `odom_source:=mola` repeat is **UNRUN** (waiting for C).

Full run `runs/b2_stack/d/nav_all2` (12 cases, one stack instance): **11 PASS, 1 FAIL**.

| Case | Result | Measured |
|---|---|---|
| reach_near (2.5, 5.5, 0) | PASS | REACHED, settled; true error 0.164 m / 0.014 rad |
| reach_far (9.6, 4.5, −π/2) | PASS | REACHED, settled; 0.187 m / 0.024 rad |
| reject_occupied | PASS | REJECTED `TARGET_FOOTPRINT_OCCUPIED`; 0.0 m motion |
| reject_outside | PASS | REJECTED `TARGET_OUTSIDE_MAP`; 0.0 m motion |
| unreachable (obstacle on target) | PASS | TIMEOUT at 60.6 s, backend confirmed cancel; no motion after |
| new_obstacle 1.6 m ahead | PASS | replanned, REACHED; 0 collisions |
| cancel | PASS | CANCELLED; last motion 0.70 s after cancel |
| operator_inhibit | PASS | gate `OPERATOR_DISABLED` 0.06 s; CANCELLED; stopped 0.66 s |
| localization_loss (3 s lidar blackout) | PASS | gate `LOCALIZATION: STALE` 0.48 s; CANCELLED; 0.0 m motion while invalid; last motion 1.26 s after blackout start (legacy 1.2 s; not a pre-registered check) |
| teleop_hold (extra) | PASS | teleop turned 1.09 rad through the full chain; HOLD → gate 0.06 s, still 0.38 s later |
| teleop_no_localization (extra) | PASS | blackout ⇒ gate `LOCALIZATION…`, 0.0 rad turned under teleop |
| teleop_collision (extra) | **FAIL** | see below |

teleop_collision in `nav_all2`:
- **What happened:** after 11 earlier cases, placing the test obstacle 1.8 m ahead dropped the legacy scan/map match
  score from about 0.99 to 0.67, and stayed low after the obstacle was removed (pose shifted by about 0.26 m). The gate
  inhibited teleop (DEGRADED → LOST) before collision_monitor engaged, so the check "collision_monitor limited teleop"
  could not be shown.
- **Safety:** the system failed closed with 0 collisions.
- **Isolated reruns:** `nav_tc1` and `nav_tc2` (teleop_hold + teleop_collision): **2/2 PASS**. collision_monitor limited
  a 0.3 m/s teleop command to 0.04 m/s, and the robot halted 1.02 m centre-to-centre from the obstacle with 0
  collisions. The final hold came from the gate (localization DEGRADED near the unmapped box), which is recorded in
  the case output.
- **Coverage:** the deterministic hard zero for teleop is covered by the graph test above.
- **Open:** localization robustness to close unmapped obstacles is open for Agent C.

Findings fixed during this work (raw runs kept locally):
1. **`nav_all1`, `nav_retry1/2`: slowdown zone throttled rotation.** With a Humble *slowdown* rectangle, a new obstacle
   inside it also scaled in-place rotation. RPP and the rotation shim accel-limit against *measured* odometry, so the
   replanning turn converged to about 0.05 rad/s. The unmapped obstacle then eroded the scan-match score, localization
   went DEGRADED, and the gate cancelled (new_obstacle FAIL). Replaced by a footprint **APPROACH** circle (r 0.6 m,
   1.5 s), which does not throttle rotation in place.
2. **`nav_retry3`: stop zone blocked the turn away.** A 0.75 m stop rectangle was entered while the velocity smoother
   decelerated. Humble stop zones also block turning away, so the run deadlocked until Nav2 FAILED. The stop zone was
   shrunk to 0.65 × ±0.45 m (last resort inside the approach layer). Then new_obstacle REACHED (`nav_retry4`, `nav_all2`).
3. **Nav2 startup without TF.** Nav2 Humble logged errors on SIGINT when it was stuck activating without TF. Lifecycle
   autostart is now off; `nav_contract_server` requests STARTUP once `/clock` > 0 and `map → base_link` exist.

Limitations / TO_VERIFY:
- Collision polygons, limits, watchdog and the footprint are PROPOSED from the sim circle (r 0.55 m); the real B2 +
  payload footprint is TO_VERIFY.
- In Humble, collision_monitor skips a source older than `source_timeout` (not fail-closed). Scan freshness is enforced
  upstream only through localization STALE for autonomy and for teleop with the default flag.
- A Humble stop zone blocks all motion, including turning away; recovery is clearing the obstacle or a human action.
- One map and one hall; static scene apart from test obstacles.
- `nav2_real.yaml` was not run (no real sensors); spin/backup are not loaded for real.
- `state/map_info` is assumed to be Agent C's topic name. Without it, the map comes from `map_meta`.
- STVL (`use_stvl:=true`) is UNRUN.
- Teleop launch (joy/keyboard) was not exercised with devices (no input devices in the container).

## Agent E — Mission, exploration, ladder search
_No results yet._

## Agent F — Sim parity, visualization, integration
_No results yet._

## M6 — Full-system simulation acceptance
_Not assembled. Reviewer decision: pending (human)._
