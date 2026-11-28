# ULTRON B2 ROS 2 stack — end-to-end runtime plan (sim + real) for fleet

Fleet briefs: [README.md](README.md) · [FLEET_COMMON.md](FLEET_COMMON.md) · `AGENT_A.md` … `AGENT_F.md`.

Status: **PLAN, not evidence.** All numbers are PROPOSED until measured. Nothing here
authorizes a robot connection, hardware setting change or motion
(see `site-safety-patrol-ULTRON/docs/SIM_TO_HARDWARE.md`).

## 1. Problem and approach

Build one ROS 2 Humble colcon workspace that runs the **same** B2 autonomy stack
in three modes: `sim` (ULTRON MuJoCo sensor bench, later Isaac), `replay` (real
rosbag2) and `live` (real B2 + Jetson Orin). It covers drivers, preprocessing,
3D lidar odometry (MOLA-LO), 2D map/localization (slam_toolbox), Nav2 motion
planning, the motion gate, the real B2 command bridge (disabled by default),
bounded exploration, ladder search, operator visualization (Foxglove + laptop
RViz) and edge deployment.

**Do not restart from zero.** The team repo already contains validated sim code
(see §2). The fleet ports it into ament packages, adds the real-sensor path,
3D odometry and the mission/search layer, then runs a documented full-system sim
gate before any real motion.

Decisions made with the user:

| Topic | Decision |
|---|---|
| Location | `site-safety-patrol-ULTRON/ros2_ws/` on new branch `om/b2-ros-stack` (from `origin/main`). Never push/merge to `main` without user approval. |
| SLAM | **MOLA-LO** (3D lidar(-inertial) odometry + 3D map) → `odom→base_link`; **slam_toolbox** 2D map + localization → `map→odom` for Nav2. **KISS-ICP** is a swappable odometry backend behind the same interface. MOLA-LO is **GPLv3**: run it as a separate process/binary package, never link it into our code; keep KISS-ICP (MIT) as the permissive fallback. |
| Simulation | ULTRON MuJoCo bench now (kinematic, Helios-32 sim, already validated); Isaac Sim parity as a later phase behind the same topic contract. |
| Visualization | Both: `foxglove_bridge` on the Orin (whitelisted, throttled topics) **and** laptop RViz via `domain_bridge` (or `zenoh-bridge-ros2dds` fallback) into a separate visualization domain. Raw lidar never crosses Wi-Fi. |
| Real command bridge | Included, implemented and tested in sim/mock; **disabled by default**, requires arming, behind motion gate + watchdog; hardware use needs separate authorization. |
| Fleet | 6 parallel agents (workstreams A–F, §7). |

## 2. Current state (verified from repo, 28 Nov)

Team repo `site-safety-patrol-ULTRON` (current local branch `om/pass7-main-reconciliation`;
`origin/sahil/sim-slam-nav2` already merged into `main`):

| Exists | Where | Evidence |
|---|---|---|
| Simulated Helios-32 lidar (32 ch placeholder, 10/20 Hz, noise, dropout, motion distortion), test hall with 7 ladders, walking people | `src/ultron/autonomy/{lidar,hall_scene}.py`, `config/autonomy/lidar_helios32*.yaml` | `docs/SAHIL_VALIDATION.md` Phase 1, 5 |
| ROS 2 sim bridge: `/clock`, `/ultron/sim/lidar/points`, `/ultron/sim/scan`, `/ultron/sim/odom`, TF, ground truth, `/ultron/sim/cmd_vel`; domain 71 + localhost guard | `autonomy/sim_bridge.py, driver.py, odometry.py, scan2d.py`, `scripts/sim-slam.sh` | Phase 2 |
| slam_toolbox mapping (ATE ≈ 0.05 m), map save with `free_thresh` fix, posegraph, `map_meta.json` | `run_mapping.py`, `slam_toolbox_mapping.yaml` | Phase 3 |
| Localization on saved map + `LocalizationState` monitor (VALID/DEGRADED/LOST/STALE, later-scan rule) | `localization_state.py`, `localization_monitor.py`, `slam_toolbox_localization.yaml` | Phase 4 |
| Nav2 (NavFn + RotationShim/RPP) behind `motion_gate_node`; adapter with contract statuses; failure cases N1/N2 pass | `nav2_adapter.py`, `motion_gate_node.py`, `navigation_contract.py`, `nav2_params.yaml` | Nav N0–N2 |
| Mission/supervisor, SQLite memory, viewpoints, inspection, dashboard, assessment | `src/ultron/{supervisor,memory,viewpoints,mission,...}.py` | `docs/HARDWARE_READINESS.md` |
| Organizer B2 MuJoCo (DDS, no walking), Isaac feasibility smoke test | `scripts/organizer_b2*.py`, `docs/ISAAC_SIM_FEASIBILITY.md` | docs |

Gaps: no ament packages/launch files; no real-sensor drivers/configs; no 3D
odometry; no B2 URDF/TF for real sensors; no real command bridge; Nav2 N3
(supervisor-driven mission) unrun; no exploration/ladder search over an observed
map; no edge deployment/visualization; no replay of the real bag.

### Real recording (`/home/azut25r/workspace/heck_aachen/metadata.yaml`)
rosbag2 v5, sqlite3, 36.6 s, 71,375 msgs, file `rosbag2_2026_11_28-00_48_42_0.db3`.

| Topic | Type | ≈Rate | Notes |
|---|---|---|---|
| `/rslidar_packets` | `rslidar_msg/msg/RslidarPacket` | ~866 Hz | raw RoboSense packets → can re-decode with `rslidar_sdk` |
| `/scan_cloud` | `sensor_msgs/PointCloud2` | ~10 Hz | producer + fields (ring/time?) **TO VERIFY** |
| `/imu/data` | `sensor_msgs/Imu` | ~100 Hz | Xsens MTi (`xsens_mti_ros2_driver`); also `/imu/acceleration`, `/imu/angular_velocity`, `/imu/mag`, `/filter/quaternion`, `/filter/euler`, `/imu/time_ref`, `/status` |
| `/camera_depth/head/{color/image_raw,depth/image_rect_raw,*/camera_info,*/metadata}` | Image/CameraInfo/realsense2 msgs | ~30 Hz | RealSense (briefing says D457); `extrinsics/depth_to_color` |
| `/tf` | TFMessage | ~100 Hz | publisher **TO VERIFY** (likely Xsens orientation) |
| `/tf_static` | TFMessage | 1 msg | sensor mount frames **TO VERIFY** |
| absent | — | — | **No `/sportmodestate`, `/lowstate`, `/odom`, `/joint_states`**: B2 leg odometry is not in the bag → lidar odometry is required for replay. |

Briefing (unverified): B2, Jetson Orin, RoboSense Helios-32, RealSense D457,
ROS 2 Humble, ≈10 × 10 m course, 6–8 ladders, labels ≈80 cm high. Organizer kit:
real Unitree robots use DDS domain 0 on the robot network; sims domain 1 on `lo`;
ULTRON sim uses domain 71 localhost-only.

## 3. Target architecture

```
                 ┌──────────────────────── Jetson Orin (edge, live) / dev PC (sim, replay) ───────────────────────┐
 DRIVERS         rslidar_sdk ─► lidar/points      xsens_mti ─► imu/data      realsense2 ─► cam_head/*     unitree_ros2 ─► sportmodestate
 (live)          (sim: ultron_b2_sim publishes the same relative topics; replay: rosbag2 + remaps)
                        │                              │                          │
 PREPROCESS      self_filter + crop + voxel ─► lidar/points_filtered ─► pointcloud_to_laserscan ─► scan
                        │                              │
 STATE EST.      MOLA-LO (or KISS-ICP) ── TF odom→base_link + odom_lo (Odometry) + 3D local map (throttled)
 2D MAP/LOC      slam_toolbox (mapping | localization) ── TF map→odom + /map
                 localization_monitor ─► localization_state (VALID/DEGRADED/LOST/STALE)   map_manager (save/load/version)
 NAVIGATION      Nav2: planner(NavFn/Smac2D) + controller(RotationShim+RPP) + costmaps(static+obstacle/STVL+keepout)
                 └► nav/cmd_vel_raw ─► velocity_smoother ─► control/in/autonomy
 CONTROL         teleop (joy/keyboard) ─► control/in/teleop        ops/hold (lock, highest priority)
 (one path)      twist_mux (HOLD > teleop > autonomy) ─► control/cmd_vel_muxed
                 ─► collision_monitor (common, all sources) ─► control/cmd_vel_safe
                 ─► motion_gate (common, all sources; sole publisher of cmd_vel) ─► cmd_vel
                 ─► SIM: sim_driver  | LIVE: b2_sport_bridge (subscribes cmd_vel only; Move/StopMove, watchdog, armed only)
                 Above all software: B2 handheld RC priority + physical E-stop (hardware, not bypassable by this stack)
 MISSION         mission_manager: PATROL→EXPLORE→DETECT→APPROACH→SETTLE→CAPTURE→ASSESS(hand-off)→COMMIT→RETURN_BASE, HOLD from any state
                 frontier_explorer (bounded zone) · ladder_detector → ladder_localizer (depth+TF@stamp) → ladder_tracker → memory_bridge (SQLite)
                 view_pose_planner (4-pose ring + gates) · capture_coordinator · coverage_reporter
 OPS / VIZ       foxglove_bridge (whitelist, :8765) · viz_throttle (map/cloud downsample, compressed images) · domain_bridge → laptop RViz domain
                 rosbag2 (mcap, zstd) recording profiles · diagnostics aggregator · health/heartbeat
                 └────────────────────────────────────────────────────────────────────────────────────────────┘
 Laptop: Foxglove Studio (ws://orin:8765) and/or RViz2 in viz domain; ULTRON dashboard (existing) consumes mission events.
```

### 3.1 Frames and TF authority (single publisher per edge)

| Edge | Publisher | Notes |
|---|---|---|
| `map → odom` | slam_toolbox | only in mapping/localization modes |
| `odom → base_link` | MOLA-LO **or** KISS-ICP **or** (fallback) B2 `sportmodestate` odom adapter | exactly one, selected by launch arg `odom_source:={mola,kiss,b2,sim}`; a `tf_authority_check` node fails the launch on duplicates |
| `base_link → {lidar_link, imu_link, cam_head_*}` | `robot_state_publisher` from `ultron_b2_description` | real values from bag `/tf_static` + measurement; sim values from `lidar_helios32.yaml` |
| Ground truth (sim) | never on TF | existing rule preserved |

Base frame for planar navigation: introduce `base_footprint` (gravity-aligned, z=0)
if IMU attitude is applied to `base_link`, to avoid tilted `pointcloud_to_laserscan`
slices (integrated ch.04 caveat). Decision recorded by Workstream C.

### 3.2 Topic contract and isolation

All stack nodes use **relative** topic names and launch under a namespace:
`/ultron/sim`, `/ultron/replay` or `/ultron/live`. This keeps the existing
`/ultron/sim/*`-only test valid and makes cross-mode mixing visible.

| Logical | Relative name | Type |
|---|---|---|
| lidar raw | `sensors/lidar/points` | PointCloud2 (x,y,z,intensity,ring,time) |
| lidar filtered | `sensors/lidar/points_filtered` | PointCloud2 |
| 2D scan | `sensors/scan` | LaserScan |
| IMU | `sensors/imu/data` | Imu |
| head camera | `sensors/cam_head/{color/image_raw,color/camera_info,depth/image_rect_raw,depth/camera_info}` | Image/CameraInfo |
| odometry | `state/odom` | Odometry |
| localization state | `state/localization` | `ultron_b2_interfaces/LocalizationState` |
| map | `map`, `map_metadata` | OccupancyGrid |
| 3D map (throttled) | `viz/map_cloud` | PointCloud2 |
| autonomy command | `nav/cmd_vel_raw` → (velocity_smoother) → `control/in/autonomy` | Twist |
| teleop command | `control/in/teleop` | Twist |
| HOLD lock | `ops/hold` | std_msgs/Bool (twist_mux lock, highest priority) |
| muxed → safe → gated | `control/cmd_vel_muxed` → (collision_monitor) → `control/cmd_vel_safe` → (motion_gate) → `cmd_vel` | Twist |
| active source | `control/active_source` | std_msgs/String (`hold`/`teleop`/`autonomy`/`none`) |
| operator enable | `ops/operator_enable` | `ultron_b2_interfaces/OperatorEnable` (heartbeat) |
| gate status | `safety/motion_gate` | `ultron_b2_interfaces/MotionGateStatus` |

**Command-path invariant (all modes, all sources).** Every velocity command
that can reach `sim_driver` or `b2_sport_bridge` passes, in this order:
source mux → `collision_monitor` → `motion_gate` → `cmd_vel`. `motion_gate`
is the only node allowed to publish `cmd_vel`; the bridge and sim driver
subscribe to `cmd_vel` only. There is no teleop, recovery or test shortcut
past the collision monitor or the gate. The gate always enforces: HOLD,
operator-enable heartbeat, command age, non-finite rejection and velocity
limits. For `autonomy` it also requires VALID, fresh localization for the
loaded map. For `teleop` the localization check is controlled by
`teleop_requires_localization`, which defaults to `true` (fail-closed);
relaxing it is an open decision for the safety owner (see Unknowns). The B2
handheld RC and physical E-stop sit above this software chain and are not
replaced by it.
| ladder candidates | `perception/ladder_detections`, `perception/ladder_candidates` | interfaces |
| mission | `mission/status`, `mission/events` | interfaces |

Live drivers keep vendor names; bringup remaps them into the contract
(e.g. `/scan_cloud`→`sensors/lidar/points`, `/imu/data`→`sensors/imu/data`,
`/camera_depth/head/color/image_raw`→`sensors/cam_head/color/image_raw`).

DDS/domains (PROPOSED, verify on site):

| Mode | Domain | Transport |
|---|---|---|
| sim | 71 | `ROS_LOCALHOST_ONLY=1` (existing guard) |
| replay | 72 | localhost only; **no** control nodes launched |
| live (Orin) | 0 (must see Unitree `/sportmodestate`, `/api/sport/*`) | CycloneDDS bound to robot NIC; config in `ultron_b2_bringup/config/dds/` |
| laptop viz | 42 | `domain_bridge` whitelist from 0→42 (throttled topics only); fallback `zenoh-bridge-ros2dds` with allow-list when multicast/Wi-Fi is unreliable |

Foxglove: `foxglove_bridge` on Orin with `topic_whitelist` (map, viz/map_cloud ≤1 Hz voxelized,
scan, tf, state/*, nav plans/costmap ≤2 Hz, compressed cam_head color ≤5 Hz, mission/*,
diagnostics). Never whitelist raw `sensors/lidar/points` or raw images by default.

## 4. Package layout (`site-safety-patrol-ULTRON/ros2_ws/src/`)

| Package | Build | Contents |
|---|---|---|
| `ultron_b2_interfaces` | ament_cmake | msgs: `LocalizationState`, `MotionGateStatus`, `OperatorEnable`, `LadderDetection(Array)`, `LadderCandidate(Array)`, `MissionStatus`, `MissionEvent`, `CoverageReport`, `MapInfo`; srvs: `Arm`, `SaveMap`, `LoadMap`, `SetPatrolZone`; actions: `NavigateToView`, `CaptureViews`, `RunMission`. Mirrors existing JSON schemas (`ultron.localization_state/v1`, `ultron.navigation_goal/v1`) with explicit versions. |
| `ultron_b2_description` | ament_cmake | B2 xacro (from `unitree_ros`/`unitree_mujoco` B2 model, BSD-3 provenance kept), sensor mounts (`lidar_link`, `imu_link`, `cam_head_*_optical_frame`), `calibration/{sim,real}.yaml`, `footprint.yaml` |
| `ultron_b2_bringup` | ament_python | `sim.launch.py`, `replay.launch.py`, `live.launch.py`, composable sub-launches (`drivers`, `preprocess`, `state_estimation`, `slam`, `nav`, `control`, `mission`, `viz`, `record`); `mode:={mapping,localization,nav,mission}`, `odom_source`, `viz:={none,foxglove,rviz_bridge,both}`; DDS xml; driver configs (`rslidar_sdk`, `xsens`, `realsense`); `tf_authority_check`; preflight node (domain/namespace/hardware flags) |
| `ultron_b2_preprocess` | ament_cmake (C++ for throughput) or Python+numpy first | self-filter crop box, range/height crop, voxel downsample, optional deskew; `pointcloud_to_laserscan` config |
| `ultron_b2_localization` | ament_python | MOLA-LO + KISS-ICP launch/config, slam_toolbox mapping/localization configs (ported from `config/autonomy`), `localization_monitor` (port), `map_manager` (save 2D pgm/yaml/posegraph + 3D MOLA map + `map_meta.json` with hashes, `free_thresh` fix, versioning), B2-odom adapter (fallback) |
| `ultron_b2_navigation` | ament_python | Nav2 params `{sim,real}.yaml`, keepout/zone filter masks, `collision_monitor` + `velocity_smoother` configs, `motion_gate` (port), `nav_contract_server` (`NavigateToView` action wrapping `NavigateToPose`, pre-checks, settle, CAPTURE_READY; port of `nav2_adapter.py`) |
| `ultron_b2_control` | ament_cmake/python | `twist_mux` config (HOLD lock > teleop > autonomy, output `control/cmd_vel_muxed`, upstream of the collision monitor and gate), `active_source_monitor` (publishes `control/active_source` from the same priority table; replace both with a minimal custom mux if they cannot be kept consistent), teleop (joy/keyboard → `control/in/teleop`), `b2_sport_bridge` (subscribes **only** gated `cmd_vel`; Twist → `unitree_api/Request` Move api 1008, StopMove on timeout/disarm, rate + accel limits, armed-only, refuses unless `hardware_enable:=true` and preflight passes), `sim_driver` adapter (also subscribes only `cmd_vel`) |
| `ultron_b2_sim` | ament_python | wrapper launching `ultron.autonomy.sim_bridge` with contract remaps/namespace `/ultron/sim`, RGB-D head camera render (MuJoCo) on contract topics, scenario hooks; later `isaac/` launcher behind same contract |
| `ultron_b2_perception` | ament_python | `ladder_detector` (backends: `oracle_sim` [labelled ORACLE], `yolo`, `owlv2` open-vocab; model weights from perception owner), `ladder_localizer` (depth + TF at image stamp, reject stale TF), `ladder_tracker` (map-scoped association, ambiguity kept), `label_capture` hooks |
| `ultron_b2_mission` | ament_python | `mission_manager` (wraps existing `supervisor.py` actions `INSPECT/SURVEY/RETURN_BASE/RETURN_BASE_WITH_GAPS/AWAIT_OPERATOR`), `frontier_explorer` (bounded by PatrolZone; m-explore-ros2 or own), `view_pose_planner` (port `viewpoints.py`, six gates), `capture_coordinator`, `memory_bridge` (SQLite via `memory.py`, append-only), `coverage_reporter`, dashboard event publisher |
| `ultron_b2_viz` | ament_python | Foxglove layouts (`.json`), RViz configs, `viz_throttle` node, `domain_bridge` yaml, zenoh allow-list config |
| `ultron_b2_tools` | ament_python | `bag_inspect` (topics, rates, TF tree, cloud fields, time gaps), `bag_to_contract` remap/replay helper, eval tools (port `eval_slam.py`, `eval_localization.py`), latency/resource probe (port) |

Docker: `docker/dev.Dockerfile` (x86, Humble, all deps) and `docker/orin.Dockerfile`
(arm64, L4T/JetPack base TO VERIFY, Humble) + `compose` per mode. CI: `colcon build` +
`colcon test` (unit + `launch_testing`) in dev container; existing `unittest` suite must stay green.

## 5. Open-source references (pin versions/commits in `ros2_ws/deps.repos`)

| Component | Source | Licence | Role / note |
|---|---|---|---|
| MOLA, `mola_lidar_odometry`, `mola_state_estimation` | github.com/MOLAorg/mola, /mola_lidar_odometry; docs.mola-slam.org; Humble binaries | **GPLv3** (MOLA-LO) / BSD (core parts) — verify per package | Primary 3D LO + 3D map; separate process only |
| `mp2p_icp`, `mrpt_ros` | github.com/MOLAorg/mp2p_icp, MRPT | BSD-3 | MOLA deps; map files |
| KISS-ICP | github.com/PRBonn/kiss-icp (ROS 2 wrapper) | MIT | Permissive odometry fallback |
| GLIM (optional, later) | github.com/koide3/glim | MIT | GPU factor-graph mapping; not critical path |
| slam_toolbox | github.com/SteveMacenski/slam_toolbox (Humble) | LGPL-2.1 | 2D map + localization (already validated) |
| Nav2 | github.com/ros-navigation/navigation2 (Humble 1.1.x) | Apache-2.0 / mixed | planner, controller, BT, costmaps, collision_monitor, velocity_smoother, keepout filters, simple_commander |
| spatio_temporal_voxel_layer | github.com/SteveMacenski/spatio_temporal_voxel_layer | LGPL-2.1 | 3D obstacle layer from lidar (optional vs obstacle layer) |
| pointcloud_to_laserscan | github.com/ros-perception/pointcloud_to_laserscan | BSD-3 | 3D→2D scan |
| robot_localization (optional) | github.com/cra-ros-pkg/robot_localization | BSD-3 | fuse B2 odom + IMU if needed |
| rslidar_sdk / rslidar_msg | github.com/RoboSense-LiDAR/rslidar_sdk | verify (BSD-3 expected) | decode `/rslidar_packets`; Helios-32 config |
| Xsens MTi ROS 2 driver | github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client | verify | IMU |
| realsense-ros | github.com/IntelRealSense/realsense-ros | Apache-2.0 | D457 head camera |
| unitree_ros2 / unitree_sdk2 | github.com/unitreerobotics/unitree_ros2 (pin; ch.04 cites `3ff13ea0`) | BSD-3 | `/api/sport/request` (Move 1008, StopMove), `/sportmodestate`; verify B2 API on real firmware (R-67) |
| B2 model | github.com/unitreerobotics/unitree_ros (`b2_description`), unitree_mujoco | BSD-3 | URDF/MJCF |
| m-explore-ros2 | github.com/robo-friends/m-explore-ros2 | BSD | frontier exploration reference |
| twist_mux, teleop_twist_joy | ros-teleop | Apache-2.0 / BSD | command priority |
| foxglove_bridge | github.com/foxglove/ros-foxglove-bridge | MIT | edge visualization |
| domain_bridge | github.com/ros2/domain_bridge | Apache-2.0 | 0→viz domain whitelist |
| zenoh-bridge-ros2dds | github.com/eclipse-zenoh/zenoh-plugin-ros2dds | EPL-2.0/Apache-2.0 | fallback laptop bridge |
| rosbag2_storage_mcap | ros2/rosbag2 | Apache-2.0 | edge recording |
| nvblox / Isaac ROS (optional) | NVIDIA-ISAAC-ROS | Apache-2.0 | later 3D costmap on Orin |
| Ladder detection | OWLv2 (Apache-2.0), GroundingDINO (Apache-2.0), YOLO (Ultralytics **AGPL-3.0** — licence review) | — | Perception owner (Om) supplies weights |

Team sources to reuse: `docs/SAHIL_*.md`, `docs/HARDWARE_READINESS.md`,
`docs_om/integrated/chapters/04_patrol_navigation_poses.md` (stack decision R-43,
TF caveats, B2 sport API notes), `05_ladder_discovery_reid.md`, `09_plan_tests_gates.md`,
`docs_om/original/chunk_3_*` (ladder discovery / 3D pinning / registry),
`chunk_4_*` (B2 pipeline), `chunk_extra_*` (minimum views).

## 6. Milestones and gates (ordered; cut-lines marked)

| M | Goal | Exit criteria (PROPOSED) |
|---|---|---|
| M0 | Workspace + contracts + bag facts | `colcon build` clean in dev container; interfaces compile; `bag_inspect` report on the real bag (TF tree, frames, cloud fields, rates, gaps); unknowns table filled |
| M1 | Sim parity in ament | `ros2 launch ultron_b2_bringup sim.launch.py mode:=mapping odom_source:=sim` reproduces Phase 3 metrics (ATE RMSE ≤ 0.10 m) using the new packages |
| M2 | Real-data replay SLAM | replay of real bag: MOLA-LO odometry + slam_toolbox map produced; map saved/reloaded; LocalizationState sane; resource use measured on dev PC (and Orin if available). No ground truth → report qualitative + loop-consistency metrics only |
| M3 | Nav + safety in sim | Nav2 N1/N2 cases re-pass with new packages through the single chain (mux → collision_monitor → motion_gate → `cmd_vel`); teleop commands are also stopped by the collision monitor and zeroed by a closed gate; graph check shows `motion_gate` is the only `cmd_vel` publisher; `b2_sport_bridge` tested against a mock `/api/sport/request` subscriber: disarmed = no messages, timeout → StopMove, limits enforced |
| M4 | Autonomous search in sim (**cut-line A: must have**) | `mode:=mission`: bounded explore → ladder detection (oracle labelled + real detector if available) → view pose → settle → capture → resume → return base; coverage report with gaps; HOLD/operator inhibit from every state |
| M5 | Edge deployment + viz (**cut-line B**) | Orin container runs replay mode; Foxglove shows map/pose/plan/ladders at whitelisted rates; laptop RViz via domain_bridge; bandwidth measured |
| M6 | Full-system sim acceptance record | `docs/b2_ros_stack/VALIDATION.md` (M6 section) per SIM_TO_HARDWARE.md; reviewer sign-off. **Only then** propose a bounded, separately authorized hardware plan (read-only sensors first, then supervised motion with RC priority, watchdog, E-stop) |
| M7 | Isaac parity (**post-event**) | Isaac lidar/RGB-D/hall publish same contract; M1/M3/M4 cases re-run |

Hardware gates (unchanged): no robot connection by any agent. Live mode code ships with
`hardware_enable:=false`, arming service required, preflight refuses wrong domain/namespace,
and a mock bridge is the only tested consumer until the user authorizes a bounded test.

## 7. Fleet workstreams (6 agents)

Each agent: branch off `om/b2-ros-stack` as `om/b2-ros-stack-<letter>`; follow team
`AGENTS.md`/`CLAUDE.md`; write results only to its own section of
`docs/b2_ros_stack/VALIDATION.md`; PASS/FAIL/UNRUN with artifacts; never commit bags, maps, weights, runs.

### A — Foundation: workspace, interfaces, description, bringup, containers, CI
- Create `ros2_ws/` skeleton, `deps.repos` (pinned), `ultron_b2_interfaces`, `ultron_b2_description` (B2 xacro + sensor frames; sim calibration from `lidar_helios32.yaml`, real placeholders `source: TO_VERIFY`), `ultron_b2_bringup` launch skeleton with all args, namespace/domain preflight, `tf_authority_check`, DDS configs.
- Docker dev (x86) + Orin (arm64) images; compose per mode; CI script (`colcon build && colcon test` + existing `unittest`).
- Accept: clean build/test in container; launch with `mode:=*` starts placeholders without errors; preflight rejects live mode without `hardware_enable` and wrong domain.

### B — Real-sensor ingestion, replay and preprocessing (needs bag)
- `ultron_b2_tools/bag_inspect` + report on the real bag; decide `/scan_cloud` vs re-decoding `/rslidar_packets` with pinned `rslidar_sdk` (need ring/time for deskew).
- Driver configs: `rslidar_sdk` (Helios-32, frame `lidar_link`), Xsens, RealSense D457 (head); remaps into contract; `replay.launch.py` (`--clock`, `use_sim_time`, namespace `/ultron/replay`, no control nodes).
- `ultron_b2_preprocess`: self-filter (measure robot body returns from bag), crop, voxel, `pointcloud_to_laserscan` (height band per ch.04, TO VERIFY), timing/QoS checks (sensor data QoS best-effort vs bag reliable).
- Extract real `base_link→sensor` from `/tf_static`; fill unknowns table (lidar variant, mount, IMU frame/orientation, camera extrinsics, clock source).
- Accept: replay publishes contract topics at bag rates; filtered cloud has no self-returns; scan shows walls; report committed (no data).

### C — State estimation, mapping, localization (depends A; B for real)
- MOLA-LO launch/config for Helios-32 (+ optional Xsens IMU), publishing `odom→base_link` with configured frame names; KISS-ICP alternative with identical outputs; B2-odom adapter fallback (`/sportmodestate` → Odometry) for live.
- Port slam_toolbox configs, `localization_monitor`, `map_manager` (2D + 3D map save/load, versioning, `free_thresh` fix, `map_meta.json` hashes); `base_footprint` decision.
- Eval: sim (ground truth) for MOLA vs KISS vs sim-odom; real bag: drift on loop if the route closes, map sharpness, CPU/RAM/latency; Orin measurement when available.
- Accept: M1 metrics in sim with `odom_source:=mola` and `:=kiss`; M2 on real bag; LocalizationState failure cases (blackout, kidnap) re-pass.

### D — Navigation, safety chain and B2 control (depends A, C-sim)
- Port Nav2 params to `{sim,real}.yaml`; footprint from measured B2 (sim 0.55 m circle until measured); costmaps: static + obstacle (scan) + optional STVL from filtered cloud; keepout/zone mask from PatrolZone; `velocity_smoother`, `collision_monitor`.
- Port `motion_gate` and `nav_contract_server` (`NavigateToView` action, pre-checks, settle, late-result rejection, timeout, cancel).
- Single command path for every source: `twist_mux` (HOLD > teleop > autonomy) → common `collision_monitor` → common `motion_gate` → `cmd_vel` → sim driver or bridge. No source bypasses the monitor or gate.
- `ultron_b2_control`: `twist_mux`, `active_source_monitor`, teleop, `b2_sport_bridge` (Move/StopMove via pinned `unitree_ros2` msgs, watchdog, armed-only, limits PROPOSED), `sim_driver` adapter; mock sport subscriber for tests.
- Accept: N1/N2 cases re-pass through the full chain in sim; teleop-path tests (teleop toward obstacle stopped by collision monitor; teleop with gate closed ⇒ zero `cmd_vel`); `motion_gate` is the sole `cmd_vel` publisher; bridge tests (disarmed silence, timeout StopMove ≤ PROPOSED 0.3 s, limit clamp, non-finite reject, domain/namespace refusal).

### E — Mission, exploration and ladder search (depends A, D-sim; perception weights optional)
- `frontier_explorer` bounded by PatrolZone, gaps reported (unknown/unreachable separate); fallback: configured survey route.
- `ultron_b2_perception`: detector node (oracle_sim clearly labelled; yolo/owlv2 backends loading Om's weights), `ladder_localizer` (depth + TF at stamp, reject stale), `ladder_tracker` (map-scoped, ambiguity preserved; port from `memory.py`/`observations.py`).
- `mission_manager` wrapping `supervisor.py`; `view_pose_planner` (port `viewpoints.py`, 4-pose ring + gates), `capture_coordinator` (capture request/ack), `memory_bridge` (SQLite, append-only, map-scoped), `coverage_reporter`, events to existing dashboard.
- Accept: M4 scenario in sim with two patrols (ID retained, new ladder → candidate, ambiguous pair kept, missing handled); HOLD from every state; arrival never reported as inspection result.

### F — Sim parity, visualization/ops, integration testing and docs (depends A; integrates all)
- `ultron_b2_sim`: wrap existing bridge with contract names, add MuJoCo RGB-D head camera topics, scenario hooks; later Isaac launcher (M7).
- `ultron_b2_viz`: Foxglove layouts (mapping, navigation, mission), `viz_throttle` (voxelized map cloud ≤1 Hz, compressed images), `foxglove_bridge` whitelist, `domain_bridge` 0→42 yaml, zenoh allow-list fallback, RViz configs.
- Recording profiles (mcap+zstd): `minimal` (tf, state, scan, mission), `debug` (+filtered cloud, compressed images).
- End-to-end `launch_testing` scenarios (M1, M3, M4), `docs/b2_ros_stack/RUNBOOK.md` (fresh machine, sim, replay, Orin, viz, stop/recovery), `docs/b2_ros_stack/VALIDATION.md` sections and M6 record assembly.
- Accept: one-command sim mission + Foxglove view; replay on Orin with measured bandwidth; runbook reproduced by another agent.

### Dependency summary
A → (B, C, D, E, F start on stubs immediately); B → C(real); C(sim) → D; D(sim) → E; all → F(M6).

## 8. Unknowns (owner, fallback)

| Unknown | Owner | Fallback |
|---|---|---|
| `/scan_cloud` producer & fields (ring/time) | B | re-decode packets with rslidar_sdk |
| `/tf` 100 Hz publisher, `/tf_static` content | B | own description values, flagged TO_VERIFY |
| B2 odometry availability on robot (`/sportmodestate`) | C | MOLA/KISS lidar odometry only |
| Orin JetPack/L4T, CPU/GPU budget, storage | A | run SLAM on laptop over Ethernet (not Wi-Fi) |
| Network topology, multicast on venue Wi-Fi | F | zenoh bridge; Foxglove only |
| Real footprint, stopping distance, B2 Move limits/firmware API | D (+ operator) | sim values, motion stays NO-GO |
| Camera extrinsics/intrinsics (D457 head) | B/E | camera_info from bag; extrinsics TO_VERIFY |
| Ladder detector weights | Om (perception) | oracle_sim in sim only, labelled |
| MOLA GPLv3 acceptability for team distribution | user | KISS-ICP |
| Whether teleop may move without VALID localization (`teleop_requires_localization`) | safety owner (+ operator) | keep `true` (fail-closed); teleop still passes collision monitor + gate either way |

## 9. Notes
- Preserve existing tests/behaviour; port code by importing `ultron.*` modules where possible rather than duplicating.
- Thresholds stay PROPOSED until measured; sim values are not hardware settings.
- Optional cloud/VLM/System One have no route to motion or verdicts.
