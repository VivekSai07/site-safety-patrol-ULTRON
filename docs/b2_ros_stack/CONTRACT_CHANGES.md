# Shared contract changes

Log every change to `ultron_b2_interfaces`, topic names, frames or domains
before merging it. Bump the affected message's `schema_version`.

| Date | Agent | Item | Change | Reason | Version | Agreed by |
|---|---|---|---|---|---|---|
| 2026-11-28 | A | `ultron_b2_interfaces` | Initial v1: msgs LocalizationState, MotionGateStatus, OperatorEnable, LadderDetection(Array), LadderCandidate(Array), MissionStatus, MissionEvent, CoverageReport, MapInfo; srvs Arm, SaveMap, LoadMap, SetPatrolZone; actions NavigateToView, CaptureViews, RunMission. Each msg has `schema_version` + `SCHEMA_VERSION` constant `ultron.<name>/v1` | Foundation | v1 | A (initial) |
| 2026-11-28 | A | `ultron_b2_contracts` (Python, in interfaces pkg) | JSON ↔ msg converters live in `ultron_b2_contracts`, not `ultron_b2_interfaces` (rosidl owns that Python module name) | Name collision | v1 | A |
| 2026-11-28 | A | `ultron_b2_bringup` build type | ament_cmake + ament_cmake_python (was ament_python in PLAN §4) | `tf_authority_check` is C++: rclpy Humble exposes no publisher GID | — | A |
| 2026-11-28 | A | Sim TF authority | `robot_state_publisher` (description, `calibration/sim.yaml`) owns `base_link → lidar_link/imu_link/cam_head_*`; `sim_bridge`'s own static `base_link → lidar_link` must be disabled by the sim wrapper (F) | Single TF authority | — | A (F to confirm) |
| 2026-11-28 | A | Replay sub-launches | `replay` includes description, drivers, preprocess, state_estimation, slam, viz, record; never nav, control, mission | No command path in replay | — | A |
| 2026-11-28 | A | DDS localhost profile | `CYCLONEDDS_URI=…/cyclonedds_localhost.xml` (no `<Interfaces>`, `MaxAutoParticipantIndex` 120) set by dev.sh/compose/tests | Loopback discovery limit | — | A |
| 2026-11-28 | F | `ultron_b2_interfaces/srv/SimScenario` | New sim-only service `sim/scenario` (command, target, values → accepted, message, event_json): obstacle, lidar_blackout/dropout, kidnap, ladder_move/hide/show | Scenario hooks for E/M6 | v1 | F |
| 2026-11-28 | F | Sim topics (`ultron_b2_sim`) | Contract topics under `/ultron/sim`: `sensors/lidar/points`, `sensors/scan`, `sensors/imu/data` (synthetic), `sensors/cam_head/{color,depth}/…`, `state/odom` (only `odom_source:=sim`, else `sim/leg_odom`), `/clock`. Sim-only: `sim/ground_truth/base_pose`, `sim/ground_truth/ladders` (latched JSON, evaluation only), `sim/oracle/cam_head/ladder_labels`, `sim/bridge_status`. Input: `cmd_vel` (D's gate is the sole publisher) | Sim parity | — | F |
| 2026-11-28 | F | Sim TF authority (confirms A) | Wrapper always disables sim_bridge's static `base_link→lidar_link` (`--no-static-tf`); `odom→base_link` only when `odom_source:=sim` | Single TF authority | — | F |
| 2026-11-28 | F | `sensors/scan` owner in sim | Sim bridge publishes `sensors/scan` by default; set `sim_scan_topic:=sim` (→ `sim/scan`) once B's preprocess owns `sensors/scan` | Avoid two publishers | — | F (B to confirm) |
| 2026-11-28 | F | `record:=` | Added `full` (`none,minimal,debug,full`); mcap + zstd; output `runs/bags/<mode>_<profile>_<UTC>/` | Recording profiles | — | F |
| 2026-11-28 | F | Viz domain 42 naming | domain_bridge maps whitelisted `/ultron/<mode>/<rel>` to `/ultron/viz/<rel>` (`viz/` prefix stripped) plus `/tf`, `/tf_static`; Foxglove on `127.0.0.1:8765`, whitelist from `ultron_b2_viz/whitelist.py` | Mode-independent laptop RViz | — | F |
| 2026-11-28 | F | bringup exec_depends | `ultron_b2_sim`, `ultron_b2_viz`, `rosbag2_storage_mcap`, `ros2bag` | Sub-launches filled | — | F |
| 2026-11-28 | F | Large-sample QoS (sim) | Best-effort readers drop ~0.7 MB clouds/images over loopback (default 212 KB `rmem_max`); sensor consumers in sim should subscribe RELIABLE depth 1 (publishers are RELIABLE) | Measured failure | — | F (B, C, E to note) |
| 2026-11-28 | F | Dev image | No change: `ultron-b2-dev:humble` already has mujoco, Pillow, websockets, foxglove_bridge, domain_bridge, rosbag2 mcap. zenoh-bridge-ros2dds not installed (UNRUN) | — | — | F |
