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
