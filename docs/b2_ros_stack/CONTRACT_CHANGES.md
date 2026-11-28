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
| 2026-11-28 | B | Replay bag `/tf` | Bag Xsens `/tf` (`world→imu_link`, orientation only) remapped to `/ultron/replay/bag/tf_xsens`, off the TF tree; bag `/tf_static` (RealSense `head_link→…`) replayed unchanged | `imu_link` must have one parent (RSP `base_link→imu_link`); no `world` frame in the stack | — | B (C/F to confirm) |
| 2026-11-28 | B | Description `head_link` | `calibration/real.yaml` adds `head_link` (parent `cam_head_link`, identity, TO_VERIFY) so the bag's RealSense static tree joins the robot; real images keep frame_ids `head_{color,depth}_optical_frame` — consumers use `header.frame_id`, not `cam_head_*_optical_frame` | Connect the replayed camera tree | — | B (C/F to confirm) |
| 2026-11-28 | B | Real `lidar_link` z | `real.yaml` `lidar_link` z 0.18 → 0.10 m (base_footprint→lidar_link 0.75 m from the bag floor), TO_VERIFY | Bag floor at −0.75 m in `lidar_link` | — | B (C/F to confirm) |
| 2026-11-28 | B | `sensors/lidar/points_filtered` | Fields x,y,z,intensity f32, ring u16, `time` f32 (s after header stamp), point_step 24, frame unchanged (`lidar_link`), **reliable** depth 5; header stamp = input stamp (first point) unless deskew | Agent C input; matches sim layout | — | B (C/F to confirm) |
| 2026-11-28 | B | `sensors/scan` | `pointcloud_to_laserscan` from `points_filtered`, frame `base_footprint`, band 0.45–1.00 m above `base_footprint` (= −0.20…0.35 m vs `base_link`), 0.35–12 m, 1,800 beams, **best-effort** (upstream SensorDataQoS) | Agent B deliverable | — | B (C/F to confirm) |
| 2026-11-28 | B | Replay QoS | Sensor consumers in replay must subscribe **reliable** (bag player offers reliable; images transient_local). Best-effort readers lost ~all 1 MB samples on this host. `cloud_filter` param `input_reliability` (default reliable) | Measured on real-bag replay | — | B (C/F to confirm) |
| 2026-11-28 | B | `sensors/cam_head/*` | Replay also provides `color/metadata`, `depth/{camera_info,metadata}`, `extrinsics/depth_to_color` (realsense2_camera_msgs) | Bag content | — | B (C/F to confirm) |
| 2026-11-28 | B | Preprocess in sim/replay | `preprocess` sub-launch now starts `cloud_filter` + `pointcloud_to_laserscan` (configs `*_{real,sim}.yaml` by `calibration_profile`) in sim and replay; drivers in replay without `bag:=` start nothing | Replaces placeholders | — | B (C/F to confirm) |
| 2026-11-28 | B | Bag player node | `/rosbag2_player` runs in the root namespace (Humble `ros2 bag play` has no `--ros-args`); its topics are remapped to absolute `/ultron/replay/...` names | Tool limitation | — | B (C/F to confirm) |
