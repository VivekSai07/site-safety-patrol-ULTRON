# Agent A — Foundation: workspace, interfaces, description, bringup, containers, CI

Read [FLEET_COMMON.md](FLEET_COMMON.md) and [PLAN.md](PLAN.md) first.
Branch: `om/b2-ros-stack-a`. **Start immediately; everyone else depends on you.**
Deliver a buildable skeleton in your first commits so B–F can start on stubs.

## Kickoff prompt

> You are Agent A for the ULTRON B2 ROS 2 stack. Follow AGENTS.md, CLAUDE.md,
> docs/b2_ros_stack/FLEET_COMMON.md, PLAN.md and AGENT_A.md. Create the ros2_ws
> foundation: interfaces, B2 description with sensor frames, bringup launch
> skeleton with preflight and TF-authority check, DDS configs, Docker dev/Orin
> images and CI. Ship a buildable skeleton early, then harden. No hardware
> connection. Record actual results in docs/b2_ros_stack/VALIDATION.md (Agent A).

## Deliverables

1. **Workspace** `ros2_ws/src/` with packages from PLAN §4 (empty-but-buildable
   where owned by others: `package.xml`, `setup.py`/`CMakeLists.txt`, README
   naming the owner agent). `ros2_ws/deps.repos` with pinned refs for
   slam_toolbox (binary preferred), MOLA/mola_lidar_odometry (binary preferred),
   kiss-icp, rslidar_sdk + rslidar_msg, Xsens driver, realsense-ros,
   unitree_ros2 (`unitree_api`, `unitree_go`, `unitree_hg` msgs), foxglove_bridge,
   domain_bridge, m-explore-ros2. Mark which are apt binaries vs source.
2. **`ultron_b2_interfaces`** (ament_cmake) — msgs/srvs/actions listed in PLAN §4.
   Each msg has `string schema_version`. Field meanings mirror existing JSON
   contracts: `ultron.localization_state/v1` (`src/ultron/autonomy/localization_state.py`),
   `ultron.navigation_goal/v1` and statuses `ACCEPTED, ACTIVE, REACHED, REJECTED,
   CANCELLED, TIMEOUT, FAILED` (`navigation_contract.py`), supervisor actions
   `INSPECT, SURVEY, RETURN_BASE, RETURN_BASE_WITH_GAPS, AWAIT_OPERATOR`.
   Add `NavigateToView.action` (goal: goal_id, map_id, map_version, target
   PoseStamped, tolerances, timeout, reason, asset_id; feedback: status, pose;
   result: status, reason, last_valid_pose, settled). Add a Python round-trip
   converter test for each JSON ↔ msg.
3. **`ultron_b2_description`** — B2 xacro based on the official Unitree B2
   model (`unitreerobotics/unitree_ros` `b2_description` or the kit's MJCF;
   keep BSD-3 licence/provenance file). Frames: `base_link`, `base_footprint`,
   `lidar_link`, `imu_link`, `cam_head_link`, `cam_head_color_optical_frame`,
   `cam_head_depth_optical_frame`. Mount values loaded from
   `calibration/sim.yaml` (copy from `config/autonomy/lidar_helios32.yaml`:
   lidar `[0.342, 0, 0.18]`) and `calibration/real.yaml` (all values
   `source: TO_VERIFY`, filled later by Agent B from the bag `/tf_static`).
   `footprint.yaml` (sim: circle r = 0.55 m; real: TO_VERIFY).
4. **`ultron_b2_bringup`** — `sim.launch.py`, `replay.launch.py`,
   `live.launch.py`, each including sub-launches `drivers, preprocess,
   state_estimation, slam, nav, control, mission, viz, record` (stubs that
   other agents fill). Args: `mode:={mapping,localization,nav,mission}`,
   `odom_source:={sim,mola,kiss,b2}`, `viz:={none,foxglove,rviz_bridge,both}`,
   `record:={none,minimal,debug}`, `hardware_enable:=false`, `bag:=<path>`.
   All nodes pushed into the mode namespace (`PushRosNamespace`) and use
   relative topics.
   - `preflight` node/script: refuses to start if `ROS_DOMAIN_ID` /
     `ROS_LOCALHOST_ONLY` / namespace do not match the mode table; refuses
     `live` with `hardware_enable:=true` unless `ULTRON_HW_AUTHORIZED` file
     token exists (document; agents never create it).
   - `tf_authority_check` node: subscribes `/tf`, fails (non-zero exit +
     diagnostic ERROR) if any child frame has more than one publisher or if the
     ground-truth frame appears.
   - DDS: `config/dds/cyclonedds_localhost.xml`, `cyclonedds_robot.xml`
     (interface name placeholder `TO_VERIFY`), with SHM off by default.
5. **Containers** `docker/dev.Dockerfile` (osrf/ros:humble-desktop, x86) and
   `docker/orin.Dockerfile` (arm64; base image TO_VERIFY against the Orin's
   JetPack — parameterize via build-arg; do not guess), `docker/compose.*.yaml`
   for sim and replay (host network only for localhost modes, domain env set).
6. **CI script** `ros2_ws/scripts/ci.sh`: rosdep install, colcon build,
   colcon test (incl. `launch_testing`), and the repo's existing unittest
   suite. Add a GitHub Actions workflow file only if the user approves later —
   for now, script only.

## Acceptance (record in VALIDATION.md → Agent A)

- `ci.sh` passes in the dev container from a clean clone.
- `ros2 launch ultron_b2_bringup sim.launch.py mode:=mapping` starts stubs
  without errors and with all nodes under `/ultron/sim`.
- Preflight unit tests: wrong domain → refuse; live + hardware_enable without
  token → refuse; replay never launches control nodes.
- `tf_authority_check` test: two publishers of `odom→base_link` → failure.
- Interface converter tests pass.

## Do not

Build drivers/SLAM/Nav logic (other agents). Guess Orin base image or NIC
names. Add hardware tokens.
