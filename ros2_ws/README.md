# ULTRON B2 ROS 2 workspace (`ros2_ws/`)

ROS 2 Humble stack for the Unitree B2 patrol. Plan: `docs/b2_ros_stack/PLAN.md`.
Simulation and replay only until the documented full-system simulation gate and independent
real-world checks pass. Agents never launch `live`, use `ROS_DOMAIN_ID` 0 or create hardware tokens.

## Quick start (dev container)

```bash
# once: build the dev image (x86_64, ROS 2 Humble + Nav2, slam_toolbox, MOLA, ...)
docker build -f ros2_ws/docker/Dockerfile.dev -t ultron-b2-dev:humble ros2_ws/docker

ros2_ws/scripts/dev.sh bash -lc "scripts/import_deps.sh && colcon build --symlink-install"
ros2_ws/scripts/dev.sh bash -lc "source install/setup.bash && colcon test --packages-select-regex '^ultron_b2_' && colcon test-result --verbose"
ros2_ws/scripts/dev.sh scripts/ci.sh                    # full CI: deps, build, test, repo unittests
ros2_ws/scripts/dev.sh bash -lc "source install/setup.bash && scripts/ultron_launch.sh sim mode:=mapping odom_source:=mola"
ULTRON_MODE=replay ULTRON_BAG_DIR=/abs/bag ros2_ws/scripts/dev.sh bash -lc \
  "source install/setup.bash && scripts/ultron_launch.sh replay mode:=mapping odom_source:=mola bag:=/abs/bag"
```

For simulation navigation or mission, load a saved map using `map:=<map-directory>` and launch
`sim mode:=nav` or `sim mode:=mission`. Without a map, localization remains INITIALIZING.
Launching mission mode starts its nodes; it does not by itself start a mission or grant operator
enable. The sim ladder detector uses labelled ground-truth **ORACLE** detections, not real perception.
See `docs/b2_ros_stack/VALIDATION.md` for measured results and outstanding integration checks.
For a step-by-step sensor-stream probe and mission limitations, see
[`docs/b2_ros_stack/SIM_TEST_GUIDE.md`](../docs/b2_ros_stack/SIM_TEST_GUIDE.md).
The guide also covers the explicit simulation-only mapping teleop option and map saving.

`dev.sh` mounts the repo at the same absolute path, runs as your uid/gid with
`HOME=ros2_ws/.dev_home`, and sets `ROS_LOCALHOST_ONLY=1`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
`CYCLONEDDS_URI=…/cyclonedds_localhost.xml` and `ROS_DOMAIN_ID` 71 (`ULTRON_MODE=sim`, default) or
72 (`ULTRON_MODE=replay`). `ULTRON_DEV_NET=bridge` (default) gives each container a private
loopback so parallel agents never see each other's DDS graph; `host` shares the host loopback
(still localhost-only); `none` has no network. Extra docker args: `ULTRON_DEV_ARGS`.
`ULTRON_BAG_DIR` mounts a bag directory read-only at the same path (it must be outside the repo).

The localhost Cyclone profile raises `MaxAutoParticipantIndex` to 120: loopback has no multicast,
and the default index limit (9) refuses participants once the stack plus CLI tools exceed it
("Failed to find a free participant index"). Do not also list `lo` in `<Interfaces>`
(`ROS_LOCALHOST_ONLY=1` already selects it; Cyclone rejects it twice).

## Packages (PLAN §4)

| Package | Owner | State |
|---|---|---|
| `ultron_b2_interfaces` | A | msgs/srvs/actions + `ultron_b2_contracts` JSON converters |
| `ultron_b2_description` | A | B2 xacro, `calibration/{sim,real}.yaml`, `config/footprint.yaml` |
| `ultron_b2_bringup` | A | `{sim,replay,live}.launch.py`, preflight, `tf_authority_check` (C++), DDS profiles |
| `ultron_b2_preprocess`, `ultron_b2_tools` | B | replay inputs, cloud filtering and scan conversion |
| `ultron_b2_localization` | C | lidar odometry, SLAM, localization monitoring and map storage |
| `ultron_b2_navigation`, `ultron_b2_control` | D | Nav2, command safety chain and disabled-by-default B2 bridge |
| `ultron_b2_perception`, `ultron_b2_mission` | E | ladder detection interfaces, tracking and mission logic |
| `ultron_b2_sim`, `ultron_b2_viz` | F | kinematic simulation, visualization and recording |

Bringup includes the sub-launches in `ultron_b2_bringup/launch/sub/` and pushes the mode namespace
(`/ultron/{sim,replay,live}`) and `use_sim_time`. Some components are inactive or placeholders
outside their applicable mode. The shared sub-launch arguments are
`run_mode, mode, odom_source, viz, record, bag, hardware_enable, use_sim_time, calibration_profile,
publish_description`. Use relative topic
names. `replay` never includes `nav`, `control` or `mission`.

### Bringup arguments

`mode:={mapping,localization,nav,mission}`, `odom_source:={sim,mola,kiss,b2}`,
`viz:={none,foxglove,rviz_bridge,both}`, `record:={none,minimal,debug,full}`, `hardware_enable:=false`,
`bag:=<path>`, `namespace` (must equal the mode's), `publish_description`, `tf_check`.
Replay only: `replay_rate` (1.0), `replay_loop` (false), `replay_start_offset` (0.0 s),
`replay_delay` (2.0 s), `replay_cameras` (true). The bag player runs `ros2 bag play --clock 100`
with only the contract topics remapped into `/ultron/replay` (table:
`ultron_b2_bringup/replay_topics.py`); the bag's Xsens `/tf` goes to `bag/tf_xsens`, off the TF
tree. Large clouds/images need **reliable** subscribers on this host: best-effort readers lose
~1 MB fragmented samples (see `docs/b2_ros_stack/VALIDATION.md`, Agent B).

Preflight refuses (exit 2, nothing else starts) when `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`,
namespace or `odom_source` do not match the mode table, when `hardware_enable:=true` outside
`live`, or when enabling hardware in `live` without a valid operator token at `/etc/ultron/ULTRON_HW_AUTHORIZED`
(or `$ULTRON_HW_AUTHORIZED_FILE`; YAML `operator`, `plan`, timezone-aware `expires`). Agents never
create this token. Even with hardware disabled, live mode joins the robot network; do not launch it
without explicit authorization. `ros2 launch` opens a DDS participant before the in-launch
preflight runs, so use `scripts/ultron_launch.sh`, which checks the environment first.

`tf_authority_check` watches `/tf` and `/tf_static` and reports (diagnostics ERROR; exits 1 when
`duration_s` > 0 or `fail_on_violation`) any child frame with more than one publisher, a frame on
both static and dynamic TF, a child with two parents, or a ground-truth frame
(`ultron_sim_world`). In sim, `robot_state_publisher` (description) owns `base_link → sensor`
frames; the sim wrapper disables the legacy `sim_bridge` static `base_link → lidar_link`.

## Visualization

Set `viz:=foxglove`, `viz:=rviz_bridge` or `viz:=both` explicitly; the default is `none`.
Foxglove binds to loopback on port 8765 and exposes an allow-list of throttled, read-only data.
The default private Docker network isolates that loopback from the host; for local viewing,
`ULTRON_DEV_NET=host` shares host loopback and its port with other local users. Laptop RViz uses
the separate visualization domain 42; the Zenoh option is not yet validated. Setup, SSH tunnels,
topic restrictions and limits are in `docs/b2_ros_stack/VIZ_GUIDE.md`.

## Hardware boundary

The real B2 bridge is disabled by default and has only been exercised against a mock. No launch
example here authorizes hardware connection or motion. Complete-system hardware deployment remains
NO-GO until full-system simulation acceptance and independent real-world checks are documented.
Missing or stale robot responses and sport-state messages are diagnostic information, not an
independent hardware watchdog or an arming interlock.
Any real motion also requires a responsible operator, verified RC priority, an independently tested
watchdog and a functional physical E-stop; see `docs/HARDWARE_READINESS.md` and
`docs/SIM_TO_HARDWARE.md`.

## Dependencies (`deps.repos`)

`scripts/import_deps.sh` imports pinned sources into `external/` (gitignored). `external/build/*`
is built with the workspace (`unitree_ros2` msgs `unitree_api`, `unitree_go`, `unitree_hg`;
`rslidar_msg`). `external/ref/*` are pinned references with `COLCON_IGNORE`. The dev image
installs these as apt binaries (recorded in `/opt/ultron/apt_optional.txt`): slam_toolbox,
MOLA + mola_lidar_odometry + mola_state_estimation, mp2p_icp, robot_localization,
spatio_temporal_voxel_layer, realsense2_camera_msgs, foxglove_bridge, domain_bridge. No Humble
binary exists for `kiss_icp` (pip `kiss-icp==1.2.3` is installed; the ROS wrapper is a pinned
source ref) or `rslidar_msg` (built from source).

unitree_ros2 has no B2-specific message package: B2 uses `unitree_go` (`SportModeState`,
`LowState`, …) and `unitree_api` (`Request`/`Response`). B2 sport API ids are TO_VERIFY.

## Containers

- `docker/Dockerfile.dev` — `ultron-b2-dev:humble`, x86_64, `ros:humble-ros-base` + apt stack +
  repo python extras (mujoco, fastapi, …) so the repo unittest suite runs in-container.
- `docker/Dockerfile.orin` — arm64, `BASE_IMAGE` build-arg required (JetPack/L4T TO_VERIFY).
  **UNRUN.**
- `docker/compose.{sim,replay}.yaml` — host network with `ROS_LOCALHOST_ONLY=1`, domains 71/72.
- `ci/github-workflow.ros2_ws.yml` — inactive template (manual trigger) pending user approval.
