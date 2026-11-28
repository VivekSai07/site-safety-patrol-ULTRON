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
ros2_ws/scripts/dev.sh bash -lc "source install/setup.bash && scripts/ultron_launch.sh sim mode:=mapping"
ULTRON_MODE=replay ULTRON_BAG_DIR=/abs/bag ros2_ws/scripts/dev.sh bash -lc "source install/setup.bash && scripts/ultron_launch.sh replay bag:=/abs/bag"
```

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
| `ultron_b2_preprocess`, `ultron_b2_tools` | B | stub |
| `ultron_b2_localization` | C | stub |
| `ultron_b2_navigation`, `ultron_b2_control` | D | stub |
| `ultron_b2_perception`, `ultron_b2_mission` | E | stub |
| `ultron_b2_sim`, `ultron_b2_viz` | F | stub |

Each sub-launch in `ultron_b2_bringup/launch/sub/<name>.launch.py` (drivers, preprocess,
state_estimation, slam, nav, control, mission, viz, record) currently starts a silent
`<name>_placeholder` node. Owners replace the body of their sub-launch; bringup already pushes the
mode namespace (`/ultron/{sim,replay,live}`) and `use_sim_time`, and passes the launch arguments
`run_mode, mode, odom_source, viz, record, bag, hardware_enable, use_sim_time, calibration_profile,
publish_description`. Use relative topic
names. `replay` never includes `nav`, `control` or `mission`.

### Bringup arguments

`mode:={mapping,localization,nav,mission}`, `odom_source:={sim,mola,kiss,b2}`,
`viz:={none,foxglove,rviz_bridge,both}`, `record:={none,minimal,debug}`, `hardware_enable:=false`,
`bag:=<path>`, `namespace` (must equal the mode's), `publish_description`, `tf_check`.
Replay only: `replay_rate` (1.0), `replay_loop` (false), `replay_start_offset` (0.0 s),
`replay_delay` (2.0 s), `replay_cameras` (true). The bag player runs `ros2 bag play --clock 100`
with only the contract topics remapped into `/ultron/replay` (table:
`ultron_b2_bringup/replay_topics.py`); the bag's Xsens `/tf` goes to `bag/tf_xsens`, off the TF
tree. Large clouds/images need **reliable** subscribers on this host: best-effort readers lose
~1 MB fragmented samples (see `docs/b2_ros_stack/VALIDATION.md`, Agent B).

Preflight refuses (exit 2, nothing else starts) when `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`,
namespace or `odom_source` do not match the mode table, when `hardware_enable:=true` outside
`live`, or in `live` without a valid operator token at `/etc/ultron/ULTRON_HW_AUTHORIZED`
(or `$ULTRON_HW_AUTHORIZED_FILE`; YAML `operator`, `plan`, timezone-aware `expires`). Agents never
create this token. `ros2 launch` opens a DDS participant before the in-launch preflight runs, so
use `scripts/ultron_launch.sh`, which checks the environment first.

`tf_authority_check` watches `/tf` and `/tf_static` and reports (diagnostics ERROR; exits 1 when
`duration_s` > 0 or `fail_on_violation`) any child frame with more than one publisher, a frame on
both static and dynamic TF, a child with two parents, or a ground-truth frame
(`ultron_sim_world`). In sim, `robot_state_publisher` (description) owns `base_link → sensor`
frames; the legacy `sim_bridge` static `base_link → lidar_link` must be disabled by its wrapper.

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
