# B2 ROS stack: clone, build and bag replay

This handoff is for a second Linux PC and its coding agent. Read `AGENTS.md`,
`CLAUDE.md`, this file, `ros2_ws/README.md`, and
`docs/b2_ros_stack/VALIDATION.md` first. The branch is a work in progress:
the full autonomous simulation gate has **not** passed. Do not describe this
stack as hardware-ready.

## What this branch contains

`ros2_ws/src/` contains the Humble interfaces, description, sensor
preprocessing, odometry and SLAM, navigation and motion gate, simulation,
perception and mission packages. `ros2_ws/deps.repos` pins external source
repositories; `ros2_ws/scripts/import_deps.sh` downloads them under ignored
`ros2_ws/external/`. This repository currently has no Git submodules. The
dev image supplies other ROS and Python dependencies. `runs/`, ROS build
outputs, local maps, bags, credentials and model weights are not transferred.

The ROS hall simulation is a kinematic B2 base with synthetic sensors and
oracle detections, not the organizer's Unitree DDS simulator or validated
walking. Replay is sensor/SLAM analysis only: it does not start navigation,
mission or control, and it does not command a robot. The real B2 adapter is
disabled and unverified. Camera registration, extrinsics, timestamps and
physical stopping need independent verification.

## Clone and prepare

On an x86-64 Linux PC, install Git and Docker Engine with permission to run
Docker, and ensure the PC has internet access and sufficient disk space for
the image, dependencies, build and bag. Docker requires a supported platform;
the Orin/arm64 image is unvalidated. From the directory where you want the
repository:

```bash
git clone --branch om/b2-ros-stack --single-branch \
  https://github.com/VivekSai07/site-safety-patrol-ULTRON.git
cd site-safety-patrol-ULTRON
git status --short --branch
```

From the project root, build the image and workspace:

```bash
docker build -f ros2_ws/docker/Dockerfile.dev -t ultron-b2-dev:humble ros2_ws/docker
ros2_ws/scripts/dev.sh bash -lc 'scripts/import_deps.sh && colcon build --symlink-install'
ros2_ws/scripts/dev.sh bash -lc 'source install/setup.bash && cat /opt/ultron/apt_optional.txt'
```

The import requires access to the pinned upstream repositories. `mola` is
optional in the image build: check `/opt/ultron/apt_optional.txt` before using
`odom_source:=mola`; if unavailable, diagnose the installation rather than
assuming it is installed. `odom_source:=kiss` uses the image's pinned Python
KISS-ICP dependency. Do not silently substitute one estimator or claim it is
validated for the robot.

For ROS package checks:

```bash
ros2_ws/scripts/dev.sh bash -lc \
  'source install/setup.bash && colcon test --packages-select-regex "^ultron_b2_" && colcon test-result --verbose'
```

This is not full-mission acceptance; opt-in tests may be skipped. Existing
`colcon test-result` files from unrelated packages can affect its summary.
To repeat from a clean checkout, build and test there rather than deleting
someone else's build or runs.

## Obtain and inspect the bag

The recorded B2 bag is **not in Git** and no public download URL has been
verified. Obtain it separately through an authorized private channel; do not
upload it to GitHub or another service. Extract it outside the repository so
the bag directory contains `metadata.yaml` and the storage file(s). Set
`BAG_DIR` to that directory, not its parent or ZIP file:

```bash
export BAG_DIR="$(realpath /absolute/path/to/rosbag-directory)"
if test -f "$BAG_DIR/metadata.yaml"; then
  ULTRON_MODE=replay ULTRON_BAG_DIR="$BAG_DIR" ros2_ws/scripts/dev.sh \
    bash -lc 'source install/setup.bash && ros2 bag info "$1"' _ "$BAG_DIR"
else
  echo 'Missing rosbag2 metadata.yaml'
fi
```

The dev script mounts the bag read-only and refuses a bag inside the
repository. Do not expose private recordings, maps or derived output without
authorization and rights/privacy review.

The previously inspected recording is described in
`docs/b2_ros_stack/BAG_FACTS.md`; it is one particular recording, not a
generic sensor specification. Its expected lidar topic is `/scan_cloud`,
IMU is `/imu/data`, and camera topics are under `/camera_depth/head/`.
It does not contain B2 odometry or measured body-to-sensor extrinsics.
For a different bag, inspect topic types, frames, timestamps and calibration
before replay; the fixed topic table is
`ros2_ws/src/ultron_b2_bringup/ultron_b2_bringup/replay_topics.py`.

## Replay and inspect

Start from the project root, with `BAG_DIR` set as above:

```bash
ULTRON_MODE=replay ULTRON_BAG_DIR="$BAG_DIR" ros2_ws/scripts/dev.sh \
  bash -lc 'source install/setup.bash && scripts/ultron_launch.sh replay mode:=mapping odom_source:=mola bag:="$1" replay_delay:=12 replay_rate:=0.5 replay_cameras:=true' \
  _ "$BAG_DIR"
```

Use `odom_source:=kiss` only as a separate comparison. Do not use
`replay_loop:=true` for the first check. The replay graph uses domain 72,
localhost-only DDS and a private Docker bridge. It plays the bag once and
the launch remains running; Ctrl-C stops the launch. The fixed RealSense
and lidar mount transforms include unverified assumptions. Raw depth is not
registered to color: do not infer valid real perception or label reading.

While playback is running, open a second host terminal, identify **your**
container, and enter that same container; a new `dev.sh` container has a
different private DDS graph:

```bash
docker ps --filter ancestor=ultron-b2-dev:humble
docker exec -it <your-container-id> bash
source install/setup.bash
ros2 node list --no-daemon
ros2 topic list -t
ros2 run ultron_b2_tools stream_probe --namespace /ultron/replay --duration 10 \
  --topics /clock sensors/lidar/points sensors/scan sensors/imu/data \
  sensors/cam_head/color/image_raw sensors/cam_head/depth/image_rect_raw \
  sensors/cam_head/color/camera_info state/odom
```

Check delivered counts, frames and timestamps during playback; a listed
topic alone is not evidence of delivery. After the bag ends, the replay
clock watchdog should mark localization `STALE`, not `VALID`. Do not
interpret a map or an empty output as validated localization or coverage.
For the specific previously inspected bag, an optional automated mapping
diagnostic writes private local results to a **new** directory:

```bash
mkdir -p ros2_ws/runs
ULTRON_MODE=replay ULTRON_BAG_DIR="$BAG_DIR" ros2_ws/scripts/dev.sh \
  bash -lc 'scripts/accept/replay_mapping.sh "$PWD/runs/replay-check-01" "$1" mola' _ "$BAG_DIR"
```

This diagnostic defaults to cameras off and 0.7x replay. Review its
`checks.json`, `summary.json`, `audit.json`, timing, logs and map rather
than treating a successful script exit as hardware or full-system acceptance.
The output directory must not already exist. For other recordings, its
fixed topics, required observations and timeout may not apply.

## Simulation and separate organizer kit

For the ROS hall simulation, mapping teleoperation, snapshot capture and map
saving, use `docs/b2_ros_stack/SIM_TEST_GUIDE.md`. Maps from this machine are
not included: build one with the same scan pipeline on the new PC. See
`docs/b2_ros_stack/VALIDATION.md` for failed full mission attempts and
remaining localization, identity, frontier and stop-acknowledgment issues.
The simulated pitch experiment is not integrated into this ROS stack.

The separately inspected organizer-kit checkout can be obtained beside this
repository if needed; it is **not** a required ROS workspace submodule:

```bash
cd ..
git clone https://github.com/smarobix/ctrl-hackathon-2026-kit.git
git -C ctrl-hackathon-2026-kit checkout 8f5de5806fcd432d339ca1dbb61a1f0f9ea1d6cd
cd site-safety-patrol-ULTRON
```

That is the source and revision examined locally, not a claim that the
repository is an official upstream. Follow `docs/ORGANIZER_B2_SIM.md` for
its separately isolated DDS smoke test. Do not mix its DDS simulation with
the ROS hall graph or claim a working high-level B2 walking service.

## Agent safety boundary

Do not launch `live`, connect to a robot, use domain 0, or issue hardware
commands without explicit authorization. Complete integrated deployment
requires documented full-system simulation acceptance and independent
real-world checks. Any real motion additionally requires a responsible
operator, verified RC priority, an independently tested watchdog and a
functional physical E-stop. Preserve unknown/conflicting inspection evidence:
`AUTOMATED_PASS` is scoped pre-screening, not a safety certificate.
