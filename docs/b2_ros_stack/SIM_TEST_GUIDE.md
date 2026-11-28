# B2 ROS stack: isolated simulation test

This is the existing `om/b2-ros-stack` worktree, not the organizer's Unitree
walking controller. Its B2 base moves kinematically; its RGB-D, lidar and IMU
are synthetic. The mission detector uses **ORACLE** simulator labels. These
checks do not validate B2 locomotion, real perception or inspection results.
The full mission simulation gate is **not accepted**; see
[VALIDATION.md](VALIDATION.md#m6--full-system-simulation-acceptance).
For the separate organizer-provided Unitree MuJoCo DDS simulator and its
viewer/stand limitations, see [ORGANIZER_B2_SIM.md](../ORGANIZER_B2_SIM.md).
The organizer simulator is not the ROS hall stack described below.

## Start the sensors and mapping graph

From the repository root, using the existing Humble dev image and built
workspace:

```bash
ros2_ws/scripts/dev.sh bash
```

In the container:

```bash
source install/setup.bash
scripts/ultron_launch.sh sim mode:=mapping odom_source:=sim
```

The dev script uses a private Docker network, ROS domain 71 and localhost-only
DDS. `odom_source:=sim` uses the kinematic simulator's odometry, not MOLA or
real B2 odometry. Mapping requires movement to observe additional space; merely
starting the graph does not create a complete map. Stop the foreground launch
with Ctrl-C. If the image or workspace is absent, use the build instructions in
[`ros2_ws/README.md`](../../ros2_ws/README.md).

## Inspect the same isolated graph

In a second host terminal, identify **your** container with
`docker ps --filter ancestor=ultron-b2-dev:humble` and enter it:

```bash
docker exec -it <your-container-id> bash
source install/setup.bash
ros2 node list --no-daemon
ros2 run ultron_b2_tools stream_probe --namespace /ultron/sim --duration 10 \
  --topics /clock sensors/lidar/points sensors/scan sensors/imu/data \
  sensors/cam_head/color/image_raw sensors/cam_head/depth/image_rect_raw \
  sensors/cam_head/color/camera_info state/odom
```

Check the counts, rates and frame IDs in the printed JSON. A zero count or
missing topic is a failed stream check, not a pass. Image output can be slow
on a software renderer. The probe's subscriber reliability adapts to the
publisher; a topic name alone does not demonstrate delivered data.

## Drive a first map with simulated teleop

The default motion gate requires a loaded map and valid localization, even for
teleop. For **simulation mapping only**, stop the previous launch and explicitly
opt in to teleop without localization:

```bash
scripts/ultron_launch.sh sim mode:=mapping odom_source:=sim \
  sim_teleop_without_localization:=true
```

In another terminal inside the **same container**, keep this simulated operator
heartbeat running while driving:

```bash
source install/setup.bash
ros2 topic pub -r 10 /ultron/sim/ops/operator_enable ultron_b2_interfaces/msg/OperatorEnable \
  '{schema_version: ultron.operator_enable/v1, enabled: true, operator_id: sim-operator}'
```

In a third terminal inside that container, use the keyboard controls:

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/ultron/sim/control/in/teleop -p speed:=0.2 -p turn:=0.4
```

The program displays its keys. Use Ctrl-C to stop the keyboard publisher, then
stop the heartbeat. This option changes **only** the localization requirement
for teleop in sim mapping: autonomy still requires localization, and teleop
still passes through the mux, collision monitor and final gate with an operator
heartbeat. For this simulation-only override, the final gate also requires a
fresh, nonempty scan with at least one valid return. A missing, invalid or
stale scan inhibits teleop; this is a limited simulation check, not proof of
complete obstacle coverage. This simulated heartbeat is not a hardware safety device.
The sim navigation and mission planners use a proposed 0.90 m planning
clearance, larger than the simulated collision monitor's 0.85 m approach
radius. This conservative simulation-only setting can make views or routes
unreachable; it is not a measured B2 footprint or a real-world clearance.

To capture a simulated camera frame, stop the keyboard publisher, then from a
terminal in the same container:

```bash
mkdir -p "$HOME/sim_captures"
ros2 run ultron_b2_tools sim_snapshot --output "$HOME/sim_captures/capture-01"
```

Choose a new directory for each capture; the tool refuses to overwrite an
existing one. It writes `color.png` and `metadata.json` with the image timestamp,
frame and SHA-256. It subscribes only to the fixed simulation color topic in
domain 71 and never sends motion commands. It does not authenticate the
publisher or verify that the simulated robot has settled; visually review the
image. Capturing a frame does not read a label or fill an inspection checklist.
Do not upload captures without a rights/privacy review.
The separate [label-first workflow](../LABEL_FIRST_USAGE.md) supports
reviewed simulated evidence and draft forms; this PNG is not automatically
imported into that workflow or associated with a ladder.

After surveying, save a map in another container terminal:

```bash
source install/setup.bash
ros2 service call /ultron/sim/map/save ultron_b2_interfaces/srv/SaveMap \
  '{schema_version: ultron.save_map/v1, map_id: sim-teleop, save_3d: false}'
```

Check `success` and the returned directory; an incomplete or poorly observed
map is not a validated map. Saved maps and captures are local run artifacts.
Build and reload maps with the same scan pipeline: the older SLAM evaluation
bridge projects a different height band and angular resolution than this
bringup's `cloud_to_scan` pipeline. An old evaluation map may therefore be
inconsistent with the mission scan, even if it loads successfully.

The simulated flow is lidar → preprocessing → scan and odometry → SLAM /
localization; RGB-D and simulated labels feed the mission perception nodes.
The simulation-only localizer uses pixel-level oracle labels paired with the exact
color and depth timestamps. It rejects missing masks instead of substituting box
depth; it remains privileged simulator truth, not a real segmentation model.
Mapping mode does **not** start the mission perception nodes. For a mission
graph, stop mapping and use a previously saved and validated map:

```bash
scripts/ultron_launch.sh sim mode:=mission odom_source:=sim map:=<saved-map-directory>
```

This starts nodes but does **not** start a mission or enable motion. A missing
map leaves localization INITIALIZING. Even with a map, a full mission is not
currently demonstrated: localization/TF staleness, excess candidate identities,
frontier preemption and navigation/stop acknowledgments need resolution. The
latest diagnostic also reported many rejected simulated collision attempts.
The mission's captures are simulated and are not an end-to-end Nesseler checklist
or label-reading result. For map saving and alternate odometry backends see
[`ultron_b2_localization/README.md`](../../ros2_ws/src/ultron_b2_localization/README.md).
This ROS hall bridge does not expose a simulated `Euler` pitch service. The
separate offline evidence-sweep pitch experiment in the other team worktree is
not wired into this ROS stack or the organizer's Unitree DDS simulator.

## Before any real-world integration

Replay is separate from mission/control and uses a read-only bag in domain 72;
follow [`ros2_ws/README.md`](../../ros2_ws/README.md). Camera-to-body and sensor
extrinsics, intrinsics, registration, timestamps, TF ownership and actual driver
and B2 firmware interfaces must be measured and verified. The recorded color and
depth cameras have a baseline; the localizer now refuses unregistered fusion
outside the simulated co-located camera configuration. Supplying a flag alone
does not register images.

Live mode and the hardware command bridge have not been validated. Complete
integrated deployment requires the documented full-system simulation gate and
independent real-world checks. Any connection needs explicit authorization;
any real motion additionally requires a responsible operator, verified RC
priority, an independently tested watchdog and a functional physical E-stop.
