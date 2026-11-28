# Viewing ULTRON B2 data: Foxglove and laptop RViz (Agent F)

Visualization is read-only. Nothing in this guide can publish commands, call services or set
parameters on the robot stack. Raw lidar clouds and raw images are never exposed; the viewer sees
only the throttled copies made by `viz_throttle` plus small topics (scan, odom, TF, maps, status).
All ports, rates and limits are PROPOSED, sim-measured values. Verify them on the robot Wi-Fi.

| `viz:=` | Starts (in the stack namespace `/ultron/<mode>`) |
|---|---|
| `none` (live default) | nothing |
| `foxglove` (sim/replay default) | `viz_throttle` + `foxglove_bridge` on `127.0.0.1:8765` |
| `rviz_bridge` | `viz_throttle` + `domain_bridge` (stack domain → 42) |
| `both` | all three |

The allow-list has one source of truth: `ultron_b2_viz/whitelist.py`. After editing it, regenerate
the configs with `ros2 run ultron_b2_viz gen_viz_configs`. A test checks that they are in sync.

## Why there is no shared DDS over Wi-Fi

The stack runs on the Orin in domain 71 (sim), 72 (replay) or 0 (live), with loopback-only DDS.
DDS discovery over Wi-Fi is unreliable and multicast-heavy. Joining the laptop to the stack
domain would also give it a path to command topics. Instead:

- **Foxglove (default):** one WebSocket on the Orin's loopback, reached through an SSH tunnel.
- **Laptop RViz:** `domain_bridge` copies the whitelist into a separate **viz domain 42** on the
  Orin. Domain 42 never carries the stack's raw or command topics. The laptop reaches domain 42
  over an SSH-tunnelled zenoh link, or plain DDS on a trusted wired LAN.

## Option 1: Foxglove (recommended)

On the Orin, start the stack with `viz:=foxglove` (sim and replay default to it):

```bash
ros2 launch ultron_b2_bringup sim.launch.py mode:=mapping viz:=foxglove
```

On the laptop:

```bash
ssh -N -L 8765:127.0.0.1:8765 <user>@<orin>
```

Then, in Foxglove, open a connection to `ws://localhost:8765` (Foxglove WebSocket).

- Import a layout from `ros2_ws/src/ultron_b2_viz/foxglove/<mode>/ultron_{mapping,nav,mission}.json`.
  Layouts are generated per mode because topic names carry the mode namespace.
- The bridge binds `127.0.0.1` only. Binding a LAN/Wi-Fi address (`foxglove_address:=...`) needs
  user approval.
- Client publishing, services and parameters are disabled. The service, parameter and client-topic
  whitelists are set to match nothing, and only the `connectionGraph` and `assets` capabilities are
  enabled.
- The port can be changed with `foxglove_port:=`.

## Option 2: laptop RViz via viz domain 42

On the Orin, run with `viz:=rviz_bridge` (or `both`). The stack's `ROS_DOMAIN_ID` must match the
mode (sim 71, replay 72); the launch refuses otherwise. The bridged names are mode-independent:

| Domain 42 topic | From |
|---|---|
| `/ultron/viz/points` | `viz/points` (voxel 0.10 m, ≤30k pts, ≤2 Hz) |
| `/ultron/viz/map_cloud` | `viz/map_cloud` (when C publishes a 3D map) |
| `/ultron/viz/cam_head/color/image/compressed` | JPEG ≤640 px wide, ≤5 Hz |
| `/ultron/viz/map`, `global_costmap`, `local_costmap`, `plan` | rate-limited relays |
| `/ultron/viz/sensors/scan`, `state/odom`, `state/localization`, `robot_description`, `control/active_source`, `safety/motion_gate`, `perception/ladder_{detections,candidates}`, `mission/{status,events}` | stack topics of the same relative name |
| `/tf`, `/tf_static` | global |

### 2a: zenoh over an SSH tunnel (no DDS on Wi-Fi)

This option is UNRUN: `zenoh-bridge-ros2dds` is not in the dev image.

1. On the Orin, add `viz_zenoh:=true`. The launch then starts `zenoh-bridge-ros2dds -c
   config/zenoh_bridge_orin.json5`. It is a router on `tcp/127.0.0.1:7447`, reads domain 42 and
   has an allow-list with per-topic frequency caps. If the binary is missing, it logs `UNRUN` and
   continues.
2. On the laptop, open the tunnel with `ssh -N -L 7447:127.0.0.1:7447 <user>@<orin>`.
3. On the laptop, run `zenoh-bridge-ros2dds -c zenoh_bridge_laptop.json5 -e tcp/127.0.0.1:7447`.
   This runs in client mode and recreates the subscribers locally in domain 42.
4. On the laptop, set `export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1`, then run
   `rviz2 -d ultron_viz42.rviz` (from `ros2_ws/src/ultron_b2_viz/rviz/`).

### 2b: plain DDS on a trusted wired link

On a dedicated Ethernet cable or isolated LAN only, never shared Wi-Fi:

- Laptop and Orin both use `ROS_DOMAIN_ID=42`, with a Cyclone profile bound to that interface.
- The Orin stack itself stays in its own domain on loopback; only domain 42 leaves the machine.

This needs user approval because it opens DDS on a network interface.

### RViz notes

`ultron_viz42.rviz` uses fixed frame `map` (switch to `odom` in mapping without a map). The camera
display subscribes to the `.../image` base name with the `compressed` transport. If the Image
display shows nothing, set its *Transport Hint* to `compressed`.

## Bandwidth (sim, measured 2026-11-28)

Measurements come from the dev container, sim `mode:=mapping viz:=both`, over 20 s. Artifacts are
in `runs/agent_f/` (git-ignored).

| Stream | Raw in stack | Viz copy |
|---|---|---|
| lidar points (10 Hz, ~0.78 MB/msg) | 6.99 MB/s | 0.20 MB/s (2 Hz, voxel 0.10 m) ≈ 34× less |
| head colour image (10 Hz, 640×360 rgb8) | 6.64 MB/s | 0.074 MB/s (≈4.4 Hz JPEG q70) ≈ 90× less |
| everything in domain 71 (incl. depth, oracle labels) | 172.8 Mbit/s | — |
| everything arriving in domain 42 | — | 2.59 Mbit/s |
| Foxglove client subscribed to every advertised channel | — | 2.53 Mbit/s |

`state/odom` and `/tf` are bridged at their full 50 Hz in domain 42 (about 0.33 Mbit/s together).
The zenoh Orin config caps odom at 10 Hz for Wi-Fi.

Throttling is wall-clock based. `viz/throttle_status` reports per-stream in/out bytes per second
every 5 s.

## Recording (rosbag2, mcap + zstd)

Use `record:={none,minimal,debug,full}`. Bags are written to
`runs/bags/<mode>_<profile>_<UTC>/` (or `$ULTRON_RUNS_DIR/bags`). `runs/` is git-ignored; never
commit bags.

| Profile | Contents | Sim size |
|---|---|---|
| minimal | TF, `/clock`, state, scan, map, mission/safety/control status, diagnostics | 1.2 MiB / 52 s |
| debug | minimal + IMU, `viz/*`, command chain, plan, perception, sim ground truth/status, camera_info | 0.74 MiB / 23 s (viz off) |
| full | everything in the mode namespace incl. raw lidar/images | 143 MiB / 23 s (≈6.2 MB/s) |

Profiles are defined in `ultron_b2_bringup/config/record/profiles.yaml`, and zstd "Fast" settings
in `mcap_zstd.yaml`. Stop the stack with Ctrl-C (SIGINT) so the recorder writes `metadata.yaml`.
A SIGTERM-killed recorder leaves an mcap without metadata. `ros2 bag reindex` can repair that.
