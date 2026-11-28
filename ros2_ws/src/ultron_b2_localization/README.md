# ultron_b2_localization

**Owner: Agent C** (see `docs/b2_ros_stack/AGENT_C.md`, PLAN.md §4). This package covers state
estimation, 2D SLAM and localization, LocalizationState monitoring and versioned maps for the
ULTRON B2 stack. Every topic name is relative; the bringup applies the mode namespace
(`/ultron/sim`, `/ultron/replay`, ...). The integration points are
`ultron_b2_bringup/launch/sub/{state_estimation,slam}.launch.py`.

## Outputs (consumed by navigation and mission)

| Interface | Type | Producer |
|---|---|---|
| TF `odom → base_link` (planar: z = roll = pitch = 0) | tf2 | exactly one node, chosen by `odom_source` (below) |
| `state/odom` | nav_msgs/Odometry (frame `odom`, child `base_link`) | same node as the TF |
| TF `map → odom` | tf2 | slam_toolbox (mapping, or localization when a map is loaded) |
| `map` | nav_msgs/OccupancyGrid (latched) | slam_toolbox |
| `state/localization` | ultron_b2_interfaces/LocalizationState (latched, 0.5 s heartbeat) | `localization_monitor` |
| `map/info` | ultron_b2_interfaces/MapInfo (latched) | `map_manager` |
| `map/save`, `map/load` | ultron_b2_interfaces/SaveMap, LoadMap | `map_manager` |

Inputs: `sensors/lidar/points_filtered` (PointCloud2 in `lidar_link`, used by MOLA and KISS) and
`sensors/scan` (LaserScan for slam_toolbox and the monitor). Agent B's preprocessing provides
both. Static TF `base_link → lidar_link` comes from `robot_state_publisher`.

## `odom_source` (`launch/odometry.launch.py`)

| value | `odom → base_link` + `state/odom` | notes |
|---|---|---|
| `sim` | the simulation driver | this package starts nothing |
| `mola` (default) | `mola_lidar_odometry` (MOLA-LO, GPLv3, a separate process, apt `ros-humble-mola-lidar-odometry`) → `mola_odom_planarizer` | MOLA's own `/tf` and pose are remapped to private `mola/tf_raw` and `mola/odom_raw`. The planarizer republishes x, y, yaw only, because MOLA's planar option does not constrain the published ICP pose (\|z\| ≤ 0.11 m and roll/pitch ≤ 2.6° were measured on the bag). |
| `kiss` | `kiss_icp_odometry` (pip kiss-icp 1.2.3, MIT; a Python ROS wrapper in this package) | latest-only worker thread; it de-skews with per-point `timestamp`/`time` when present |
| `b2` | `b2_odom_adapter`: `/sportmodestate` (unitree_go/SportModeState) → Odometry | fallback; **mock-tested only** and never run against a robot |

## SLAM and localization (`launch/slam.launch.py`)

- `mode:=mapping` runs slam_toolbox online-async (config ported from `config/autonomy`). The
  monitor scores scans against the *live* map, so this is a consistency check only.
- `mode:=localization|nav|mission map:=<map_id>[:<version>]|<dir>|<map.yaml>` checks the map's
  sha256 hashes and then starts slam_toolbox localization on its pose graph. The monitor scores
  against the announced map. Without `map:=`, no backend starts and the state stays INITIALIZING.
- `localization_monitor` is a port of `ultron.autonomy.localization_state`, and a parity test
  covers it. States are VALID, DEGRADED, LOST, STALE and INITIALIZING. A scan gap over 0.5 s or a
  stale `map → base_link` gives STALE. A pose jump (> 0.5 m or 0.35 rad between scans, as in a
  kidnap) counts as a failed match: the state drops to DEGRADED and reaches LOST after 5 bad scans
  or 3 s. Thresholds are the simulation-validated set and are PROPOSED for the robot.
- `map_manager` writes `<map_root>/<map_id>/v<N>/` (`map.pgm`/`.yaml`, slam_toolbox
  `map.posegraph`/`.data`, and with `save_3d` and MOLA also `map3d.mm`/`.simplemap`). It also
  writes `map_meta.json` with sha256 hashes, provenance and version. Versions are never
  overwritten. `map.yaml`'s `free_thresh` is rewritten from 0.25 to 0.196, so unknown cells do not load as
  free (the ported SAHIL fix). The map root is `map_root:=`,
  else `$ULTRON_MAP_ROOT`, else `$ROS_HOME/ultron_maps`. Maps are run artifacts: never commit them.

## Caveats

- The IMU is not fused (MOLA and KISS are lidar-only). MOLA's `/initialpose` subscription is
  absolute (not namespaced) and unused.
- The scan must arrive reliably and completely. `pointcloud_to_laserscan` (best effort) delivered
  only 0.3–79 % of the bag's 1.5 MB clouds and broke slam_toolbox. The eval tool
  `tools/cloud_to_scan.py` (reliable QoS) delivered 364/364; at integration it was replaced by the
  installed `ultron_b2_preprocess` `cloud_to_scan` node (RELIABLE), the only `sensors/scan` publisher.
- slam_toolbox warns that `min_laser_range` 0.3 is below the scan's `range_min` (1.0 m on the
  real profile). The warning is harmless.
- No result here is hardware or real-perception validation. See VALIDATION.md (Agent C).

## Dev tools (`tools/`, not installed)

- `eval_bag.sh <mola|kiss> <out> [mapping|localization] [map_ref]` replays the real bag in
  container domain 72. It is driven by `replay_eval.py` and needs the bag mounted read-only at
  `/bag` through `ULTRON_DEV_ARGS`.
- `eval_sim.sh <sim|mola|kiss> <out> [seed]` evaluates against simulation ground truth in domain
  71. It uses `eval_sim_bridge.py`, `eval_sim.launch.py` and `sim_ate.py`, which reuse
  `src/ultron/autonomy/{sim_bridge,eval_slam}.py`.

Tests: `python3 -m pytest -q src/ultron_b2_localization/test`.
