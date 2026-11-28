# Agent B — Real-sensor ingestion, replay and preprocessing

Read [FLEET_COMMON.md](FLEET_COMMON.md) and [PLAN.md](PLAN.md) first.
Branch: `om/b2-ros-stack-b`. Depends on A's skeleton (start on a local stub
package if A is not ready; rebase later).

## Input data

Real B2 recording (local, **never commit or upload**): rosbag2 sqlite3,
36.6 s, metadata at `/home/azut25r/workspace/heck_aachen/metadata.yaml`
(the user will supply the `.db3` path; ask the operator for it via the
user if absent — mark tasks needing it UNRUN until then). Topics:
`/rslidar_packets` (rslidar_msg/RslidarPacket ~866 Hz), `/scan_cloud`
(PointCloud2 ~10 Hz), `/imu/data` (~100 Hz, Xsens) + `/imu/*`, `/filter/*`,
`/status`, `/camera_depth/head/{color,depth}/{image_raw|image_rect_raw,camera_info,metadata}`
(~30 Hz), `/camera_depth/head/extrinsics/depth_to_color`, `/tf` (~100 Hz),
`/tf_static` (1 msg). **No B2 odometry / sportmodestate / joint states.**

## Kickoff prompt

> You are Agent B for the ULTRON B2 ROS 2 stack. Follow AGENTS.md, CLAUDE.md,
> FLEET_COMMON.md, PLAN.md and AGENT_B.md. Characterize the real bag, build
> the replay launch and driver configs that map real topics into the ULTRON
> contract, and implement point-cloud preprocessing and 3D→2D scan
> conversion. Fill the calibration unknowns table from data, not guesses.
> Never commit or upload the bag. No hardware connection.

## Deliverables

1. **`ultron_b2_tools/bag_inspect`** (Python, rosbag2_py): per topic count,
   rate, first/last stamp, header-vs-receive time offsets, gaps > 3 periods,
   frame_ids; full `/tf_static` dump and dynamic `/tf` parent→child table with
   rates; PointCloud2 field list, point count stats, `is_dense`, ring/time
   availability; CameraInfo summary. Output Markdown + JSON to
   `runs/bag_inspect/<bag-name>/` (ignored). Commit only a **redacted summary**
   (topics/fields/frames, no data) to `docs/b2_ros_stack/REAL_BAG_FACTS.md`.
2. **Decide the lidar source.** If `/scan_cloud` lacks per-point `time`/`ring`,
   re-decode `/rslidar_packets` with pinned `rslidar_sdk` (Helios-32 config,
   `send_by_rows`/timestamp settings documented) and compare. Record which
   producer the live stack uses.
3. **Driver configs** in `ultron_b2_bringup/config/drivers/`:
   `rslidar_helios32.yaml`, `xsens_mti.yaml`, `realsense_d457_head.yaml`
   (values from the bag/metadata where observable, else `TO_VERIFY`).
4. **`replay.launch.py`** body: `ros2 bag play --clock` (rate arg, loop arg,
   start offset), `use_sim_time:=true` for all nodes, remaps vendor names →
   contract (`/scan_cloud`→`sensors/lidar/points`, `/imu/data`→
   `sensors/imu/data`, `/camera_depth/head/...`→`sensors/cam_head/...`),
   filter out bag `/tf` edges that our stack will own (use the existing
   `src/ultron/autonomy/tf_filter.py` idea), namespace `/ultron/replay`,
   domain 72, **no control/command nodes**. QoS: sensor-data subscribers must
   accept the bag's reliable/volatile offers (document overrides).
5. **`ultron_b2_preprocess`**: node `cloud_filter` — self-filter crop box
   (derive body-return box from the bag; sim box from existing
   `lidar.py` self-filter), range min/max, height band, voxel grid
   (PROPOSED 0.05 m for odometry input, 0.10–0.20 m for viz), optional deskew
   using per-point time + IMU (only if time field exists), output
   `sensors/lidar/points_filtered`. Implement in C++ (PCL) or vectorized
   numpy; measure CPU per frame. Config for `pointcloud_to_laserscan`
   (`target_frame: base_footprint`, height band from integrated ch.04
   −0.20…0.35 m PROPOSED, range 0.35–12 m PROPOSED) → `sensors/scan`.
6. **Calibration**: fill `ultron_b2_description/calibration/real.yaml` from
   `/tf_static` with `source: bag_tf_static` per field; anything not in the bag
   stays `TO_VERIFY`. Document IMU frame orientation and whether `/tf` from the
   bag contains an orientation-only edge.

## Acceptance (VALIDATION.md → Agent B)

- REAL_BAG_FACTS.md committed (no raw data).
- Replay: contract topics publish at bag rates (±10 %), `/clock` drives nodes,
  no node in `/ultron/replay` publishes any `cmd_vel` or `/api/*`.
- Filtered cloud: < 1 % of points inside the self box; per-frame filter time
  reported (dev PC; Orin UNRUN until available).
- `sensors/scan` visibly shows walls in a Foxglove/RViz screenshot (local).
- Unit tests for filter maths and remap table.

## Do not

Guess sensor extrinsics, upload/commit any data, or connect to the robot.
