# REAL_BAG_FACTS — pointer

The redacted facts for the real bag `rosbag2_2026_11_28-00_48_42` (topics, fields, frames, rates,
QoS, TF, calibration unknowns; no raw data) are in [BAG_FACTS.md](BAG_FACTS.md). This file exists
because AGENT_B.md names it; keep one source of truth there.

Replay mapping actually used (`ultron_b2_bringup/replay_topics.py`, namespace `/ultron/replay`):

| Bag topic | Replayed as | Note |
|---|---|---|
| `/scan_cloud` | `sensors/lidar/points` | `lidar_link`; x,y,z,intensity f32, ring u16, `timestamp` f64 absolute |
| `/imu/data` | `sensors/imu/data` | `imu_link`; zero covariances; header leads receive by ~31 ms |
| `/camera_depth/head/{color,depth}/{image_raw\|image_rect_raw,camera_info,metadata}`, `/camera_depth/head/extrinsics/depth_to_color` | `sensors/cam_head/...` (same suffix) | frame_ids stay `head_{color,depth}_optical_frame` |
| `/tf` (Xsens `world→imu_link`, orientation only) | `bag/tf_xsens` | kept for inspection, **off the TF tree** |
| `/tf_static` (RealSense `head_link→…optical_frame`) | `/tf_static` unchanged | joined to the robot by `cam_head_link→head_link` (identity, TO_VERIFY) |
| `/clock` | from `ros2 bag play --clock 100` | all stack nodes `use_sim_time:=true` |
| packets, Xsens extras, `/status`, `/rosout`, … | not played | `/status` type is not installed |
