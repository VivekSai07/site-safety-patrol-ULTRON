# Real B2 bag facts — `rosbag2_2026_11_28-00_48_42`

Facts and summary numbers only. **No raw data, images or point clouds are in this
repository**; the bag stays local (outside any repo) and must not be uploaded.
Derived with `ros2_ws/tools/bag_inspect.py` (rosbags 0.11.11, no ROS) and a local,
uncommitted KISS-ICP check. Simulation-independent, but this is one 36.6 s
recording: values are observations, not calibrations. Anything marked
**TO_VERIFY** is not established by the bag.

## Provenance and integrity

| Item | Value |
|---|---|
| Source | `rosbag2_2026_11_28-00_48_42.zip`, 1,987,480,517 B (same size as the browser's completed download; no partial `.crdownload`) |
| Zip structure | valid: 3 entries, deflate with data descriptors, zip64 extra for the 4,324,941,824 B `.db3`; EOCD present at end. The earlier "End-of-central-directory signature not found" was most likely `unzip` reading the file while it was still being copied into the workspace (final mtime 07:05); `unzip -l` now succeeds |
| Extraction | `recover_bag_zip.py` central-directory mode, CRC verified; `PRAGMA integrity_check` = `ok` |
| Salvage | **100 % of every topic** (71,375 / 71,375 messages, matches `metadata.yaml`); no recovery needed |
| Storage | sqlite3, rosbag2 schema 3, `ros_distro` humble, single file, no message definitions stored |
| Recorder | `rosbag2_recorder` (only `/rosout` publisher, 30 info lines); duration 36.556 s |

## Topics

Rates from header stamps (receive-time rates agree). QoS is the recorded offer:
all **reliable**; images, `/tf_static`, extrinsics and `/rosout` are
**transient_local**, the rest volatile.

| Topic | Type | Count | Hz | frame_id | Notes |
|---|---|---:|---:|---|---|
| `/scan_cloud` | PointCloud2 | 364 | 10.00 | `lidar_link` | no header gaps; header = first-point time; received 0.102 s after header (end of sweep) |
| `/rslidar_packets` | rslidar_msg/RslidarPacket | 31,667 | 872 | `lidar_link` | **incomplete**: 58 % of expected MSOP packets (see lidar) ; 22 DIFOP packets have zero stamps |
| `/imu/data` | Imu | 3,645 | 99.81 | `imu_link` | header stamps lead receive time by 31 ms median (−54…+1 ms); delivery batched ~60 ms |
| `/imu/{acceleration,angular_velocity,mag}`, `/filter/{quaternion,euler}` | Vector3Stamped / MagneticField / QuaternionStamped | 3,640–3,645 | 99.8 | `imu_link` | Xsens driver outputs |
| `/imu/time_ref` | TimeReference | 3,645 | 99.8 | `` (empty) | |
| `/status` | xsens_mti_ros2_driver/XsStatusWord | 3,645 | 99.8 | – | not decoded (definition not registered) |
| `/tf` | TFMessage | 3,645 | 99.8 | – | one edge, see TF |
| `/tf_static` | TFMessage | 1 | – | – | RealSense-internal frames only |
| `/camera_depth/head/color/{image_raw,camera_info,metadata}` | Image / CameraInfo / Metadata | 1,090 | 29.99 | `head_color_optical_frame` | |
| `/camera_depth/head/depth/{image_rect_raw,camera_info,metadata}` | Image / CameraInfo / Metadata | 1,085 | 29.82 | `head_depth_optical_frame` | one 0.167 s gap at t≈10.1 s |
| `/camera_depth/head/extrinsics/depth_to_color` | realsense2_camera_msgs/Extrinsics | 1 | – | – | |
| `/temperature`, `/pressure`, `/filter/free_acceleration`, `/imu/utctime`, `/parameter_events`, `/events/write_split` | – | 0 | – | – | subscribed, never published |

Not present: **B2 odometry, `/sportmodestate`, joint states, `base_link`**.

## TF tree

Two disconnected trees; roots `world` and `head_link`. No frame has two parents.

```
world ──(/tf, 99.8 Hz)──► imu_link
head_link ─► head_color_frame ─► head_color_optical_frame      (/tf_static)
head_link ─► head_depth_frame ─► head_depth_optical_frame      (/tf_static)
lidar_link   (only as a header frame_id; no TF edge at all)
```

| Edge | Source | Value |
|---|---|---|
| `world → imu_link` | `/tf`, Xsens filter | **orientation only**: translation constantly (0,0,0); roll median 0.38°, pitch median 1.57°; yaw −49.0° → −8.0°. Not odometry |
| `head_link → head_depth_frame` | `/tf_static` | identity |
| `head_link → head_color_frame` | `/tf_static` | xyz (−0.0003, −0.0591, 0.0000) m, rpy (0.04, −0.15, 0.15)° |
| `head_*_frame → head_*_optical_frame` | `/tf_static` | xyz 0, quaternion (−0.5, 0.5, −0.5, 0.5) (standard optical rotation, rpy −90°, 0, −90°) |

**Missing extrinsics (TO_VERIFY, measure):** `base_link`↔`lidar_link`,
`base_link`↔`imu_link`, `base_link`/`lidar_link`↔`head_link`. The bag cannot
supply them.

## Lidar

| Item | Value |
|---|---|
| `/scan_cloud` fields | `x,y,z,intensity` FLOAT32 @0/4/8/12, `ring` UINT16 @16, `timestamp` FLOAT64 @18; point_step 26, little-endian (rslidar_sdk XYZIRT layout) |
| Layout | unorganized: height = N, width = 1 (a column, not a ring × azimuth grid); `is_dense` true, no NaN or zero-range points |
| Points / scan | median 38,920 (37,918–39,594) of 57,600 possible (32 × 1,800) |
| Rings | 32 values, 0…31 |
| Per-point time | absolute seconds (FLOAT64); min = header stamp (±0.2 ms); span 0.100 s per scan → **deskew is possible** |
| Range | min 0.22 m, p5 0.28 m, median 2.81 m, p95 11.06 m, max 40.7 m (sampled every 3rd scan) |
| Intensity | 1…255, median 10 |
| Near returns (< 1 m) | ≈4,570 points / scan in box x −0.14…0.66, y −0.62…0.81, z −0.74…−0.02 m (p1–p99, lidar frame); mix of robot body and nearby floor — self-filter box TO_VERIFY |
| Floor | ≈0.75 m below the lidar origin (5th-percentile z at 1–3 m horizontal range; median −0.747 m), lidar roughly z-up |
| Packets | 1,248 B MSOP, id `55 AA 05 5A`, 12 blocks × 32 channels with `FF EE` flags, azimuth step 0.20° (150 packets / rev at 10 Hz); DIFOP id `A5 FF 00 5A 11 11 55 55` |
| Model | **RoboSense RS-Helios-32 class** (MSOP layout + 32 rings + 0.2° at 10 Hz); 5515 vs 1615 variant TO_VERIFY from the DIFOP/label |
| Packet completeness | only **58 %** of the MSOP packets a full 36.3 s at 10 Hz would need were recorded; 2,254 azimuth jumps > 2 packets; 217 frame-begin flags for 364 scans. Re-decoding packets cannot reproduce full scans from this bag; **`/scan_cloud` is the lidar source of record** |
| Producer | node name not recorded; layout and `lidar_link` frame match rslidar_sdk publishing both packets and the cloud. Exact driver config TO_VERIFY |

## IMU (Xsens MTi)

- Frame `imu_link`, 99.8 Hz, z up: mean acceleration (−0.35, 0.05, 9.78) m/s², |a| 9.90 m/s².
- Gyro std (0.115, 0.108, 0.504) rad/s, |max| (0.64, 0.58, 1.26) rad/s.
- All covariance matrices are **zero** (unknown): consumers must set their own.
- Header stamps are ~31 ms ahead of the recorder's receive clock (driver/device
  time base); for LIO, align IMU and lidar time (TO_VERIFY on robot).

## Head camera (RealSense, D455/D457 family)

| Stream | Resolution | Encoding | Intrinsics (fx, fy, cx, cy) | Distortion |
|---|---|---|---|---|
| Color | 1280 × 720 | `rgb8` | 643.637, 642.901, 643.497, 361.957 | `plumb_bob` (−0.05437, 0.06552, −0.00077, 0.00008, −0.02152) |
| Depth (`image_rect_raw`) | 848 × 480 | `16UC1` | 431.055, 431.055, 425.932, 241.043 | `plumb_bob`, zeros |

- CameraInfo is constant for the whole bag (one version per stream).
- Depth units: depth scale is **not recorded**; values are consistent with the
  RealSense default 1 mm/unit (median 3.98 m, p95 10.1 m, min 0.48 m; ≈94 % valid
  pixels). Use 0.001 m TO_VERIFY against the device.
- `depth_to_color`: translation (−0.05913, 0.00008, 0.00046) m (|t| 59.1 mm),
  rotation 0.22°; matches `head_link → head_color_frame` in `/tf_static`. The
  59 mm baseline, 87–90° fields of view and 848×480 depth identify the D455/D457
  family (D457 per team hardware list).
- Metadata JSON keys: `actual_fps` (29,988 = 29.988 Hz), `clock_domain`
  (`global_time`), `frame_number`, `frame_timestamp`, `hw_timestamp`,
  `backend_timestamp`, `time_of_arrival`, `raw_frame_size`.

## Motion

The robot **moved** — evidence from sensors only (no odometry in the bag):

- IMU orientation yaw range 365° (≈ one full turn), gyro-z integral range 369°,
  net +50°; total |Δyaw| 811°.
- Lidar odometry (KISS-ICP, deskewed with per-point time, 0.3 m voxel, 1–30 m;
  local check, **unvalidated estimate**): path ≈ **19.4 m**, net displacement
  0.27 m (returns near start), extent 3.1 × 5.1 m, height change 0.23 m, speed
  median 0.51 m/s (p95 0.98, max 1.37), moving in 96 % of scan intervals; yaw net
  +41°, range 366°, consistent with the IMU.

## Consequences for the stack (proposals)

- Replay remaps: `/scan_cloud` → `sensors/lidar/points` as-is (fields already
  XYZIRT; `timestamp` must be mapped to the `time` name/relative form if a
  consumer expects it). Subscribers must accept reliable offers; for latched
  `/tf_static` and extrinsics use transient_local.
- Filter or drop bag `/tf` (`world → imu_link`) before our stack owns
  `odom → base_link`; `world` is the Xsens orientation frame, not a map.
- `base_link`, lidar, IMU and head-camera extrinsics remain **TO_VERIFY** in
  `calibration/real.yaml`; only the RealSense-internal transforms are
  `source: bag_tf_static`.
- Odometry must come from lidar (MOLA/KISS) or a future B2 state topic; the bag
  has none.
