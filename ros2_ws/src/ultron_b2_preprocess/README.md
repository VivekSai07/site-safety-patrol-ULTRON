# ultron_b2_preprocess

**Owner: Agent B.** Lidar preprocessing for the topic contract (PLAN.md §3.2), vectorized numpy.

| Node | In | Out |
|---|---|---|
| `cloud_filter` | `sensors/lidar/points` (PointCloud2; rslidar_sdk XYZIRT with f64 absolute `timestamp`, or sim `time` f32) and, only with `deskew:=true`, `sensors/imu/data` | `sensors/lidar/points_filtered`: `x,y,z,intensity` f32, `ring` u16, `time` f32 (s after header stamp), frame unchanged (`lidar_link`), reliable QoS |
| `pointcloud_to_laserscan` (upstream, config only) | `sensors/lidar/points_filtered` | `sensors/scan` (LaserScan in `base_footprint`, best-effort `SensorDataQoS`) |

Pipeline (all in the cloud frame): drop non-finite → range `[range_min, range_max]` → height
`[z_min, z_max]` → remove self box → optional rotation-only deskew (mean IMU angular velocity over
the sweep, constant-rate model; translation is not compensated; output stamp = reference time and
`time` = 0) → voxel grid keeping the first point per voxel (real attributes, scan order).

Configs: `config/cloud_filter_{real,sim}.yaml`, `config/pointcloud_to_laserscan_{real,sim}.yaml`,
selected by `calibration_profile` in `ultron_b2_bringup/launch/sub/preprocess.launch.py`. Every
threshold is PROPOSED; the real self box is derived from the bag (see the config comments and
`docs/b2_ros_stack/VALIDATION.md`, Agent B) and the lidar-to-body extrinsic is TO_VERIFY.

Tests: `colcon test --packages-select ultron_b2_preprocess` (filter maths + node on synthetic clouds).
