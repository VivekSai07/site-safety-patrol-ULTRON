# ultron_b2_sim — MuJoCo sim bridge on the topic contract (Agent F)

SIMULATION ONLY. Kinematic base motion, primitive-scene renders and oracle labels; not validated
walking, real perception or inspection results.

`sim_contract_bridge` subclasses `ultron.autonomy.sim_bridge.SimBridge` (imported from the repo's
`src/`; set `ULTRON_SRC` if the package cannot locate it) and keeps its domain-71/localhost guard.

| Topic (relative to `/ultron/sim`) | Type | Rate (PROPOSED) | Notes |
|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | 50 Hz | simulation clock |
| `sensors/lidar/points` | PointCloud2 (x,y,z,intensity,ring,time) | 10 Hz | frame `lidar_link` |
| `sensors/scan` | LaserScan | 10 Hz | `scan_topic:=sim` moves it to `sim/scan` when preprocess owns `sensors/scan` |
| `sensors/imu/data` | Imu | 50 Hz | kinematic synthetic (gyro z, planar accel + g); no orientation |
| `state/odom` | Odometry | 50 Hz | only with `odom_source:=sim` (+ `odom->base_link` TF); otherwise `sim/leg_odom`, no TF |
| `sensors/cam_head/color/{image_raw,camera_info}` | Image rgb8 640×360 | 10 Hz | BAG_FACTS intrinsics ×0.5, fx = fy, centred principal point |
| `sensors/cam_head/depth/{image_rect_raw,camera_info}` | Image 16UC1 mm 424×240 | 10 Hz | 0 = no return; 0.3–10 m |
| `sim/oracle/cam_head/ladder_labels` | Image 16UC1 | 10 Hz | ORACLE per-pixel ladder index (for E's `oracle_sim` only) |
| `sim/ground_truth/base_pose` | PoseStamped | 50 Hz | frame `ultron_sim_world`, EVALUATION ONLY, never on TF |
| `sim/ground_truth/ladders` | String JSON (latched) | on change + 1 Hz | `ultron.sim_ladder_truth/v1`; includes `odom_origin_world` |
| `sim/bridge_status` | String JSON (latched) | 1 Hz | includes camera render stats |
| `sim/cmd_vel` (input) | Twist | — | from the control `sim_driver`, which forwards motion_gate's gated `cmd_vel`; nothing else feeds the sim |
| `sim/scenario` (service) | ultron_b2_interfaces/srv/SimScenario | — | obstacle, lidar_blackout, lidar_dropout, kidnap, ladder_move/hide/show |
| `sim/sim_control` (input) | String JSON | — | the base bridge's original test hook |

TF: `base_link -> sensor` belongs to robot_state_publisher (the bridge's own static
`base_link -> lidar_link` is disabled); `odom -> base_link` only when `odom_source:=sim`.

```bash
ros2 launch ultron_b2_sim sim_bridge.launch.py namespace:=/ultron/sim      # standalone
ros2 launch ultron_b2_bringup sim.launch.py mode:=mapping                   # via bringup drivers sub-launch
ros2 service call /ultron/sim/sim/scenario ultron_b2_interfaces/srv/SimScenario \
  "{schema_version: ultron.sim_scenario/v1, command: ladder_move, target: ladder_03, values: [6.0, 4.0, 0.0]}"
```

Bringup passes `sim_rate`, `sim_people`, `sim_route`, `sim_scenario`, `sim_hidden_ladders`,
`sim_scan_topic`, `sim_camera`, `sim_seed`, `sim_out` (evaluation logs incl. ground_truth.csv) through to this launch when given on the command line.

Camera rendering uses MuJoCo EGL (Mesa llvmpipe in the dev container, `LP_NUM_THREADS=4`) in a
worker thread; frames are skipped (counted in `sim/bridge_status`) if a render is still running.
Requires the `mujoco` Python package (pip; present in `ultron-b2-dev:humble`).

Isaac: `launch/isaac.launch.py` is an UNRUN stub behind the same contract (`isaac/README.md`).
