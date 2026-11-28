# Isaac Sim backend (M7) — plan and stub, UNRUN

Status: **UNRUN stub**. MuJoCo (`launch/sim_bridge.launch.py`) is the validated simulator for the
stack. `launch/isaac.launch.py` declares the same contract and refuses to start; `smoke:=true` only
runs `scripts/isaac/run-smoke.sh` (RTX lidar on `/ultron/sim/isaac/points`, not a contract topic).

Contract the Isaac backend must publish under `/ultron/sim` (same as the MuJoCo wrapper):
`/clock`, `sensors/lidar/points` (x,y,z,intensity,ring,time; frame `lidar_link`), `sensors/scan`,
`sensors/imu/data`, `state/odom` + `odom->base_link` only when `odom_source:=sim`,
`sensors/cam_head/{color/image_raw,color/camera_info,depth/image_rect_raw,depth/camera_info}`,
`sim/ground_truth/base_pose`, `sim/ground_truth/ladders` (schema `ultron.sim_ladder_truth/v1`),
service `sim/scenario` (`ultron_b2_interfaces/srv/SimScenario`). Input: gated `cmd_vel` only.
Never publish `base_link->sensor` TF (robot_state_publisher owns it) or ground-truth frames on TF.

Work items (from `docs/ISAAC_SIM_FEASIBILITY.md`, ~3–4 days kinematic parity): Helios-32 emitter
pattern, USD hall from `hall_scene.py`, mesh-free B2 import driven by `driver.py`, Isaac-side bridge
(clock, odometry drift, TF, 2D scan, ground truth, scenario hooks), RGB-D camera at the
`cam_head_link` pose from `ultron_b2_description/calibration/sim.yaml`. Isaac uses its internal
Humble rclpy (Python 3.12) and Fast DDS; keep domain 71 + localhost.
