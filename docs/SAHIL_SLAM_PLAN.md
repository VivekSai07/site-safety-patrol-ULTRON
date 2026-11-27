# Sahil: 2D lidar SLAM in simulation — phased plan

**Status:** plan, not evidence. Each phase is complete only when its acceptance checks have actually run and their results are recorded in `docs/SAHIL_VALIDATION.md`. Numeric thresholds below are simulation choices made before evaluation; they are not hardware settings.

**Scope:** sensor-driven 2D SLAM (slam_toolbox) from a simulated RoboSense Helios-32 on the ULTRON MuJoCo B2, with ground truth kept only for evaluation. Nav2, semantic memory and coverage follow in a separate plan once Phase 4 passes. No hardware connection, no real robot topics and no command path to a robot are created here.

## Fixed decisions

| Topic | Decision | Reason |
|---|---|---|
| SLAM backend | `slam_toolbox` (ROS 2 Humble binary already installed), online async for mapping, localization mode for reload | Mature 2D pose-graph SLAM, map serialization and localization mode; feeds Nav2 directly. |
| Sensor | Simulated Helios-32 profile; the real robot carries a RoboSense lidar at the front (team photos). | Organizer-supplied model. Exact variant and specs must be confirmed from the organizer documentation (see Phase 1). |
| Simulator | Existing ULTRON MuJoCo scene and kinematic base motion; lidar via `mujoco.mj_multiRay`. | Reuses the regression baseline; organizer kit has no lidar or walking. Kinematic motion is disclosed, not called walking. |
| Python/ROS boundary | `.venv-ros` on `/usr/bin/python3` 3.10 with `--system-site-packages` (rclpy) + pip `mujoco==3.3.7`, `numpy`, `Pillow`. | Keeps ROS's interpreter untouched and avoids the conda `mjlab` env (Python 3.12, incompatible with Humble rclpy). |
| ROS isolation | `ROS_DOMAIN_ID=71`, `ROS_LOCALHOST_ONLY=1`, `use_sim_time:=true`, all sim topics under `/ultron/sim/`. | Matches existing RViz replay isolation; nothing can reach a robot on the network. |
| Frames | `map -> odom` (slam_toolbox), `odom -> base_link` (simulated odometry), `base_link -> lidar_link` (static, from config). REP-103/105: metres, radians, x forward, z up. | Standard ROS semantics, so Nav2 and real-robot adapters can replace parts later. |
| Ground truth | Published only on `/ultron/sim/ground_truth/pose` and logged to file; **never** on TF and never read by SLAM nodes. | Handoff rule: ground truth for evaluation only. A test enforces this. |

## Phase 0 — Environment, branch and configuration scaffold

**Goal:** reproducible environment and non-actuating configuration before any sensor code.

- Set the commit identity, merge updated `main` into `sahil/sim-slam-nav2`, keep tests green.
- `scripts/setup-ros-venv.sh`: create `.venv-ros`, pin `mujoco==3.3.7`, print versions.
- `config/autonomy/lidar_helios32.yaml`: sensor profile with every value marked `source: datasheet|assumed|measured`; mounting pose marked `assumed` until measured on the robot.
- `config/autonomy/sim_slam.yaml`: domain ID, topic names, frame names, odometry noise model, rates.
- Package skeleton `src/ultron/autonomy/` (only modules actually used in Phase 1–4).

**Acceptance:** existing 52 tests still pass; rendered tests pass in `.venv-ros` (`ULTRON_RENDER_TESTS=1`); `python -c "import rclpy, mujoco"` succeeds in `.venv-ros`; configuration loader rejects missing/unknown keys.

## Phase 1 — Lidar sensor model and realistic test hall (no ROS)

**Goal:** a tested, ROS-free Helios-32 simulator producing timestamped point clouds from the MuJoCo scene.

Sensor model (`autonomy/lidar.py`):
- Beam pattern from the profile: 32 channels with the supplied vertical angles, 360° horizontal at the chosen resolution, scan rate, min/max range.
- `mj_multiRay` from the lidar origin; returns range, hit geom ID and per-point time offset across the sweep.
- Self-occlusion: robot body, compute box, sensor mast and floodlight are modeled as primitives; rays hitting them are returned as hits and then removed by a configurable self-filter, as on the real robot. The resulting blind sector is reported per scan.
- Noise: Gaussian range noise, dropout probability and max-range misses; seeded RNG for reproducibility.
- Optional (later): motion distortion by evaluating the pose at each point's timestamp.

Test hall (`autonomy/hall_scene.py`, does not alter the protected demo scene):
- Approx. 10 × 10 m bounded area per the briefing, walls with doors/recesses, pillars/ducts, tables, a crate/cart, a 1 × 2 m mat.
- Ladder models from the photos at real-world scale: Layher double-sided A-frame stepladders with strap spreaders, combination ladders with top rollers (leaning and A-frame), trestles. 6–8 ladders. Primitive shapes; dimensions marked `assumed`.
- Static human cylinders (dynamic ones in Phase 5).

**Acceptance:**
- Analytic tests: a wall at known distance is measured within noise σ×3; a box behind the robot is invisible inside the blind sector; channel count/angles match the profile.
- Point count per scan matches profile minus filtered self-hits; no NaN/inf in published points.
- Performance: one full scan in ≤ 50 ms on this laptop (10 Hz real-time budget), measured and recorded.
- Saved sample scans (`.npz`) render in a quick top-down PNG for visual review.

## Phase 2 — ROS 2 simulation bridge

**Goal:** the simulator publishes realistic sensor streams on ROS with simulated time; a driver moves the robot kinematically.

Node `ultron_sim_bridge` (run in `.venv-ros`):
- `/clock` from simulation time.
- `/ultron/sim/lidar/points` (`sensor_msgs/PointCloud2`, fields x,y,z,intensity,ring,time) at the profile rate, frame `lidar_link`.
- `/ultron/sim/scan` (`sensor_msgs/LaserScan`): 2D projection of points within a configured height band in `base_link` (closest return per angle bin); blind sector as `inf`. Implemented in the bridge because `pointcloud_to_laserscan` is not installed.
- `/ultron/sim/odom` + TF `odom -> base_link`: integrated from true motion with configurable scale error, yaw-rate bias and random walk, labeled `SIMULATED_ODOMETRY`.
- Static TF `base_link -> lidar_link` from config.
- `/ultron/sim/ground_truth/pose` (evaluation only).
- Kinematic driver accepting `geometry_msgs/Twist` only on `/ultron/sim/cmd_vel`, with clamped speed/acceleration, footprint collision check against the scene and a command timeout that stops the base. It publishes nothing on `/cmd_vel`.
- Scripted exploration routes (YAML) for repeatable runs; rosbag recording script.

**Acceptance:**
- Measured topic rates within ±10 % of configuration; TF tree exactly `odom -> base_link -> lidar_link` (no `map` until SLAM runs).
- Header stamps are sim time and monotonic; scan and point cloud of one sweep share a stamp.
- Command timeout stops the base within one control period; collision check rejects motion into an obstacle.
- Test asserts no ground-truth frame on `/tf` and no topic outside `/ultron/sim/`, `/clock`, `/tf`, `/tf_static`.
- RViz config shows cloud, scan, odom and robot.

## Phase 3 — Online mapping with slam_toolbox

**Goal:** build a map from the scan + odometry only, save it, and measure it against ground truth.

- `config/autonomy/slam_toolbox_mapping.yaml` (online async) tuned for the profile: resolution 0.05 m, range limits, scan topic `/ultron/sim/scan`, frames as above.
- `scripts/sim-slam.sh map ROUTE`: launches bridge + slam_toolbox + scripted route + bag record; at the end saves `map.pgm/yaml` and serialized pose graph under `runs/slam/<run>/` with a `map_meta.json` (map_id, version, config hashes, sensor profile, bag reference, sim/odometry provenance).
- Evaluation tool `autonomy/eval_slam.py`:
  - Absolute trajectory error (translation RMSE, max; yaw RMSE) of `map -> base_link` vs ground truth.
  - Map quality: rasterize true scene geometry in the scan height band; occupied-cell precision/recall and free/occupied IoU after alignment.
  - Compare with the odometry-only trajectory to show what SLAM corrects.
- Same run replayed from the recorded bag gives a map within tolerance (replay determinism).

**Proposed acceptance (set before evaluation, simulation-only):** translation ATE RMSE ≤ 0.10 m, max ≤ 0.25 m, yaw RMSE ≤ 2°, occupied-cell recall ≥ 0.85 within 0.10 m on walls; ladder feet appear as occupied clusters (reported, not required). Odometry-only error must be clearly larger, otherwise the test is not exercising SLAM.

## Phase 4 — Map reload, localization state and failure gates

**Goal:** load a saved map, localize, and publish an honest `LocalizationState`.

- `scripts/sim-slam.sh localize MAP`: slam_toolbox localization mode on the serialized map, robot starting at a configured base pose (with an initial-pose offset to test convergence).
- `autonomy/localization_state.py`: typed `LocalizationState` per the team contract — `stamp`, `clock_id`, `frame_id`, `map_id`, `map_version`, position (m), quaternion (x,y,z,w), `state` (`INITIALIZING`, `VALID`, `DEGRADED`, `LOST`, `STALE`), covariance, age and age limit, source (`SLAM_ESTIMATE`; ground truth never). Published on `/ultron/sim/localization_state` and as JSON for non-ROS consumers.
- Validity rules: pose becomes `VALID` only after scan-match quality and covariance meet thresholds for N consecutive updates, not because data arrived; `STALE` when the newest transform is older than the age limit; map-version mismatch with a requested map is `REJECTED`.
- `transform_at(stamp)` helper: returns the transform valid at an observation timestamp or rejects it (stale/extrapolation), for later use by perception/memory.

**Acceptance:**
- Reload: localization error ≤ Phase 3 tolerances after convergence from a 0.5 m / 15° initial offset.
- Loss: blank or blocked scans for > age limit → `STALE`/`LOST` within one age limit; recovery after scans return, with the state change logged.
- Kidnap: teleport the robot 3 m; state must leave `VALID` (not silently report the old pose) and recover or stay `LOST`.
- Map ID/version mismatch and stale timestamp tests reject.

## Phase 5 — Robustness and realism (as time permits)

- Moving humans (scripted cylinders) during mapping and localization.
- Long low-feature wall segment (corridor degeneracy) and higher odometry drift levels.
- Lidar motion distortion on, scan rate variants, point dropout bursts.
- Resource budget: CPU/RAM of bridge + slam_toolbox, scan-to-pose latency.

**Acceptance:** each case recorded as PASS/FAIL with the Phase 3/4 metrics; failures documented, not tuned away silently.

## Phase 6 — Evidence and hand-off for SLAM

- `docs/SAHIL_STACK_DECISION.md` (stack, pinned versions, I/O table, environment boundaries, resources, blockers).
- SLAM sections of `docs/SAHIL_RUNBOOK.md` and `docs/SAHIL_VALIDATION.md` with revision, config hashes, results and artifact paths.
- Short screen recording: cloud, scan, growing map, estimated vs true path.
- Unresolved real-robot inputs table (owner + fallback): lidar variant/topic/rate, mounting pose, IMU availability, odometry source and rate, clock sync, TF from the robot, network.

## Schedule against the team plan

Team baseline freeze is Saturday 28 Nov ~13:30. Target: Phases 0–3 before the freeze, Phase 4 immediately after, so Nav2 integration can start on a valid `LocalizationState`. Phase 5 is optional before hand-in (Sunday 29 Nov noon). If a phase slips, ship the last passing phase with honest status rather than skipping acceptance.

## Out of scope here

Nav2 goals/costmaps, semantic ladder memory, coverage accounting (next plan); 3D SLAM; real robot topics, recordings or hardware connection; ladder detection from lidar (camera detector owns discovery).
