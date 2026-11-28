# Agent F — Sim parity, visualization/ops, integration tests, runbook and validation

Read [FLEET_COMMON.md](FLEET_COMMON.md) and [PLAN.md](PLAN.md) first.
Branch: `om/b2-ros-stack-f`. Depends on A; integrates B–E. Start with the sim
wrapper and visualization immediately (they unblock everyone's debugging).

## Kickoff prompt

> You are Agent F for the ULTRON B2 ROS 2 stack. Follow AGENTS.md, CLAUDE.md,
> FLEET_COMMON.md, PLAN.md and AGENT_F.md. Wrap the existing MuJoCo sensor
> bench behind the ROS topic contract (with an RGB-D head camera), build
> Foxglove and laptop-RViz visualization that never sends raw lidar over
> Wi-Fi, recording profiles, end-to-end launch tests, the runbook and the
> full-system sim validation record. No hardware connection.

## Deliverables

1. **`ultron_b2_sim`**: launch wrapping `ultron.autonomy.sim_bridge` (via
   `.venv-ros` interpreter or the dev container) under `/ultron/sim`, with
   contract topic names (`sensors/lidar/points`, `sensors/scan`,
   `state/odom` when `odom_source:=sim`, `cmd_vel` input). Keep the existing
   domain-71/localhost guard and existing tests green (add remaps, do not
   rename the bridge's internals unless all tests are updated). Add a MuJoCo
   head RGB-D camera publishing `sensors/cam_head/{color,depth}` +
   `camera_info` at PROPOSED 10 Hz, pose from `calibration/sim.yaml`; oracle
   segmentation topic for E's `oracle_sim` backend. Scenario hooks
   (obstacle spawn, blackout, kidnap) exposed as services.
2. **`ultron_b2_viz`**
   - `viz_throttle` node: voxelized `viz/map_cloud` (≤ 1 Hz), downsampled
     `viz/points` (≤ 2 Hz, ≤ 30k pts), compressed color (≤ 5 Hz, JPEG),
     costmap/plan republish ≤ 2 Hz.
   - `foxglove.launch.py`: `foxglove_bridge` port 8765 (PROPOSED), explicit
     `topic_whitelist` (viz/*, map, tf, tf_static, state/*, nav plan/costmap,
     mission/*, safety/*, diagnostics). Foxglove layouts JSON for mapping,
     navigation and mission.
   - `rviz_bridge.launch.py`: `domain_bridge` YAML copying only the throttled
     topics from the stack domain to domain 42 (laptop RViz); RViz configs.
   - Fallback doc + config for `zenoh-bridge-ros2dds` with an allow-list
     (for multicast-hostile Wi-Fi).
   - Bandwidth measurement script (`ros2 topic bw` aggregator) per profile.
3. **Recording**: `record.launch.py` with mcap + zstd; profiles `minimal`
   (tf, tf_static, state/*, sensors/scan, mission/*, safety/*, diagnostics) and
   `debug` (+ points_filtered, compressed images). Output under `runs/` only.
4. **Integration tests** (`launch_testing`, opt-in env var like existing
   `ULTRON_SLAM_TESTS`): M1 sim mapping, M3 nav failure suite through full
   chain, M4 mission; each writes a JSON summary used by the validation record.
5. **Docs**: `docs/b2_ros_stack/RUNBOOK.md` (fresh machine; dev container;
   sim one-command mission; replay of the real bag; Orin container build/run
   in replay mode; Foxglove/RViz connection; stop/recovery; known limits) and
   `docs/b2_ros_stack/VALIDATION.md` (sections per agent + M6 full-system sim
   acceptance record in the format required by `docs/SIM_TO_HARDWARE.md`:
   revision, config, scope, data provenance, results/artifacts, limitations,
   open risks/owners, reviewer decision/date — reviewer is a human).
6. **Isaac parity (M7, post-event)**: plan + stub launcher following
   `docs/ISAAC_SIM_FEASIBILITY.md`, same contract topics. Not critical path.

## Acceptance (VALIDATION.md → Agent F)

- `ros2 launch ultron_b2_bringup sim.launch.py mode:=mission viz:=foxglove`
  runs end to end; Foxglove shows map, robot, plan, ladders, mission state
  (local screenshot).
- Replay on dev PC with `viz:=both`: measured bandwidth to viz consumers
  (target PROPOSED < 5 Mbit/s for `minimal` view); raw lidar not bridged.
- Orin replay run: UNRUN until hardware access is authorized — record so.
- Another agent reproduces the RUNBOOK sim mission from a clean clone.
- M6 record assembled with every required case PASS/FAIL/UNRUN; acceptance
  decision left to the human reviewer.

## Do not

Bridge raw clouds/images by default, expose foxglove/VNC beyond loopback/robot
LAN without user approval, or sign the acceptance record yourself.
