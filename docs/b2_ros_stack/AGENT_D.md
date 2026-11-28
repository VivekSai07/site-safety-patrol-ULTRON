# Agent D — Navigation (Nav2), safety chain and B2 control bridge

Read [FLEET_COMMON.md](FLEET_COMMON.md) and [PLAN.md](PLAN.md) first.
Branch: `om/b2-ros-stack-d`. Depends on A and C (sim). Start from the
existing validated sim Nav2 stack (`docs/SAHIL_NAV_PLAN.md`, Nav N0–N2 in
`docs/SAHIL_VALIDATION.md`).

## Kickoff prompt

> You are Agent D for the ULTRON B2 ROS 2 stack. Follow AGENTS.md, CLAUDE.md,
> FLEET_COMMON.md, PLAN.md and AGENT_D.md. Port Nav2 + motion gate + navigation
> contract into ament packages, add velocity smoothing, collision monitor,
> twist_mux and a B2 sport-mode command bridge that is disabled by default and
> tested only against a mock. Re-run the N1/N2 failure cases through the full
> chain in sim. No hardware connection, no live launch.

## Command chain (single path, order fixed)

```
Nav2 controller → nav/cmd_vel_raw → velocity_smoother → nav/cmd_vel_smoothed
 → collision_monitor → nav/cmd_vel_safe → motion_gate → nav/cmd_vel_gated
 → twist_mux (teleop priority > autonomy; lock topic = HOLD) → cmd_vel
 → sim_driver (sim)  |  b2_sport_bridge (live, armed only)
```

## Deliverables

1. **`ultron_b2_navigation`**
   - `config/nav2_{sim,real}.yaml` from `config/autonomy/nav2_params.yaml`
     (NavFn or Smac2D planner; RotationShim + RPP controller; behavior_server
     with spin/backup **disabled** for real by default; bt_navigator).
     Costmaps: static, obstacle layer (`sensors/scan`), optional
     STVL from `sensors/lidar/points_filtered` (param flag), inflation;
     keepout + speed filter masks generated from a PatrolZone polygon.
     Footprint from `ultron_b2_description/footprint.yaml`.
   - `velocity_smoother` and `collision_monitor` configs (polygons PROPOSED;
     stop/slowdown zones from the footprint; source: `sensors/scan`).
   - Port `motion_gate_node.py` (VALID+fresh localization for loaded map,
     operator enable heartbeat ≤ 1.0 s, command age ≤ 0.3 s, else zero +
     cancel request) — reuse existing module logic; publish
     `safety/motion_gate`.
   - `nav_contract_server`: `NavigateToView` action wrapping
     `NavigateToPose`; port pre-checks from `nav2_adapter.py` (map id/version,
     frame, occupied/unknown/outside-map, finite pose), timeout, cancel,
     late-result rejection, settle → `settled=true` (CAPTURE_READY).
2. **`ultron_b2_control`**
   - `twist_mux.yaml`: teleop (priority 100), autonomy (10), lock topic
     `ops/hold` (priority 255).
   - Teleop launch (joy + keyboard) publishing to the teleop input.
   - `sim_driver`: adapter from `cmd_vel` to the existing sim bridge input.
   - **`b2_sport_bridge`** (C++ or Python): subscribes `cmd_vel`; publishes
     `unitree_api/msg/Request` on `/api/sport/request` with Move (api_id 1008,
     JSON `{"x":vx,"y":vy,"z":vyaw}`) and StopMove; reads
     `/api/sport/response` and `/sportmodestate` for health. Rules:
     `hardware_enable` param false ⇒ publishes nothing; `Arm` service required
     (armed state latched off on any fault); watchdog: no fresh `cmd_vel`
     within PROPOSED 0.3 s ⇒ StopMove once and zero; clamp vx/vy/vyaw and
     accelerations to PROPOSED limits from config; reject non-finite; refuse
     if domain/namespace preflight fails; publish diagnostics. API ids must be
     taken from the pinned `unitree_ros2` source (verify B2 vs Go2 headers;
     note R-67 API drift), not from memory.
   - `mock_sport_server` for tests (records requests, simulates responses).
3. Launch includes for `nav` and `control` sub-launches (A's skeleton).

## Acceptance (VALIDATION.md → Agent D; thresholds as in SAHIL_NAV_PLAN unless stated)

- Sim, full chain, `odom_source:=sim`: N1 reach near/far; occupied, outside
  map, unreachable (timeout), new obstacle (replan), cancel, operator inhibit,
  localization loss — all PASS or reported FAIL.
- Collision monitor stops before a spawned obstacle inside the stop polygon.
- HOLD (lock topic) zeroes `cmd_vel` within one control period.
- Bridge vs mock: disarmed ⇒ 0 requests; armed + stream ⇒ Move at rate;
  stream stops ⇒ exactly one StopMove ≤ watchdog + 0.05 s; clamp + non-finite
  tests; wrong domain ⇒ refuse.
- Repeat a subset with `odom_source:=mola` after C delivers.

## Do not

Launch the bridge against anything except the mock. Enable Nav2 recovery
motions for real by default. Change verdict/inspection policy.
