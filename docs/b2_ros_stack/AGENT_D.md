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

## Command chain (single path, order fixed, applies to every source)

```
Nav2 controller → nav/cmd_vel_raw → velocity_smoother → control/in/autonomy ─┐
teleop (joy/keyboard) ─────────────────────────────────→ control/in/teleop  ─┤
ops/hold (lock) ─────────────────────────────────────────────────────────────┤
                                                                             ▼
 twist_mux (HOLD > teleop > autonomy) → control/cmd_vel_muxed
 → collision_monitor (common) → control/cmd_vel_safe
 → motion_gate (common; sole publisher of cmd_vel) → cmd_vel
 → sim_driver (sim)  |  b2_sport_bridge (live, armed only)
```

Invariant: no command source reaches `sim_driver` or `b2_sport_bridge`
without passing the collision monitor and then the motion gate. Selection
happens **before** safety, never after. Only `motion_gate` publishes
`cmd_vel`; the bridge and sim driver subscribe to nothing else. No teleop,
recovery, test or debug shortcut may publish `cmd_vel` directly. The B2
handheld RC and physical E-stop remain above this chain.

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
   - Port `motion_gate_node.py` as the **common final gate** for all
     sources: input `control/cmd_vel_safe`, output `cmd_vel`. Always
     enforced: HOLD, operator enable heartbeat ≤ 1.0 s, command age ≤ 0.3 s,
     non-finite reject, PROPOSED velocity limits; else zero + cancel request.
     When `control/active_source` is `autonomy`, also require VALID+fresh
     localization for the loaded map. For `teleop`, apply the localization
     check when `teleop_requires_localization` is `true` (default,
     fail-closed; relaxing it is the safety owner's decision). Unknown or
     stale active source ⇒ treat as autonomy (strictest). Reuse existing
     module logic; publish `safety/motion_gate`.
   - `nav_contract_server`: `NavigateToView` action wrapping
     `NavigateToPose`; port pre-checks from `nav2_adapter.py` (map id/version,
     frame, occupied/unknown/outside-map, finite pose), timeout, cancel,
     late-result rejection, settle → `settled=true` (CAPTURE_READY).
2. **`ultron_b2_control`**
   - `twist_mux.yaml`: inputs `control/in/teleop` (priority 100),
     `control/in/autonomy` (10); lock topic `ops/hold` (priority 255);
     output `control/cmd_vel_muxed` (feeds the collision monitor, never the
     bridge).
   - `active_source_monitor`: publishes `control/active_source` from the
     same priority table and input freshness. If it cannot be kept
     consistent with twist_mux, replace both with a minimal custom mux that
     publishes the selected command and its source together.
   - Teleop launch (joy + keyboard) publishing to `control/in/teleop` only.
   - `sim_driver`: adapter from gated `cmd_vel` to the existing sim bridge input.
   - **`b2_sport_bridge`** (C++ or Python): subscribes only gated `cmd_vel`; publishes
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
- Collision monitor stops before a spawned obstacle inside the stop polygon,
  for **both** autonomy and teleop input.
- Teleop with the gate closed (no operator heartbeat, or HOLD) ⇒ zero
  `cmd_vel`; teleop with stale command ⇒ zero.
- Graph check (`ros2 topic info -v cmd_vel` in the launch test): exactly one
  publisher, `motion_gate`; bridge and sim driver subscribe to `cmd_vel`
  only; nothing else publishes `cmd_vel` or `/api/sport/request` except the
  bridge.
- HOLD (lock topic) zeroes `cmd_vel` within one control period.
- Bridge vs mock: disarmed ⇒ 0 requests; armed + stream ⇒ Move at rate;
  stream stops ⇒ exactly one StopMove ≤ watchdog + 0.05 s; clamp + non-finite
  tests; wrong domain ⇒ refuse.
- Repeat a subset with `odom_source:=mola` after C delivers.

## Do not

Launch the bridge against anything except the mock. Enable Nav2 recovery
motions for real by default. Change verdict/inspection policy.
