# ultron_b2_navigation

**Owner: Agent D** (`docs/b2_ros_stack/AGENT_D.md`). Nav2, PatrolZone masks, collision monitor and the
**common final motion gate**. Results: `docs/b2_ros_stack/VALIDATION.md` → Agent D.

Command chain (relative names under the mode namespace, e.g. `/ultron/sim`):

```
NavigateToView (nav/navigate_to_view) -> nav_contract_server -> Nav2 NavigateToPose
Nav2 controller -> nav/cmd_vel_raw -> velocity_smoother -> control/in/autonomy ┐
teleop -> control/in/teleop ───────────────────────────────────────────────────┤ twist_mux (ops/hold lock)
control/cmd_vel_muxed -> collision_monitor -> control/cmd_vel_safe -> motion_gate -> cmd_vel
cmd_vel -> sim_driver (sim/cmd_vel) | b2_sport_bridge (/api/sport/request, mock only)
```

`motion_gate` is the **only** publisher of `cmd_vel`. It always enforces HOLD, an operator-enable
heartbeat ≤ 1.0 s, command age ≤ 0.3 s, finite values and PROPOSED limits (out-of-limit → zero,
not clamped). It requires VALID, fresh localization for the loaded map when the source is
`autonomy`, and for `teleop` when `teleop_requires_localization` is true (the default). An unknown
or stale `control/active_source` is treated as autonomy. It publishes `safety/motion_gate`.

| Node / file | Role |
|---|---|
| `nav_contract_server` | `NavigateToView` action on `nav/navigate_to_view` wrapping `navigate_to_pose`; pre-checks (schema, frame `map`, map id/version, finite pose, VALID localization, outside map / occupied / unknown / keepout), timeout, cancel, late-result rejection, settle → `settled` (CAPTURE_READY). Cancels the goal when the gate inhibits for any reason except `NO_COMMAND`/`COMMAND_STALE`, or when gate status is missing or stale. Requests Nav2 lifecycle STARTUP only once `/clock` runs and `map → base_link` exists (`nav2_startup: on_tf`). |
| `motion_gate` | Final gate (`gate_logic.py`). |
| `zone_mask_server` | `nav/set_patrol_zone` (SetPatrolZone) → `nav/keepout_mask` + `nav/speed_mask` (+ filter infos). Without a zone it publishes unrestricted masks (map bounds). |
| `zone_masks_tool` | Offline PatrolZone → mask PGM/YAML. |
| `config/nav2_{sim,real}.yaml` | NavFn + RotationShim/RPP, keepout + speed filters, optional STVL (`use_stvl`). In real, spin/backup are not loaded. |
| `config/collision_monitor.yaml` | Stop polygon plus footprint APPROACH circle (PROPOSED, sim footprint; real TO_VERIFY). |
| `sim_glue`, `sim_nav_cases` | **TEMPORARY** legacy-sim glue and the N1/N2 full-chain case runner (until Agents F/C deliver). |

Run the sim cases (dev image, non-zero domain, localhost; the map comes from `scripts/sim-slam.sh` / `run_mapping`):

```bash
ROS_LOCALHOST_ONLY=1 PYTHONPATH=$REPO/src:$PYTHONPATH \
  install/ultron_b2_navigation/lib/ultron_b2_navigation/sim_nav_cases ../runs/b2_stack/d/nav_X --map ../runs/b2_stack/d/map_seed7
```

Simulation results are not hardware evidence. Polygons, limits and the real footprint are PROPOSED or TO_VERIFY.
