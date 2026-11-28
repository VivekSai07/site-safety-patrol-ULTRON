# Sahil: Nav2 navigation in simulation — phased plan

**Status:** plan, not evidence. Results go to `docs/SAHIL_VALIDATION.md`. Thresholds are simulation choices fixed before evaluation. Builds on the SLAM plan (map, slam_toolbox localization, `LocalizationState`). No hardware connection or real command path is created; all velocity output stays on `/ultron/sim/cmd_vel`.

## Architecture

```
slam_toolbox (localization) --map->odom, /map--> Nav2 (planner, controller, costmaps, BT)
                                                   | /ultron/sim/nav/cmd_vel_raw
LocalizationState --+                              v
operator_enable ----+--> motion gate --> /ultron/sim/cmd_vel --> kinematic driver (bridge)
map id/version -----+        | inhibit -> zero command + cancel active goal
supervisor decision --> Nav2 adapter (NavigationGoal/Result, settle, capture readiness)
```

| Decision | Choice | Reason |
|---|---|---|
| Stack | Nav2 1.1.20 (Humble binary) | Installed, standard, cancellable `NavigateToPose` action; DimOS remains optional |
| Localization | slam_toolbox localization on the Phase 3 map (not AMCL) | Already validated in Phase 4 with an honest validity monitor |
| Planner / controller | NavFn global planner; Regulated Pure Pursuit with rotation shim | Simple, well understood; the kinematic B2 turns in place |
| Footprint | circle 0.55 m (same as the simulator's collision check) + inflation | One footprint everywhere; real B2 footprint still to be measured |
| Obstacles | static layer from the map; obstacle layer from `/ultron/sim/scan` | New obstacles appear in the local costmap |
| Safety gate | separate motion gate between Nav2 and the driver | Nav2 has no notion of LocalizationState or operator permission; missing inputs default to inhibit |

## Phase N0 — Contracts, gate and adapter (no simulation needed)
- `NavigationGoal` / `NavigationResult` records (team contract), statuses `ACCEPTED, ACTIVE, REACHED, REJECTED, CANCELLED, TIMEOUT, FAILED`, explicit Nav2 status mapping.
- Pre-goal checks: map ID/version match, frame `map`, `accept_pose` VALID and fresh, operator enabled, target inside map and not occupied/unknown (saved map), finite pose and timeout.
- Motion gate logic: pass commands only while LocalizationState VALID and fresh, operator enable true and fresh, map matches, command fresh; otherwise publish zero and request cancellation. Missing input = inhibit.
- Late results cannot complete another goal; timeouts cancel; capture readiness only after REACHED plus a settle condition (speed ~0 for a configured time and localization VALID).
- Supervisor mapping: `INSPECT`, `RETURN_BASE`, `RETURN_BASE_WITH_GAPS` -> goals; `SURVEY` -> configured survey waypoints; `AWAIT_OPERATOR` -> no motion.
- **Acceptance:** unit tests for every status mapping, every gate input (missing/stale/invalid), late-result rejection, timeout, settle.

## Phase N1 — Nav2 bring-up on the saved map
- `config/autonomy/nav2_params.yaml` derived from the package defaults (changes commented); launch through `scripts/sim-slam.sh nav`.
- **Acceptance:** lifecycle nodes active; goal to an open pose reached: final position error ≤ 0.25 m, yaw ≤ 0.35 rad (true pose); no collision stop; no velocity reaches `/ultron/sim/cmd_vel` with operator enable absent.

## Phase N2 — Failure behaviour
| Case | Acceptance |
|---|---|
| Occupied / unknown / outside-map goal | REJECTED before Nav2, with reason; no motion |
| Unreachable goal (enclosed) | FAILED or TIMEOUT within its timeout; no endless retry; robot stops |
| New obstacle on the path (spawned in sim) | replan around it or stop; no collision stop counted |
| Cancellation | CANCELLED; base stops within command timeout |
| Operator inhibit mid-goal | gate zeroes output within one control period; goal CANCELLED |
| Localization loss (lidar blackout) mid-goal | gate inhibits within the STALE detection time; goal CANCELLED; no motion while not VALID |
| Late result | a result for an old goal ID never completes the current goal |

## Phase N3 — Mission integration
- Supervisor-driven run: select target -> view pose -> REACHED -> settle -> CAPTURE_READY -> (fixture) capture acknowledgement -> resume -> return to recorded base.
- Base unreachable -> AWAIT_OPERATOR with preserved reason.
- **Acceptance:** end-to-end run with fixture perception; every transition logged; return-to-base error ≤ 0.25 m; arrival never reported as an inspection result.

Out of scope: real B2 transport, walking dynamics, docking/charging, semantic memory and coverage (next).
