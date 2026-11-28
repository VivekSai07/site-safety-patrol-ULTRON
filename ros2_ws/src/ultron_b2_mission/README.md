# ultron_b2_mission

**Owner: Agent E** (`docs/b2_ros_stack/AGENT_E.md`). Started by `ultron_b2_bringup` `mode:=mission`.
Relative topic names; numeric parameters are **PROPOSED** (`config/mission_sim.yaml`).

The mission **never publishes velocities**. It moves the robot only through Agent D's
`nav/navigate_to_view` (`ultron_b2_interfaces/action/NavigateToView`). HOLD = latch `ops/hold`
(`std_msgs/Bool`, twist_mux lock) **and** cancel the active nav/capture goal.

## Nodes
* `mission_manager` — `mission/run` (RunMission), services `mission/hold` (SetBool), `mission/resume`,
  `mission/abort` (Trigger), `mission/set_patrol_zone`; publishes `mission/status`, `mission/events`,
  `mission/coverage`, `mission/frontiers`, `ops/hold`. Consumes `state/localization`, `map`,
  `perception/ladder_candidates`, `ops/operator_enable`, `ops/hold`.
* `capture_coordinator` — `mission/capture_views` (CaptureViews); stores frames as `.npy` + JSON manifest
  under `evidence_dir` (local only, never committed) and returns `sha256:<hex>:<path>` refs.

## State machine (`mission_core.py`, pure Python)
`PATROL → EXPLORE → DETECT → APPROACH → SETTLE → CAPTURE → ASSESS → COMMIT → next | RETURN_BASE`.
From every state: inhibit (mission hold, `ops/hold`, operator enable lost/stale, localization not VALID or
stale, map mismatch) ⇒ `HOLD` (goal cancelled, no new goals, late results ignored), automatic resume when
causes clear (mission hold needs `mission/resume`); prolonged localization loss or hold ⇒ `AWAIT_OPERATOR`
(needs resume/abort). Unreachable view poses ⇒ ladder `LADDER_UNREACHABLE` (not inspected).
ASSESS only hands off (`PENDING_ASSESSMENT`); the mission never produces a verdict, and
`AUTOMATED_PASS` remains pre-screening, never safe-to-use.

## Libraries
`grid.py` (occupancy, clearance, reachability, line of sight), `zone.py` (PatrolZone, world→map),
`frontier_explorer.py` (frontiers inside the zone; unreachable reported separately), `view_pose_planner.py`
(stand-off views with rejection reasons), `coverage_reporter.py` (explorable unknown vs unreachable gaps,
ladder gaps), `memory_bridge.py` (SQLite registry via `src/ultron/memory.py`, `events.jsonl` for the dashboard).

## Tests
`python3 -m pytest test` on the host (core + planning). In the dev container also
`test_mission_e2e_mock.py` (real nodes vs a mock NavigateToView server). `test_mission_sim_e2e.py` is the
full-sim test and stays **UNRUN** (skipped) until C/D/F land; run with `ULTRON_E2E_SIM=1`.
Simulation and oracle results are not hardware or perception evidence.
