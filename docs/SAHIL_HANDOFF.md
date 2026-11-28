# Sahil: SLAM, mapping and navigation

> **Update 28 November 2026:** spatial memory moved to Kaan and lives in `src/ultron/pipeline/` (see [PIPELINE.md](PIPELINE.md)). Section 4 below is kept as written; what it needs from this branch is the pose and a stable map id.

**Owner:** Sahil and his coding agent. **Branch:** `sahil`. **Base:** published `main` at `1643f27800b615a10883b818a1997c48e4cad08a` (27 November 2026).

Build ULTRON's autonomous spatial layer: establish a map, localize the B2, discover reachable inspection targets from observations, navigate to approved views, remember assets across patrols, resume search and return to the recorded base. DimOS is welcome; you own the stack choice. Deliver runnable software, integration contracts and measured evidence, with unimplemented parts clearly identified.

## Start here

```bash
git clone https://github.com/VivekSai07/site-safety-patrol-ULTRON.git
cd site-safety-patrol-ULTRON
git switch --track origin/sahil
# Existing clean clone instead: git fetch origin && git switch sahil
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

Read, in order: [AGENTS.md](../AGENTS.md), [CLAUDE.md](../CLAUDE.md), this handoff, [organizer alignment](ORGANIZER_ALIGNMENT.md), [integration](INTEGRATION.md), [supervisor](AUTONOMOUS_SUPERVISOR.md), [simulation/hardware gates](SIM_TO_HARDWARE.md), [Pass 7 reconciliation](RECONCILIATION_PASS7.md). Consult [design](DESIGN.md) and [v0.4 runtime limits](RUNTIME_V04.md) before claiming existing functionality.

Agent kickoff prompt:

> You are implementing Sahil's autonomy workstream on branch `sahil`. Follow AGENTS.md, CLAUDE.md and docs/SAHIL_HANDOFF.md. First inspect the existing interfaces and available organizer inputs, then implement milestones in order. Choose DimOS or another justified stack, pin its revision and keep dependencies isolated. Preserve the existing simulation and inspection policy. Produce runnable mapping/localization/navigation/memory integration with repeat-patrol and failure evidence. Never substitute world truth for sensor-based SLAM or claim physics walking from kinematic animation. This assignment does not authorize a physical robot connection. Commit focused changes to this branch and prepare a PR against main with reproducible commands, actual results and remaining gaps. Do not wait for unrelated perception/model work to start: use explicitly labeled replay/synthetic adapter fixtures first.

## What exists, and what does not

| Existing code | Reuse / boundary |
|---|---|
| `src/ultron/navigation.py` | Optional `DimosNavigator` goal/poll/cancel adapter. Tests use injected interfaces; real DimOS runtime/B2 integration is unverified. |
| `src/ultron/planning.py` | Inflated static 2D grid A*. Simulation planner, not SLAM, a live costmap or dynamic-obstacle avoidance. |
| `src/ultron/supervisor.py` | Deterministic target selection, survey/return decisions and explicit gaps. Preserve its separation from inspection verdicts. |
| `src/ultron/memory.py` | Retired as spatial memory (identity is `src/ultron/pipeline/`); kept for the MuJoCo mission. SQLite assets/missions/inspection history. `associate`, `append`, `query` are useful seams; identity association is positional/type-based and limited. `remember_objects` consumes simulator world truth. |
| `src/ultron/dimos_bridge.py` | Optional offline ingestion of synthetic object/history records into DimOS. It is not live semantic mapping. |
| `src/ultron/observations.py`, `runtime_perception.py` | Experimental camera/depth projection and model tracks. No measured real calibration or robust appearance re-ID. |
| `src/ultron/viewpoints.py` | Experimental fixed-map-axis candidates; no measured ladder orientation. Full rings only for explicitly open A-frames with missing rear evidence. |
| `src/ultron/mission.py`, `mujoco_sim.py` | Working rendered workflow, kinematic base motion and oracle detections by default. Preserve as a regression baseline. |

The SMAROBIX kit is the authoritative organizer simulator/interface starting point. At the reviewed revision it provides physics, DDS joint state/commands and a standing B2. **It has no walking controller, simulated lidar/camera, SLAM or autonomous navigation baseline.** Do not spend the critical path training a quadruped policy or assume SDK `Move` is implemented by that simulator. Use ULTRON's explicitly kinematic execution for mission tests, a reproducible sensor/replay environment for SLAM tests, and keep their results separate. Combining them into a sensor-driven navigation simulation is a deliverable only once actually connected and tested.

Organizer source snapshots and pinned Unitree revisions are in ORGANIZER_ALIGNMENT.md. On Vivek's machine the organizer clones are siblings of `/home/vivek/ULTRON`; other machines must clone them. Further robot documents/data may require access to the organizer's gated Hugging Face collection. The local display-performance fixes made after this base are not included in this branch: begin headless rather than assuming the old viewer is lightweight.

The supplied challenge briefing names B2 / Jetson Orin / RoboSense Helios-32 / RealSense D457 / ROS 2 Humble. Verify actual topics, message types, rates, clocks, IMU availability, odometry, calibration, TF and network settings. Do not infer an integrated lidar IMU or invent device topics. Put unresolved inputs in a table with an owner and fallback. Hardware access requires the separate gate below.

## Workstream ownership

You own sensor/replay adapters for localization, map lifecycle, transforms, costmaps/routes, safe navigation state, observed semantic memory, coverage accounting and return-to-base navigation. Own the bridge from ULTRON's supervisor to your chosen stack. Add visualization of map/pose/path/coverage/asset IDs using Rerun or RViz where practical.

Other workstreams own ladder detection models, OCR/VLM damage assessment, final inspection decisions, nesseler checklist export and pitch/UI packaging. Consume their observations and return navigation/capture-ready states. You may add fixtures to unblock tests, but do not take over model training or change safety verdict policy. Record integration needs in your PR; do not silently redefine shared fields.

## Milestones and deliverables, in priority order

### 1. Stack decision and interface discovery

Deliver `docs/SAHIL_STACK_DECISION.md`: chosen stack and pinned commits/versions; why it is practical on the available compute; input/output table; Python/ROS environment boundaries; estimated startup/resource requirements; known blockers. Choose DimOS if it supplies the needed functionality and can be exercised here; another ROS/non-ROS stack is acceptable behind the same contracts. Do not install DimOS into ROS's system interpreter or assume its Go2 blueprint supports B2. Verify upstream APIs before implementation.

Deliver a configuration template and adapter interface inventory. Unknown hardware topics/calibration stay explicitly unset; no guessed physical configuration should activate. First integration checkpoint is a replay/synthetic observation entering the adapter and a cancellable navigation goal leaving it, with no hardware command path enabled.

### 2. Sensor-driven mapping and localization

Deliver launch/replay commands that build or load a versioned map from timestamped sensor data and publish robot pose with validity/age/uncertainty. Retain raw-source provenance and distinguish prerecorded odometry, estimated SLAM pose and ground truth used only for evaluation. Persist occupancy geometry and the configuration/calibration needed to reproduce it. Demonstrate restart/load and localization loss/recovery.

Define `map -> odom -> base_link -> sensor` semantics (or document the equivalent), metres/radians, transform direction, optical-camera convention and clocks. Each observation must use a transform valid at its acquisition timestamp, not merely the latest pose. Reject stale/unavailable transforms. A pose must not become valid merely because data arrived. If genuine sensor-driven SLAM cannot run with available inputs, deliver the adapter/replay scaffold and mark this milestone BLOCKED/UNRUN with exact missing inputs; do not rename the scripted ULTRON map as SLAM.

### 3. Navigation integrated with the mission supervisor

Deliver an isolated backend with goal submission, status, cancellation, timeouts and stop behavior. Load a bounded operating zone, footprint/inflation and occupancy map; route to valid standoff poses, pause for capture, then resume survey. Handle occupied/unreachable goals without looping forever. Replan or stop for new obstacles according to tested backend capabilities. Reject motion when localization, map, transform, operator permission or command state is invalid/stale. Missing gates default to disabled.

Use the real existing supervisor action names: `INSPECT`, `SURVEY`, `RETURN_BASE`, `RETURN_BASE_WITH_GAPS`, `AWAIT_OPERATOR`. Extend through explicit adapters rather than passing them straight to a robot driver. Proposed goal statuses are `ACCEPTED`, `ACTIVE`, `REACHED`, `REJECTED`, `CANCELLED`, `TIMEOUT`, `FAILED`; map backend statuses explicitly. The current `DimosNavigator` is only a starting seam, not evidence of watchdog or transport readiness.

Record the base pose in a known map at mission start or require an explicit configured base. Return there when known inspections/search are finished or when gaps require return. If base is unreachable, stop/request operator and preserve the reason. This is navigation to a waypoint, not charging/contact docking. Ladder capture requests must wait for a tested arrival/settle criterion; a successful navigation status alone is not an inspection result.

### 4. Observed semantic/spatial memory across patrols

*Now Kaan's (`src/ultron/pipeline/`). From SLAM it needs camera poses in the map frame and the same map id on every patrol of a site.*

Deliver a live/replay observation ingestion path with persistent map-scoped ladder/object identities, positions/uncertainty, first/last-seen times and provenance. Query nearby assets and pending inspections. Preserve separate observations, persistent assets, missions and append-only inspection history. Two views of one stationary ladder should associate; ambiguous neighboring ladders must remain ambiguous rather than merge. Handle moved/missing assets and map resets explicitly. Do not erase unresolved prior flags or overwrite previous evidence.

A minimal positional association is acceptable if its limits are evaluated and documented. Do not promise identity through arbitrary movement without supporting evidence. Avoid schema breakage: explicit migration or a new DB is required. DimOS may back the implementation, or ULTRON's registry may remain the canonical inspection record with a synchronized spatial index; document the source of truth and replay/deduplication semantics.

### 5. Bounded exploration and honest completion

Deliver search progress that distinguishes traversed paths, observed sensor space, obstacles, reachable unexplored space, unreachable space and unknown coverage. Configure the search region and stopping conditions. Specify whether coverage is navigational or visibility-based; neither proves ladder recall by itself. Do not use waypoint completion, object count or a preloaded fixture list as proof the whole site was inspected.

Finish each mission with known/inspected/pending/unreachable/missing counts, remaining coverage gaps and a reason for return/stop. “All ladders found” needs an independently enumerated evaluation course; do not use a fixture list as production perception. If complete autonomous exploration cannot be delivered in time, ship a configured bounded survey with accurate gap reporting, clearly marked as the fallback.

### 6. Integration evidence and hand-back

Deliver `docs/SAHIL_RUNBOOK.md` (fresh-machine setup, one-command replay/demo, resource settings, expected outputs, stop/recovery, known limitations) and `docs/SAHIL_VALIDATION.md` (revision, configuration/data provenance, each acceptance case and actual result/artifact reference). Add a short demo recording showing map, robot pose/path, persistent ladder IDs, inspection pause/resume and return or explicit stop. Keep large recordings/maps/bags local or in an approved artifact location; commit small fixtures/metadata only. No dataset upload is authorized by this branch assignment.

## Integration contracts to implement and agree with perception

The following are **proposed logical records**, not existing ROS topics or validated hardware APIs. Implement typed validation and publish the exact chosen schema/transport in the stack decision. Optional fields must not conceal missing required information.

| Record | Required meaning |
|---|---|
| Pose/localization | `stamp`, `clock_id`, `frame_id`, `map_id`, `map_version`, position in metres, quaternion with stated ordering, validity/state, uncertainty/covariance and age limit. |
| Object observation | `observation_id`, sensor timestamp/frame, map ID/version, world position/uncertainty, type proposal, detection confidence semantics, source/model, evidence reference and transform provenance. Use serial identity only when actually read. |
| Navigation goal | `goal_id`, mission/map IDs, target pose/frame, tolerances and timeout from configuration, reason/asset ID, operator-enable and freshness prerequisites. Unknown frame/map is rejected. |
| Navigation result | Same goal ID, explicit status/reason, timestamp, last valid pose, cancellation/stop acknowledgement; late responses cannot complete another goal. |
| Capture readiness | Goal/asset/view IDs, arrival state, timestamped pose/transform and tested settle condition; perception acknowledges capture/failure before inspection resumes. |
| Memory/mission event | Stable event/asset/mission/map IDs, source timestamp, append-only payload/evidence references, deduplication key and synthetic/replay/live provenance. |
| Coverage result | Region/map version, definition and denominator, observed/unobserved/unreachable quantities, limitations and explicit completion/return reason. |

Proposed package layout: `src/ultron/autonomy/` for contracts, input adapters, localization/map backend, navigation backend, semantic memory bridge and coverage; a separate ROS package if needed; `config/autonomy/` for non-actuating templates; `scripts/` for launch/replay; `tests/` for contracts and failure scenarios. These paths are suggested ownership boundaries, not a requirement to create empty modules. Coordinate small changes to `mission.py`, `supervisor.py`, `navigation.py`, `memory.py` and CLI; do not replace the repository wholesale.

## Acceptance checklist

Define numeric tolerances, age limits, footprint and resource budgets before evaluation and record how they were selected. Simulation values are not physical settings. For each case record PASS / FAIL / UNRUN, data revision, backend and any stubs; no planned command counts as a pass.

- [ ] Clean environment starts from pinned dependencies using the runbook; no undisclosed downloads or hardware connection.
- [ ] Sensor/replay input produces an estimated map/pose; save/reload preserves declared map semantics. Ground truth is separate from estimator input.
- [ ] Bad timestamps, absent/stale transforms, localization loss and map-version mismatch inhibit goals and cancel active navigation as appropriate. Restart/relocalization cannot reuse invalid identities/poses silently.
- [ ] At least two patrols: an observed stationary ladder retains its ID/history, a new ladder creates a candidate, ambiguity remains explicit, and moved/missing handling is demonstrated.
- [ ] Target selection -> valid view -> arrival/settle -> capture acknowledgement -> resume runs end-to-end with declared perception/inspection fixtures if models are unavailable.
- [ ] Occupied/unreachable inspection goal, new obstacle, timeout, cancellation and operator inhibition lead to bounded replan/stop/review, without false arrival or stale goal completion.
- [ ] Return to the recorded base succeeds in a reachable case. An unreachable base produces an operator-wait/stop outcome with preserved evidence.
- [ ] Unknown/unreachable coverage and pending targets remain visible. Completing configured waypoints alone does not report complete area coverage or perfect ladder recall.
- [ ] Existing hard-fail precedence, uncertainty handling, human sign-off and history regression tests remain green; hardware outputs stay disabled by default.
- [ ] Record map/localization metrics where reference truth exists, goal success/timeout counts, ID merges/splits, gap counts, CPU/RAM and sensor-processing latency. If no reference truth exists, state the metric is unavailable rather than report invented accuracy.

Baseline checks (with dependencies available):

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
PYTHONPATH=src MUJOCO_GL=egl ULTRON_RENDER_TESTS=1 python3 -m unittest discover -s tests -v
```

These test ULTRON's existing workflow; passing them does not validate your SLAM stack. Add relevant backend/replay checks and retain separate results. Run headless first; do not stress a laptop with simultaneous uncapped viewers or assume xclock proves GPU rendering.

## Hardware and team coordination

This task assignment authorizes development and branch publication, **not physical B2 connection or operation**. Follow SIM_TO_HARDWARE.md: full integrated deployment needs scoped simulation acceptance and separate supervised readiness. Targeted component tests have their own bounded authorization/validation path. Any motion needs verified RC priority, independent watchdog/stop, E-stop, measured environment and named operator. Do not copy simulation DDS values, Go2 WebRTC drivers or the kit's standing script onto the real B2. VLM text and optional System One have no motion/verdict authority.

If inputs are unavailable, keep independent implementation moving, document the dependency/owner and report partial status honestly. Do not contact organizers, upload recordings or change shared hardware settings from the agent. Keep raw recordings, reference PDFs, credentials, weights and generated runs out of Git.

## Commit and PR delivery

Commit incrementally on `sahil`; keep `main` unchanged. Suggested checkpoints: contracts/configuration; mapping/replay; supervisor/navigation; persistent memory/coverage; integrated evidence/runbook. Add tests around real failure semantics, not mock-only success claims. Rebase/merge updated main only after preserving local work; never force-push teammates' changes.

Final PR against `main` must name delivered milestones, exact launch commands, stack revisions, schema/DB changes, actual acceptance results, demo/artifact paths, dependencies on perception and remaining simulation/hardware gaps. End with a concise hand-back: what works, what is stubbed, what is blocked, and the next command Vivek can run. Leave unfinished milestones explicitly unchecked rather than calling the full autonomous stack complete.
