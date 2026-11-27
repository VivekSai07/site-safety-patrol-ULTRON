# ULTRON simulation architecture

## Requested scope and choices
The user requested a lightweight MuJoCo/RViz POC for Unitree B2, Rerun visualization and fuller use of DimOS-style spatial memory. This revision supersedes the first text-only spike. It is a team-side simulation artifact, not the real-hardware deliverable. Author choices: primitive B2 model, deterministic oracle perception, grid A*, SQLite registry, portable telemetry, optional viewer adapters. The requested simulation scope authorizes this implementation directly.

A full Unitree DDS simulator would require a locomotion controller; its low-level bridge does not provide the high-level walking/navigation service. Installing full DimOS would bring substantial dependencies and does not establish B2 support. The implementation therefore isolates these future adapters and runs a small B2 mission workflow without DDS or an LLM.

## Executable flow
1. Build a MuJoCo scene from the preserved official B2 MJCF. Remove meshes, retain joint/inertial/collision definitions, add ladders/walls/crates and an explicitly assumed inspection camera.
2. Inflate obstacle rectangles/zone boundaries by a 0.65 m circular approximation and plan four-connected A* paths on a 0.2 m grid.
3. Set the B2 free-base pose along those paths; call `mj_forward` and render. This is kinematic placement, not `mj_step` locomotion. Default motion now uses forward body-velocity and turn profiles from `motion.py`; bounded rotation precedes translation. Joint cycling is cosmetic, not contact dynamics. SDK-shaped commands are recorded but never sent by the demo.
4. At survey waypoints, confirm three rendered observations. MuJoCo segmentation selects actual rail pixels; metric depth and intrinsics back-project them into MuJoCo camera coordinates (right/up/back), then transform into world/map coordinates. This is an explicitly privileged detector/type oracle.
5. Spatially associate compatible ladder types within 0.65 m in the same map. Multiple neighbors produce a separate ambiguous candidate. Fixture keys are used only by the simulation oracle and per-mission duplicate suppression; persistent LAD IDs come from the spatial registry. Missing frames, localization error, moved-asset identity and appearance re-ID remain unimplemented.
6. Pause patrol; request overview and close-up goals. Capture after a nominal 0.8 s simulated settle. The renderer preserves RGB/depth, camera matrix, model geometry, timestamp and mission state. Settle is a simulated timestamp increment, not measured stabilization.
7. Scripted component/label findings feed deterministic rules. FAIL or overdue label -> FLAGGED before completeness. Missing required parts -> MANUAL_REVIEW. Otherwise -> AUTOMATED_PASS within the demo visual scope; `safe_to_use=false` always. One optional side-pose recovery fills scripted A-frame gaps. A blocked goal retains unknowns.
8. Commit one inspection per asset/mission and resume the remaining survey waypoints. Across patrols reuse map-scoped IDs, append events, and expose verdict changes. Prior flags remain unresolved across future observations until human resolution; this spike has no flag-release action. The fresh observation status is retained separately. The second patrol changes LAD-001's foot geometry and fixture finding. Optional missing variant removes the A-frame and marks its prior ID MISSING_EXPECTED.
9. Generate local HTML/JSON, SHA-256 evidence manifest, portable JSONL, RGB/depth and GIF. Optional Rerun export and ROS/RViz replay consume those recordings; an optional DimOS adapter ingests inspection events into a real `SqliteStore`.

## Scope of checks
Type-specific demo fields mirror the teammate bible's proposed subsets. They are not the complete DGUV checklist and not nesseler's official protocol. Label dates come from fixtures, not OCR. A complete broad ladder view is not automatically proof that every component was seen. The prototype's component PASS/FAIL values are scripted even though camera images themselves are rendered. The default 2026-11-27 inspection date is a fixed scenario date; it does not use the current date.

## Mission and storage contracts
`telemetry.jsonl`: simulation time (seconds), pose [x m,y m,yaw radians], camera position/rotation, intrinsics, image/depth paths, robot primitives, initial mission scene primitives, state and mission ID. Camera matrices use MuJoCo right/up/back; Rerun applies diag(1,-1,-1) to produce right/down/forward optical coordinates. RGB is 640×360, depth is float32 metres.

`events.jsonl`: mission start, waypoint, confirmation, evidence, recovery, rejection, commit, resume and finish. Each event is synthetic. `protocol.json`: missions, asset records and ordered histories. SQLite: `missions`, `assets`, `inspections`; WAL and a unique(asset,mission) constraint. Failed renders preserve partial recordings rather than delete them. A failed mission can remain without a summary; do not count it as completed.

Recorded scene XML includes the changed scene for the last patrol; per-frame scene snapshots preserve earlier patrol geometry. `upstream.xml` remains unmodified. Depth arrays dominate disk usage. Run folders are ignored by Git. Persisted events include an originating run ID and absolute local evidence root. Each export copies evidence referenced from older runs into `history/{run_id}/` and rewrites export paths; keep original run folders available while sharing a database.

## Limits and future work
No sensor fusion SLAM, validated model inference, actual tactile checks, executed real B2 transport, external data upload, n.core API, formal PDF exporter, continuous live dashboard, volumetric area coverage, robust real-world re-ID, or physics-validated controller. Rerun supports replay and a `--follow` tail of live simulation telemetry/events. RViz remains a replay path; stdout/JSONL is the dependency-free live event interface. Rerun/DimOS imports are optional and fail with useful instructions if absent. SDK runtime validation is pending; graphical/ROS operation is restricted in this execution environment.

A real baseline must replace oracle observations with aligned detector/depth/TF outputs, preserve unknowns, verify the official required components and calibrate evidence quality. Connect approved high-level navigation to organizer B2 control, retaining RC priority/watchdog. Real autonomy is an independent integration gate.

Version 0.3 adds a deterministic supervisor, scene-object memory, synthetic detection-label export, SDK-shaped motion and separately invoked optional model adapters. See AUTONOMOUS_SUPERVISOR.md and PERCEPTION_MOTION.md for the boundaries.
