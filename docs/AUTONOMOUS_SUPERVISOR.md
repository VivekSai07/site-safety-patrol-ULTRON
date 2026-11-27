# Autonomous inspection supervisor

The operational objective is: patrol an approved bounded site, discover ladders, retain world locations/history, inspect from reachable useful angles, preserve uncertainty, then return to base/inspector. The current implementation is sim-only. Real DimOS navigation/semantic mapping requires a working B2 connection and actual sensors; installing DimOS alone is not proof of complete site discovery.

## Layers and responsibilities

- **Navigation:** current simulation uses a bounded inflated A* grid and SDK-shaped velocity profiles. Real navigation should reuse the verified event/DimOS navigation runtime behind a B2 motion supervisor. Camera findings never bypass costmap/RC/watchdog.
- **World memory:** static scene entities/parts, location and map version are stored in SQLite from simulator truth. Persistent ladder IDs, inspection events, current flags and missing expected assets remain separate. `dimos-ingest` writes both object and inspection streams into the upstream memory API when available.
- **Perception:** oracle in the protected patrol demonstration; optional YOLOE/Qwen adapters are a separate experimental image path. Real mapping observes objects over time; it cannot silently mark unseen space searched or guarantee that every object is recognized.
- **Intelligence:** `supervisor.decide` selects a reachable uninspected known target, continues survey while search remains, or requests return-to-base/operator. `inspection_queue` re-evaluates target priority at the current robot position between inspections. Typed state and geometry drive actions; VLM prose does not.
- **Inspection:** type-scoped components, evidence, one bounded alternate view, hard-fail-first rules and human escalation. All model confidence values remain uncalibrated until evaluated. The model report retains the image set, rationale and hashes.
- **Completion:** handled known assets and searched area are separate. The supervisor uses RETURN_BASE_WITH_GAPS if coverage is unverified, targets are unreachable, or expected assets are missing. An unreachable dock requests AWAIT_OPERATOR. Existing MISSION_COMPLETE in a synthetic protocol refers to the fixture/waypoint benchmark, not verified real site coverage.

## State flow

SETUP/MAP -> SURVEY -> DETECT/CONFIRM -> BIND/RE-ID -> SELECT TARGET -> VALIDATE VIEW -> APPROACH -> CAPTURE -> ASSESS -> SECOND LOOK OR HUMAN REVIEW -> COMMIT -> RESUME -> RETURN BASE.

The current demo starts at a nominal base and returns there. It does not implement a charging dock handshake, fiducial docking, contact charging, battery telemetry or human tracking. The return goal is a map waypoint, not an actual docking behavior.

## Angle selection

`candidate_views` can propose a full ring only for an A-frame when the caller explicitly confirms open space and missing rear evidence; without both attestations it uses the simulator's nominal front candidates. Platform and wall/leaning ladders do not receive full-ring candidates. Angles use a fixed map axis, not measured ladder orientation. The grid filters candidate footprints and available paths in simulation only: neither an attestation nor a free grid proves physical clearance or visibility. The protected runtime keeps one optional side recovery; full-orbit execution remains an isolated Tier 3 proposal.

## Next real integration gate

Connect aligned RGB/depth/CameraInfo/TF and actual ladder observations to registry/policy; consume observed semantic objects from DimOS; use its verified localization/navigation services behind the B2 control boundary. Measure search coverage and detector recall against an independently enumerated course. Replay a real bag before live motion. A learned language planner can suggest tasks, while the deterministic supervisor validates them and retains explicit budgets/unknowns.
