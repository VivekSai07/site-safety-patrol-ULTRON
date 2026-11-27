# ULTRON jury strategy and delivery gates

The official weighting is 30% real-hardware operation, 25% reliability, 20% technical quality, 15% creativity and 10% pitch (Kickoff Slides p.10 / Recap p.2). A beautiful simulation earns no evidence for the real-hardware factor. Use it to de-risk interfaces and rehearse the story, then prioritize the course run.

| Factor | Demonstrable evidence to collect | What this repo contributes |
|---|---|---|
| Works, 30% | Robot finds/approaches a ladder, captures real images, generates actual record | Mission flow, camera/evidence contract, local report |
| Reliability, 25% | 5–10 repeated runs; unreadable label, blocked pose, network failure and missed ladder outcomes | Deterministic review/hard-fail logic, seeded failure scenarios |
| Technical quality, 20% | Reproducible code, raw sources, timestamps, calibration, correct metrics, approved dataset | Tests, SQLite history, evidence hashes, replay, provenance |
| Creativity, 15% | Robot notices an evidence gap, takes one useful second look, remembers changed/missing asset | Bounded recovery and cross-patrol memory prototype |
| Pitch, 10% | Five-minute explanation with an uncut ≤2-minute hardware video | Storyboard and honest simulation preview |

## Freeze order

1. Real sensor frame -> one detected ladder -> evidence crop -> official protocol field -> report. Record this early.
2. Integrate Sahil's verified mapping/navigation adapter for a bounded, operator-approved route. The organizer kit supplies standing/DDS simulation, not a navigation baseline. Keep model inference outside the control/watchdog loop.
3. Add persistent identity/history and explicit unresolved items. Measure duplicate IDs and missing expected assets.
4. Freeze the real baseline before second-look navigation, tilt, new mapping stacks or model fine-tuning. Team target: Saturday 13:30.
5. Capture the backup real-hardware video by Saturday 22:00. Final packaging on Sunday before noon.

## Metrics that mean something

- Unsafe-to-scoped-pass errors: hard-fail seeded ladders emitted AUTOMATED_PASS / total seeded hard-fail ladders; report count and sample size, not just percentage. Target zero, then measure useful completion too so abstaining on everything is visible.
- Ladder recall: unique physical ladders observed / independently enumerated test-course ladders; simulation oracle recall is a separate result.
- Re-ID: duplicate/incorrectly merged assets over repeated passes; moved/missing cases separate.
- Evidence completion and recovery: resolved required gaps / attempted second looks; record blocked and unsuccessful retries.
- Autonomy: interventions, confirmations, blocked goals and time. Mode labels must change when a human drives.
- Coverage: reached route points are not an area coverage estimate. A mapped visibility/occupancy coverage measure is future work.

The generated protocol records seeded oracle hard-fail cases and false scoped passes, waypoint completion, recovery attempts, interventions and missing expected IDs. No simulated result is presented as measured real perception or hardware autonomy.

## Keep scope small

Use one good detector and literal label reader before a second VLM. Grounding masks/depth and stable IDs matter more than many models. Spatial memory should answer operational questions: which ladder changed, where is it, what was observed last time, which expected assets are missing? Embedding search can come later; the current persistent registry already answers these deterministically.
