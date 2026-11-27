# ULTRON event handoff

This is the team-side simulation repository requested by the owner. No organizer contact or permission step is part of this workflow. Organizer source documents and team proposals are distinguished in SOURCE_REVIEW.md.

## Four-person work split

1. Robot/operator owner: starter bring-up, measured sensors/TF, mapping/localization, bounded navigation, RC/watchdog, early real video and bag collection.
2. Perception owner: ladder detection, rail-depth projection, stationary evidence, literal label reading, unknown/conflict handling.
3. Registry/report owner: stable asset IDs, mission/event history, actual nesseler field mapping, usable report and exports.
4. Integration/evaluation owner: repeatable tests, Rerun replay, seeded runs, privacy-approved dataset, packaging and pitch.

## First hardware session

Record actual sensor topics/types, frame IDs, QoS, calibration, image encoding, time synchronization, B2 control wrapper and recording commands. Take one overview and close-up of a ladder under supervision. Get the real template, label semantics and defect rubric from materials supplied at kickoff. Treat illustrative sensor counts/IPs/remote button sequences in the team bible as unverified until physically checked.

## Minimum vertical slice

One real image -> detected ladder -> evidence -> official field -> reviewed record. Then bounded patrol, deduplication, re-ID and mission coverage. Keep the kinematic simulation and actual hardware success criteria separate. Team baseline target Saturday 13:30; backup ≤2-minute hardware video Saturday 22:00; submission Sunday noon.

## Dataset package

Collect permitted team captures plus calibration, aligned depth metadata, labels, crops, timestamps, model versions and train/eval split by physical ladder/session. Include negatives, unknowns and failures. Remove identifying bystanders/private information. Keep source rights/license explicit; no automatic CC-BY assumption. Generate a dataset card and manifest, then upload to Hugging Face only when the user/team requests publication. Current synthetic runs must be labeled synthetic and never counted as real sensor data.
