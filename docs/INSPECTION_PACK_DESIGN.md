# Inspection prompt pack and updated ownership

User instruction, 27 November 2026: Om owns ladder detection; Kaan (also written Kann) owns persistent spatial memory; Sahil owns simulation SLAM/mapping/navigation; Vivek owns the seven-section API prompts, structured assessments/evaluation and integration with Aravind's dashboard. This supersedes the earlier role table. Coordinate the observed-asset/pose handoff between Sahil and Kaan rather than implementing competing memories.

Deliver a reusable prompt/rubric/schema pack, a deterministic request builder/result-to-dashboard adapter and a ZIP of image-generation prompts for photorealistic real-world construction photography. No paid API calls or image generation are part of creating this pack. Synthetic intended defects are generation instructions, not reviewed labels or real inspection validation. MuJoCo is used separately for capture-pipeline tests, not as the requested dataset style.

Current dashboard contract inspected at `feature/dashboard` commit `42330758a62197122b56d56098a49c8c442350bc`: `dashboard/models.py`, `checklist.py`, `app.py`. Existing `feature/protocol-reader` at `698a571c4b3631cf3ed535212a80b660ee071fe2` also contains a VLM reader/publisher. This pack does not merge either branch or replace that reader. It uses the dashboard's 23 exact criterion IDs across seven sections and its `OK/DEFECT/UNKNOWN/NOT_CHECKED/N_A` states. Any change to dashboard keys requires an explicit contract update.

The upstream live-assessment milestone requires a sampling audit of eight
target-relative azimuth sectors (45-degree spacing), each with lower/middle/
upper evidence: 24 accepted keyframes, plus label detail when possible.
That is a requirement of that particular adapter, not a proven minimum or a
prerequisite for the current operator-positioned label-first draft. Angles
refer to the observed ladder frame, not the robot's own yaw. Distinct bins
alone do not establish surface visibility, structural integrity or functional
operation. The grid and thresholds are proposals, not measured hardware
guarantees.

Unavailable or blocked rear/upper views must fail the specific full-coverage
gate and be displayed as incomplete; do not call a partial survey 360.
Preserve partial defect findings and route gaps to human review. No path may
cross obstacles, reduce keep-outs or command unverified tilt/height movement
just to complete the grid. The isolated `inspect360` runner uses virtual
pan/tilt, kinematic movement and oracle visibility; it does not establish
physical completion or all-surface visibility. The optional experiment does
not supersede the baseline or hardware gates.

API requests carry one asset's keyframes and explicit image IDs, all seven section prompts and a strict schema with every criterion present. A bundle fingerprint binds exact images/job context/prompts; stale answers cannot be applied to a different mission or changed evidence. The model proposes evidence only. Local validation checks IDs, states, confidence, references and human-only constraints. An uncalibrated clean proposal never emits an automated pass. The dashboard adapter preserves `safe_to_use=false`, functional checks and sign-off pending. It requires an actual map position; unknown position is an integration error rather than a made-up origin. Published photo IDs are content/asset-bound and distinct from local frame names, preventing cross-ladder overwrites within a mission.

Dashboard publication order: start the mission, upload evidence JPEGs, emit progress events, then post one final inspection. Its current duplicate rule rejects a second verdict for the same asset/mission, so incremental coverage/proposals remain events until finalization. No cloud credentials go to the browser. No notification channels or live endpoints are invoked by preparing or testing this pack.

The generation ZIP contains prompts, review templates and API assets,
including source-derived rubric material with unverified reuse rights. Do
not upload it or generate images externally without separate authorization
and rights/privacy review. If generation is approved, keep all views and
condition variants of one physical identity in one split. Intended labels
stay `reviewed=false` until the actual image is inspected. Real held-out
ladder/session data is required for real performance claims.
