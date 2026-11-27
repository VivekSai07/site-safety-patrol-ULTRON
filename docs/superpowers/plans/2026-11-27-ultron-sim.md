# ULTRON simulation proof of concept implementation plan

Goal: deliver the requested lightweight MuJoCo/RViz B2 workflow demonstration with Rerun export and spatial history.

Scope: a simulation spike, authorized directly by the user's correction; native execution in the existing team repository. The repo directory is renamed ULTRON. No physical robot control or organizer contact.

Architecture: Unitree's B2 MJCF reduced to collision geometry, with its original kinematics/inertials retained; kinematically prescribed base motion along an inflated-grid A* route. MuJoCo renders RGB, depth and segmentation. Oracle segmentation supplies detections; ladder findings are scenario fixtures, explicitly separated from pixel evidence. SQLite maintains map-scoped assets and append-only inspections. Rerun logs 3D geometry, camera, depth and verdicts; a separate Humble replay publishes RViz markers/TF/images from the same portable recording.

- [x] Read organizer recap/slides, teammate bible and supplied DGUV inspection chapter/checklist; distinguish organizer requirements from teammate proposals.
- [x] Write failing tests for hard-fail precedence, review, spatial association, history and bounded planning; implement and run.
- [x] Build a three-ladder construction scene and camera capture with the official primitive B2 model.
- [x] Implement discovery-confirmation, approach/capture/recovery/resume, two-patrol changes and missing-asset scenario.
- [x] Add Rerun export, RViz replay and optional DimOS memory ingestion using inspected upstream APIs.
- [x] Run rendered end-to-end scenarios, inspect evidence images, verify history, failure modes and replay.
- [x] Rewrite README/agent instructions, record sources/limits and jury delivery priorities; commit local work.

Review focus: ambiguous nearby ladders stay separate; hard failures win even with missing evidence; absent assets are mission gaps; blocked goals never teleport through obstacles; missing optional SDKs report their limitation and preserve portable data.
