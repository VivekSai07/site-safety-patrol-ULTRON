# Local validation, 27 November 2026

Executed using the host Python 3.14 environment, MuJoCo 3.13.0, NumPy and Pillow. Compatibility target remains Python 3.10+; actual Python 3.10 MuJoCo runtime was not available here.

- Full suite: `PYTHONPATH=src MUJOCO_GL=egl ULTRON_RENDER_TESTS=1 python3 -m unittest discover -s tests -v`.
- Two-patrol normal demo: `PYTHONPATH=src MUJOCO_GL=egl python3 -m ultron demo --out runs/ultron-demo`.
- Rendered regressions: missing expected asset, blocked A-frame pose, and blocked close-up cannot produce a premature pass.
- Pure regressions: a prior FLAGGED status cannot clear itself on a clean observation; historical evidence from colliding filenames is copied into an export under its originating run ID.
- Read-only review found the three regressions above; they were fixed and re-reviewed.
- Spatial/text query locates LAD-001 and its two ordered inspections from the on-disk SQLite database.
- ROS Humble messages imported; 32 B2 RViz markers were constructed from real recorded telemetry without starting a ROS node. `bash -n scripts/rviz.sh` and Python compile checks passed. This is not an RViz runtime result.
- Headless render images and the preview were visually inspected, including actual B2 primitive visibility and a side view of the A-frame.
- Missing Rerun and DimOS SDK paths produce explicit installation errors rather than fake recordings. No `.rrd` or DimOS store is claimed to have been generated.

The normal synthetic run discovers 3/3 scene ladders in each patrol, reuses three persistent IDs and performs one second look per patrol. There are 1 then 2 fixture-defined hard-fail cases; neither patrol emits a scoped pass for those cases. These are tests of workflow/rule behavior with oracles, not measured detector accuracy or real-hardware autonomy.

Remaining validation: Rerun 0.32 runtime, full DimOS memory ingest, GUI/RViz desktop replay, Python 3.10 simulation dependency resolution, actual B2 drivers/control/sensors, calibrated perception, official protocol, real course runs and required hardware video.

## Version 0.3 extension

New core tests cover motion velocity limits and endpoint geometry, exact obstacle-boundary drift, stale-state/operator inhibition in the SDK adapter, low-confidence/unseen/duplicate/invalid model proposals, site-coverage-aware supervisor completion, dataset label normalization, and idle Rerun event polling. Actual optional model or physical B2 inference/control is still unverified; injected-client tests establish only adapter contract behavior.

The simulator records SDK-shaped velocity commands, a world-object memory inventory and visible detector annotations. Model reports default to review without weights, with source images preserved. The supplied source is publishable separately from large ignored run/dataset folders; reference PDFs, model weights and service credentials are excluded.

Final v0.3 local gate: all 36 tests passed, including the three EGL-rendered integration tests. A two-patrol smooth-motion run retained three ladder IDs, returned to base in both patrols and recorded coverage uncertainty. It stored 10 scene entities and 5,290 SDK-shaped command samples. The synthetic exporter produced 524 detector frames/labels. These are simulation/workflow measurements, not real-model or real-robot performance.
