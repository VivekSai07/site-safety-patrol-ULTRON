# Local validation, 27 November 2026

## Main reconciliation with `0e7eff6`

In the reconciled worktree, Python 3.10.12 with MuJoCo 3.17.0, NumPy 2.2.6,
Pillow 12.3.0 and the dashboard dependencies ran:

```bash
PYTHONPATH=src:tests python -m unittest discover -s tests -v
PYTHONPATH=src:tests MUJOCO_GL=egl ULTRON_RENDER_TESTS=1 python -m unittest discover -s tests -v
```

The fast run completed 213 tests with 15 skipped; the rendered run completed
213 with seven skipped, with all remaining tests passing. Rendered skips were
two PDF checks requiring the unavailable source form, one check requiring a separately generated
`runs/history-demo`, and four optional browser checks. The interpreter used
the dashboard environment with a separate local simulation site-packages
path; for reproduction install the declared `sim` and `dashboard` extras.
Python 3.12 was not tested locally.

A separate clean-source run generated a two-patrol synthetic history and ran
28 dashboard tests with two PDF checks skipped; the history conversion check
passed. A clean wheel included the synthetic schema, dashboard assets and
both assessment packs, without the excluded local schema or hall.

A separate `ultron.inspect360` run recorded 24/24 accepted synthetic
sampling cells and return to the simulated base. It used kinematic movement,
virtual camera pan/tilt and segmentation-oracle visibility; this is not a
validated minimum image count, complete physical surface coverage or hardware
readiness. No real robot or live external API was contacted.

## Organizer B2 headless smoke test and label-first status

An isolated organizer-kit headless test on 27 November 2026 used image
`ultron-organizer-b2:1127b2001`, kit commit
`8f5de5806fcd432d339ca1dbb61a1f0f9ea1d6cd`, private loopback/domain 71,
and no viewer or stand controller. Its log records B2 startup with 12
actuators and 601 `/lowstate` samples received at 199.99 Hz, with finite
first 12 joint positions and IMU values. `/lowcmd` and `/sportmodestate`
topic types were observed, not a high-level walking service. The test cleaned
up task-owned containers. See [organizer B2 simulation](ORGANIZER_B2_SIM.md)
for the exact commands, isolation, source and local evidence path.

Viewer, stand-up, walking and capture-to-draft integration were not
established by that DDS test.

## Organizer B2 isolated viewer and stand probe

A later bounded team-wrapper probe used the organizer simulator and stand
controller in private container loopback. Viewer HTTP returned 200; noVNC
WebSocket returned 101 with an RFB greeting; the captured PNG was visually
inspected and shows a standing simulated B2. The probe observed 1,432 finite
`/lowcmd` samples and 598 finite `/lowstate` samples at approximately
198.93 Hz. The finite checks covered the first 12 motor/joint positions and,
for LowState, IMU values. The wrapper used a project-specific XKB cache,
read-only roots and no host ports; task containers were cleaned up.

See [organizer B2 simulation](ORGANIZER_B2_SIM.md) for the exact command and
the local ignored evidence at `runs/organizer-b2-viewer-20261127e/`. Browser
GUI/host access, walking, hardware and the unmodified host-published Compose
viewer launch remain untested. Sport request/response topics from the stand
client do not establish a walking service.

## Derived capture-to-draft probe

Separately, a 27 November 2026 rendered probe exercised the derived B2
kinematic scene at 1280×720. HTTP requests created a session, captured a
synthetic overview and label ROI, moved the simulated base by a bounded
increment, issued STOP and observed rejection of a subsequent move. A
manually transcribed synthetic fixture manifest produced a separate local
draft and review page; one recorded run retained 41 unresolved fields, two
unresolved date roles and pending human review.

The run used the earlier local source-specific catalog. That catalog is
excluded from distribution and may be used locally only with verified
local-use rights, which remain unverified. An independently authored minimal
synthetic public example and an `ULTRON_FORM_SCHEMA` override for a compatible
local schema are now available; this earlier run does not validate them.
The public example is not the official form or a complete mapping.

That probe tested HTTP endpoints, not browser keyboard operation. The imagery,
label values and motion are synthetic; no OCR, physical walking, hardware,
real inspection or safe-use conclusion is established. This is not a
full-system simulation acceptance or hardware-readiness gate. The ignored
local evidence remains at `runs/teleop-render-probe-20261127-2243/`; no raw
output is copied into these documents. See
[LABEL_FIRST_BASELINE.md](LABEL_FIRST_BASELINE.md) and
[LABEL_FIRST_USAGE.md](LABEL_FIRST_USAGE.md).

The separate host-accessible derived teleop launcher was also exercised:
the host-loopback page, PNG frame, session and capture HTTP endpoints
responded successfully; the task-owned resources were removed. The local
environment used the launcher's loopback relay when Docker did not publish
the port. That HTTP probe did not exercise browser keyboard input. This derived probe is
separate from the isolated organizer viewer/stand probe above. Its local log
is `runs/teleop-host-ui-final3/host-probe.log`; no raw output is copied here.
The host-local teleop UI has no user authentication; localhost does not
protect it from other local users. These HTTP checks do not establish a
multi-user access boundary.

## Derived teleop browser checks

On the simulated derived UI, headless Playwright Chromium exercised WASD
press/release, a directional button hold/release, STOP while held, preview,
overview capture and label-ROI capture with a linked original and crop.
An earlier focus-switch check in that automated setup did not produce the
expected lifecycle transition; those preceding checkpoints did pass.

A separate headed stock Firefox 132.0 run with session-local Xvfb/twm and
native keyboard/mouse input held W while switching to another tab. The
server received `FOCUS_LOST` and `PAGE_HIDDEN` STOP requests; after returning,
pressing W without rearming produced no new movement request. Opening a new
window produced `FOCUS_LOST` and disarmed the session. Earlier headless and
automation-distributed browser focus attempts did not produce the expected
transition; the cause was not established. The successful Firefox run used
session-restricted networking/filesystem access, and its content/RDD/GPU
sandboxes were disabled because nested user-namespace setup was denied.
Task-owned services were cleaned up. These checks cover only these simulated
setups, not every browser or an independent physical stop.

Ignored local evidence: `runs/teleop-browser-ui-20261127/browser-focus-evidence.json`
and its referenced logs. Real OCR, live API, walking, hardware and a
completed inspection remain unverified; no raw output is copied here.

The following historical suite was executed using the host Python 3.14
environment, MuJoCo 3.13.0, NumPy and Pillow. Python 3.10 MuJoCo runtime was
not available in that earlier run; later results must be recorded separately.

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

The simulator records SDK-shaped velocity commands, a world-object memory inventory and visible detector annotations. Model reports default to review without weights, with source images preserved. Publication requires explicit approval and applicable rights/privacy review. Reference PDFs, images, private research, generated runs, datasets, model weights and service credentials are outside the approved scope; the local reference-derived checklist catalog is not approved for public distribution. Retain required source and license provenance for included material.

Final v0.3 local gate: all 36 tests passed, including the three EGL-rendered integration tests. A two-patrol smooth-motion run retained three ladder IDs, returned to base in both patrols and recorded coverage uncertainty. It stored 10 scene entities and 5,290 SDK-shaped command samples. The synthetic exporter produced 524 detector frames/labels. These are simulation/workflow measurements, not real-model or real-robot performance.
