# Site safety sweep (fire / first aid / escape routes / PPE)

![ULTRON safety sweep, model-backed](images/ultron-safety-sweep-model.gif)

| PPE OK | PPE violation (flagged) |
|---|---|
| ![helmet and vest](images/safety-ppe-ok.jpg) | ![no helmet, no vest](images/safety-ppe-violation.jpg) |

## Model-backed confirmation and judgment (synthetic scene)

With model backends configured, confirmation and criterion proposals use
pixels, but target kinds, positions and routing still come from the simulated
scene. This is not independent site discovery or a real-world inspection.
Model outputs are uncalibrated proposals requiring human review.
The following optional model command downloads weights; first review the
specific model and dependency licenses and obtain any required approval.

    HF_HUB_DISABLE_IMPLICIT_TOKEN=1 PYTHONPATH=src MUJOCO_GL=osmesa python3 -m ultron demo \
      --out runs/model-safety --patrols 1 --safety \
      --detector yolo --weights weights/yoloe-11s-seg.pt \
      --inspector qwen --qwen Qwen/Qwen3-VL-2B-Instruct --allow-download

- Station confirmation: YOLOE open-vocab box near the station
  (`source=ULTRALYTICS_MODEL`) or Qwen3-VL visual confirmation
  (`source=QWEN_VL_PROPOSAL`). Neither → `SAFETY_CHECK_INCOMPLETE`
  `reason=NOT_DETECTED_BY_MODEL`.
- Judgment: `SAFETY_STATION_JUDGED` carries per-criterion `OK/DEFECT/UNKNOWN`
  states and the model's literal reason (`source=MODEL_PROPOSAL`);
  `SAFETY_VIOLATION` fires when the model judges `psa.helm`/`psa.warnweste`
  DEFECT.
- A teammate reported a local synthetic-scene smoke run on 2026-11-28:
  YOLOE detected nothing on primitive renders; Qwen3-VL-2B proposed
  confirmation and judgments for five stations. This reconciliation did
  not reproduce the model run and it is not real-world accuracy evidence.
- The segmentation oracle runs only with the default flags (CI/no-GPU).

## Live dashboard (end-to-end, as site manager)

Three terminals — dashboard, live tailer, autonomous patrol:

    # 1. Dashboard (http://127.0.0.1:8000)
    PYTHONPATH=src python3 -m ultron.dashboard --db runs/dashboard.sqlite --media runs/dashboard-media

    # 2. Live tailer: streams the run to the dashboard as it is written
    PYTHONPATH=src python3 -m ultron.dashboard.live_tail runs/live-e2e --url http://127.0.0.1:8000 --idle-timeout 600

    # 3. Autonomous, model-backed patrol (no scripted route, no sim-oracles)
    HF_HUB_DISABLE_IMPLICIT_TOKEN=1 PYTHONPATH=src MUJOCO_GL=osmesa LP_NUM_THREADS=2 python3 -m ultron demo \
      --out runs/live-e2e --patrols 1 --safety --nav auto \
      --detector yolo --weights weights/yoloe-11s-seg.pt \
      --inspector qwen --qwen Qwen/Qwen3-VL-2B-Instruct --allow-download

What the site manager sees live: robot pose on the zone map (bounds/keepouts
from the mission), the front camera, the event feed — `PATROL_GOAL_SELECTED`
(autonomous coverage goals, `ultron.navigation_goal/v1` records aligned with
the Nav2 contract on `sahil/sim-slam-nav2`), `SAFETY_STATION_JUDGED`,
`SAFETY_VIOLATION` — and, at mission end, review-pending ladder inspections
with evidence photos. Verified 2026-11-28: PPE violations arrived in the live
feed with `source=MODEL_PROPOSAL`; ladder records posted as `MANUAL_REVIEW`.

`--nav auto` honesty: autonomy here is goal selection (coverage planner on
the A* grid); base motion stays kinematic. Real locomotion and Nav2 execution
live in the organizer simulator / `sahil/sim-slam-nav2` lane.

Beyond the ladder protocol, patrol 0 can sweep nesseler-style site-safety
stations. Checklists are data: `src/ultron/assets/site_safety_v1/` mirrors
`inspection_v1` (rubric, strict response schema, system + section prompts) and
covers 9 criteria in 4 sections (`ext.*`, `firstaid.*`, `flucht.*`, `psa.*`),
derived from the BG BAU first-aid/fire-protection template and the
risk-assessment effectiveness check in the
[nesseler data](https://github.com/ctrl-rwth/ctrl-hackathon-2026/tree/main/site-safety-patrol/nesseler-data).

## Run

    PYTHONPATH=src MUJOCO_GL=egl python3 -m ultron demo --out runs/safety-demo --patrols 1 --safety

(Use `MUJOCO_GL=osmesa` where EGL is unavailable or wedged; see
`docs/DISPLAY_TROUBLESHOOTING.md`.)

The sweep emits events (`events.jsonl`) and writes `safety_sweep.json`:

| Event | Meaning |
|---|---|
| `SAFETY_TARGET` | grid-validated sweep goal chosen for a prop (`asset_kind`) |
| `SAFETY_ASSET_CONFIRMED` | segmentation oracle confirmed the prop (scene ground truth) |
| `SAFETY_EVIDENCE_CAPTURED` | keyframe saved for the VLM bundle |
| `SAFETY_VIOLATION` | `violation`: `PPE_HELMET_MISSING` / `PPE_VEST_MISSING`, with evidence frame |
| `SAFETY_CHECK_INCOMPLETE` | approach blocked or prop not visible — never silently OK |
| `SAFETY_SWEEP_SAVED` | `safety_sweep.json` manifest written |

## VLM assessment (offline builder, no API call)

    PYTHONPATH=src python3 -m ultron.assessment request runs/safety-demo/safety_sweep.json --model YOUR_MODEL --out /tmp/body.json

The manifest carries `"pack": "site_safety_v1"`; the builder produces a strict
9-criterion request. Images in one sweep bundle are different stations of the
same confirmed site visit. Clean proposals stay review-pending; there is no
automated pass, and `dashboard_payload` (ladder `Inspection` verdicts) rejects
this pack by design — site safety publishes events, not verdicts.

## Dashboard (note for Aravind)

No model change required today: `SAFETY_*` flow through the existing `Event`
contract (free-form `type` + `data`) and photos through the normal upload path
(`tests/test_site_safety_pack.py::DashboardEventCompatTests` pins this).
Optional follow-up owned by the dashboard: render `SAFETY_VIOLATION` events red
in the live feed and group `SAFETY_*` under a "Site safety" card. Any schema
change to `Event` is a contract update and needs both sides.

## Limits

Default prop confirmation and PPE flags use segmentation/fixture oracles.
With model backends enabled, confirmation and judgments are model proposals
for known synthetic target positions, not independent discovery. The VLM
judges only captured frames; unseen stations stay UNKNOWN/NOT_CHECKED.
Nothing here certifies the site as safe.
The other three nesseler checklists (scaffold, low-dust, subcontractor
briefing) are future data-only pack additions.
