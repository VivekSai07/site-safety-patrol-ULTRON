# ULTRON · Site Safety Patrol

> **Recording data on the real B2?** Follow [RECORD_ON_B2.md](RECORD_ON_B2.md).

A Unitree B2 walks a construction site, finds every ladder and the fire and first-aid equipment, and fills in nesseler's checklists with a photo as evidence for each point. Results arrive on a live dashboard while the robot is still walking.

```
robot (ROS 2)                     GPU machine                            anywhere
scripts/ros_relay.py ──TCP──▶ python -m ultron.pipeline patrol ──HTTP──▶ python -m ultron.dashboard
 camera, depth, pose,           survey → check → inspect                  mission, live view,
 lidar if there is no pose      (detector + vision model)                 checklist records, events
```

1. **Survey.** An open-vocabulary detector (YOLOE) finds ladders, fire extinguishers, hose cabinets, alarm boxes and first-aid kits in the camera stream. Depth and the robot's pose place each one in the map; tracking keeps one object from becoming several. The best frames per object and viewing side are kept.
2. **Check.** A vision-language model looks at each object once and confirms, corrects or rejects what the detector called it.
3. **Inspect.** For each object the pipeline works out which views its form still lacks, asks navigation for them, and reads the form: nesseler's 23-row ladder Kontrollblatt, or the fire and first-aid rows of their site checklist.
4. **Report.** Records, evidence photos and events go to the dashboard, which exports nesseler's original form as a PDF.

A form row is only filled when the cited photo resolves enough detail to show it: a model cannot report "no cracks" from across the room. Function checks (does the spreader lock, are the hinges greased) always stay with a person, and every result is a proposal for a qualified inspector to countersign. **Nothing here is a safe-to-use certificate.**

![ULTRON B2 patrol preview](docs/images/ultron-patrol.gif)

*The preview shows the simulation described in the second half of this page.*

## What has been shown, and on what

| Part | Shown on | Result |
|---|---|---|
| Survey | 36 s recording of the real B2 in the hackathon hall | 5 objects, each real and correctly typed: two ladders, fire extinguisher, hose cabinet, alarm box |
| Object check | same recording, Qwen3-VL 8B | ladders, extinguisher and hose cabinet confirmed in every run |
| Ladder form | same recording | shape rows filled with photo and box (22 % and 31 % of visual rows); the rest reported as "close-up needed" |
| Whole chain, live | the recording replayed as ROS topics through the relay | mission, live view, both ladders and all events on the dashboard |
| SLAM and map reload | recorded B2 LiDAR and simulation | see [validation record](docs/SAHIL_VALIDATION.md) |
| Dashboard goals through Nav2 and the motion gate | MuJoCo, ROS 2 Humble, stand-in B2 SDK | valid goal reached, unsafe requests rejected, active goal canceled on replacement |
| Patrol, coverage planner, safety sweep, 360° capture | MuJoCo simulation with ground-truth detections | see below |

Not yet shown: the relay on the robot itself, a pose from running localisation, the robot driving to a requested view, and any close-up (about 0.4 m) of a label or inspection sticker. Until a close-up exists, no sticker date and no fine row (cracks, wear, burrs) has been read for real. Thresholds are fitted to one recording. Details and numbers: [docs/PIPELINE.md](docs/PIPELINE.md).

## Run it

```bash
python -m venv .venv && source .venv/bin/activate
pip install -e '.[protocol,dashboard,bags,detector]'      # GPU machine
export OPENAI_API_KEY=ollama OPENAI_BASE_URL=http://HOST:11434/v1          # any OpenAI-compatible vision model
export ULTRON_VLM_MODEL=qwen3-vl:8b-instruct ULTRON_TRIAGE_MODEL=qwen3-vl:8b-instruct

python -m ultron.dashboard --port 8010 &
python -m ultron.pipeline patrol --source relay:5600 --detector yolo --interval 0 \
    --mission HALL-1 --site my-site --dashboard http://127.0.0.1:8010
```

On the robot, in a shell with ROS 2 sourced (needs only `rclpy`, `tf2_ros`, `numpy`; it subscribes and never commands):

```bash
python3 scripts/ros_relay.py --to GPU_MACHINE:5600
```

Without a robot, run the same pipeline on a recorded bag: `--source bag:PATH,rate=8 --calibration bag:PATH`. Press Ctrl-C once to end the survey and start the inspection. `--navigator` chooses how missing views are reached: `passive` (not at all), `operator` (a person drives), `dashboard` (goal through the dashboard bridge to Nav2) or `contract` (the `ultron.autonomy` navigation contract with `--map DIR`). Only `passive` has been exercised.

## Where things are

| Package | What it is | Owner |
|---|---|---|
| `src/ultron/pipeline/` | Spatial memory (per-site object map: identity across patrols, moved objects), survey, keyframes, prompt calibration, object check, view planning, inspection, navigation adapters | Kaan |
| `src/ultron/protocol/` | Frame sources (bag, relay, ROS 2), nesseler's ladder form as data, vision-model client, section-by-section reader, rules, evidence store, dashboard publisher | Kaan |
| `scripts/ros_relay.py` | Robot-side relay | Kaan |
| `src/ultron/dashboard/`, `scripts/dashboard_bridge.py` | Live dashboard, PDF export into nesseler's form, alerts, goal bridge to Nav2 | Aravind |
| `src/ultron/autonomy/`, `config/autonomy/` | Simulated lidar, slam_toolbox mapping and localisation, Nav2 behind a motion gate, navigation contract | Sahil |
| `src/ultron/` (top level), `src/ultron/assets/` | MuJoCo simulation and its mission store (`memory.py`, retired as spatial memory), patrol and coverage planner, assessment and site-safety prompt packs, teleop, model adapters | Vivek |
| `src/ultron/detection_world/`, `detection_*.py` | Synthetic detection worlds, box-level evaluation, label-first baseline | Om |
| `src/site_safety_patrol/`, `scenarios/` | Earlier standard-library fixture demo, kept for reference | |

Documents: [pipeline](docs/PIPELINE.md) · [hardware readiness](docs/HARDWARE_READINESS.md) · [dashboard](docs/DASHBOARD.md) · [navigation goals from the dashboard](docs/dashboard_nav_goal.md) · [autonomy handoff](docs/SAHIL_HANDOFF.md), [SLAM plan](docs/SAHIL_SLAM_PLAN.md) and [navigation plan](docs/SAHIL_NAV_PLAN.md) · [site safety sweep](docs/SITE_SAFETY.md) · [API prompt packs](docs/API_PROMPTS.md) · [weekend plan and team split as first written](docs/WEEKEND_PLAN.md).

## Tests

```bash
pip install -e '.[sim,dashboard,autonomy,protocol]'
python -m unittest discover -s tests
```

388 tests; CI runs them on Python 3.10 and 3.12. They need no GPU, no ROS and no network: detectors, vision models and Nav2 are replaced by stand-ins, so passing tests say the logic holds, not that a model sees correctly. `MUJOCO_GL=egl ULTRON_RENDER_TESTS=1` adds the camera-rendered simulation tests.

---

# The simulation

**Readiness:** [Integrated launch commands, tested evidence and remaining hardware blockers](docs/HARDWARE_READINESS.md). Hardware motion remains NO-GO; SLAM/Nav2 and real sensor/control validation are outstanding.

**API prompts:** [Versioned ladder and site-safety prompt packs, schemas and request-building adapter](docs/API_PROMPTS.md). Targeted `inspection_v2` wording is available as an opt-in candidate; the evaluated v1 baseline is preserved. [Evaluation plan](docs/PROMPT_V2_EVALUATION.md).

**A lightweight Unitree B2 MuJoCo simulation for the nesseler challenge.** The existing demo has a robot model patrol a bounded construction scene, discover ladders, approach for evidence, request one second look, and remember ladder identities across patrols. Actual simulated RGB/depth frames become a traceable inspection report. Rerun export and ROS 2 Humble/RViz replay consume the same recording. The label-first baseline has a rendered synthetic capture-to-draft probe with manual fixture transcription, plus bounded browser-control checks; real OCR and real-world operation remain unverified. See the [baseline and source overview](docs/LABEL_FIRST_BASELINE.md).

This is ULTRON's team-side working repository. The organizer starter remains the integration authority for real hardware. The simulation demo does not contact organizers or send robot commands; the optional hardware hook is not verified for real operation.

An independently authored minimal synthetic draft schema and an
`ULTRON_FORM_SCHEMA` override for a compatible, locally supplied schema are
available. The public example is not the official Nesseler form or a complete
mapping. The source-specific catalog remains local, is excluded from
distribution and may be used locally only with verified local-use rights.
Local catalogs are fixture-only in the optional OpenAI adapter; live use is
not enabled for them. Earlier probe results do not validate this example or
override.

**Organizer starter alignment:** retain ULTRON for inspection workflows and use the SMAROBIX Unitree simulator for SDK/DDS/ROS tests. See the [source comparison, launch commands and nesseler checklist export](docs/ORGANIZER_ALIGNMENT.md). The organizer simulator stands with physics and joint interfaces; it has no walking controller, camera or lidar.

Version 0.4 adds optional model-backed simulation observations, grouped synthetic corpus generation, training entry points and a DimOS goal adapter. [Commands and verification limits](docs/RUNTIME_V04.md) distinguish implemented interfaces from executed integrations.

For the separate simulation-only ladder and inspection-label detection test,
see the [detection world](docs/DETECTION_WORLD_USAGE.md), [public-source
review](docs/DETECTION_WORLD_SOURCES.md), and [offline evaluation
guide](docs/DETECTION_EVALUATION.md). A [yellow-card label baseline](docs/DETECTION_LABEL_BASELINE.md)
exercises the evaluator using image pixels; it is not a trained label model.
Synthetic ground truth is not a detector.


## Seven-section assessment and photoreal dataset prompt pack

The [assessment runbook](docs/ASSESSMENT_RUNBOOK.md) describes an optional
live adapter and a separate 24-view simulation experiment. The current
[integrated patrol worker](docs/HARDWARE_READINESS.md#reproduce-on-the-laptop)
uses the user-authorized OpenAI path when explicitly enabled; images and prompts
leave the laptop and can incur API charges. A one-keyframe cloud response was verified from the user laptop;
a verified-reference synthetic comparison subsequently detected the seeded missing rung.
Real-data accuracy and performance across other defects remain unverified.
Default simulation makes no cloud calls.

The existing [generation-prompt ZIP](deliverables/ULTRON_inspection_prompt_pack_v1.zip)
contains scenario recipes and prompts, not reviewed images or a validated
dataset. It includes source-derived rubric material whose reuse rights remain
unverified. Do not upload it, its prompts or images to any external service
without separate authorization and rights/privacy review.

```bash
# Optional 24-view experiment; virtual pan/tilt, kinematic motion, oracle visibility.
PYTHONPATH=src MUJOCO_GL=egl LP_NUM_THREADS=2 python3 -m ultron.inspect360 --out runs/my-360-capture
# Build a private Responses API JSON body locally; does not send it or spend credits.
PYTHONPATH=src python3 -m ultron.assessment request runs/my-360-capture/keyframes.json --model YOUR_AVAILABLE_VISION_MODEL --out runs/my-360-capture/request.json
```

## Simulation-first; hardware is separately gated

Do most development and validation in simulation. Complete integrated hardware deployment requires documented full-system simulation acceptance, followed by separate real-world readiness, explicit authorization and operator supervision. Simulation acceptance records unresolved assumptions; it does not validate physical locomotion, real sensors or optional SDKs.

Targeted component hardware tests may happen before full-system simulation acceptance, but only with component-appropriate tests and risk assessment, authorization and a bounded plan. Any motion also requires verified RC priority, an independent watchdog/stop, a functional E-stop, a measured environment and a controlled operator. Any hardware connection requires explicit user/operator authority and independent safety validation; this reconciliation authorizes none. See [simulation-to-hardware gates](docs/SIM_TO_HARDWARE.md).

Optional advanced intelligence has no control authority. Proposed System One remains off by default, offboard and read-only/audit-only after the hard-fail scan, with no route to motion, verdict or veto. Inspection verdicts are prescreening, never safe-to-use certification.

Numeric thresholds and launch, network and device details in plans or examples are proposed until verified for their intended environment; simulation settings are not hardware approvals. The [Pass 7 Build Bible](docs/bible/BUILD_BIBLE_PASS7.md) and [reconciliation record](docs/RECONCILIATION_PASS7.md) distinguish current implementation from proposed work; neither authorizes hardware operation.

## Run the simulation

Separate organizer-kit probes observed B2 DDS samples and a standing B2
viewer image via isolated container loopback; browser GUI/host access,
walking and hardware remain untested. For exact commands and limits see
[organizer B2 simulation](docs/ORGANIZER_B2_SIM.md).
For the distinct simulated capture UI and local manifest-to-draft commands,
see [label-first usage](docs/LABEL_FIRST_USAGE.md). The separate
[validation record](docs/VALIDATION.md) scopes the rendered synthetic probe.
The host-local teleop UI has no user authentication; localhost does not
protect it from other local users. Use a trusted host and non-sensitive data.

From the repository root, on a machine with MuJoCo, NumPy and Pillow (verify dependencies and launch details locally):

```bash
PYTHONPATH=src MUJOCO_GL=egl python3 -m ultron demo --out runs/ultron-demo
```

## Site safety sweep beyond ladders

The optional `--safety` simulation visits fixture fire, first-aid, escape and
PPE props and writes `safety_sweep.json` for the 9-criterion
`site_safety_v1` pack. By default, discovery and PPE flags use simulator
oracles, not learned perception or a site inspection. `--nav auto` replaces
the scripted route with coverage-planner goal selection, and
`ultron.dashboard.live_tail` streams a running patrol to the dashboard. See
[docs/SITE_SAFETY.md](docs/SITE_SAFETY.md).

```bash
PYTHONPATH=src MUJOCO_GL=egl python3 -m ultron demo --out runs/safety-demo --patrols 1 --safety
```

Omit `--out` for a unique run folder. Existing folders are refused. Default: two patrols with acceleration-limited motion; simulated duration follows the route and turn profile, and rendering speed depends on the machine. Run data is roughly 200–300 MB because depth arrays are preserved.

After generating a run at `runs/ultron-demo/`, open `report.html` directly, or serve it:

```bash
python3 -m http.server 8000 --bind 127.0.0.1 --directory runs/ultron-demo
```

Open <http://localhost:8000/report.html>. `preview.gif` shows the rendered robot and camera without requiring a viewer.

For a fresh machine (Python 3.10+):

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[sim,rerun]'
ultron demo --out runs/demo
```

Install the simulation extra in a suitable isolated environment if those dependencies are not already available.

## What to demonstrate

| Case | Implemented behavior |
|---|---|
| Patrol 1: intact leaning ladder | Overview + close-up; `AUTOMATED_PASS` within demo visual scope |
| Missing rung / overdue label | `FLAGGED`; hard-fail early exit before completeness/recovery |
| A-frame rear feet/spreader unknown | At most one approved side pose, fused evidence under one LAD ID |
| Patrol 2: newly missing foot cap | Same LAD-001; append history and highlight status change |
| Missing expected ladder | Mission becomes `COVERAGE_INCOMPLETE` |
| Blocked approach | Reject goal, preserve evidence, unresolved components require review |

`AUTOMATED_PASS` is the teammate bible's scoped pre-screening status. It is **not a declaration that a ladder is safe to use**. All records retain `safe_to_use=false`, human sign-off, and pending tactile/functional checks.

```bash
PYTHONPATH=src python3 -m ultron demo --variant missing
PYTHONPATH=src python3 -m ultron demo --variant blocked --patrols 1
PYTHONPATH=src python3 -m ultron demo --no-recovery --patrols 1
PYTHONPATH=src python3 -m ultron demo --variant intervention --patrols 1
PYTHONPATH=src python3 -m ultron query runs/ultron-demo/memory.sqlite --text FLAGGED
PYTHONPATH=src python3 -m ultron query runs/ultron-demo/memory.sqlite --near 3 6 --radius 1
```

Use `--database runs/team-memory.sqlite` to retain history across separate runs on the same map. Prior flags remain unresolved until human resolution (a release workflow is future work). Old history evidence is copied into each export; retain source run folders while reusing the database. Query is explicit text/spatial filtering, not an LLM or embedding search.

## View in MuJoCo, Rerun or RViz

```bash
# Native MuJoCo viewer, on a desktop with working display access
bash scripts/launch-demo.sh
# Rerun SDK 0.32.0 required; save ultron.rrd and open viewer
PYTHONPATH=src python3 -m ultron rerun runs/ultron-demo --spawn
# ROS 2 Humble + RViz + system Pillow required; isolated localhost domain 71
scripts/rviz.sh runs/ultron-demo
```

Rerun logs the B2, site primitives, route, camera transforms, RGB/depth, spatial asset pins, and events. RViz replay publishes only `/ultron/sim/*` visualization topics and TF. It publishes **no `/cmd_vel`, DDS low-level commands or hardware goals**. Primitive capsule visuals are approximated by bounding boxes in Rerun and cylinders in RViz; MuJoCo renders original primitives.

**Verification limits:** headless EGL rendering and SQLite workflows have been exercised in simulation. Rerun/DimOS adapters were not verified end to end in the recorded validation; their availability and graphical/ROS restrictions vary by environment. Portable recordings remain available for supported viewers.

## What is real in the simulation

The B2 description comes from Unitree's official MJCF, retaining its joint tree and inertials with primitive geometry. MuJoCo renders RGB, metric depth, occlusion and semantic segmentation. Rail pixels are projected with camera intrinsics/extrinsics to map coordinates. A* avoids inflated obstacles/zone edges, and SQLite associates spatial observations independently of fixture IDs.

**Motion is kinematically prescribed; this is not a walking controller or a dynamics validation. Detection/type and inspection readings use explicit simulation oracles.** The patrol demo still uses oracles. Optional YOLOE/YOLO-World and Qwen3-VL image-analysis adapters and a B2 SportClient transport hook are implemented separately; their runtime inference/hardware operation is unverified. Real SLAM, sensor calibration and physical walking control remain integration work. Discovery confirmation occurs in stationary waypoint bursts. A-frame evidence recovery is scripted, not proof of actual component visibility. Waypoint completion is reported separately from unknown area coverage. See [design and limitations](docs/DESIGN.md).

## Motion, perception, dataset and mission intelligence

The default motion profile turns towards travel, accelerates/decelerates, and records `Move/StopMove` commands in `motion_commands.jsonl`. B2 SDK calls are behind an explicit connection hook; the demo never connects to hardware. The leg cycle is cosmetic animation, not a contact controller.

A deterministic supervisor prioritizes reachable known targets, records gaps and returns to base. It distinguishes completed known inspections from verified site coverage. Static scene objects are stored in memory from **simulation world truth**, not a working semantic mapper. The DimOS adapter can ingest these objects and inspection history when its runtime is available.

```bash
# Export visible synthetic ladder boxes and annotation metadata for dataset development
PYTHONPATH=src python3 -m ultron export-dataset runs/ultron-intelligent --out runs/synthetic-dataset --split train
# Selected images of one ladder: optional local Qwen3-VL, no hardware commands
PYTHONPATH=src python3 -m ultron analyze front.jpg side.jpg --qwen /path/to/local/model --out runs/model-review --ladder-type A_FRAME_STEPLADDER
# Watch the capture stream while the demo is running (Rerun SDK required)
PYTHONPATH=src python3 -m ultron rerun runs/live-patrol --spawn --follow
```

A normal COCO YOLO checkpoint lacks a ladder class; use a ladder-trained checkpoint or open-vocabulary YOLOE/YOLO-World. Model confidence is not a calibrated safety probability. Unseen/weak evidence is UNKNOWN, and unvalidated model PASS proposals require human review. Synthetic defect metadata does not establish pixel visibility. See [motion/perception](docs/PERCEPTION_MOTION.md), [supervisor architecture](docs/AUTONOMOUS_SUPERVISOR.md), and [data collection](docs/DATA_COLLECTION.md).

Start with [source review](docs/SOURCE_REVIEW.md), [jury strategy](docs/JURY_STRATEGY.md), [B2/DimOS integration](docs/INTEGRATION.md), [handoff](docs/EVENT_HANDOFF.md) and [pitch/video plan](docs/PITCH.md). The organizer source PDFs are not included in this team checkout. The [baseline overview](docs/LABEL_FIRST_BASELINE.md) cites the locally supplied Nesseller checklist's source revision; the original remains outside this team repository. The local reference-derived catalog is not approved for public distribution.
