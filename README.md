# ULTRON · Site Safety Patrol

**A lightweight Unitree B2 MuJoCo simulation for the nesseler challenge.** The existing demo has a robot model patrol a bounded construction scene, discover ladders, approach for evidence, request one second look, and remember ladder identities across patrols. Actual simulated RGB/depth frames become a traceable inspection report. Rerun export and ROS 2 Humble/RViz replay consume the same recording. The label-first baseline has a rendered synthetic capture-to-draft probe with manual fixture transcription, plus bounded browser-control checks; real OCR and real-world operation remain unverified. See the [baseline and source overview](docs/LABEL_FIRST_BASELINE.md).

![ULTRON B2 patrol preview](docs/images/ultron-patrol.gif)

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

## Weekend target architecture

**Later team integration plan, not a claim of completed integration.** The current baseline is operator-driven simulation capture, starting with an overview and inspection label, followed by a local evidence-linked draft. Autonomous acquisition and the OpenAI worker are later integrations; any live API use requires separate authorization and rights/privacy review for both images and text. A deterministic supervisor owns movement and completion; a deterministic policy owns screening decisions. The seven nesseler sections are report categories, not seven defects a camera can always prove absent.

```mermaid
flowchart TB
    subgraph inputs["Inputs and execution environments"]
        SIM["ULTRON MuJoCo RGB and depth<br/>Existing kinematic workflow; oracle baseline"]
        REAL["B2 camera, depth, lidar and robot state<br/>Physical integration separately gated"]
        KIT["Organizer Unitree MuJoCo<br/>Standing and DDS interface tests only<br/>No walking, camera or lidar"]
    end

    SYNC["Timestamp, calibration and frame adapters<br/>Reject stale or invalid data"]
    MAP["Sahil: SLAM, localization and costmaps<br/>DimOS or justified alternative"]
    DET["Om: local ladder detector<br/>YOLO-World or validated ladder checkpoint"]
    MEM["Sahil: observed spatial memory<br/>Map-scoped IDs, provenance and uncertainty"]
    SUP["Vivek: deterministic mission supervisor<br/>Search, target, capture, resume, return"]
    NAV["Sahil: cancellable navigation<br/>Reachable poses, freshness and stop gates"]
    EXEC["Execution adapter<br/>Kinematic simulation OR verified B2 walking interface"]
    CAP["Vivek: evidence capture and selection<br/>Overview, label, feet, rungs and useful side views"]
    STORE["Local evidence store<br/>Image IDs, hashes, poses, timestamps and asset IDs"]
    JOB["Kaan: asynchronous inspection jobs<br/>Versioned checklist, prompts and image manifest"]
    VLM["OpenAI Responses API<br/>Same-ladder keyframes to structured proposals"]
    VAL["Kaan: schema and evidence validation<br/>Check IDs, completeness, refusal and errors"]
    RULE["Vivek: deterministic screening policy<br/>Hard fail first; unknown or unvalidated pass needs review"]
    REC["Persistent inspection history<br/>Seven-section nesseler protocol and pending human checks"]
    EVT["Vivek and Aravind: versioned event API<br/>Mission, navigation, evidence and inspection updates"]
    UI["Aravind: live inspector dashboard<br/>Map, ladder list, checklist, images and API status<br/>Rerun for spatial diagnostics"]
    HUMAN["Human inspector<br/>Resolve uncertainty and complete functional checks"]
    COV["Coverage and pending-target accounting<br/>Return with gaps or wait if base is unreachable"]

    SIM --> SYNC
    REAL --> SYNC
    SYNC --> MAP
    SYNC --> DET
    MAP --> MEM
    DET --> MEM
    MEM --> SUP
    MAP --> NAV
    SUP --> NAV
    NAV --> EXEC
    EXEC --> SYNC
    NAV -->|Arrival and capture readiness| CAP
    SUP --> CAP
    CAP --> STORE
    STORE --> JOB
    JOB --> VLM
    VLM --> VAL
    VAL --> RULE
    RULE --> REC
    REC --> MEM
    RULE -.->|Bounded evidence-gap request only| SUP
    SUP --> COV
    MEM --> COV
    COV --> SUP
    MAP --> EVT
    SUP --> EVT
    STORE --> EVT
    JOB --> EVT
    REC --> EVT
    EVT --> UI
    STORE -->|Referenced evidence| UI
    UI --> HUMAN
    HUMAN -->|Explicit reviewed completion| REC
    KIT -.->|Separate interface checks only| EXEC

    classDef existing fill:#183c32,stroke:#52b788,color:#ffffff
    classDef planned fill:#183247,stroke:#66b3ff,color:#ffffff
    classDef external fill:#43351c,stroke:#e9b44c,color:#ffffff
    class SIM,STORE existing
    class SYNC,MAP,DET,MEM,SUP,NAV,EXEC,CAP,JOB,VAL,RULE,REC,EVT,UI,COV planned
    class REAL,KIT,VLM,HUMAN external
```

Green marks the existing rendered source/evidence baseline; blue marks target integrations, even where partial modules already exist; gold marks external systems or people. The organizer simulator's dashed connection does not imply it implements high-level walking. No API model output is an actuator command. Implement the simulation and physical adapters separately, preserving their provenance.

### Inspection strategy: useful coverage before full orbit

For Om's first baseline, try YOLO-World with a saved `ladder`/`stepladder` vocabulary, then compare a ladder-trained checkpoint if available. Ultralytics documents [custom vocabulary and saving the prepared model](https://docs.ultralytics.com/models/yolo-world/); this supplies an integration route, not demonstrated ladder recall on our course. Keep whichever model meets measured recall/latency, and retain its license/version provenance.

1. **Discover and bind.** Om's detector processes sampled RGB; aligned depth plus timestamp-correct transforms locate the candidate. Sahil's memory associates it with a map-scoped ID, preserving ambiguous matches. Use the same detector adapter for rendered and real images, but evaluate those domains separately. Simulator segmentation remains labeled oracle ground truth for tests, never model performance.
2. **Approach and capture.** Start with a whole-ladder overview and an aisle-facing label close-up, then add views for unresolved visual fields as needed. Preserve uncropped originals and crop coordinates. Select sharp, sufficiently exposed, distinct views of the same ladder; nearby ladders must not leak into its evidence bundle. No minimum image count has been established.
3. **Inspect reachable surfaces.** Add one approved second look when it can resolve a specific gap. A full circle is optional, not a prerequisite. The current experimental full-ring path is simulation-only for an explicitly open A-frame with missing rear evidence. Wall-backed ladders, blocked paths and unknown clearances must not trigger an orbit. A fixed camera cannot promise a top-down view; do not assume a gimbal, arm or body-tilt interface. Record unseen upper/rear/underside surfaces as unknown.
4. **Queue evidence, then continue bounded work.** Kaan's worker receives the immutable image manifest and seven-section rubric. API processing must not block the motion watchdog or sensor pipeline. While analysis is pending, the supervisor may resume its configured route; a later result updates the same asset/job. Any re-observation is a separately validated goal with a budget, never a free-form VLM instruction.
5. **Validate, preserve and report.** Check every returned image/criterion ID against the request. Retain raw response, model ID, prompt/schema version, timestamps and usage. Fuse evidence without clearing earlier hard failures. Weak/conflicting/unseen evidence, unreadable dates, API failure or a model-only clean result require human review. Return to base with any pending/gap counts visible; navigation completion and analysis completion are separate states.

### Seven-section rubric for the API worker

An authorized local reviewer should compare independently authored criteria against the source; a single coarse PASS per section is insufficient. Do not publish or externally transmit reference-derived form text without separate authorization and rights/privacy review. Output literal observations with evidence image IDs, visibility, unknowns, raw model confidence and reasons. Confidence is uncalibrated and is not a probability of safe use. Human-only criteria remain pending even when the visible surface looks intact.

| Proposed review area | Image-based proposals to request | What remains human/unknown without other evidence |
|---|---|---|
| Rails/stiles | Visible bends, cracks, corrosion, splinters and wear | Hidden damage, sharpness by touch, structural strength, wood treatment when not observable |
| Rungs/steps/platform | Missing/deformed rungs, visible joints, wear and contamination | Joint tightness, play and load-bearing function |
| Spreader | Presence, visible detachment/damage and observed configuration | Locking/security and functional operation |
| Fittings | Missing hardware, visible damage/rust and accessible connections | Fastener security, mechanism function and lubrication adequacy |
| Feet/rollers | Missing/damaged caps, visible support contact and rollers | Grip, brake function and stability under load |
| Accessories | Visible presence/damage of supports, extensions and stabilizers | Required applicability, secure attachment and function |
| Markings | Manufacturer/instructions/pictograms; transcribed label text and ambiguous dates | Qualified inspection/sign-off and unreadable/unverified label semantics |

Track inspection-sticker status and deployment hazards (pallet support, leaning inclination) separately from those seven sections. Geometry needs calibrated depth/reference planes and uncertainty; VLM angle estimates alone are not a measured inclination. Do not infer next-inspection dates from sticker colour or a universal annual expiry. Extract readable text first, then apply explicit verified date semantics in code.

A future prompt package should use independently authored, rights-reviewed criteria and approved applicability guidance and examples, plus unknown/human-only rules and the requested JSON schema. The local reference-derived catalog is not approved for publication or external transmission. This is prompt context, not fine-tuning. Treat text printed in images as scene data, never instructions. Proposed criterion states: `DEFECT_OBSERVED`, `NO_VISIBLE_DEFECT`, `UNKNOWN`, `NOT_APPLICABLE_PROPOSED`; applicability proposals require validation. A new adapter must map these into existing policy without treating `NO_VISIBLE_DEFECT` as certification. All seven sections remain present even if most criteria need review.

### OpenAI API and budget plan

Use a **server-side OpenAI API worker**, not the ChatGPT website or a browser-exposed key. The Responses API accepts image inputs, including multiple images in one request. Use a vision-capable model available to the team's credited project and strict Structured Outputs where supported; pin the tested model ID/configuration after Kaan benchmarks correctness, latency and cost. A valid JSON schema does not guarantee correct findings. See the official [vision guide](https://developers.openai.com/api/docs/guides/images-vision?api-mode=responses) and [Structured Outputs guide](https://developers.openai.com/api/docs/guides/structured-outputs).

Start with one multi-image inspection request per ladder, with one bounded additional request for genuinely new evidence if useful. Use a configurable small worker pool, timeout/backoff, request deduplication and a persistent job queue. Cache only by the complete evidence/rubric/model context; a changed inspection cannot reuse an unrelated old result. Refusal, malformed/incomplete response, exhausted budget or network failure produces `MANUAL_REVIEW` with saved evidence, not a fabricated result. Measure P50/P95 latency and actual token usage before setting the stopped-inspection deadline.

Any project/account credit is a resource to verify, not an assumed pooled balance. Use a backend-owned environment key, never commit it or send it to the dashboard. Log estimated cost from observed usage and the verified model price; impose an application-side per-run spend/request cap and reserve credits for the final demo. No streaming every frame, no five-key rotation scheme, and no model training on the critical path. No API calls or credits have been spent by this architecture update.

## Team owners and deliverables

These assignments keep Sahil and Aravind's current work intact. Vivek owns the integration contract; each teammate supplies a runnable adapter plus a small representative fixture so other workstreams can progress independently. The paths below are proposed new boundaries, not existing services.

| Owner | Workstream and concrete output | First handoff / acceptance | Ownership boundary |
|---|---|---|---|
| **Sahil** | SLAM, localization, maps/costmaps, navigation, observed spatial memory and coverage; DimOS or justified alternative. The fetched work was on [`sahil/sim-slam-nav2`](https://github.com/VivekSai07/site-safety-patrol-ULTRON/tree/sahil/sim-slam-nav2), with its [handoff](https://github.com/VivekSai07/site-safety-patrol-ULTRON/blob/sahil/sim-slam-nav2/docs/SAHIL_HANDOFF.md). Refresh branch status before use. | Pose/map stream, observed asset records and cancellable goal adapter. Demo capture pause/resume, repeated IDs, blocked goal and return/unreachable-base outcomes. Real SLAM must be sensor-driven; disclose replay/kinematic parts. | `autonomy/` package if chosen; coordinate `navigation.py`, `memory.py` and map schema with Vivek. No inspection verdict changes. |
| **Om** | Ladder detector and evidence localization; prepare a ladder vocabulary/checkpoint, run it on synthetic and real samples, produce boxes/tracks plus depth/TF observations. Own detection dataset annotation and recall evaluation with negatives/occlusion. | First deliver saved detector config/weights provenance, annotated preview and timestamped detection fixture. Then report recall/false detections, FPS/latency and ID/depth failures on held-out physical ladder/session groups. | `perception/` detector adapter, observation schema implementation, dataset/evaluation tooling. No API inspection worker or navigation control. |
| **Kaan** | OpenAI multi-image inspection worker; versioned seven-section rubric/prompt/schema; label transcription; bounded retries/budgeting; criterion evidence and nesseler field mapping. Own inspector-reviewed assessment cases. | First deliver an inspection-job fixture and schema-valid result fixture without needing the robot. Only after separate approval and rights/privacy review, run an API smoke test and evaluate defect/unreadable/occluded cases; retain raw responses and failure-to-review results. | Proposed `inspection_api/`, prompt/schema files and checklist mapping in `nesseler.py`. Coordinate policy mapping with Vivek; no final model-only safe verdict. |
| **Vivek** | Mission orchestrator and integration lead; own shared contracts, frame-to-asset evidence bundles, capture quality/pose budgets, deterministic policy, persistent jobs/events and end-to-end runner. Coordinate hardware readiness with a named authorized operator. | First publish versioned contracts/fixtures to everyone. Wire one detection -> capture bundle -> API result -> policy -> dashboard. Then integrate Sahil's goal/status adapter, repeat patrol, cloud outage and evidence-gap scenarios. | `mission.py`, `supervisor.py`, capture/bundle code, event/worker plumbing and integration tests. Resolve shared-file changes; do not rewrite teammates' backends. |
| **Aravind** | Live dashboard with ladder/map list, navigation and coverage state, seven-section evidence cards, pending/API-failed states, human completion and protocol download. Rerun may supply spatial diagnostics; the usable inspection UI is the deliverable. | Build now against Vivek/Kaan's fixtures; display image-linked findings and live event updates. Validate that pending/unknown never looks green and that final reports survive API failure/reconnect. | Dashboard frontend and its read API/SSE or WebSocket adapter, coordinated with Vivek. Browser has no OpenAI credentials or direct robot transport. A UI stop request is not an independent E-stop. |

### Contract freeze and immediate next actions

**First integration checkpoint: one ladder, one evidence bundle, one seven-section reviewed report on the dashboard.** Mapping and navigation develop in parallel; no teammate needs to wait for the whole robot stack.

| Producer -> consumer | Contract to agree before deeper implementation |
|---|---|
| Om -> Sahil/Vivek | `DetectionObservation`: observation ID, sensor timestamp/clock/frame, bbox/class/score, source model, evidence reference; map position and uncertainty only when depth/TF are valid. A detector track ID is not a persistent asset ID. |
| Sahil -> Vivek/Aravind | `LocalizationState`, `AssetObservation`, `CoverageState`; map ID/version, validity/age and uncertainty. `NavigationGoal/Result` share goal IDs, explicit status/reason and cancellation acknowledgement. |
| Vivek -> Kaan | `InspectionJob`: job/mission/asset/map IDs, observation time, ladder-type proposal, criterion/rubric version and immutable view manifest (image ID/hash, original/crop reference, pose/TF, angle/quality where measured). One bundle identifies one target. |
| Kaan -> Vivek | `InspectionProposal`: job/asset IDs, all criterion IDs with evidence/unknowns, literal label text/date proposal, raw confidence, model/prompt/schema versions, response reference, usage and error/refusal state. No command or final safety certification. |
| Vivek -> Aravind | Ordered, replayable `MissionEvent` plus REST-style snapshots/evidence access; include event sequence, job/asset IDs, provenance and proposed states `CAPTURED`, `ANALYZING`, `FLAGGED`, `MANUAL_REVIEW`, `HUMAN_COMPLETED`. Human completion does not mean safe use; preserve the signed decision separately. |

Vivek defines/versions these shared schemas and publishes success, pending and failure fixtures first. Om and Kaan implement separate modules against them. Aravind develops without robot/API access. Sahil maintains his dedicated branch. Refresh the [branch inventory](docs/RECONCILIATION_PASS7.md) before relying on names or merge status. Agree before changing a shared schema or database; preserve evidence/history through explicit migrations.

### Integration gates through submission

| Order | Team target / acceptance evidence |
|---|---|
| **Now** | Agree contracts and fixture IDs. Om evaluates rights-reviewed sample data; Kaan produces an offline seven-section proposal fixture; Aravind renders its evidence; Vivek joins them. Reference photos are examples, not labeled defect ground truth. |
| **Baseline** | One ladder end-to-end, persistent record, human review and a cloud-failure case. Then two ladders and correct image-to-asset binding. Sahil supplies sensor/replay mapping and navigation with explicit validity/gap states. |
| **Protected demo** | Integrate bounded patrol -> discover -> approach -> capture -> report -> resume -> return. Measure detector recall against an independently enumerated course; report false-pass count, abstentions, duplicate/merged IDs, interventions, processing latency and spend. Run representative blocked/unreadable/network-failure cases. |
| **Freeze** | Team planning target: Saturday 28 Nov around 13:30 for the reliable baseline; preserve a backup hardware video by 22:00 if authorized access is available. These are planning targets, not evidence the system is ready. Drop full orbits/fine-tuning before risking the working slice. |
| **Hand-in** | Organizer material records Sunday 29 Nov noon hand-in and a hardware video up to two minutes; confirm current event instructions. Finish README/runbook, repository, pitch/demo video, one-pager and provenance-reviewed dataset package. Public data upload needs its own team/user instruction. |

Physical testing follows the separate gates below; this plan does not grant robot-operation authority. A replay/kinematic demo is the honest fallback if hardware access or validated autonomy is unavailable. The minimum useful outcome is an evidence-backed seven-section screening report with explicit human work remaining, not an autonomous “safe to use” certificate.

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
PYTHONPATH=src MUJOCO_GL=glfw python3 -m ultron demo --viewer --patrols 1
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

## Repository

```text
src/ultron/           mission, MuJoCo scene, policy, memory, planning, viewers
src/ultron/assets/   official B2 MJCF + upstream license/provenance
scripts/             ROS/RViz replay
config/              RViz configuration
src/site_safety_patrol/  earlier standard-library fixture demo, retained for reference
scenarios/           earlier fixture scenarios
tests/               pure policy/planning/history tests + opt-in rendered workflow
```

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

## Verify

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
PYTHONPATH=src MUJOCO_GL=egl ULTRON_RENDER_TESTS=1 python3 -m unittest discover -s tests -v
```

The second command includes actual camera rendering, two-patrol status changes, recovery, missing expected assets, and blocked approach behavior. Synthetic results are not real-hardware success metrics.
