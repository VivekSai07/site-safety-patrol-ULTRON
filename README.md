# ULTRON · Site Safety Patrol

**A lightweight Unitree B2 MuJoCo simulation for the nesseler challenge.** A robot model patrols a bounded construction scene, discovers ladders, approaches for evidence, requests one second look, and remembers ladder identities across patrols. Actual simulated RGB/depth frames become a traceable inspection report. Rerun export and ROS 2 Humble/RViz replay consume the same recording.

![ULTRON B2 patrol preview](docs/images/ultron-patrol.gif)

This is ULTRON's team-side working repository. The organizer starter remains the integration authority for real hardware. Nothing contacts organizers or sends robot commands.

## Run on this machine

The existing `python3` environment already has MuJoCo, NumPy and Pillow:

```bash
cd /home/vivek/ULTRON
PYTHONPATH=src MUJOCO_GL=egl python3 -m ultron demo --out runs/my-demo
```

Omit `--out` for a unique run folder. Existing folders are refused. Default: two patrols with acceleration-limited motion; simulated duration follows the route and turn profile, and rendering speed depends on the machine. Run data is roughly 200–300 MB because depth arrays are preserved.

A generated example lives in `runs/ultron-demo/`. Open `report.html` directly, or serve it:

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

The local `.venv` created during preparation is empty: package downloads failed on DNS. Use the already installed `python3` command above here, or install extras when networking is available.

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

**Verification limits here:** headless EGL rendering and SQLite workflows were executed. Rerun/DimOS SDKs are absent and cannot be downloaded in this restricted shell. Their adapters are based on inspected upstream APIs but remain unverified end to end. ROS Humble/RViz are installed; ROS node creation and interactive windows are blocked by session socket/display restrictions. Portable recordings remain available for all viewers.

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

Start with [source review](docs/SOURCE_REVIEW.md), [jury strategy](docs/JURY_STRATEGY.md), [B2/DimOS integration](docs/INTEGRATION.md), [handoff](docs/EVENT_HANDOFF.md) and [pitch/video plan](docs/PITCH.md). Organizer and teammate PDFs are preserved locally in `docs/` and ignored by Git; these notes identify exact pages. No dataset or repository has been uploaded.

## Verify

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
PYTHONPATH=src MUJOCO_GL=egl ULTRON_RENDER_TESTS=1 python3 -m unittest discover -s tests -v
```

The second command includes actual camera rendering, two-patrol status changes, recovery, missing expected assets, and blocked approach behavior. Synthetic results are not real-hardware success metrics.
