# Site patrol pipeline (`ultron.pipeline`)

A skeleton to branch from. Two stages share one workspace folder on disk:

```
SURVEY      frame + pose ─▶ detect ─▶ lift to map ─▶ object map ─▶ keyframes
            (robot walks its route; everything is saved after every frame)

INSPECTION  for each found object:
            missing views? ─▶ navigator ─▶ capture ─▶ pick keyframes ─▶ read form ─▶ report
```

It takes the ideas that fit from two papers and leaves their systems out: map-anchored objects
with their keyframes (STaR), and choosing frames per object by what they show instead of by time
(DAAAM). Unlike both, it only looks for the kinds listed in `catalog.py`, and all state is plain
JSON and JPEG that is written as it goes, so pause, resume and snapshot need no extra code.

## Run

```bash
pip install -e '.[protocol,dashboard]'
export OPENAI_API_KEY=...

# stage 1: Ctrl-C whenever; run it again to continue the same map
python -m ultron.pipeline survey --source ros2:/IMAGE,pose=/ODOM,cloud=/POINTS --calibration calibration.json \
    --mission PATROL-1 --dashboard http://127.0.0.1:8000
# what was found
python -m ultron.pipeline status
# stage 2: passive (no motion), operator (a person drives), or dashboard (goal -> bridge -> Nav2)
python -m ultron.pipeline inspect --navigator dashboard --camera ros2:/IMAGE,pose=/ODOM,cloud=/POINTS \
    --calibration calibration.json --mission PATROL-1 --dashboard http://127.0.0.1:8000
```

Workspace (`--data`, default `runs/pipeline`): `objects.json`, `keyframes.json`, `keyframes/*.jpg`,
`ladders/<id>/` (Kontrollblatt sheets and evidence), `objects/<id>/` (other checklists), `events.jsonl`.

## Modules

| File | Does | State |
|---|---|---|
| `catalog.py` | Kinds to look for, detector prompts, views and sides each form needs | works; values are first guesses |
| `types.py` | Records passed between modules | works |
| `detect.py` | `VlmDetector` (vision model returns boxes), `YoloDetector` (open-vocabulary YOLO), `ReplayDetector` | VLM and replay tested with fakes; **YOLO never run** |
| `lift.py` | Detection → map position from lidar, depth image or assumed range | tested on synthetic data; **assumes the camera looks along the robot's +x from its origin** |
| `objects.py` | Object map: associate sightings, stable ids, saved every change | works |
| `keyframes.py` | Bounded keyframe store per object × view range × viewing direction; selection for the reader | works; quality score is a simple heuristic |
| `coverage.py` | Missing views → goal poses around the object | works; no occlusion or part visibility |
| `navigate.py` | `Passive`, `Operator`, `DashboardNavigator` (dashboard `/api/goal` → bridge → Nav2) | tested against the dashboard app; **never run with Nav2** |
| `survey.py` | Stage 1 loop | works |
| `inspection.py` | Stage 2 loop; ladders → `ultron.protocol` Kontrollblatt → dashboard; other kinds → checklist JSON | works with fake models |
| `checklists.py` | Fire and first-aid rows quoted from nesseler's form, prompt, answer rules | rules tested; **prompt never run on a model** |

`ultron.protocol` (the earlier reader) supplies frames, sources, the vision client, the ladder
form and rules, the ladder store and the dashboard publisher.

## Interfaces to build against

- **Detector**: `detect(frame) -> list[Detection(kind, box, score)]`, box in thousandths of the image.
- **Navigator**: `go_to(ViewRequest) -> bool`, True once the robot stands at `x, y, yaw` (map frame).
- **Occupancy**: `is_free(x, y) -> bool`, passed to `Inspection`; nothing provides it yet.
- **Camera**: `camera() -> Frame` with `pose` and `points` or `depth` filled.
- **Events** (`events.jsonl`, also live to subscribers): `object_found`, `look`, `view_requested`,
  `view_failed`, `inspecting`, `protocol` (ladder done), `checklist` (other object done), `error`.

## Work packages

Each is one branch off `feature/site-pipeline` and touches the listed files only.

| Branch | Goal | Files |
|---|---|---|
| `pipeline/detector` | A detector that finds ladders, extinguishers, hydrants and first-aid kits in the hall: YOLO weights and prompts, thresholds, tracking between frames | `detect.py`, `catalog.py` |
| `pipeline/sensors` | Real topics and calibration: depth image in the ROS 2 source (not read yet), camera extrinsics in `lift.py`, a measured `calibration.json` | `protocol/sources.py`, `lift.py`, `protocol/lidar.py` |
| `pipeline/keyframes` | Better frame choice: which ladder part is in view, occlusion, DAAAM-style joint selection | `keyframes.py`, `coverage.py` |
| `pipeline/navigation` | `is_free` from the SLAM occupancy map, goals through Nav2, check on arrival that the object is in view | `navigate.py`, `inspection.py` |
| `pipeline/forms` | Prompts tuned on real photos: ladder rows per section with the matching keyframes, fire and first-aid rows | `protocol/vlm.py`, `checklists.py`, `inspection.py` |
| `pipeline/dashboard` | Show non-ladder objects, checklist results and view requests; today only ladders reach the dashboard | `protocol/publish.py`, `dashboard/` |
| `pipeline/stages` | One process that switches between survey and inspection on a trigger, instead of two commands | `__main__.py` |

## Not yet verified

- No real model, detector, robot topic or Nav2 goal has been exercised. Tests use fake detectors,
  fake model answers and synthetic poses, depth and lidar.
- Viewing directions are map bearings in 45° steps; the pipeline does not know a ladder's front.
- An object seen only without range gets an assumed range of 2.5 m and is marked approximate.
- Without poses, objects are split by a 6 s gap in time, and no view can be requested.

Tests: `PYTHONPATH=src python -m unittest tests.test_pipeline`
