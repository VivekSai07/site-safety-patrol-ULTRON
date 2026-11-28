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
| `detect.py` | `YoloDetector` (open-vocabulary YOLOE with tracking and masks), `VlmDetector` (vision model returns boxes), `ReplayDetector` | YOLO works on the real recording; VLM detector tested with fakes only |
| `lift.py` | Detection → map position from lidar, depth image or assumed range | depth path run on the real recording; **assumes the camera looks along the robot's +x from its origin** |
| `objects.py` | Object map: associate sightings by track and position, merge fragments, stable ids, saved every change | works on the real recording |
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
| `pipeline/detector` | More recordings: first-aid kits, ladders side by side, other lighting; re-fit thresholds and size ranges | `detect.py`, `catalog.py` |
| `pipeline/sensors` | Live topics as in the recording (the ROS 2 source does not read depth yet), pose from SLAM instead of offline odometry, camera offset from the robot origin | `protocol/sources.py`, `protocol/bag.py`, `lift.py` |
| `pipeline/keyframes` | Better frame choice: which ladder part is in view, occlusion, DAAAM-style joint selection | `keyframes.py`, `coverage.py` |
| `pipeline/navigation` | `is_free` from the SLAM occupancy map, goals through Nav2, check on arrival that the object is in view | `navigate.py`, `inspection.py` |
| `pipeline/forms` | Prompts tuned on real photos: ladder rows per section with the matching keyframes, fire and first-aid rows | `protocol/vlm.py`, `checklists.py`, `inspection.py` |
| `pipeline/dashboard` | Show non-ladder objects, checklist results and view requests; today only ladders reach the dashboard | `protocol/publish.py`, `dashboard/` |
| `pipeline/stages` | One process that switches between survey and inspection on a trigger, instead of two commands | `__main__.py` |

## On the real robot's recording

`ws:~/data/b2_real/rosbag2_2026_11_28-00_48_42` (36 s, 17 m loop in the hall; RealSense head camera
colour 1280×720 and depth 848×480, lidar cloud at 10 Hz, IMU; no pose and no camera-to-lidar
transform recorded). Read without ROS by `protocol/bag.py`:

```bash
pip install -e '.[protocol,bags,detector]'    # bag reader, lidar odometry, YOLOE (GPU)
python -m ultron.pipeline survey --source bag:BAG,rate=8 --calibration bag:BAG --detector yolo --interval 0
```

Result: 5 objects, each a real one with the right kind: the A-frame ladder (84 sightings, seen
from two sides), the leaning ladder (48), the fire extinguisher (19), the red hose cabinet (32) and
the red alarm box (28). A second extinguisher was in view for one frame only and is not kept. The
whole pass takes about 30 s on an RTX 5090.

What it took, in the order the recording forced it:

- **Depth** is re-projected into the colour camera with the recorded intrinsics and extrinsics.
- **Pose** comes from lidar odometry over the scans: the loop closes to within 15 cm and its heading
  agrees with the IMU. The camera is assumed to look along the lidar's +x from the lidar's position.
- **Detector**: YOLOE (`yoloe-11l-seg`), 0.1 s per frame. Compared on twelve frames with Grounding
  DINO (slower, muddled labels, confident false hits) and OWLv2 (finds everything but floods ladder
  parts and doors). YOLOE's misses were prompt wording, see `catalog.py`.
- **Tracking**: YOLOE's tracker ids keep an object's identity from frame to frame; feed it several
  frames per second. Two boxes in one frame are always two objects.
- **Range from the mask**, not the box: a ladder's box is mostly background.
- **Size check**: each kind has a plausible height. From 4 m the alarm box scored 0.6 as an
  extinguisher; at 0.36 m tall it cannot be one.
- **Merging fragments**: an object seen far less often than a same-kind neighbour within reach is
  folded into it (half-ladder boxes, a cabinet seen twice at slightly different ranges).

Limits of this evidence: one recording, one hall. Thresholds and size ranges are fitted to it. The
two ladders stand 3 m apart; two ladders side by side rely on tracking and on being seen in one
frame at least once, which is tested with synthetic data only.

## Not yet verified

- The inspection stage has not run on real data: no vision-model call has been made (no API key
  where this was built), so form reading quality is unknown.
- Nav2 goals, the live ROS 2 source and the Unitree source have not been exercised.
- Viewing directions are map bearings in 45° steps; the pipeline does not know a ladder's front.
- An object seen only without range gets an assumed range of 2.5 m and is marked approximate.
- Without poses, objects are split by a 6 s gap in time, and no view can be requested.
- The dashboard is optional: without `--dashboard` nothing is posted and nothing from
  `ultron.dashboard` is imported.

Tests: `PYTHONPATH=src python -m unittest tests.test_pipeline`
