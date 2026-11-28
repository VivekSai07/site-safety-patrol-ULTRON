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
| `detect.py` | `VlmDetector` (vision model returns boxes), `YoloDetector` (open-vocabulary YOLOE), `ReplayDetector` | YOLO run on the real recording; VLM detector tested with fakes only |
| `lift.py` | Detection → map position from lidar, depth image or assumed range | depth path run on the real recording; **assumes the camera looks along the robot's +x from its origin** |
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
| `pipeline/sensors` | Live topics as in the recording (the ROS 2 source does not read depth yet), pose from SLAM instead of offline odometry, camera offset from the robot origin | `protocol/sources.py`, `protocol/bag.py`, `lift.py` |
| `pipeline/keyframes` | Better frame choice: which ladder part is in view, occlusion, DAAAM-style joint selection | `keyframes.py`, `coverage.py` |
| `pipeline/navigation` | `is_free` from the SLAM occupancy map, goals through Nav2, check on arrival that the object is in view | `navigate.py`, `inspection.py` |
| `pipeline/forms` | Prompts tuned on real photos: ladder rows per section with the matching keyframes, fire and first-aid rows | `protocol/vlm.py`, `checklists.py`, `inspection.py` |
| `pipeline/dashboard` | Show non-ladder objects, checklist results and view requests; today only ladders reach the dashboard | `protocol/publish.py`, `dashboard/` |
| `pipeline/stages` | One process that switches between survey and inspection on a trigger, instead of two commands | `__main__.py` |

## First run on the real robot's recording

`ws:~/data/b2_real/rosbag2_2026_11_28-00_48_42` (36 s, 17 m loop in the hall; RealSense head camera
colour 1280×720 and depth 848×480, lidar cloud at 10 Hz, IMU; no pose and no camera-to-lidar
transform recorded). Read without ROS by `protocol/bag.py`:

```bash
pip install rosbags kiss-icp ultralytics      # bag reader, lidar odometry, YOLOE (GPU)
python -m ultron.pipeline survey --source bag:BAG,rate=3 --calibration bag:BAG --detector yolo --interval 0
```

- Depth is re-projected into the colour camera with the recorded intrinsics and extrinsics.
- Poses come from lidar odometry over the scans: the loop closes to within 15 cm and its heading
  agrees with the IMU. The camera is assumed to look along the lidar's +x from the lidar's position.
- YOLOE (`yoloe-11l-seg`, threshold 0.5) on an RTX 5090: 109 frames in under 30 s including model
  load. It finds the three ladders and the fire extinguisher in view; false hits scored below 0.4.
- Result: 7 objects for 4 real ones in view. Clean: the leaning ladder, the extension ladder, the
  extinguisher. Wrong: the stepladder's object also holds two frames of the extension ladder
  standing 0.5 m from it, and there are two spurious two-sighting objects (one ladder fragment, one
  repeat of the extinguisher). Position alone cannot separate objects that close: that needs
  tracking or appearance, the `pipeline/detector` package.
- "fire hydrant" and "fire hose cabinet" prompts did not reach 0.5 on the red wall boxes.

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
