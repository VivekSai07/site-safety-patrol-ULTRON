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

## One system: robot → relay → patrol → dashboard

```
robot (ROS 2)                         GPU machine                          anywhere
scripts/ros_relay.py  ──TCP 5600──▶  python -m ultron.pipeline patrol ──HTTP──▶ python -m ultron.dashboard
 camera, depth, TF pose,              survey → check → inspect             mission, live view,
 lidar if TF has no pose              (YOLOE, vision model)                ladder records, events
```

```bash
# GPU machine
python -m ultron.dashboard --port 8010 &
python -m ultron.pipeline patrol --source relay:5600 --detector yolo --interval 0 \
    --mission HALL-1 --site ctrl-hall --dashboard http://127.0.0.1:8010
# robot, in a shell with ROS sourced (needs only rclpy, tf2_ros, numpy)
python3 scripts/ros_relay.py --to GPU_MACHINE:5600
```

Add `--record DIR` to the relay to also keep a ROS 2 bag of the run on the robot (camera,
depth, lidar, TF, IMU; about 7 GB a minute). Costs and options: [RECORDING.md](RECORDING.md).

The survey runs until the relay stops, `--survey-seconds` pass or Ctrl-C is pressed once; then
the objects are checked and inspected and the mission is closed. `--navigator` chooses how missing
views are reached: `passive` (not at all), `operator` (a person drives), `dashboard` (goal through
the dashboard bridge to Nav2) or `contract` (`ultron.autonomy` with `--map DIR`).

Pose: the relay sends the robot's pose from TF (`map` ← `base_link`) when localisation publishes
it, so objects land in the SLAM map and view goals are in the frame Nav2 expects. Without it the
pipeline computes lidar odometry from the forwarded clouds, in a frame of its own.

Tested by replaying the hall recording as ROS topics in a container on the workstation
(`ros2 bag play` + the relay): 36 s of stream, live odometry, 6 objects, 5 confirmed by the vision
model, both ladders on the dashboard, the other objects in its event list. Not tested: the real
robot, a TF pose from running localisation, and any navigator other than `passive`.

Also tested with relay and pipeline together on a stand-in for the robot computer (ROS 2 Humble,
Python 3.10, network off; `scripts/b2_standin/`, steps in [RECORD_ON_B2.md](../RECORD_ON_B2.md) §6):
5 objects, 5 found; a second patrol on the same map id kept the 5 ids, lined up by 0.22 m and
marked nothing moved; the sheets were read afterwards from the saved keyframes on another machine.

## Run the stages separately

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

With a map id (`--map-id ID`, or `--map DIR` of a saved `ultron.autonomy` map, keyed `ID-vVERSION`)
`objects.json` and the keyframes live in `--sites/<site>/<map id>/` (default `runs/sites`) and are
shared by every patrol on that map; events and ladder sheets stay in `--data`. Give a map id only
when poses are in that map's frame (localisation): lidar odometry starts a new frame each run, and a
survey on a shared map warns (`localisation_unconfirmed`) when its frames say their pose is odometry.
Each patrol inspects again the objects it saw itself, even if an earlier mission inspected them; an
object it did not see is not re-inspected from old keyframes. Lining up with the stored objects
(`anchor.py`): one landmark decides nothing (the robot may be off, or the object moved); two that
disagree wait for a third; of three or more, one lying over 1 m from where the agreeing others put it
was moved (`object_moved` with `seen_at`). Until the patrol is lined up nothing is announced, not
even stored objects it re-sees (one may be taken for another of its kind nearby); a patrol that
never sees two stored objects reports when its survey settles. Objects created before then are
discarded if it turns out suspect, and a suspect patrol announces nothing, so it re-inspects
nothing an earlier mission inspected (it still inspects stored objects never inspected).

## Modules

| File | Does | State |
|---|---|---|
| `catalog.py` | Kinds to look for, detector prompts, views and sides each form needs | works; values are first guesses |
| `types.py` | Records passed between modules | works |
| `detect.py` | `YoloDetector` (open-vocabulary YOLOE with tracking and masks; drops detections `lift.doubt` finds on or through glass; extra classes from `catalog.ALSO`; input size option), `VlmDetector` (vision model returns boxes), `ReplayDetector` | YOLO works on the real recording; VLM detector tested with fakes only |
| `lift.py` | Detection → map position from lidar, depth image or assumed range; floor plane removed; ladders lying on the floor (`flat_on_floor`, `lying`); `doubt` (no depth in the outline, or seen over a sill) | depth path run on the real recording and the simulated yard; lying and glass tests rendered only; **assumes the camera looks along the robot's +x from its origin** |
| `discover.py` | Lidar candidates for the camera: upright ladder-sized blobs, and low long blobs (lying ladders) | hall recording (`bag:...,lidar=1`), sim yard, rendered sweeps |
| `objects.py` | Object map: associate sightings by track, then by floor footprint (radius without one), merge fragments, stable ids, saved every change | works on the real recording |
| `anchor.py` | Lines a patrol up with the objects stored by earlier ones on the same map; gross-error guard | replay of the real recording with injected pose errors; synthetic |
| `moved.py` | Looks through a stored object's footprint with the depth image: still there, hidden, or gone; a verdict means the place was in clear view (the survey also counts detections in such views to judge wall-backed objects) | real recording (no false "moved"; planted empty spots and dropped detections found); simulated yard; synthetic |
| `keyframes.py` | Bounded keyframe store per object × view range × viewing direction; selection for the reader | works; quality score is a simple heuristic |
| `coverage.py` | Missing views → goal poses around the object | works; no occlusion or part visibility |
| `navigate.py` | `Passive`, `Operator`, `DashboardNavigator` (dashboard `/api/goal` → bridge → Nav2) | tested against the dashboard app; **never run with Nav2** |
| `survey.py` | Stage 1 loop | works |
| `inspection.py` | Stage 2 loop; ladders → `ultron.protocol` Kontrollblatt → dashboard; other kinds → checklist JSON | run on the real recording, passive (no navigation) |
| `checklists.py` | Fire, alarm and first-aid rows quoted from nesseler's form, prompt, answer rules | run on the real recording with Qwen3-VL |
| `calibrate.py` | Chooses detector prompt and threshold per kind, judged by the vision model | run on the real recording |
| `verify.py` | Vision model confirms, relabels or rejects each object | run on the real recording |

`ultron.protocol` (the earlier reader) supplies frames, sources, the vision client, the ladder
form and rules, the ladder store and the dashboard publisher.

## Interfaces to build against

- **Detector**: `detect(frame) -> list[Detection(kind, box, score)]`, box in thousandths of the image.
- **Navigator**: `go_to(ViewRequest) -> bool`, True once the robot stands at `x, y, yaw` (map frame).
- **Occupancy**: `is_free(x, y) -> bool`, passed to `Inspection`; nothing provides it yet.
- **Camera**: `camera() -> Frame` with `pose` and `points` or `depth` filled.
- **Events** (`events.jsonl`, also live to subscribers): `object_found` (once seen 3 times in this
  patrol and not a likely fragment), `object_merged`, `object_moved`, `object_back`,
  `localisation_anchored`, `localisation_suspect` (gross error: the patrol changes nothing more),
  `localisation_unconfirmed` (shared map, but poses from odometry), `objects_discarded`, `look`, `view_requested`, `view_failed`,
  `inspecting`, `protocol` (ladder done), `checklist` (other object done), `error`. The dashboard
  hears about an object only after `object_found`; fragments never reach it.

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
- **Footprints, not points**: the measured point of a 1.5 m ladder is its visible face, which moved
  0.7 m between a side and a front view and split it. Each sighting now projects the depth pixels in
  its mask onto a 10 cm floor grid (gated to the kind's extent around the measured range, and per
  image column to the nearest surface, so the wall seen between rungs is dropped); objects keep the
  cells seen twice (and in 5 % of its sightings, so stray cells do not pile up over many patrols)
  and a sighting joins the footprint it touches within 0.4 m. Measured on the
  recording: the two passes of one wall box lie up to 0.32 m apart, the two ladders 1.6 m apart edge
  to edge. The survey ends with the five objects at 4, 6, 8, 12 and 16 frames per second (radius
  matching ended with six at four of those rates); remaining part boxes are merged before anyone
  hears of them. Positions are the footprint's middle.
- **Pose error between passes**: one wall box measured on the two passes of the loop lies 0.3-0.6 m
  apart (the extinguisher 0.2 m). This is the noise floor for lining up later patrols.

Limits of this evidence: one recording, one hall. Thresholds and size ranges are fitted to it. The
two ladders stand 3 m apart; two ladders side by side rely on tracking and on being seen in one
frame at least once, which is tested with synthetic data only.

## On simulated patrols (ladderworld, sim evidence)

Three patrols of one simulated ladder yard (Isaac Sim, the B2's sensor rig at its real mounts;
51 ladders, scaffolds, containers): A from the south end, B from the north end, C from the south
with the A-frame L49 moved 2 m. Read by `protocol/sim.py`; the poses are the simulator's, so all
three are in one map frame:

```bash
python -m ultron.pipeline survey --source sim:PATROL,rate=5,depth=noisy --calibration sim:PATROL --detector yolo \
    --interval 0 --map-id s3h --site yard        # error=DEG/DX/DY on the source misplaces every pose
```

Result, A then B then C on one map id, scored against the yard's ground truth:
- A: 23 objects: 22 real ladders, each once, each inside the ladder's footprint (the scorer cannot
  see errors smaller than that); one rolling scaffold taken for a ladder (detector).
- B: lined up with A's objects (shift 0.01 m); every object A found keeps its id; 7 ladders only
  seen from the north are added; no duplicates.
- C: lined up; L49's old place is seen empty from three places and reported `object_moved`; the
  ladder at its new place is a new object inside its new footprint. Nothing else changes.
- With every pose of B misplaced: 0.4 m and 2 degrees (about the yard's corner) lines up and keeps
  every id, but the fit stays a shift: stored objects move up to 0.24 m (0.06 m without the error)
  and ladders B adds in the north lie 0.3-0.4 m further off. 1.5 m, 5 degrees, and 3 degrees with
  0.6 m are suspect: nothing is announced, kept or re-inspected.
- The first-aid point is never detected: YOLOE gives "first aid kit" nothing on a clear view of
  it, even at score 0.05 ("first aid sign" scores 0.33). Prompt calibration, not association.

What the open yard forced (the hall recording had none of it):
- **Range from the nearest dense depth peak that spans the outline**, not a percentile: seen 10 m
  away a ladder is a quarter of its outline, the rest background between the rails, and the 35th
  percentile fell behind it (the footprint then lay up to 4.5 m beyond the ladder and split it).
  A small object in front, caught by a sloppy outline, does not span it and is passed over.
- **The floor is removed**: a plane fitted to the lower half of each depth image (camera 0.87 m
  above it in the simulator; on the hall recording found in all 265 frames at rate 8, camera
  0.74-0.79 m above it). Masks that take in an object's shadow otherwise put it on the floor
  in front, and a stored height band that reaches below the floor made the empty floor where a
  moved ladder stood look occupied.
- Patrol D (north start, noisy depth, on the A+B map; L10 removed, L49 moved 4 m, two identical
  stepladders 0.3 m apart edge to edge): lined up (0.02 m); L49's old place reported moved, the
  ladder at its new place a new object; the twins two objects, each on the right one; no
  duplicates. L10's removal is not reported: D passed no nearer than 5.3 m.
- Patrol D2 on D's map (the twins re-seen at 3.3 m, together and L52 alone; a stop facing L10's
  empty place): seen together from 5-8 m in D, each twin's footprint had picked up a thin layer
  of the other's cells, so every later sighting touched both and was left unassigned. A sighting
  touching two objects known to be different now goes to the one whose footprint holds its cells
  clearly more strongly (counts, not just presence). D2 then re-observes both twins, each on the
  right one (23 -> 143 and 29 -> 151 sightings), and makes no third. The detector found L52 alone
  in only 1 of 57 frames.
- **Gone against a wall cannot be looked through** (measured at D2's stop: 29 "occupied", no
  "empty"): the look-through test sees an empty place only in front of open space. Where L10
  leaned, the container wall stands at the footprint's distance and reads as "something there". The
  same holds for every wall-mounted kind (extinguisher, hose cabinet, alarm box).
- **So gone is also "reliably found, now missed in clear view"** (`survey.py`). Every object counts,
  per viewing side (45 degree steps), the looks that had its place in clear view (footprint inside
  the image at 1-4.5 m, not hidden: `look_through` gives a verdict) and those that detected it. A
  look is what the robot sees until it moves 0.2 m or turns 10 degrees: frames from one spot count
  once, so the counts barely depend on the frame rate (hall recording, rates 4-16: the ladders and
  the extinguisher qualify, the hose cabinet never). A side counts once the object was found in at
  least 40 % of at least 5 clear looks from it. A
  lined-up patrol that never detects a stored object, but has its place in clear view from such
  sides in distinct looks from two places 0.5 m apart, until missing a present object every time would be under 2 % likely by those
  sides' rates (at least 4 looks), marks it moved (`object_moved`, `reason: "not detected in clear
  view"`). It is judged when the survey settles, after fragments are merged, so it does not flip
  during the survey (detected during inspection later, it is `object_back`). `status` lists per object the sides it can be judged from (`gone_if_missed_from`).
  - Why per side: in the simulated yard the leaning ladders L10 and L11 were found in 42/42 and 49/49
    clear views from one side and 13/67 and 0/33 from another; with one overall rate, present L10
    went unfound for 9 looks from 3 places (C) and L11 for 6 (D): that would have been a false
    "moved". Per side, present objects' longest such run is 1 look on C, D and D2 and 2 on the
    hall recording (no verdict). D2's stop
    faces L10's empty place from its poorly detected side: no verdict there either.
  - Why at settle: on the hall recording at 16 frames a second the A-frame's sightings went to a
    fragment for 14 looks; the fragment joins it only when the survey settles.
  - Real recording, run 2 on run 1's map with one object's detections dropped (the object stays in
    the depth image, as a wall would): LAD-001 (9 looks), LAD-002 (16), FEX-001 (5), ALM-001 (7)
    reported moved; HYD-001 (found in 2 of 7 clear views) is not judged. Unchanged run 2 at rates 4,
    8 and 16: nothing moved. Sim C, D, D2 (noisy): only L49, by look-through.
  - Real recording split in its two passes (map from pass 1, pass 2 as the next patrol, rates 8 and
    16): no false "moved", the four objects pass 2 sees keep their ids; LAD-002 with its detections
    dropped is not judged (pass 2 sees it only from a side pass 1 did not find it reliably from).
    One pass of a patrol is too little record: the check stays silent rather than guess.
  - Patrol 1 must run on this code (`views`/`sides` are new; older maps have none) and see the object
    within 1-4.5 m from the sides the next patrol will. At 2 frames a second the extinguisher gets
    only 2 sightings and is not announced: run at 4 or more.
  - Its limit: an object present that the detector stops finding from its good sides (new lighting,
    a cover) is reported moved, and back when found again in a later patrol.
- **Sightings beyond 12 m are not used.** With the D457's noise (`depth=noisy`, fitted by the
  simulator to the real camera: 6 mm at 2 m, 97 mm at 8 m) ladders seen up to 12.5 m still land
  inside their footprint (90th percentile); from 15 m on they lie 0.25-0.9 m off, a ladder 40 m
  away became a stray object and one seen from 20 m was split. Half the yard's sightings were that
  far. With the limit, noisy and noise-free depth give the same result: A keeps the 14 ladders it
  passes within 12 m, each inside its footprint; B and C as above. The figures above (23 objects,
  7 added by B) are from before the limit.

## Hardening for the demo room (lying ladders, glass)

The demo room is unseen apart from 7 phone photos (about 10 x 8 m, dark carpet, one long wall of
interior glass above a radiator band that mirrors the ladders, a wooden extension ladder lying on
the carpet, aluminium A-frames, a stepladder, a step stool). Shots to record:
[REAL_DATA_SHOTS.md](REAL_DATA_SHOTS.md).

**Ladders lying on the floor** (`lift.flat_on_floor`). Before, a lying ladder was dropped at every
step: the 8 cm floor band removed most of its pixels, its foreshortened box failed the size check
(0.5-8 m), and per image column only its near 25 cm survived. Now, for kinds marked `lying` in the
catalog (ladders), an outline below the horizon with at most 10 % of its measured pixels higher
than 0.4 m over the floor plane lies flat. Its footprint is where its rays meet a plane 5 cm over
the floor, scaled to the measured depth (one scale per outline: floor depth is too noisy per pixel
at range) and restricted to pixels at least 4 cm over the floor when there are enough of them.
The sighting has no height, so the size check passes it, and `lying=True`; its zband is floor to
measured top, so `lift.lying(site_object)` tells from `objects.json` alone.
- Floor about its fitted plane on the hall recording (71 frames): sd 0.9 cm at 1-2 m, 1.4 cm at
  2-3 m, 2.6 cm at 4-6 m. The rendered tests use depth noise that wide (and the simulator's).
- Rendered tests: 3-4.5 m ladders 7 cm thick, side-on at 1.5-6 m, end-on at 2.5-6 m, at 45 and 60
  degrees: found lying, over 90 % of the visible footprint covered. The footprint reaches past the
  far edge by up to the floor an outline takes in beyond it (up to 0.5 m at 6 m), the safe side
  for an obstacle. A survey of five such frames makes one ladder object, marked lying.
- Hall recording at rate 8: the same 5 objects as before at the same places; 0 of 228 sightings
  lying.
- Without a floor plane (no depth in the image's lower half: dark carpet, poor light) nothing is
  called lying, and a lying ladder takes the old path and fails the size check. Untested on carpet.
- Not done in this branch (files owned elsewhere): `objects.py` does not keep `Sighting.lying` (the
  zband does); `survey.py` gives the vision model the box height of a keyframe (about 0.1 m for a
  lying ladder), which may make it reject the object.
- **At the defaults nothing finds a lying ladder on the robot yet**: the detector runs at 640 (the
  lying prompt is inert there), lidar candidates are opt-in (`--candidates lidar`), and the relay
  does not forward the cloud. Recommended for the demo until it is recorded: keep 640 (1280 finds
  the lying ladder in photos but splits objects on the hall recording); forward the cloud from the
  relay and run `--candidates lidar` so lying candidates exist; have the navigation stack treat a
  lying candidate's cells as an obstacle (`Candidate.cells`) even if the camera never confirms it.

**Can the camera see one?** On the 7 demo-room photos (phone, 1.5 m up) YOLOE never boxed the
lying wooden ladder with "ladder", "wooden ladder" or any other ladder prompt at its usual input
size 640 (at most 0.13). At 1280, "ladder lying on the floor" boxed it at 0.44, 0.44, 0.70 and
0.83 in the 4 photos with it in full view (0.12 cut off at the edge), "ladder" still never. So
`catalog.ALSO` runs that prompt beside the chosen ladder prompt (0.4), and `YoloDetector` takes
`imgsz` (`ULTRON_DETECTOR_IMGSZ`, default 640). By an estimate (no measured field of view for the phone),
the B2 at 1280 sees about as many pixels per degree as these photos at 1280 (14 against about 17),
but from half the height, so the ladder is flatter still.
On the hall recording at 1280 the prompt boxed a low paving platform as lying at 0.11-0.28 and
nothing else. But 1280 itself costs on the hall recording: in 3 of 3 surveys at 1280 (twice with
the extra prompt, once without) one ladder was split into two objects 0.3-0.45 m apart and the
vision model relabelled the hose cabinet an extinguisher; in 2 of 2 at 640 (with and without the
extra prompt) the 5 objects came out right. So 640 stays the default until the demo room is
recorded. The lidar is the other way to find a lying ladder (below, and ONLINE.md).

**Glass** (`lift.doubt`, applied in `YoloDetector`). Behind the operator's fence, which is the main
defence, a detection is dropped when:
- its outline has depth in under 15 % of its pixels (clear glass often gives none; the range would
  then fall back to an assumed 2.5 m, inside the room, where no fence catches it). Real outlines on
  the hall recording: 0.43 and up, ladders 0.68 and up; a rendered ladder in front of glass that
  gives no depth: 0.42-0.50. Neither test is applied when the whole image has depth in under half
  its pixels (sensor off, or a scene of mostly glass and dark carpet);
- it is seen over a sill: right below its outline (0.3 m at its range) at least half the measured
  pixels are an upright surface nearer by 0.5 m and 15 % of its range. A window or glass wall
  starts above a wall or radiator band; from the robot's 0.75 m a reflected ladder or one in the
  next room shows only its upper part, with the sill right below. Below a real object is floor,
  its own feet, or the wall it hangs on at its own range. Through an open door the floor runs on,
  so a door passes.

Measured: on the hall recording the depth camera reads 12-18 m through the interior window where
the wall around it is 2.5-6 m away; YOLOE boxed nothing on that glass. Every detection at score
0.1 and up at rate 4: at input 640 none of the 124 real sightings above threshold is doubted, 3
other boxes are (all below 0.14); at 1280 one of 135 real ones is (the alarm box, with the leaning
ladder standing nearer below it). On the demo-room photos the mirrored ladders are boxed by
"ladder" at 0.29-0.67 (3 of 9 over the 0.5 threshold; 0.88 at 1280), by "wooden ladder" at up
to 0.90 (8 of 10 over 0.5): choosing a material prompt makes reflections worse. These photos have no
depth, so whether the B2's depth on that glass shows holes, the room behind or the mirror depth,
and whether the sill test catches the reflections from 0.75 m, is untested: shots 2.1-2.4.
A reflection of a ladder standing within 0.5 m of the glass is not caught (its mirror image is
less than the gap behind the sill); the fence, drawn inside the glass, is.

## Tuning itself: the vision model supervises the detector

A vision-language model (Qwen3-VL 8B through Ollama, any OpenAI-compatible endpoint works) is
asked a few dozen times per patrol, never per frame.

```bash
export OPENAI_API_KEY=ollama OPENAI_BASE_URL=http://HOST:11434/v1
export ULTRON_VLM_MODEL=qwen3-vl:8b-instruct ULTRON_TRIAGE_MODEL=qwen3-vl:8b-instruct
python -m ultron.pipeline calibrate --source bag:BAG,rate=2,pose=none --detector yolo   # writes prompts.json
python -m ultron.pipeline survey  --source bag:BAG,rate=8 --calibration bag:BAG --detector yolo --interval 0
python -m ultron.pipeline inspect --calibration bag:BAG --detector yolo
```

| Step | File | What happens | On the hall recording |
|---|---|---|---|
| Prompt choice | `calibrate.py` | Every candidate prompt of a kind (listed in `catalog.py`) is run on sample frames; the model judges six of its boxes; the prompt expected to deliver the most real boxes wins | 80 s. Picked "ladder" and "red fire extinguisher", as tuned by hand before; refuted all 14 "first aid" boxes (there is no kit in the recording) |
| Object check | `verify.py`, `Survey.verify` | Each well-seen object is shown to the model in up to three keyframes; a majority confirms, relabels or rejects it | Ladders, extinguisher and hose cabinet confirmed in every run; the alarm box confirmed or left "unclear" (never mislabelled). The model is told each object's measured height, and a relabel that contradicts it is refused: before that it called the 0.8 m hose cabinet an alarm box |
| Resolution rule | `protocol/assess.py`, `form.py` | A form row counts only if the cited photo resolves enough pixels per metre for that kind of finding; label text and sticker dates need a close-up | See below |

**The form reader fabricates when allowed to.** In the first real inspection Qwen3-VL 8B marked all
23 rows of both ladders fine at 0.86 to 0.99 confidence from blurred frames two to three metres
away, and invented an inventory number, a manufacturer and an article number. Requiring a photo
and a confidence does not stop a model that supplies both. With the resolution rule the same run
gives what the photos support: five shape rows fine (rails and rungs not bent, no rung missing,
spreader and feet there), twelve rows "close-up needed", sticker not readable, shown grey at 29 %
coverage. Still wrong after the rule: the leaning ladder is typed "Stehleiter" and both "Stahl".

Reading a label or sticker needs about 1500 pixels per metre: 0.4 m from the object with the B2's
head camera. The recording has no such view, so no row needing it has ever been filled.

## Joined with the rest of the repository

This branch contains `main` and Sahil's `sahil/sim-slam-nav2` (`ultron.autonomy`). Where they meet:

| Piece from the team | How the pipeline uses it |
|---|---|
| Dashboard (`ultron.dashboard`) | `--dashboard URL`: mission, live frame, pose, events and each ladder's record. Run on the hall recording: both ladders arrive as "manual review" with photos and evidence crops |
| Ladder prompt pack (`assets/inspection_v1/rubric.json`) | `protocol/sections.py` reads the form one section at a time with that guidance, tells the model each photo's distance and marks each criterion as judgeable or too far |
| Navigation contract (`autonomy/navigation_contract.py`) | `navigate.ContractNavigator` turns view requests into `ultron.navigation_goal/v1` goals and waits for "capture ready"; `navigate.free_space` filters goals with the saved map's footprint check. Tested against the contract code with a scripted backend, never with Nav2 running |

Section-by-section reading on the hall recording (Qwen3-VL 8B, eight calls per ladder, about 20 s):
the model now answers the shape rows with a photo and a box, says "unknown" for rows the photos
cannot show, and differs between the two ladders (feet in view on one, not on the other).

Not joined yet: main's wider safety pack (escape routes, protective equipment), its coverage
planner, Om's detection evaluation, and non-ladder objects on the dashboard.

## Not yet verified

- Form reading has run on real frames only from two to three metres, with one small model. How
  well any model reads the fine rows and the sticker from a real close-up is unknown.
- A second patrol from a different start and an object moved between patrols: simulated patrols
  (above) and synthetic tests only; on real data, replays of the one recording (same start; injected
  pose errors; fake objects planted on open floor). No real localised (`map`) pose yet. Pose errors of 0.7-1 m are flagged as gross: stored
  landmarks are known only to 0.3-0.6 m, so the guard cannot tell them apart and changes nothing.
- Removal judged by non-detection: on real data only with detections dropped from a replay, not
  with an object physically taken away; the detection rates it relies on come from one recording.
- Displacements over 2 m (beyond the reach in which an unmatched sighting is taken for a stored
  object) are not caught by the guard: such an object is created anew.
- Two ladders closer than about 0.4 m edge to edge and never seen in one frame become one object.
- Nav2 goals, the live ROS 2 source and the Unitree source have not been exercised.
- Viewing directions are map bearings in 45° steps; the pipeline does not know a ladder's front.
- Lying ladders and glass: rendered tests, phone photos and the hall recording's controls only; no
  B2 recording of a lying ladder or of a glass wall exists ([REAL_DATA_SHOTS.md](REAL_DATA_SHOTS.md)).
- An object seen only without range gets an assumed range of 2.5 m and is marked approximate.
- Without poses, objects are split by a 6 s gap in time, and no view can be requested.
- The dashboard is optional: without `--dashboard` nothing is posted and nothing from
  `ultron.dashboard` is imported.

Tests: `PYTHONPATH=src python -m unittest tests.test_pipeline`; the footprint, site-map, anchoring and
moved-object tests render simple scenes (depth image plus masks) in `tests/test_pipeline.py`.
