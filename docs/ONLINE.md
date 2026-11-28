# Online patrol (`python -m ultron.pipeline online`)

`patrol` surveys first and inspects after the stream ends. `online` reads each object while the
robot walks on: one ladder's form is filled by the vision model in the background while the robot
already collects the next one.

```
frame ─▶ survey (detect, lift, object map, keyframes)            main thread, every frame
           │ object_found                     lidar candidate (optional) ─▶ "look there"
           ▼
         COLLECTING ──▶ object check (vision model, early, quick: confirm / relabel / reject)
           │
           │ done with it when: all views it needs | left behind (unseen 6 s and 3.5 m away)
           │                    | time budget (120 s) | operator types n
           ▼
         READING  (worker thread: form reader on frames copied into memory)
           ▼
         DONE ─ sheet / checklist saved, dashboard told
           │ robot comes back with views it lacked at the first read
           └──▶ COLLECTING again (at most --max-reads reads)
```

- **Only model calls run on the worker.** Every write to the object map, the keyframes and the
  ladder store stays on the main thread, so the survey code is unchanged. The worker serves object
  checks before form reads.
- **When the robot is done with an object, and which comes next, is decided by geometry and time.**
  The model's check can drop an object from the queue (rejected) or rename it (relabelled), as it
  does offline; form answers steer nothing. The focus is the nearest object still collecting. Its next missing view is announced as a `focus` event. With
  `--navigator dashboard|contract` it is also sent as a goal, off-thread, so frames keep flowing
  while the robot walks.
- **A second read never clears the first.** The worse status stays, earlier results are kept in
  `result["earlier"]`, and the ladder sheet keeps every inspection with its own evidence.
- **Leftovers are handled as before.** When the stream ends, the ordinary inspection settles the
  survey (merges fragments, judges missing objects) and reads whatever was not read online. That
  covers objects that stood beside one of their kind, which are only announced at settle.
- **Events.** New events are `object_ready` (why it was flushed and which views it lacked),
  `object_reopened`, `focus`, `candidate`, `candidate_explained` and `candidate_dismissed`. All of
  them are forwarded to the dashboard event list.

## Run it

```bash
export OPENAI_API_KEY=ollama OPENAI_BASE_URL=http://localhost:11434/v1
export ULTRON_VLM_MODEL=qwen3-vl:8b-instruct ULTRON_TRIAGE_MODEL=qwen3-vl:8b-instruct

# simulated patrol at recorded pace, lidar candidates on
python -m ultron.pipeline online --source sim:~/data/sim_hall/patrol_A_south,rate=5,depth=noisy,lidar=1 \
    --calibration sim:~/data/sim_hall/patrol_A_south --detector yolo --interval 0 --realtime \
    --candidates lidar --floor 0 --data runs/online-simA --mission SIM-A

# hall recording
python -m ultron.pipeline online --source bag:BAG,rate=8 --calibration bag:BAG --detector yolo --interval 0 --realtime

# robot: a person drives (teleop), follows the focus events and types n + Enter when done with one
python -m ultron.pipeline online --source relay:5600 --detector yolo --interval 0 --navigator operator \
    --dashboard http://127.0.0.1:8010 --mission HALL-ONLINE
```

Tuning: `--leave-seconds`, `--leave-distance`, `--budget-seconds`. `--max-reads 2` reads an object
again when the robot comes back with views it lacked. It is off by default because the dashboard
takes one ladder verdict per mission (HTTP 409 for the second): a re-read ladder is in its local
sheet only.

## Discovery from the lidar

`discover.LidarCandidates` looks at the cloud (map frame, z up) about once a second:
1. Keep points 0.3–2.8 m above the floor and 1–10 m from the robot.
2. Bin them on a 20 cm grid and find connected blobs.
3. Keep blobs whose longest side from above is 0.2–1.8 m and that reach at least 1 m high.

Walls, containers and fences are larger than that, so they are dropped. A candidate that shows up
in two clouds counts. An object found within 1.5 m explains it. If the camera looks at it from
1–5 m in five looks (at least 1 s apart) and finds nothing, it is dismissed for this patrol. While no object is collecting, the nearest
open candidate becomes a look goal: stand 2.5 m away and face it.

Simulated patrol A, scored against the yard's ground truth: all 11 ladders the route passes within
8 m were found, including the five leaning on containers. There were also 12 candidates with no
ladder within 1.2 m (not identified further), which the camera has to dismiss.

**On the real robot it is not wired yet.** Two things are missing:
1. The relay has to forward the cloud with the frame, transformed to `map` with TF
   (`map ← lidar`). Today it uses the cloud only for odometry.
2. The floor height has to be set (`--floor`), or estimated per cloud, which is the default.

Do not put the cloud in `frame.points`: `lift.py` then ranges detections from the lidar instead
of the depth image. The attribute is `frame.cloud`.

## Exploration, goals and where the robot may go

Contract: [EXPLORATION_CONTRACT.md](EXPLORATION_CONTRACT.md).

**Explorer: m-explore-ros2 (`explore_lite`), not our own.**
- It is a frontier explorer built on Nav2's costmap, and the dashboard already plans for it. It
  answers the question "where is unmapped space".
- `LidarCandidates` answers a different one: "what nearby is worth a look". It proposes looks at
  things already in sensor range, so the two work together.
- Frontier exploration has no boundary of its own (no polygon parameter in its documented
  options). Through a window it sees "free" space and would chase it. So it reads the fenced
  map (`/map_fenced`), not the raw SLAM map.
- Nav2 has never run on the B2, so the demo depends on neither. The ladder is: explore, then
  manual dashboard goals, then teleop. `online` behaves the same at every rung.

**What the pipeline does while exploring** (`--navigator dashboard`):
1. When an object found is within 4 m and the gate has a goal for a view it still lacks, the
   pipeline pauses the explorer through the dashboard. Nearby objects are batched into the same
   pause.
2. While paused it sends view goals and renews the lease.
3. It resumes the explorer when the views are in, no reachable view is left, or 90 s
   (`--pause-cap`) have passed. Each object pauses the explorer once per patrol.
4. Resume also happens on Ctrl-C, on an error, and at the end of the stream. If the pipeline dies,
   the dashboard resumes on its own when the lease runs out.

**One owner of Nav2: operator first.**
- An operator goal underway is never replaced; the pipeline waits until it ends, plus 10 s.
- A goal refused because the explorer drives, or because there is no navigation, is held and
  retried. It is not counted against the place.
- Only a goal Nav2 failed, or one reached without the view, marks that side as tried.

**Goal gate** (`gate.py`, used by `online`, `patrol` and `inspect`). A goal goes out only if:
- it is inside the fence (`--fence FILE` or the dashboard's), by the clearance;
- it is known free in the dashboard's latest map and reachable from the robot through free space;
- it is clear of every known object's footprint, a ladder lying on the floor included.

Where the middle of a view's distance range is not standable (tight room), the view is placed
nearer or farther. Sightings outside the fence are dropped before they become objects.

**Done with an object** also when its views have not grown for 20 s (`--stall-seconds`), or when
no reachable view is left. In a tight room the robot may never get 3.5 m away from an object, so
"left behind" alone would not fire.

**Known footprints go to the navigation stack** through the dashboard (`/api/obstacles`), for an
obstacle layer. That layer is what keeps Nav2's own paths, and the explorer's, off a ladder on the
floor. The pipeline's gate only covers its own goals. **Not configured in the navigation stack yet.**

## Tested

These runs used YOLOE and Qwen3-VL 8B through Ollama on the RTX 5090 workstation, replayed with
`--realtime`.

**Simulated patrol A** (86 s, sim evidence, lidar candidates on):
- 14 ladders found. All 9 with usable keyframes were read online; the other 5 were only ever seen
  beyond 4.5 m and have no keyframes, the same as offline.
- The first sheet arrived 19 s into the patrol. Five sheets came in during the walk; the last four
  were drained after the stream ended.
- One ladder was reopened and read a second time with an extra side.
- 17 lidar candidates counted: 14 explained by detected ladders, 2 dismissed by looking, 1 left open.

**Hall recording** (36 s, real B2 data, no lidar candidates):
- The same 5 objects as offline.
- The fire extinguisher, alarm box and both ladders were flushed as the robot passed them; the
  extinguisher's and alarm box's checklists were back during the walk.
- The extinguisher was reopened once (another side).
- The ladder sheets and the hose cabinet were done just after the stream ended.

**Against the offline `patrol` on the hall recording:** the form reader saw the same number of frames
per object (ladders 6 and 2, extinguisher 3, cabinet 2). Ladder coverage was 0.33 vs 0.22 and
0.22 vs 0.25, which is within the model's run-to-run spread, and the checklists came out the same.
The same run with `--dashboard` delivered the `FOCUS`, `OBJECT_READY` and `OBJECT_REOPENED` events,
both ladder inspections and all four checklists.

The cost of reading early: an object flushed when the robot leaves keeps the frames it had then.
In the sim yard one ladder was read from 2 frames at coverage 0.0. Seen better later, it is read
again only with `--max-reads 2`.

Unit tests: `tests/test_pipeline_online.py`. A gated reader proves the survey keeps finding and
collecting the next ladder while the first is being read. The other tests cover reject before
read, reopen, off-thread view goals, and candidate explained or dismissed.

**Reflections in glass (demo-room photos, phone camera, 2026-11-28).** The room's glass wall
mirrors its ladders. The object check now counts "a reflection or mirror image in a window or
glass" as none. Qwen3-VL 8B on six hand-cut crops:
- the three reflected A-frames were answered none ("Reflection of ladder in glass window");
- the real A-frame, the wooden ladder lying on the carpet and the stepladder were all answered
  ladder.

A rejected reflection keeps collecting its later sightings as a rejected object, so it costs one
check, not one per pass. This is the third line of defence. The fence (objects beyond the glass
are dropped) and depth checks come first. Not yet seen through the B2's own camera and depth.

## Not verified

- Never run on the robot or with Nav2. The driving rules (gate, operator first, pause lease, crash
  resume) are tested against a fake dashboard that follows the contract (`tests/test_pipeline_drive.py`),
  not against the real bridge or explorer.
- The hall recording has no lidar in its frames, so the candidates have only run on sim data.
- A form read takes 10–40 s with Qwen3-VL 8B (eight section calls per ladder). In the sim yard the
  robot finds ladders faster than they are read, so the reader falls behind. The robot never waits
  for it, and what is left is read when the stream ends.
