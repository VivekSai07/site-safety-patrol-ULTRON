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
- **The next object is chosen by geometry and time, never by a model answer.** The focus is the
  nearest object still collecting. Its next missing view is announced as a `focus` event. With
  `--navigator dashboard|contract` it is also sent as a goal, off-thread, so frames keep flowing
  while the robot walks.
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

Tuning: `--leave-seconds`, `--leave-distance`, `--budget-seconds`, `--max-reads`.

## Discovery from the lidar

`discover.LidarCandidates` looks at the cloud (map frame, z up) about once a second:
1. Keep points 0.3–2.8 m above the floor and 1–10 m from the robot.
2. Bin them on a 20 cm grid and find connected blobs.
3. Keep blobs whose longest side from above is 0.2–1.8 m and that reach at least 1 m high.

Walls, containers and fences are larger than that, so they are dropped. A candidate that shows up
in two clouds counts. An object found within 1.5 m explains it. If the camera looks at it from
1–5 m five times and finds nothing, it is dismissed. While no object is collecting, the nearest
open candidate becomes a look goal: stand 2.5 m away and face it.

Simulated patrol A, scored against the yard's ground truth: all 11 ladders the route passes within
8 m were found, including the five leaning on containers. There were also 12 candidates with no
ladder near them (scaffold, pallet and fence parts), which the camera has to dismiss.

**On the real robot it is not wired yet.** Two things are missing:
1. The relay has to forward the cloud with the frame, transformed to `map` with TF
   (`map ← lidar`). Today it uses the cloud only for odometry.
2. The floor height has to be set (`--floor`), or estimated per cloud, which is the default.

Do not put the cloud in `frame.points`: `lift.py` then ranges detections from the lidar instead
of the depth image. The attribute is `frame.cloud`.

## How this fits exploration and map building

The dashboard now has an exploration state (frontier exploration with m-explore-ros2 through
Nav2). `/api/goal` returns 409 while exploration runs, so exploration goals and view goals
exclude each other. A full loop would be:
1. Explore.
2. On a `candidate` or `object_found` event, pause exploration and give the focus to the
   inspection goals.
3. Resume exploration when nothing is collecting.

None of that is wired. Today `online` works with a person driving, with a recording, or with goals
through the dashboard or contract navigators when exploration is idle.

## Not verified

- Never run on the robot or with Nav2. Moving navigators are tested only with a scripted navigator.
- The hall recording has no lidar in its frames, so the candidates have only run on sim data.
- A form read takes 10–40 s with Qwen3-VL 8B (eight section calls per ladder). In the sim yard the
  robot finds ladders faster than they are read, so the reader falls behind. The robot never waits
  for it, and what is left is read when the stream ends.
