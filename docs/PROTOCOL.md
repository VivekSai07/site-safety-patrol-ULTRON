# nesseler checklist reader (`ultron.protocol`)

Camera frames in, nesseler's ladder checklist out: every row of the "Kontrollblatt / Checkliste
zur Überprüfung von Leitern und Tritten" (BGI 694) read per ladder by a vision model, with a
photo as evidence for each point. It is the robot-side producer for the dashboard
(`ultron.dashboard`, see [DASHBOARD.md](DASHBOARD.md)): the dashboard displays, alerts and exports
the PDF; this package decides what goes in. Read-only: it never commands the robot.

```
frame source ─▶ quick look (is a ladder in view? which view?) ─▶ sighting ─▶ full reading
 folder / OpenCV / ROS 2 / Unitree                    best overview, detail and label frames
                                                                     │
 dashboard HTTP API ◀── publish ◀── store (JSON + JPEG) ◀── rules ◀──┘
```

| File | What it does |
|---|---|
| `form.py` | The paper form as data: master data, 23 criteria in 7 sections, the three results |
| `vlm.py` | Vision model client (any OpenAI-compatible endpoint, standard library only), prompts, replay |
| `assess.py` | Deterministic rules from model reading to per-criterion states |
| `store.py` | One folder per ladder: `sheet.json`, source photos, one `beleg-<criterion>.jpg` per point |
| `patrol.py` | Stream handling: quick look every few seconds, group frames per ladder, read in the background |
| `lidar.py` | Ladder position from lidar points inside the ladder's image box |
| `sources.py` | Frame sources |
| `publish.py` | Translates store events into the dashboard's contract and posts them |

## Run

```bash
pip install -e '.[protocol,dashboard]'
python -m ultron.dashboard --lan                       # terminal 1
export OPENAI_API_KEY=...                              # ULTRON_VLM_MODEL / ULTRON_TRIAGE_MODEL choose the models
python -m ultron.protocol check                        # is the model reachable?

# one ladder from chosen photos
python -m ultron.protocol inspect photos/ladder_04.jpg photos/ladder_10.jpg --dashboard http://127.0.0.1:8000
# a live stream
python -m ultron.protocol run --source cv:0 --dashboard http://127.0.0.1:8000 --site my-site
# on the B2, where the ROS 2 topics are visible
python -m ultron.protocol run --source ros2:/IMAGE_TOPIC,pose=/ODOM_TOPIC,cloud=/LIDAR_TOPIC \
    --calibration calibration.json --dashboard http://LAPTOP:8000
# without network or key: recorded answers for three nesseler photos
python -m ultron.protocol run --source folder:PHOTOS --replay tests/fixtures/nesseler_replay.json \
    --gap 1 --interval 0 --dashboard http://127.0.0.1:8000
```

`unitree[:INTERFACE]` reads the front camera through `unitree_sdk2py` instead of ROS 2.
Without `--dashboard` the results are only written to `--data` (default `runs/protocol`).

## How readings map to the dashboard contract

| Reader state | Dashboard item | |
|---|---|---|
| ok | `OK` | needs a photo that shows it and model confidence ≥ 0.5 |
| defect | `DEFECT` | verdict `FLAGGED` |
| not visible, or suspected defect below 0.5 confidence | `UNKNOWN` | verdict `MANUAL_REVIEW`; listed in `missing` |
| function rows (spreader works, greased, ...) | `NOT_CHECKED` | always a person, unless a defect is plainly visible |
| no such part on this ladder | `N_A` | |

An overdue sticker gives `FLAGGED`, an unreadable one `MANUAL_REVIEW`. Each assessed item carries
one evidence crop in `item_photos`; the crop's caption holds the model's one-sentence remark.
`publish.py` pairs criteria with `dashboard/checklist.py` by position and refuses to import if the
two lists stop matching.

## Not yet verified

- No real model call has been made: no API key was available while building. Run `check`, then
  `inspect` on the nesseler photos, and tune the prompts in `vlm.py` on what comes back.
- `ros2` and `unitree` sources were written without the robot. On the first
  hardware session check topic names and types (`ros2 topic list -t`) and image encodings.
- `lidar.locate` is tested on synthetic points only. It needs a measured `calibration.json`.
- Sightings are split by gaps in the quick looks or a changed marking. Two unmarked ladders side by
  side in one continuous view become one sighting.
- `tests/fixtures/nesseler_replay.json` is hand-written, not model output.
- The dashboard requires a position; without localisation the reader sends (0, 0, 0).
- The dashboard takes one verdict per ladder and mission, so a second sighting of the same ladder
  in one mission is kept in the reader's store but rejected by the dashboard (logged).

Tests: `PYTHONPATH=src python -m unittest tests.test_protocol`
