# Site Safety Patrol – 48h Build Plan

Nov 27, 2026 · @Aravind

## Goal

We build a Unitree B2 patrol that finds every ladder on the test course, checks what it can see, and writes a digital inspection record automatically. That covers nesseler's three deliverables: every ladder assessed, an automatic digital protocol, and a live demo on the test course.

This plan is the 48-hour cut of the 25-page build bible. It keeps the bible's safety logic and its minimum demo (Appendix E), and drops everything marked Tier 2 or Tier 3.

**Scope boundary:** the system is automated pre-screening, not certification. We only ever say "no defect detected within the checked visual fields". Touch-only checks (bolts, hinge play, latches) stay with a qualified person and are listed as pending on every record.

## What we build vs what we cut

We keep every item in the bible's minimum demo, but get to the ladders the low-risk way. We also use one model per job instead of a bake-off.

| Bible item | Our plan | Why |
| --- | --- | --- |
| Autonomous patrol on a saved map, 0 manual driving | **Keep** | Core of the demo |
| Dynamic approach pose generated per ladder | **Swap:** drive to overview + close-up poses recorded during setup (bible's cut-line B) | Same story, far less navigation risk |
| Ladder discovery (Grounding DINO / OWLv2 / RF-DETR) | **Swap:** SAM 3 only | One model gives box and mask |
| Rail-strip depth sampling | **Swap:** median depth inside the SAM 3 mask | Mask already skips the wall behind the rungs |
| Persistent LAD-xxx IDs, no duplicates | **Keep** | Needed for "every ladder assessed" |
| OCR stack + dual reader | **Swap:** one local Qwen3-VL reader | Less to build and debug |
| Hard-fail-first rules, 4 verdict states | **Keep** | What nesseler actually buys |
| Dashboard + PDF/JSON protocol | **Keep**, plus CSV | Deliverable 2 |
| History across two patrols | **Keep** | Cheap once the registry exists |
| Active second look (one retry) | **Optional:** second recorded pose, Saturday afternoon only | Nice demo case C, not required |
| Body tilt, side-oblique ring, 360° orbit | **Cut** | High robot risk |
| Trained defect detector, geometry, lean angle | **Cut** | Needs data and time |
| PatchCore anomaly, Isaac Sim, onboard (Jetson) detection | **Cut** | Tier 3 |
| n.core mapping, replay script | **Cut** | CSV export is enough |

## Architecture

The robot only walks, streams and stops; every model runs on the laptop, so a laptop crash never makes the robot unsafe.

&#91;embedded content: System architecture · robot, laptop, outputs\]

What each part is for:

- **Sensors:** the camera takes the inspection photos and its depth places each ladder on the map; the LiDAR is for mapping and obstacle avoidance.
- **Nav2 + saved map:** knows where the robot is, walks the patrol waypoints and drives to each ladder's recorded overview and close-up poses.
- **Safety:** remote E-stop and laptop watchdog; stops the robot if someone presses the stop or the laptop link drops.
- **Detect + pin:** SAM 3 finds ladders in the frames; the depth inside the mask turns each sighting into a fixed map position.
- **VLM reader:** Qwen3-VL reads the close-up into the checklist (label date, foot caps, rungs, spreader). It reports what it sees and never decides pass or fail.
- **Rules:** plain if/else logic turns those observations into a verdict, hard fails first. This is the part we can explain and defend to nesseler.
- **Registry:** SQLite database that gives each ladder its LAD-xxx ID and keeps its inspection history. Everything else reads from it.
- **Live dashboard:** what the site manager sees: map, ladder pins, photos, verdict and reason.
- **Protocol:** the automatic digital record (PDF, JSON, CSV), nesseler's second deliverable.

## How one patrol works

The whole patrol runs without anyone touching the sticks. Setup (Mode 1) is the only time we drive by hand.

0. **Setup, once (Mode 1):** drive the B2 by remote to build and save the LiDAR map. Record 4–6 patrol waypoints, plus an overview pose (about 2.0–2.5 m) and a close-up pose (about 0.8–1.0 m) for each ladder spot.
1. **Patrol:** the B2 localizes on the saved map and walks the waypoints. It streams 2–5 keyframes per second, with depth and its map pose, to the laptop.
2. **Find:** SAM 3 spots a ladder. The median depth inside its mask gives a 3D point, which is moved into map coordinates. After 3 sightings within 1 m it counts as confirmed.
3. **Identify:** the registry matches the point to an existing LAD-xxx, or creates a new one. A ladder already inspected in this patrol is skipped.
4. **Inspect:** the laptop sends the robot to that ladder's recorded overview pose, then close-up pose. At each one the robot stops, waits about 0.8 s to settle, and takes a short burst of still photos.
5. **Judge:** Qwen3-VL reads the close-up into the inspection schema (label date, foot caps, rungs, spreader). The rules turn that into one verdict.
6. **Log and resume:** the result is added to the ladder's history, the dashboard and PDF update, and the robot walks on to the next waypoint.
7. **Finish:** after the last waypoint the system reports coverage, including any expected ladder that was not found.

## Compute plan

Everything fits on our RTX 4060 laptops (8 GB VRAM), but only just. Nothing heavy runs on the robot.

| Part | Runs on | VRAM (approx.) | Notes |
| --- | --- | --- | --- |
| Sensors, Nav2, safety | B2 onboard | – | Organizer / MYBOTSHOP ROS 2 stack |
| SAM 3 detect + pin | Laptop 1 (4060) | \~4 GB in FP16 | Image mode, one keyframe at a time. Never video mode: its memory grows per frame |
| Qwen3-VL reader | Laptop 2 (4060) | \~3 GB (4B, 4-bit) | Fallback: 2B variant at \~2 GB. Runs only when the robot is stopped |
| Rules, registry, dashboard, PDF | Laptop 2 (CPU) | – | FastAPI + SQLite |

**If only one 4060 is free:** SAM 3 and the 4B reader together need about 7 of 8 GB. That works because they never run at the same moment: SAM 3 runs while the robot walks, the reader while it stands still.

**If the organizers' RTX 6000 Ada is available** (the bible lists it): run both models there and upgrade the reader to Qwen3-VL-8B for better label reading. The 4060s become dev machines, the dashboard screen and the backup.

These VRAM numbers come from model cards and blog posts. Check them tonight with `nvidia-smi` on a few ladder photos.

## Verdicts

Every ladder gets exactly one of three statuses. The VLM only reports what it sees; plain if/else rules decide the status, in this order:

1. **Hard fail first.** If any photo shows a confirmed defect (missing foot cap, missing or bent rung, overdue inspection sticker), the ladder is FLAGGED. This holds even when other parts were not visible.
2. **Then completeness.** If a required part was hidden, blurry, glared or unclear, the status is MANUAL\_REVIEW, and the record names exactly what is missing.
3. **Only then a pass.** All checked fields are visible and fine → AUTOMATED\_PASS, with the touch-only checks still listed as pending.

| Status | Colour | Meaning |
| --- | --- | --- |
| FLAGGED | Red | Known defect seen in at least one photo |
| MANUAL\_REVIEW | Amber | Something required is missing, unclear or contradictory |
| AUTOMATED\_PASS | Green | All checked visual fields fine; tactile checks pending |
| COVERAGE\_INCOMPLETE | Grey | Patrol-level only: a zone was not scanned or an expected ladder was not found |

The system never silently passes a ladder it could not fully see.

## Safety

Three layers stop the robot. Only the first is guaranteed by the hardware, so it must always be in someone's hands.

1. **Remote E-stop (hardware):** L1 + A on the Unitree remote puts the B2 into damping mode and it settles to the ground. One teammate is the safety person, holds the remote whenever the robot is powered, and does nothing else during runs.
2. **twist\_mux (software priority):** Nav2 has the lowest priority and the joystick sits above it, so any stick input instantly overrides autonomy. That takeover is logged as an operator intervention. Reuse the twist\_mux from the MYBOTSHOP stack rather than building one.
3. **Laptop heartbeat (watchdog):** the laptop publishes a heartbeat about 10 times a second. If it stops for 0.5 s (Wi-Fi drop, laptop crash), a twist\_mux lock blocks Nav2 and the robot stops. The joystick still works.

**Check on Friday:** what the B2 does when velocity commands just stop arriving. As a backstop, the bridge to the B2 sends zero velocity after about 0.3 s of silence.

**Before any autonomy, test in open space at 0.3–0.5 m/s:** pull the laptop's Wi-Fi while walking (stops within 500 ms), move the joystick during a Nav2 goal (joystick takes over), press L1 + A while walking (damping).

## Team and timeline

Each person owns one part and can test it alone on Friday night; we only plug the parts together on Saturday morning.

| Role | Owns | Done on Friday night when… |
| --- | --- | --- |
| Robot | B2 bring-up, safety tests, map, waypoints and ladder poses, Nav2 goals | Two safe hands-off loops on the saved map |
| Perception | SAM 3 detect + pin, Qwen3-VL reader and its prompt | Ladders pinned and schema filled correctly on photos or recorded bags |
| Backend | Pydantic schemas, rules, registry, dashboard, PDF/JSON/CSV | One ladder becomes one reasoned record on a mock stream |
| Data and demo | Staged defects, recorded bags, test runs, video, dataset, pitch | Staged ladders ready, first bags recorded |

&#91;embedded content: 48h timeline · 4 phases, 3 gates\]

After the Saturday 13:30 baseline, new work goes on a branch and only joins if it does not break the baseline.

**Cut-lines if things go wrong:**

- **Close-up approach unreliable by Saturday 16:00:** add a one-click confirm before the close-up pose (semi-autonomous, logged).
- **Navigation fails entirely:** drive by remote. Detection, registry, verdict and report stay automatic, and we say so honestly in the demo.
- **Never let a Saturday extra delay the 22:00 video.**

## Friday kickoff: first hour

Settle only these three things in the first hour; everything else can wait until the robot moves.

- [ ] **Robot interface (bible D1):** which camera and depth topics, the frame names, the DDS domain, and the approved way to send velocity commands and goals
- [ ] **Route (bible D2):** confirm Nav2 with recorded poses on the organizer stack, and that we may record waypoints in Mode 1
- [ ] **Inspection sticker (bible D5):** photograph a real Prüfplakette on the course and ask the mentor what "overdue" means on it
- [ ] Ask whether the RTX 6000 Ada is ours to use (changes the compute plan)
- [ ] Practise L1 + A once with the organizers
- [ ] Request SAM 3 weights on Hugging Face before the event if not done yet (gated access)

## Demo and deliverables

The live demo is one hands-off patrol of under 3 minutes with two or three staged ladders. Each staged defect is labelled as test evidence, never as a real finding.

| Case | What we stage | Expected result |
| --- | --- | --- |
| A – Normal | Readable, valid sticker; feet and rungs visible; no defect | AUTOMATED\_PASS, tactile checks listed as pending |
| B – Defect | Overdue sticker or a missing foot cap | FLAGGED, even if another part is hidden |
| C – Second look (optional) | A crate hides one foot in the first view | Second recorded pose shows it → verdict, otherwise MANUAL\_REVIEW |

Then run a second patrol: the same LAD-IDs come back, and the dashboard shows each ladder's history.

**Final deliverables (bible WBS-11/12, before Sunday 12:00):**

- [ ] Uncut ≤2-minute backup video of the real robot, locked by Saturday 22:00
- [ ] Repo with README and start commands
- [ ] Team-captured dataset on Hugging Face (CC-BY-4.0, own photos only)
- [ ] One-pager and 5-minute pitch

## Tools and repos

| Job | Tool | Link |
| --- | --- | --- |
| Ladder detection + mask | SAM 3 via Ultralytics (`SAM3SemanticPredictor`) | [docs](https://docs.ultralytics.com/models/sam-3) · [facebookresearch/sam3](https://github.com/facebookresearch/sam3) |
| ROS 2 wrapper for SAM 3 | ultralytics\_ros (`sam3_segmentation_node`) | [mitukou1109/ultralytics\_ros](https://github.com/mitukou1109/ultralytics_ros) |
| Fallback detector | Grounded-SAM-2 | [IDEA-Research/Grounded-SAM-2](https://github.com/IDEA-Research/Grounded-SAM-2) |
| VLM reader | Qwen3-VL-4B Instruct (8B on the RTX 6000) | [Qwen/Qwen3-VL-4B-Instruct](https://huggingface.co/Qwen/Qwen3-VL-4B-Instruct) |
| Robot middleware | ROS 2 Humble, CycloneDDS, Nav2, twist\_mux | MYBOTSHOP B2 stack ([docs](https://www.docs.mybotshop.de/b2.html)) |
| Backend | FastAPI, Pydantic v2, SQLite, WeasyPrint | Pydantic schema in bible Appendix G |
| Dashboard | React or plain HTML + Foxglove for debugging | – |
| Inspection checklist | DGUV Information 208-016 | [PDF](https://publikationen.dguv.de/widgets/pdf/download/article/450) |

Prefer Apache-2.0, MIT and BSD licences. Keep a short licence list in the repo.
