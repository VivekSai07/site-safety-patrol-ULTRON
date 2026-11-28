# Integrated readiness — 28 November 2026

**Hardware motion: NO-GO.** This baseline exercises inspection and safety plumbing in a kinematic simulation. It does not establish a complete B2 autonomy stack or authorize connecting to the robot. The required gate remains [SIM_TO_HARDWARE.md](SIM_TO_HARDWARE.md).

The step-by-step verification sequence and per-test approvals are in the
[B2 hardware test plan](B2_HARDWARE_TEST_PLAN.md).

## What is connected now

The patrol emits a checksum-bound, per-ladder `ASSESSMENT_READY` manifest as soon as acquisition ends. `dashboard.live_tail --assess-openai` runs one background Responses API request at a time using the seven-section/23-criterion prompt pack. It uploads evidence, then publishes the assessment while the patrol can continue. Dashboard ladder details retain criterion reasons, uncalibrated scores, image references and view-sampling coverage. Refusals, malformed responses, timeouts, exhausted request budgets and missing jobs become manual review, never oracle success. A known failure remains FLAGGED.

Model flags have a separate persistent assessment ledger. Repeated observations cannot clear a flag; historical defects retain their original mission, bundle, reason, score and photo hashes. New clean images are not presented as proof of an old defect. The ledger does not update the navigation map or grant motion authority. To retain flags across run folders, explicitly share `--assessment-ledger` **only when the same map/asset identity registry is shared**. A new map/identity scheme needs a new ledger. Human resolution of flags is not implemented.

`--dynamic-obstacles` checks fresh map-scoped occupancy before each smooth-motion sample, including swept-path clearance and a proposed braking margin. Invalid, stale, reordered, wrong-map or disabled observations cause a logged `StopMove`; freshness/localization failures latch inhibition. A fresh blockage can trigger at most three replans per move; persistent blockage inhibits motion. `MOTION_HALTED` reaches dashboard robot-stop alerts. The provider here is explicitly simulated occupancy injection, not camera/lidar perception or an independent watchdog.

## Per-layer status and remaining acceptance

| Layer | Evidence in this baseline | Required before integrated hardware use | Owner |
|---|---|---|---|
| SLAM/localization | Patrol uses known scene geometry and exact simulation pose. No SLAM is run by `ultron demo`. | Run Sahil's branch with timestamped scan/odom/TF, verify localization loss/recovery, map persistence and map versions. | Sahil |
| Navigation | Coverage goals in `ultron demo` use kinematic A*. The separate ROS Humble simulator test served a saved map and localization pose to the dashboard, dispatched a checked Nav2 goal through the motion gate, and verified rejection and cancellation. See [measured results](SAHIL_VALIDATION.md#dashboard-goals-and-the-sdk-wrapper). | Verify physical localization, footprint and stopping behavior in a separately authorized, supervised hardware test. | Sahil + Vivek |
| Dynamic obstacles | Swept-path guard, freshness/identity checks, stop/replan tests and rendered injection cases. | Fresh real lidar/depth costmap updates, people crossing, sensor dropout, unknown-space treatment; measure stopping distance, footprint and latency. Verify independent command watchdog, RC priority and E-stop. | Sahil + named robot operator |
| Ladder detection | Existing optional YOLO runtime; default and this 360 acceptance run use segmentation oracle. No new learned-detector recall result is claimed. | Om's ladder-capable weights, negatives/occlusion tests and annotated real replay; stock COCO classes do not establish ladder support. | Om |
| Identity/spatial memory | `src/ultron/pipeline/`: objects from detector + depth + pose, matched by floor footprint, one object map per site and map id, patrols lined up to the stored map, moved detection. Real bag: 5 objects, 5 found; sim patrols (Isaac): no duplicates. The SQLite `memory.py` registry (simulator truth) is retired as spatial memory and kept only for the MuJoCo mission. | A second real recording; a physical move on real data; a map id that stays the same across patrols from SLAM. | Kaan |
| 360 acquisition | Open A-frame patrol acquired all 8 azimuth × 3 height sampling cells with 27 total images, target boxes and image hashes; returned to base. Virtual camera aiming is explicit. | Real camera field of view/extrinsics and reachable camera/robot poses; account for walls, upper surfaces and occlusion. An inaccessible sector remains a gap requiring human review, never a claimed complete inspection. | Vivek + Sahil |
| Seven-section assessment | Strict schema/evidence validation, worker, review fallbacks, persistent failure history and dashboard plumbing tested. A one-keyframe API round trip was subsequently verified from the user laptop; the selected fixture was undamaged. | Evaluate seeded defects and unknowns against human labels; verify cloud results during a complete live dashboard patrol. Functional/tactile checks remain human-only. | Vivek |
| Other site safety | Existing fire/first-aid/escape/PPE sweep feeds events/photos; optional model mode remains MODEL_PROPOSAL. | Real-image evaluation and evidence for each visual criterion; no inference of functional extinguisher/first-aid availability from appearance alone. | Om + Vivek |
| Frontend/report | Real dashboard ASGI routes/store preserve evidence and all 23 explained findings. Browser/live TCP and PDF sign-off are not established by that test. | Aravind verifies live HTTP/browser updates, media, flags, unavailable API state, report content and restart recovery on operator laptop. | Aravind |
| B2 control | An opt-in ROS wrapper consumed gated velocity with a recording stand-in for SportClient. The simulator and the isolated ROS test exercised `Move`, `StopMove` and watchdog behavior. | Verified B2 SportClient/ROS transport, named supervised operator, bounded tests and independent safety validation. | Sahil + operator |

**Full-system simulation acceptance is incomplete:** no integrated SLAM/Nav2/detector/live-cloud/physical-controller run has passed. The tests below are scoped component/integration evidence.

## Reproduce on the laptop

Use a dedicated loopback dashboard with default dashboard-only notifications. No robot command bridge or dashboard goals are enabled. These commands use installed simulation/dashboard extras; they do not install ROS packages.

Terminal 1:

```bash
cd /home/vivek/ULTRON
PYTHONPATH=src python3 -m ultron.dashboard --port 8001 \
  --db runs/readiness-dashboard.sqlite --media runs/readiness-media
```

Terminal 2, before starting the simulation (uses the existing `.env`; never source it). `--assess-openai` explicitly sends images/prompts and can spend credits; at most eight jobs receive an API attempt. Set `ULTRON_OPENAI_MODEL` in `.env` or pass `--model` for an available vision model supporting Responses structured outputs. A failed/incomplete job is not automatically retried with another paid call.

```bash
cd /home/vivek/ULTRON
PYTHONPATH=src python3 -m ultron.dashboard.live_tail runs/my-integrated-01 \
  --url http://127.0.0.1:8001 --site ultron-readiness --tag=-R01 \
  --assess-openai --max-assessments 8
```

Terminal 3:

```bash
cd /home/vivek/ULTRON
PYTHONPATH=src MUJOCO_GL=egl LP_NUM_THREADS=2 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  python3 -m ultron demo --out runs/my-integrated-01 --patrols 1 \
  --variant open-orbit --view-policy full360 --nav auto --dynamic-obstacles clear
```

Open `http://127.0.0.1:8001`. Pick a fresh run folder and mission tag each time. This 360 mode deliberately requires the open fixture and oracle identity; it is not a real detector/robot acceptance test. Omit `--assess-openai` for a no-cloud run; then the dashboard uses explicitly synthetic baseline verdicts. `--dynamic-obstacles crossing` and `stale` inject separate safety failure cases; do not assume either run completes all inspection goals. `--safety` on a normal-site run adds existing site-safety props/events; the original bounded acquisition policy remains the default there.

## Evidence recorded in this session

- `runs/integrated-360-20261128-final`: one fixture discovered, **24/24 sampling cells**, 27 image frames with target boxes; 97.4% planner observation estimate; returned to base. Motion, visibility and pose are simulation-derived, not validated robot behavior.
- `runs/integrated-dynamic-stale-20261128-a`: stale observations emitted StopMove, latched inhibition, `COVERAGE_INCOMPLETE`; no return-to-base claim.
- `runs/integrated-dynamic-crossing-20261128-a`: dynamic-clearance stops and replan events, returned to base; zero ladders found on this scripted-route case, so `COVERAGE_INCOMPLETE`. This is a stop/replan case, not mission acceptance or moving-person perception.
- `runs/integrated-360-20261128-a/cloud-validation`: configured-key API attempt recorded `API_TRANSPORT_FAILED`; all 23 criteria become review findings. No successful cloud inference is claimed.
- New focused regressions cover missing/stale/invalid occupancy, swept path/braking margin, historical failure precedence, budget limits, publication retries/lost responses, retained evidence provenance and mission closure failure. Actual dashboard ASGI/store test uses an injected model response, inline notification dispatch and filesystem evidence verification; TCP, executor wakeups, browser rendering and media HTTP serving are not verified by it.
- Full discovery encountered the existing TestClient hang and four socket-creation errors (`EPERM`) in dashboard/teleop networking tests. The supported subset excludes these 23 infrastructure-dependent cases. The supported subset completed 212 cases: 197 passed and 15 skipped. These 23 excluded cases remain unverified, not passed. The new rendered stale-observation integration test passed separately; the existing rendered workflow suite also passed all seven cases (autonomous patrol, blocked inspections, standalone 360 capture, full ring, site safety, and two-patrol identity/history). Thus eight rendered checks passed separately. JavaScript syntax and staged whitespace checks passed.

Generated runs and model weights remain local and ignored by Git. No physical robot connection, hardware configuration change or movement was performed.

## Subsequent one-keyframe cloud validation

The user ran `runs/one-keyframe-openai-20261128T030445581961Z/retry_live.py` from the native laptop terminal. The persisted Responses result completed successfully, passed evidence-bundle validation and returned all 23 criteria using all seven prompt sections: 16 UNKNOWN and seven NOT_CHECKED, overall MANUAL_REVIEW. The source was the **undamaged** `ladder_c` open A-frame fixture, with a front/middle view that cuts off the feet. This validates one-image cloud/schema plumbing and conservative uncertainty handling, **not damage-detection accuracy**. Scores describe individual model findings, not safe-use probabilities.

A separate missing-rung frame was rendered and visually checked at `runs/one-keyframe-damaged-20261128T031003035003Z/frames/00000.jpg`. The missing fourth rung is actual omitted scene geometry, not merely an oracle verdict. The neutral request manifest contains no damage ground truth; `ground-truth-NOT-SENT.json` remains a local evaluation reference. The user subsequently ran its `retry_live.py`: the response validated all 23 criteria but returned MANUAL_REVIEW and UNKNOWN for rung damage. It noticed the larger gap but could not establish that a rung was missing.

## Failed paired defect test and optional verified references

The native-terminal run `runs/rung-evidence-comparison-20261128T031501778882Z/attempt-20261128T031956834961Z` assessed matched damaged/intact fixtures with overview and detail frames (about 78 and 220 target pixels wide). Both responses validated, both returned UNKNOWN for `sprossen.beschaedigung`, and both remained MANUAL_REVIEW. The seeded missing rung was not detected; the intact control was not flagged for rung damage. These two cases do not establish accuracy or recall. The model explicitly lacked the intended design as a reference.

The request builder now accepts optional `reference_views`: each needs a unique ID, root-confined JPEG path/hash, a matching `design_id`, explicit `design_match_verified`/`intact_verified` attestations and a nonempty `verification_source`. They are REFERENCE_ONLY, excluded from current evidence IDs, dashboard photos and coverage. References and the exact comparison policy are bound into the job digest; no-reference jobs retain their prior digests. Real references require independently verified matching equipment/design; an earlier model opinion is insufficient. This supplies intended geometry, not a safe-use certificate or an instruction to flag damage.

`runs/verified-reference-comparison-20261128T032350676057Z/run_comparison.py` prepares two API calls using the same current frames plus intact synthetic design references. The intact control images are reused as reference templates, so this is a comparison diagnostic, not held-out validation. The native-terminal run `attempt-20261128T032519138166Z` subsequently completed both calls: the damaged fixture returned DEFECT for rung damage and FLAGGED overall; the intact control returned UNKNOWN for rung damage and MANUAL_REVIEW overall. Both proposals passed all 23-criterion and evidence-bundle checks. Defect evidence IDs refer only to the current overview/detail, not reference images. This is one successful reference-assisted synthetic comparison, not held-out or real-world validation. Reference admission/evidence isolation and affected assessment/worker tests passed (26 tests); no hardware behavior changed.

### Observed rung-test outcomes

| Configuration | Missing-rung fixture | Intact control | Scope |
|---|---|---|---|
| One front keyframe | UNKNOWN / MANUAL_REVIEW | Not part of this pair | Seeded defect missed |
| Overview plus close-up | UNKNOWN / MANUAL_REVIEW | UNKNOWN / MANUAL_REVIEW | Seeded defect missed |
| Same overview/close-up plus verified design references | DEFECT / FLAGGED | UNKNOWN / MANUAL_REVIEW | One successful synthetic comparison |

The last result supports continuing reference-assisted evaluation. It does not establish detection of cracks, bent stiles, missing feet, loose spreaders, wear, label validity or any functional condition. The control's UNKNOWN is not proof that the entire ladder is intact. No result grants safe use or hardware motion authority. No additional API calls were made while validating these saved responses.
