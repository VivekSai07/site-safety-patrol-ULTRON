# From the simulation to B2 motion and model-based evidence

## Motion: SDK interface and simulator are different layers

Unitree provides the B2-specific `unitree_sdk2py.b2.sport.sport_client.SportClient`, with `Move(vx,vy,vyaw)` and `StopMove()`. The official example initializes an explicitly chosen DDS domain/NIC and the B2 client. Source: [B2 SportClient](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/b2/sport/sport_client.py), [B2 example](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/b2/high_level/b2_sport_client.py).

The official [MuJoCo bridge](https://github.com/unitreerobotics/unitree_mujoco) is a low-level simulator, not an implementation of that high-level walking service. Installing SDK2 does not make a free-base MuJoCo model walk. A proper dynamics-backed controller is a separate integration task.

This repo now uses rest-to-rest acceleration/speed-limited profiles, turns into the route direction before translating, and faces each inspection target at arrival. Default bounds are 0.5 m/s forward, 0.8 rad/s yaw, with 0.6 m/s² and 1.4 rad/s² profile accelerations. These are provisional simulation settings, not validated physical B2 limits. Every command is recorded in `motion_commands.jsonl` with the shared `Move/StopMove` semantics. `--motion legacy` preserves the old stepping path for comparison.

A cosmetic diagonal-leg cycle makes the robot's progress visible; it is explicitly joint animation, not a learned gait or contact simulation. `mj_forward` still places the base kinematically.

`B2SdkTransport` forwards validated commands to an injected real B2 client, rejects stale state/operator-disabled commands, and requests StopMove on errors. `connect_b2` is an explicit operator-authorized connection hook; the demo never calls it. Real integration still requires verified interfaces, fresh localization, a separately tested watchdog, remote-control priority and an approved motion supervisor. A check inside `send` cannot stop a robot if the entire caller stops running. Recorded command traces are NOT intended for blind open-loop hardware replay.

## Detection: choose a model that actually supports ladders

Standard COCO-trained YOLO detection checkpoints use 80 fixed classes and **do not include ladder**. A pretrained YOLO is therefore not automatically a ladder detector. Options are a ladder-trained checkpoint or open-vocabulary YOLOE/YOLO-World with a ladder prompt. [COCO classes](https://docs.ultralytics.com/datasets/detect/coco/), [YOLOE](https://docs.ultralytics.com/models/yoloe/), [YOLO-World](https://docs.ultralytics.com/models/yolo-world/).

Recommended first experiment: YOLOE small segmentation checkpoint, e.g. `yoloe-11s-seg.pt`, with `ladder` and `stepladder` prompts. Confirm the exact checkpoint name/version in the installed Ultralytics release before downloading; current upstream also documents YOLOE-26 variants. This is an experiment choice, not a claim of ladder recall on the course. `set_classes` may require a text encoder download even with a local checkpoint. Prepare/cache both ahead of an offline demo. Offline mode requires a checkpoint already saved with ladder vocabulary/embeddings and skips `set_classes`; automatic Ultralytics installation is disabled before import. Ultralytics code/weight licenses are separate from this repository; review upstream AGPL/Enterprise terms for your intended distribution.

## Inspection: selected multi-view Qwen proposals

[Qwen3-VL](https://huggingface.co/collections/Qwen/qwen3-vl) includes 2B/4B/8B and larger variants. A first offboard experiment can use [Qwen3-VL-4B-Instruct](https://huggingface.co/Qwen/Qwen3-VL-4B-Instruct), with larger variants only after measuring available memory/latency. It is not a trained or calibrated ladder defect inspector.

The adapter accepts selected images of the SAME ladder, asks for explicit per-component visibility, PASS/FAIL/UNKNOWN, rationale and confidence, and validates returned JSON. Never bundle different ladder crops and assume the model knows persistent identity. The current analysis command does not auto-associate detector boxes or connect findings into robot control; that is an explicit integration boundary.

A detector confidence means evidence for an object category, not safety. A VLM's self-reported confidence is not a calibrated probability. Similar repeated images are correlated and their scores must not be multiplied to manufacture certainty. The current experimental component threshold is 0.85: unseen/low-confidence components become UNKNOWN, known failures dominate, and even complete model PASS proposals require human review until calibrated against held-out physical ladders. Label date judgements remain proposals unless literal fields and semantics are verified.

For a recognized component outside the required type template, a visible model FAIL proposal meeting the existing threshold is retained and can flag the record; an optional PASS cannot complete required fields. This is conservative screening, not validation of the model's claim.

## Executable commands

Keep model dependencies in a compatible separate environment; model downloads and inference were not performed for this reconciliation.

```bash
# Install only the desired optional model extra
python -m pip install -e '.[perception-yolo]'
python -m pip install -e '.[perception-vlm]'

# Real detector adapter, using a local checkpoint with saved ladder vocabulary/embeddings
PYTHONPATH=src python -m ultron analyze image.jpg --weights /path/to/yoloe-checkpoint.pt --out runs/detector-review

# Selected front + side images of one ladder; model must already be cached locally
PYTHONPATH=src python -m ultron analyze front.jpg side.jpg --qwen /path/to/Qwen3-VL-4B-Instruct --ladder-type A_FRAME_STEPLADDER --out runs/qwen-review

# Explicit permission for model/processor downloads (can be several GB)
PYTHONPATH=src python -m ultron analyze front.jpg side.jpg --qwen Qwen/Qwen3-VL-4B-Instruct --allow-download --ladder-type A_FRAME_STEPLADDER --out runs/qwen-download-test
```

With no checkpoint/model arguments, `analyze` exercises evidence capture/reporting and yields MANUAL_REVIEW with unknown fields. This fallback is not inference. With unavailable dependencies/models, the command errors; it does not silently substitute oracle answers. Source image hashes, copied evidence, model outputs and reasons are written to `analysis.json` and `report.html`. Runtime inference and ladder accuracy of the optional adapters remain unverified.

## Active views and 360 degrees

`candidate_views` generates simulation-grid-checked observation candidates. A full ring is available only for an A-frame when the caller explicitly confirms open space and missing rear evidence; platforms and leaning/extension ladders use nominal front candidates. Angles use a fixed map axis, not measured ladder orientation. These attestations and grid/path checks do not prove physical clearance, line-of-sight or component visibility, and the utility does not execute a full orbit. The protected demo still permits one side recovery.

Request views to answer a named gap: obscured feet, unseen spreader, unreadable label. Stop on a hard fail, resolved scope, budget, unsafe path or operator action. Report rejected/unseen views and incomplete coverage rather than guessing. A real 360 sequence requires verified space and a controller, not just a list of points.

## Live inspection view

Start a demo with a chosen new output folder, then tail it from another terminal:

```bash
PYTHONPATH=src MUJOCO_GL=egl python -m ultron demo --out runs/live-patrol
PYTHONPATH=src python -m ultron rerun runs/live-patrol --spawn --follow
```

Rerun follows complete telemetry/event lines until Ctrl-C, visualizing frames/poses/pins/reasons while capture continues. Install the Rerun extra first. The existing local HTML report remains an end-of-run artifact; Qwen analysis is a separate offline image workflow. An authenticated inspector action/sign-off UI is future work. Rerun is an evidence visualization tool, not a safety decision authority.
