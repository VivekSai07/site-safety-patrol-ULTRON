# Synthetic detection hall

This is a MuJoCo detection test world, not a navigation, inspection or OCR
system. The default is an independently authored approximation built from the
existing team scene: seven synthetic ladders, construction-like clutter,
fictitious label cards, added lighting and a free virtual camera. It is not a
replica of Sahil's hall or the organizer course. Five default views sample
different positions, including a negative view.

The original Sahil-derived hall can be used only by explicitly supplying the
reviewed local source file as described below; that source is not distributed
with this addition. In that hall the seven category-1 (`ladder`) fixtures
include two trestles, which may not match another dataset's class definition.
Surface detail, camera effects, label text and real construction-site
variation are not represented. Simulation truth is not detector performance
or a real-world safety assessment.

## Provenance

The previously tested local scene was an unmodified copy of Sahil's
`src/ultron/autonomy/hall_scene.py` from team commit
`5add04dcd75652b2857ca78137d6ff44817573de` (snapshot file SHA-256
`92c31c79afaaba385f5d8d9e77920d05aa9ba9962917091ae7c9f3eed9000b5e`,
Git blob `940a13848be03fa56af31de21a616c74b19a0eb6`). Credit: Sahil
(`sahilrajpurkar03`); source:
https://github.com/VivekSai07/site-safety-patrol-ULTRON/blob/5add04dcd75652b2857ca78137d6ff44817573de/src/ultron/autonomy/hall_scene.py.
That file is excluded from this publication because the source snapshot has
no repository-wide license file. Do not infer a license or permission for
further distribution; review rights and privacy before sharing code or
generated datasets. The independent default does not import it.

The original Unitree B2 MJCF and its BSD-3-Clause license remain unmodified in
`src/ultron/assets/b2/`, with upstream URL and blob hash documented in
`src/ultron/assets/b2/PROVENANCE.md`. Both scenes strip B2 meshes at runtime,
retain the joint tree and primitive collision geometry, and use a stationary
kinematic B2 pose. This is not a validated walking controller,
physical B2, or hardware connection. The capture camera is free/virtual,
**not** robot-mounted and not a calibrated B2 sensor.

## Run

From the team repository root, create a local simulation environment if you
do not already have one. This requires Python's `venv` support and downloads
the declared simulation dependencies. The directory must not already exist:

```bash
test ! -e runs/detection-venv && python3 -m venv runs/detection-venv && \
  runs/detection-venv/bin/python -m pip install -e '.[sim]' 'mujoco==3.3.7'
```

With MuJoCo, NumPy and Pillow installed, for example in that environment:

```bash
# Optional graphical viewer (requires WSLg or another working desktop):
PYTHONPATH=src MUJOCO_GL=glfw runs/detection-venv/bin/python scripts/detection_world_viewer.py

PYTHONPATH=src MUJOCO_GL=egl runs/detection-venv/bin/python scripts/detection_world_capture.py
PYTHONPATH=src MUJOCO_GL=egl runs/detection-venv/bin/python scripts/detection_world_capture.py \
  --out runs/my-detection-world --width 640 --height 480 \
  --position 2 1.4 1.35 --target 4 4 1.1
PYTHONPATH=src MUJOCO_GL=egl ULTRON_RENDER_TESTS=1 runs/detection-venv/bin/python -m unittest discover -s tests -p 'test_detection_world*.py' -v
```

Default output uses a unique `runs/detection-world-<uuid>` directory; an
explicit output must not already exist. Output is never overwritten; a
failed render may leave a partial directory. Positioning moves only a virtual
camera inside bounded hall coordinates; the simulated B2 remains stationary.
It does not check collision-free robot poses or trajectories. Use repeated
invocations for custom views; no GUI is required. `MUJOCO_GL` is platform
dependent; EGL must be available for the headless example.

Open the generated PNGs in `images/` with an image viewer. On WSL, for
example, `explorer.exe "$(wslpath -w runs/my-detection-world/images)"` opens
the image folder in Windows Explorer. The capture command prints the path
when it generates a unique default directory.
The optional viewer is a stationary scene: drag and scroll to inspect it.
Its free viewer camera does not set the capture camera. Close the window or
press Ctrl+C to stop; no robot movement or control is performed.

### Optional locally supplied Sahil hall

After confirming local-use rights, set `ULTRON_HALL_SCENE` to the exact
`hall_scene.py` from the commit above in an existing local checkout that also
contains its adjacent `ultron/assets/b2` directory. A preserved local copy may
instead use the `.py.local` suffix to avoid being treated as a distributable
Python module. The loader executes that file and rejects any SHA-256 other than
the recorded snapshot; use only trusted local code. It does not download the
file. For example, in the original workspace where the local copy exists:

```bash
ULTRON_HALL_SCENE="$PWD/src/ultron/detection_world/hall_scene.py.local" \
  PYTHONPATH=src MUJOCO_GL=glfw runs/detection-venv/bin/python scripts/detection_world_viewer.py
```

Set the same variable for capture commands to reproduce that scene. An unset
variable, or an empty value, selects the independent example; a missing or
mismatched nonempty path fails rather than silently switching scenes. The local file is
ignored by Git. Build release packages from a clean Git archive, not a dirty
workspace that may contain private files or stale build artifacts.

Short GUI startup and shutdown checks succeeded in a local environment; they
were for the earlier Sahil-derived scene and do not establish camera or
robot-control functionality. Use the portable setup above for a separate
installation.

## Output contract

`instances.json` contains COCO-style `images`, `annotations`, and `categories`
(1 ladder, 2 inspection_label); image file names are relative to the output
directory. Annotation `id`, `image_id`, `category_id`, `bbox` in pixel
`[x,y,width,height]`, `area`, `iscrowd=0` and uncompressed COCO RLE
`segmentation` are present. `instance_id` groups visible parts of each
synthetic object within this scene. Bboxes and masks derive strictly from
rendered geometry IDs: occluded and off-screen pixels are excluded, fully
hidden objects have no annotation. The label card and stripes are one label
instance; their pixels are not additionally included in the ladder mask.
Thin geometry and small labels may disappear at low resolution.

Each image contains a camera record: position in world metres,
`rotation_world_from_camera` (3x3 row-major rotation; MuJoCo camera axes
right/up/back), target, vertical FOV and pinhole `fx`, `fy`, `cx`, `cy` in
pixels. These are ideal simulated intrinsics and extrinsics, not B2
calibration. Images are deterministic for the same scene, views, dimensions
and rendering environment; different graphics drivers may differ. The
manifest records the scene identity. The independent example uses group
`independent-example-v1`; the optional original retains
`detection-hall-v1` for continuity with earlier captures. Treat each entire
scene as one group and independently audit any related geometry or data across
splits. No COCO benchmark, real OCR or physical validation is implied.

## Local verification and remaining work

On Python 3.10.12 with MuJoCo 3.17.0 and EGL, the independent default
produced five 640x480 views with 23 visible ladder instances, 13 visible label
instances and one negative image. These counts are segmentation ground truth,
not detector results; no model performance is established for this scene.

In an earlier Sahil-derived local run, MuJoCo 3.3.7 with EGL generated five
640x480 PNGs
in `runs/detection-world-1128-final/`: 17 visible ladder and 13 visible label
instances, including a negative image. The evaluator accepted this dataset
with an empty-prediction fixture and separate image-based label and YOLO-World
ladder baselines. Results and limitations are in
[DETECTION_EVALUATION.md](DETECTION_EVALUATION.md). Four locally sourced real
photos were also checked with the ladder detector, but they are not a verified
held-out set; inspection labels were unassessed. These numbers do not describe
the new independent default scene.

Choose and review individual public images and their rights, annotate
inspection labels if present, establish independent physical-ladder and
capture-session groups, and run independent ladder and label predictors
before reporting held-out real results. Public-source candidates and their
limits are in [DETECTION_WORLD_SOURCES.md](DETECTION_WORLD_SOURCES.md).
Autonomous acquisition, label OCR and checklist drafting from these images
remain separate work.
