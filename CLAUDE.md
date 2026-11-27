# ULTRON Claude guidance

Follow [AGENTS.md](AGENTS.md), the shared instruction source, then [README.md](README.md) and [docs/SOURCE_REVIEW.md](docs/SOURCE_REVIEW.md).

The user authorized this team-side B2 simulation repo. Do not create organizer approval steps. New code lives in `src/ultron`; the earlier text-only demo remains as reference.

Run `PYTHONPATH=src MUJOCO_GL=egl python3 -m ultron demo`. Tests: `PYTHONPATH=src python3 -m unittest discover -s tests -v`; set `ULTRON_RENDER_TESTS=1 MUJOCO_GL=egl` for camera-rendered integration tests.

The model is official B2 kinematics with primitive visuals, kinematic base motion and simulation-oracle detections/findings. Preserve those disclosures and the Unitree license. Scoped pass is not a safe-to-use certificate. Real B2 motion, Rerun runtime, DimOS runtime and ROS/display restrictions have distinct verification boundaries documented in README.md.
