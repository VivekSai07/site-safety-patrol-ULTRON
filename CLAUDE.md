# ULTRON Claude guidance

Follow [AGENTS.md](AGENTS.md), the shared instruction source, then [README.md](README.md) and [docs/SOURCE_REVIEW.md](docs/SOURCE_REVIEW.md).

The user authorized this team-side B2 simulation repo. Do not create organizer approval steps. New code lives in `src/ultron`; the earlier text-only demo remains as reference.

Follow the [simulation-to-hardware policy](docs/SIM_TO_HARDWARE.md): do most work and validation in simulation. Complete integrated hardware deployment needs documented full-system simulation acceptance and a separate supervised hardware-readiness gate. Earlier targeted component hardware tests are allowed only with appropriate tests and risk assessment, authorization and a bounded plan; any motion also requires verified RC priority, an independent watchdog/stop, a functional E-stop, a measured environment and a controlled operator. Any hardware connection needs explicit user/operator authority and independent safety validation; this reconciliation authorizes none.

Optional advanced intelligence has no control authority. Proposed System One is off by default, offboard and read-only/audit-only after the hard-fail scan, with no route to motion, verdict or veto.

Proposed simulation launch and test examples (verify launch details and numeric thresholds for the intended environment): `PYTHONPATH=src MUJOCO_GL=egl python3 -m ultron demo`; `PYTHONPATH=src python3 -m unittest discover -s tests -v`. Set `ULTRON_RENDER_TESTS=1 MUJOCO_GL=egl` for camera-rendered integration tests.

The model is official B2 kinematics with primitive visuals, kinematic base motion and simulation-oracle detections/findings. Preserve those disclosures and the Unitree license. Scoped pass is not a safe-to-use certificate. Real B2 motion, Rerun runtime, DimOS runtime and ROS/display restrictions have distinct verification boundaries documented in README.md.

For the integrated evidence worker, full360 experiment and occupancy guard, read [docs/HARDWARE_READINESS.md](docs/HARDWARE_READINESS.md) and the integrated-patrol invariants in AGENTS.md. Cloud results carry no motion authority; unresolved flags keep their original evidence.
