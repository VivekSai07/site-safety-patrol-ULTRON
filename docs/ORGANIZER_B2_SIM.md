# Organizer B2 simulation: isolated DDS smoke test

This is simulation only. The organizer kit is an interface and dynamics
reference, not a B2 walking controller, hardware connection, inspection
camera, or complete ladder-inspection pipeline. No hardware connection or
robot motion is authorized by this guide.

## Source and observed run

The local kit checkout was inspected at revision
`8f5de5806fcd432d339ca1dbb61a1f0f9ea1d6cd`. From the team repository its
`unitree-mujoco` directory is `../ctrl-hackathon-2026-kit/unitree-mujoco`.
The isolated image `ultron-organizer-b2:1127b2001` was built from that kit;
its image ID was
`sha256:1fc95a07c9e7b7a4589f04dd758440d9c6cb4ff0a0a8e5c7fba00ea9fd1c5121`.
The team launcher adjusts build scratch paths without changing the source
checkout; the kit's pinned dependencies remain the reference.

A separate bounded **headless, no-stand** test ran this image on DDS domain
71 and private loopback. The log recorded `robot up: b2`, 12 actuators and
601 received `/lowstate` samples at 199.99 Hz, with finite first 12 joint
positions and IMU values. Observed topic types were `/lowstate`
`unitree_go/msg/LowState`, `/lowcmd` `unitree_go/msg/LowCmd`, and
`/sportmodestate` `unitree_go/msg/SportModeState`; topic presence is not
proof of a working walking service. The receive rate is one local
observation, not a guaranteed rate or hardware specification.

The run inspected read-only roots, no mounts, IPC disabled, dropped
capabilities and a private shared loopback namespace. Task-owned containers
were removed afterward. Its local log is ignored run data at
`runs/organizer-b2-headless-evidence-20261127-final/smoke.log`; do not
publish it without the applicable review.

The headless run itself did not test stand-up, viewer pixels, walking,
capture, inspection, physical hardware or the label-first workflow. It is not
the viewer/stand probe below and does not establish a full-system simulation
gate.

## Isolated organizer viewer and stand probe

The separate team wrapper ran the kit's simulator and stand controller using
the same named image and local kit checkout revision. On isolated
container loopback, the viewer returned HTTP 200 and a noVNC WebSocket
upgrade (101) with an RFB greeting. The saved viewer PNG was visually
inspected and shows a standing simulated B2. The stand log and ROS probe
recorded 1,432 `/lowcmd` messages with finite first 12 motor positions and
598 `/lowstate` samples at approximately 198.93 Hz with finite first 12 joint
positions and IMU values. These counts/rates are observations from this run,
not guarantees or hardware measurements. Sport request/response topics
appeared from the stand client; their presence does **not** establish a
simulated walking service.

The wrapper used read-only container roots, isolated IPC, dropped
capabilities, a private shared network-none loopback namespace, no host ports,
and a task-specific XKB cache under the output directory. Task containers
were cleaned up. Evidence is local ignored run data under
`runs/organizer-b2-viewer-20261127e/`; do not publish it without applicable
review. The local checkout revision is not independent image-provenance
attestation. This is a wrapped isolated probe, not proof that the stock
unmodified Compose viewer launch works.

The VNC and noVNC services do not authenticate users. Loopback binding is not
authentication on a shared host. Do not expose this viewer to a shared network
or run it unless its simulation-only isolation has been verified.

The probe command was:

```bash
python3 -B scripts/organizer_b2_viewer.py --image ultron-organizer-b2:1127b2001 --out runs/organizer-b2-viewer-20261127e
```

The output must be a **new directory directly under `runs/`**; the recorded
directory already exists, so choose a different name for a repeat. Browser
GUI interaction and host access to this viewer were not tested. No walking,
hardware operation or completed inspection was established.

## Headless usage

Run from the team repository root with Docker and Docker Compose available.
The first command validates the viewer-mode Compose configuration without
starting it; the second builds an image using the same run ID, which must be
6–24 lowercase alphanumeric characters and must not already have an image
tag. Use an unused loopback port and a fresh run ID in another run. The third
command starts a uniquely named, isolated headless project, waits for B2
startup, probes ROS and removes its own containers:

```bash
python3 -B scripts/organizer_b2.py --kit ../ctrl-hackathon-2026-kit/unitree-mujoco --run-id 1127b2001 --port 6081 config
python3 -B scripts/organizer_b2.py --kit ../ctrl-hackathon-2026-kit/unitree-mujoco --run-id 1127b2001 --port 6081 build
python3 -B scripts/organizer_b2_headless.py --image ultron-organizer-b2:1127b2001
```

The named image already exists in the recorded environment, so do **not**
rerun its `build` command with the same ID; choose a fresh ID or use the
existing image for the headless command. `config` and `build` require the kit
directory; the headless command requires an existing image. Inspect local
results and environment restrictions rather than treating this example as a
hardware launch.

## Separate host-published Compose viewer mode: unverified

`scripts/organizer_b2.py` also exposes `up`, `check` and `down` for the
same kit, run ID and port. Its separate
`config/organizer-b2.compose.yaml` uses a private internal network, shared
container loopback, domain 71 and a host-loopback-only viewer port. When
appropriate for the environment, its proposed lifecycle is:

```bash
python3 -B scripts/organizer_b2.py --kit ../ctrl-hackathon-2026-kit/unitree-mujoco --run-id 1127b2001 --port 6081 up
python3 -B scripts/organizer_b2.py --kit ../ctrl-hackathon-2026-kit/unitree-mujoco --run-id 1127b2001 --port 6081 check
python3 -B scripts/organizer_b2.py --kit ../ctrl-hackathon-2026-kit/unitree-mujoco --run-id 1127b2001 --port 6081 down
```

The proposed browser URL is `http://127.0.0.1:6081/vnc.html`. These
viewer/stand actions were **not run** in the recorded session: the upstream
runtime and display use temporary paths prohibited in that environment.
`check` expects stand logs and a viewer HTTP response in addition to the ROS
probe. Do not infer success from a resolved configuration, built image or
headless results. Use the same ID and port throughout a task-owned project;
never stop unrelated containers.

The derived label-capture adapter is a separate, kinematic simulation mode,
not the above DDS smoke test. Its host-accessible launcher and required
cleanup are documented in [label-first usage](LABEL_FIRST_USAGE.md). Host
HTTP/PNG/capture endpoints and bounded browser keyboard/focus behavior were
exercised separately in specified simulation setups. Its host UI is not the
isolated organizer viewer above, whose browser interaction remains untested. See
[label-first baseline](LABEL_FIRST_BASELINE.md) and [validation](VALIDATION.md)
for current status and boundaries.
