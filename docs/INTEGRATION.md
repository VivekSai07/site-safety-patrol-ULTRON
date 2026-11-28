# B2, DimOS and visualization integration

## Current next baseline: label-first, operator-positioned simulation

The prioritized next workflow is a simulated operator selecting/positioning
one ladder, capturing overview and readable label evidence, and producing a
minimal evidence-linked example draft for human completion. Capture, draft-contract
and local adaptive-drafting code exists; a rendered synthetic HTTP probe
produced a draft using manual fixture transcription and a locally retained
source-specific catalog, separately from the existing demo. Browser controls and focus were checked separately in
specified simulated setups; real OCR remains unverified. See
[validation](VALIDATION.md). The local source-specific catalog is excluded
from distribution pending rights review; the public synthetic example is not
the official form or a complete mapping. The source form and five other
forms require separate rights and completeness review. See
[LABEL_FIRST_BASELINE.md](LABEL_FIRST_BASELINE.md).
No source images are transmitted by this documentation change.

Autonomous acquisition is a later milestone. The existing MuJoCo kinematic
demo, the organizer DDS probes, and the derived evidence-capture mode are
separate workflows; startup or message flow in one does not prove
walking, inspection, or the others. A headless DDS smoke test observed finite
LowState messages; a separate isolated viewer/stand probe observed a standing
simulated B2 and finite command/state samples. Browser GUI/host access,
walking and hardware remain untested. See the
[organizer B2 guide](ORGANIZER_B2_SIM.md). No hardware connection or command
is authorized.

## Dashboard and assessment boundaries

The separate dashboard, PDF path and versioned assessment packs are present;
see [DASHBOARD.md](DASHBOARD.md) and [API_PROMPTS.md](API_PROMPTS.md).
`ultron.assessment` constructs a hash-bound request and validates a proposal
without calling a model. The local OpenAI adapter is a distinct, optional
live path and requires explicit image/text-sharing approval; locally selected
catalogs are fixture-only. Neither proves
integration with the teleop draft.

Dashboard goals default off. Its optional ROS bridge can forward a Nav2 action
goal, but does not independently verify isolation from real hardware. Do not
start it where a real action server is reachable. Localhost, ROS domain and
namespace settings alone are not proof of isolation; this reconciliation
authorizes no hardware connection.

## B2 is not a Go2 transport substitution

Use the event starter and verified high-level B2 interface. DimOS's documented Go2 connection/blueprint is not evidence that Go2 WebRTC will control this B2. Unitree's low-level MuJoCo bridge and B2 description are useful for model/interface study; they do not supply the autonomous walking service used by the real robot. This repo sends no hardware commands.

Keep robot drivers, localization, RC priority, watchdog and original recording local. Offboard perception can receive timestamped RGB, aligned depth, intrinsics and shutter-time camera-to-map transforms. Return candidate ladder observations, not raw LLM movement instructions. A motion supervisor must validate goals against the actual footprint, zone, costmap and operator state.

Replace `Simulation.oracle_detections` with a detector adapter and `mission.py` scripted checks with a validated inspection reader. Separate detection ID, persistent ladder ID, optional serial, model version and source evidence. Do not pass fixture keys as production identifiers. Validate types/confidence/evidence integrity and calibrate thresholds on event data. Keep hard failures, conflicting views and unavailable fields explicit.

## DimOS memory path

The SQLite registry (`memory.py`) is the MuJoCo mission's store; ULTRON's spatial memory is now `src/ultron/pipeline/`. It is not a renamed DimOS implementation and includes no semantic embeddings. An optional adapter writes inspection histories into the real upstream `SqliteStore`:

```bash
# In an existing DimOS environment with its supported Python (upstream currently <3.13)
PYTHONPATH=src python -m ultron dimos-ingest runs/ultron-demo --database /tmp/ultron-dimos.sqlite
```

Then the upstream memory stream can be queried with `store.stream('ultron_inspections').near((3,6,0),radius=2).to_list()` or time/tag filters. Runtime compatibility of this adapter remains unverified. Avoid installing all of DimOS into ROS's system interpreter; use a compatible separate environment and pin its commit.

Future live integration: an actual DimOS Module consumes observations, calls the registry/policy and emits inspection events to memory/visualization. Its B2 connection is a separate adapter. Reuse the organizer SLAM/navigation baseline rather than replace it during bring-up.

## Rerun

Rerun export reads portable telemetry and metric depth, transforms MuJoCo camera right/up/back to optical right/down/forward, and logs robot/site primitives, path, evidence, asset pins and event logs. Target is `rerun-sdk==0.32.0`, matching inspected DimOS dependencies. SDK is not installed locally: export should fail clearly until installed; the simulation remains runnable. A raw JSONL file is not an RRD and is never labeled as one.

## RViz

`scripts/rviz.sh RUN` uses system `/usr/bin/python3`, sources Humble, sets localhost-only ROS domain 71, and replays the same frames. It visualizes dynamic B2 primitives (not a robot_state_publisher URDF), scene markers, track labels, path, RGB and base TF. The depth recording/camera extrinsics are preserved for Rerun; the current ROS replay does not publish depth or CameraInfo. It is an offline visualization node, not a sensor/nav bridge. Do not reuse its simulation frames as real robot frame names.

The historical host-side RViz validation had headless MuJoCo EGL rendering but
no working display or host ROS DDS sockets. Syntax/import checks did not
establish a successful RViz session. The later organizer headless test used
isolated container loopback and does not validate RViz or host DDS. On a
normal Ubuntu/Humble desktop, run the command and verify topics and camera
display before an event demo.
