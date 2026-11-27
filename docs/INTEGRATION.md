# B2, DimOS and visualization integration

## B2 is not a Go2 transport substitution

Use the event starter and verified high-level B2 interface. DimOS's documented Go2 connection/blueprint is not evidence that Go2 WebRTC will control this B2. Unitree's low-level MuJoCo bridge and B2 description are useful for model/interface study; they do not supply the autonomous walking service used by the real robot. This repo sends no hardware commands.

Keep robot drivers, localization, RC priority, watchdog and original recording local. Offboard perception can receive timestamped RGB, aligned depth, intrinsics and shutter-time camera-to-map transforms. Return candidate ladder observations, not raw LLM movement instructions. A motion supervisor must validate goals against the actual footprint, zone, costmap and operator state.

Replace `Simulation.oracle_detections` with a detector adapter and `mission.py` scripted checks with a validated inspection reader. Separate detection ID, persistent ladder ID, optional serial, model version and source evidence. Do not pass fixture keys as production identifiers. Validate types/confidence/evidence integrity and calibrate thresholds on event data. Keep hard failures, conflicting views and unavailable fields explicit.

## DimOS memory path

The implemented SQLite registry is ULTRON's small offline memory. It is not a renamed DimOS implementation and includes no semantic embeddings. An optional adapter writes inspection histories into the real upstream `SqliteStore`:

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

This execution session permits headless MuJoCo EGL rendering but denies working display and ROS DDS sockets. Syntax/import checks do not establish a successful RViz session. On a normal Ubuntu/Humble desktop, run the command and verify topics and camera display before the event demo.
