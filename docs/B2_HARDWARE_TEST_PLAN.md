# Unitree B2 hardware verification plan

**Status: PLANNED.** Applies to `sahil/sim-slam-nav2` and
`sahil/3d-slam-nav2`. Everything starts unverified for the physical robot.
The plan covers a stationary data check, bounded supervised motion, mapping,
and a dashboard-to-Nav2 check. The 2D live-map and 3D projection runs provide
simulation evidence. The SDK velocity wrapper provided stand-in evidence.
[Simulation evidence](SAHIL_VALIDATION.md) · [Hardware prerequisites](SIM_TO_HARDWARE.md) · [Current readiness](HARDWARE_READINESS.md)

## Operator sign-off at every verification

A connected cable, an available robot, and “start the plan” are signals to
present the first verification. Before **each** row below, including each
motion direction, each branch-specific check, and each retry, the agent asks
the user for explicit approval and waits for a reply. The request names the
check ID, branch and revision, robot state, operator, permitted action,
limits, abort conditions and evidence. Ask for one check at a time. Confirm
the named on-site operator is ready before any test that could actuate the
B2. An approval expires if the setup, operator, software revision, limits
or expected physical state changes.

When a row calls for multiple trials, such as RC override and E-stop, treat
each trial as its own verification with separate approval and confirmation.
After each check, report the observation, evidence and proposed result
(`PASS`, `FAIL`, `BLOCKED` or `UNRUN`). Ask the user to confirm the
result before proceeding. The operator independently confirms motion and
stopping outcomes. A failed, uncertain or interrupted check stops the
sequence. Resume only after an agreed diagnosis and a fresh approval for
the specific retry. Retain separate records for the two branches. To reuse
shared evidence such as an E-stop check, present the original result and ask
the user to confirm that it applies to the same robot, configuration,
operator and session.

Example prompts

> Approve D1 on `sahil/sim-slam-nav2` at `<revision>`? The operator will keep
> the robot stationary and the command path disconnected. I will observe
> only the agreed topics on the verified interface for the agreed window.
> I will stop if the robot moves or an unexpected command publisher appears.

> D1 observed the agreed LiDAR, camera and IMU messages. The recorded TF has
> no LiDAR-to-base transform. Evidence is `<local run path>`. Proposed result
> is BLOCKED for downstream mapping. Please confirm.

## Session record and preconditions

### Jetson Python environment

Use ROS Humble's system Python 3.10 and a venv with system site packages. The
offline setup below can repair the existing team venv in place. For a clean
environment, substitute a fresh path for `.venv-teamultron` in both commands.
The script installs dashboard dependencies and checks imports without
starting ROS or connecting to the B2.

```bash
ULTRON_ROS_VENV="$PWD/.venv-teamultron" bash scripts/setup-ros-venv.sh --hardware-preflight
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}" .venv-teamultron/bin/python -m unittest discover -s tests -v
```

Run from the branch under test. Save the full test footer and exit code
together. The reported 424 total, 422 passes, 58 skips and two errors cannot
describe a single run. Missing imports block the affected checks until the
rerun. Check branch-specific ROS packages separately before running SLAM.

### Next-slot checklist from the 2026-11-28 session

Before the slot, prepare the checkout for each branch and record its commit,
host and exact interpreter for each process. Run the offline venv setup and
test command above.

Use `.venv-teamultron/bin/python` for the SDK check. Verify the
home-directory SDK checkout against the approved revision `814556d` and
make that checkout and its compatible CycloneDDS dependency importable
from this interpreter. Import
`cyclonedds`, `unitree_sdk2py` and the B2 SportClient interface, and check
that `Move` and `StopMove` match the wrapper contract. Record the import
paths and revision. Keep this check import-only. The previous session hit
missing SDK and CycloneDDS imports in different environments.

For D1, prepare a list of expected publishers, producer hosts, middleware
and domains, topic types, QoS, and intended consumers. After approval,
verify actual message receipt for robot state, the LiDAR cloud, and each
branch's required odometry or IMU, timestamps and TF. The previous session
observed raw Unitree low state and sport state at about 500 Hz. Verify the
raw DDS state and ROS 2 sensor topics separately on the intended hosts.
For the approved read-only DDS check, use `eno1` and domain `0`, as
observed in D1. Sample `rt/lowstate` and `rt/sportmodestate` with
`.venv-teamultron/bin/python`. Confirm the interface and domain still match
the setup before connecting.

For 2D, verify the LiDAR cloud, projected scan, selected odometry source
and required TF at slam_toolbox. For 3D, verify the cloud and TF at
ICP/RTAB-Map and check its pose and 2D occupancy outputs. Limit the check
to the publishers used by the selected stack.

At the slot, present P1 with the offline results already filled in and list
only the missing facts needed for the next check. After P1 and D1 approvals,
work through stationary data and calibration first. Request keyboard motion
only after the command path and independent protections are verified.
Proceed to mapping and then Nav2 shadow checks when their inputs pass.

### Calibration handoff for D2

Keep the supplied `calibration.zip` with the local session evidence. Before
the slot, use its matrix inventory to prepare a table of expected parent
and child frames, transform direction, units, source and confidence.
During D1 record actual frame IDs, camera resolutions and timestamp
sources. During D2 compare that inventory with live TF and marked physical
references, and verify transform availability at sensor timestamps.

The archive provides D455 stream intrinsics and factory depth-to-color
extrinsics. Camera-to-base and LiDAR-to-base mounts come from the robot
model. The IMU mount differs between the xacro and CAD references. Resolve
that difference on the installed robot before using the IMU transform.
The archive contains a LiDAR-to-IMU transform and time offset from FAST_LIO
configuration. Verify them independently before using them in this stack.
Camera-to-LiDAR alignment is nominal. Require measured alignment for any
check that uses camera-to-LiDAR projection or cross-sensor association.

Before the slot, identify the host, launch entry point and expected output
for every required producer, and record the exact consumer environment.
Enter the slot with the K1 import output and the D1 stream inventory ready
for review. During D1 check actual message receipt on the SLAM consumer
and mark any missing stream against its dependent check.

### Record fields as they become relevant

P1 is an offline review. For D1, establish read-only authorization, on-site
roles, the verified connection, physical isolation of commands, the
stationary state and abort conditions. D1 discovers actual topic, QoS and
timing details. D2 verifies calibration. P2 and P3 verify the protections
needed before motion. The results of those later checks are not P1 or D1
entry requirements.

Fill these with measured or verified hardware values before the approval for
a dependent check. Leave unknown values blank and mark the dependent check
`BLOCKED`. The on-site operator keeps direct control throughout. Use Unitree's
procedures for the actual B2 model and firmware and the site's agreed stop
procedure. A software stop or an SDK return value alone cannot establish
that the robot stopped.
[Unitree B2 manuals and remote-control guidance](https://www.unitree.com/app/b2/) · [B2 SDK source at the organizer kit revision](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/b2/sport/sport_client.py)

| Record before use | Required information |
|---|---|
| People and equipment | Named user, authorized on-site operator, responsible safety reviewer and spotter as applicable. Actual robot variant, firmware, controller/RC, batteries and alerts, sensors, compute host, ROS and SDK versions. |
| Transport and command ownership | Verified starter interface, network interface and DDS domain, message types and QoS. Identify all command publishers and the sole actuation path. Identify how that path is physically isolated for read-only and shadow tests. Keep dashboards with goal entry confined to the approved operator interface. |
| Site and limits | Measured flat test zone, floor condition, marked start and endpoints, footprint and swept turning envelope, exclusion perimeter and clear stopping reserve. Record approved max linear/angular speed, acceleration, distance/angle, command duration, allowable stop time/distance and arrival tolerance. Start at the lowest reviewer-approved setting. |
| Protection and fail-safe | Verified RC priority, functional and accessible E-stop, an independent watchdog/stop, and an operator ready to use the agreed stop procedure. Record the test method, last verified result, expected state on loss of control, and how the system remains inhibited on restart or reconnect. |
| Sensor and map contract | Actual topic/frame list, acquisition and receive timestamps, clock source, calibration/extrinsics, localization age and agreement limits, map identity/version, operating boundary and occupancy convention. Record which means will independently measure travel and heading. |

The current repo does not contain a verified real B2 command path. The
existing simulation launchers, simulation topics and stand-in log are
simulation tools. Physical command checks depend on a separately verified
B2 transport and operator-approved control procedure. Keep control out of
perception, VLM and inspection code.

## Test sequence

Each row is a separate approval, including the individual rows under
controlled motion. Follow the dependency shown in each row. A hardware case
must have the component-appropriate bench, replay or simulation evidence
and test-specific risk controls recorded first. Review live data, mapping,
and Nav2 in shadow mode before considering physical autonomy.

### Preflight and stationary data

| ID | Verification and limit | Evidence and pass condition |
|---|---|---|
| P1 | Review the session record, permissions, relevant simulation/bench evidence, site and risk controls offline. | Record offline evidence, environment issues and the entry requirements for the next check. Assign each later gap to its dependent check. |
| D1 | Stationary read-only connection on the verified interface with the command path isolated. Observe agreed sensor and robot-state topics. | Actual topic names, types, QoS, frame IDs, rates, gaps, timestamps and sample counts recorded for the agreed window. LiDAR returns and required camera/IMU or odometry streams are plausible. Missing optional inputs are recorded explicitly. Any motion or unexpected actuation publisher aborts. |
| D2 | Verify calibration, timestamps and TF while stationary against known references in the test zone. | The actual `map -> odom -> base_link -> sensor` chain is documented, including which transforms can only be verified after mapping. Required sensor transforms are available at acquisition time, angle and distance agree with the references within approved tolerances, and there are no duplicate TF authorities or unsynchronized clocks. |
| P2 | Independently verify RC priority and E-stop on the actual equipment using the reviewed Unitree/operator procedure and an appropriate safe bench setup. | Reviewer and operator confirm override and stop behavior independently of ULTRON. Any bench procedure that could move the robot needs its own bounded approval and setup. An unverified protection blocks all floor motion. |
| P3 | Verify the independent watchdog and restart inhibition in the approved bench setup. Exercise command loss and stale commands within that procedure. | Watchdog stop is observed independently of ULTRON. Loss, stale data and restart leave actuation inhibited. An SDK success code without observed confirmation is insufficient. |

### Keyboard-first qualification

Use a keyboard on the laptop or Jetson. If using SSH, run the keyboard
reader in the Jetson terminal. Route requests through the motion gate and
verified B2 transport, with a recorder as the first output. Keep the
verified RC override and independent stop available. Keep Nav2 and
dashboard motion disabled during manual control.

| ID | Verification and limit | Evidence and pass condition |
|---|---|---|
| K1 | Before D1, check the B2 SDK offline in `.venv-teamultron` without initializing DDS or opening a robot transport. | The SDK revision and B2 SportClient interface match the approved starter. Record pending runtime checks. |
| K2 | After D1, test WASD with output routed only to an isolated recorder. Approve each input case separately. | **W** requests forward motion, **S** reverse, **A** left turn and **D** right turn. Verify that a key press cannot latch motion. For a terminal that only reports key presses, verify a bounded command timeout when key events stop. Test explicit stop, SSH disconnect, process exit and restart inhibition. |

Complete K1, D1 and K2 before the keyboard movement trials, and complete the
protection checks before any floor motion. Use WASD in M2–M5, one approved
direction at a time. Then record a short mapping route and verify the map
before a Nav2 shadow goal. A physical Nav2 goal still requires N3 approval.

### Controlled motion

Requires P1–P3 and K1–K2, named operator consent and an independently checked clear
test zone. Keep Nav2, dashboard goal dispatch and automatic recovery
disabled during these checks. The operator controls each move and confirms
stationary state before another move is considered. Measure actual travel,
heading and stopping against pre-marked references. Begin with an approved
shorter move when needed to establish braking. The first distance to verify
is up to **1 m**, with a separately approved extension up to **2 m** only
after measured stopping and clearance support it.

| ID | Verification and limit | Evidence and pass condition |
|---|---|---|
| M1 | Verify the approved B2 command path at rest and controlled enable and disable. Start from the operator-approved stationary state. | An absent enable, zero command and disable produce no unexpected motion. The actual B2 transport, watchdog, operator override and sole publisher are confirmed. |
| M2 | Forward move on the marked straight segment, up to the approved distance within 1 m. | Direction, distance, heading and stop time/distance independently confirmed within limits. No movement after stop. |
| M3 | Reverse move on the marked straight segment, up to the approved distance within 1 m. | Same checks as M2, with the rear corridor and stopping reserve verified separately. |
| M4 | Left turn in place through the approved small angle. | Correct turn direction, bounded translation and yaw, clear swept footprint and confirmed stop. |
| M5 | Right turn in place through the approved small angle. | Same checks as M4 for the opposite direction. |
| M6 | Optional forward motion with a gentle turn, after M2–M5. Approve one measured arc and its endpoint and stopping reserve. | Actual turn and displacement remain inside the assessed corridor. Abort for unexpected curvature, drift or clearance loss. |
| M7 | Optional distance extension, one specific direction per approval, up to 2 m. | Measured stopping reserve supports the longer move. Direction, distance, stability and stop remain within the approved limits. |

### Mapping and localization

| ID | Verification and limit | Evidence and pass condition |
|---|---|---|
| S1 | With the operator controlling any agreed route, record a short out-and-back data set in the marked flat zone. Approve each moving segment separately under the controlled-motion limits. | Local bag and marked reference poses with actual acquisition times and frame/clock provenance. No gaps beyond approved limits. Retain data locally. |
| S2a | On the 2D branch, process the approved data offline using the verified scan and odometry inputs. | The recorded walls and marked landmarks align within the agreed error. The occupancy map and pose graph save with a map identity. |
| S2b | On the 2D branch, check live mapping with actuation isolated. | Dashboard and RViz show an evolving 2D occupancy map, stable pose and preserved unknown space. |
| S3a | On the 3D branch, process the same authorized data offline with the intended 3D backend. | Inspect vertical structure and alignment against the references. Record accepted loop edges and separate alignment evidence when a revisit is evaluated. |
| S3b | Check live ICP and RTAB-Map with actuation isolated. | Inspect the 3D map in RViz and its 2D projection beside it. Verify floor/obstacle height treatment, unknown cells, distinct vertical structure, TF, fresh projection updates and a saved map database. |
| S4 | On **each branch separately**, check localization at marked positions, map reload, loss and recovery with commands isolated. Approve each case separately. Inject faults in replay or the isolated software path. | Pose age, frame, map ID/version, measurement comparison and state transitions are logged. Dropout or a discontinuity invalidates the pose and inhibits goals. Recovery requires fresh evidence and an explicit new enable before motion. |

### Dashboard and Nav2

| ID | Verification and limit | Evidence and pass condition |
|---|---|---|
| N1 | On each branch, display the real map and pose in RViz and the dashboard while the hardware command path stays isolated. Verify the application can reject goals without dispatch. | Dashboard and Nav2 use the same approved map frame/source. For live mapping, capture the displayed revision, the newer grid used by the goal checker and the goal outcome. Test wrong-map, stale/future revision, occupied, unknown, out-of-bounds and disabled-operator cases as separately approved checks. |
| N2 | On each branch, run a Nav2 shadow goal to a marked point in known free space with velocity output routed to a recorder. Approve fault/cancel checks separately. | Record plan, goal/result ID and command timestamps. Path and footprint stay in the expected zone. Cancel, stale localization, map loss and missing operator enable inhibit further dispatch. Verify disabled automatic recovery motions and no route to actuation. |
| N3 | One supervised physical dashboard-to-Nav2 goal on a branch with documented full-system simulation acceptance, completed hardware prerequisites, and a separate go/no-go decision. Approve one waypoint inside the already verified bounded route. | Independent arrival and stop measurements meet the approved tolerances. The operator confirms that RC takeover remains available and verifies the final stationary state. Each additional goal or physical fault test needs its own approval and bounded test plan. |

**Current dependency:** N3 is blocked on the available evidence until
full-system simulation acceptance, verified real B2 transport, and the
independent hardware checks have been recorded. Earlier scoped component
tests can proceed only with their own prerequisites and authorization.

### Phase 5 — Tune 3D SLAM on the tiled floor

Run this on `sahil/3d-slam-nav2` after Phase 4. Use the Phase 3 3D bag
for repeatable replay with actuation isolated. Include a stationary tiled
patch, a repeat pass, and known obstacles at floor level in the evidence.
Keep TF, input data and playback conditions fixed. Start with the Phase 3
hardware configuration and change one parameter group at a time. Begin
each mapping replay with a fresh database and record effective parameters.

| Trial | Parameters and comparison |
|---|---|
| Baseline | Measure input and processed cloud rates, ICP odometry rate, occupancy publication rate, and acquisition-to-grid latency. Record gaps and Jetson CPU and memory use. |
| Map update cadence | Compare `Rtabmap/DetectionRate` at 1, 2 and 4 Hz against measured input and compute capacity. At the best rate, compare the baseline `RGBD/LinearUpdate` and `RGBD/AngularUpdate` gates with small thresholds chosen from the observed pose noise. Check stationary churn and update delay during movement. |
| Floor and reflection handling | Inspect raw and registered clouds over tile and near real low obstacles. Measure floor height in the grid frame before testing `Grid/MaxGroundHeight` and `Grid/NormalsSegmentation`. Compare conservative range limits with `Grid/RangeMax` and, separately, input range filtering if artifacts affect ICP. Check the 2D projection for both ghost obstacles and false free space. |
| Registration stability | If ICP jumps or drops frames, compare `Icp/VoxelSize` and then `Icp/MaxCorrespondenceDistance` around the hardware baseline. Record whether the change applies to ICP odometry or RTAB-Map registration. Compare revisit alignment, wall duplication and pose discontinuities. |

Pick the configuration that preserves real low obstacles and unknown space,
reduces persistent floor artifacts, and meets the approved update latency
and localization limits. Score the 3D map and Nav2's 2D projection
separately. Verify the selected settings on another pass, check saved-map
reload and localization, and rerun the Phase 4 shadow goal with the tuned
projection. Approve each trial separately and retain its bag, parameter
dump, map and comparison notes locally.

### Slot order and evidence reuse

1. Start with `sahil/sim-slam-nav2`. Run Phases 1–4 after offline preflight, including stationary
   topics and supervised WASD checks, then mapping and Nav2. Capture a ROS
   bag during the Phase 3 mapping pass using the same live inputs consumed by
   SLAM.
2. Switch to `sahil/3d-slam-nav2` for mapping and Nav2. Present the 2D
   stationary-topic and WASD results for reuse approval. Carry them forward
   when the robot, operator, interfaces, sensor setup and control path still
   match. Recheck only changed or 3D-specific dependencies before mapping.
3. Capture a separate ROS bag during the 3D mapping pass. For each bag, log
   the branch and revision, actual SLAM input topics, required TF, clock
   source and message counts. Include the bag scope in the mapping approval.
4. Run Phase 5 on the 3D branch using its Phase 3 bag, then verify the
   selected SLAM settings on a separate pass.

For 3D cross-patrol association, pair the saved RTAB-Map database with a
persistent map ID and reload it in localization mode. Verify camera poses
in the saved map frame at marked references after a restart before enabling
cross-patrol association. The live launcher currently creates a fresh
identity and database path per run. A marked start can aid relocalization.
Use a fixed ID only when the shared coordinate frame has been verified.

### How the branches differ

| Branch | Map and localization checks |
|---|---|
| `sahil/sim-slam-nav2` | Check the real point cloud to 2D scan projection, verified odometry, slam_toolbox map/pose graph save and reload, and the live 2D occupancy grid for dashboard goals. The prior B2 bag replay used estimated LiDAR mounting and LiDAR odometry. Replace these assumptions with approved measured transforms and a verified input chain for hardware testing. |
| `sahil/3d-slam-nav2` | Check online 6-DoF ICP odometry and RTAB-Map from the actual 3D scan. For Nav2, validate its live 2D occupancy projection and the cut used for the measured floor and obstacles. Assess 3D map alignment and restart independently. The KISS-SLAM offline runs and RTAB-Map MuJoCo runs are separate evidence. The simulation speed and stopping numbers are simulation values. |

## Abort, shutdown and evidence

Stop immediately for perimeter intrusion, abnormal behavior, uncontrolled
motion, loss of the operator/RC/E-stop/watchdog, loss of required telemetry,
invalid localization, unexpected TF authority or a breached limit. The
operator uses the agreed independent stop procedure. Inhibit command
forwarding and cancel the active goal if available. The operator confirms
the robot has stopped before anyone approaches. Log the reason, first
fault time, stop time/distance, final state and recovery decision. Keep
motion disabled after a failure until a separately approved check.

For normal shutdown, confirm the robot is stationary with the operator,
cancel any active goal, disarm the command path, stop capture, and
disconnect through the agreed procedure. Ask for approval before any
verification of the reconnect or restart behavior.

Use one local run folder per session, with a separate check record per
branch and verification. Keep bags, site images, network identifiers and
credentials local. Commit only the plan and authorized non-sensitive
summaries. Suggested record:

```text
check_id / branch / commit / config fingerprint / local run path:
user approval and timestamp / operator and readiness confirmation:
robot, transport and sensor version / verified prerequisites:
scope / limits / independent measurement method / abort criteria:
command, sensor, map, TF and pose evidence:
expected result / observed result / stop state:
proposed verdict / user confirmation / unresolved items:
```
