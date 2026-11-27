# Pass 7 reconciliation: ULTRON and Om's plan

This is the decision record for the [Pass 7 Build Bible](bible/BUILD_BIBLE_PASS7.md).
The comparison baseline is team commit `0b97edb`; the changes on this branch
are described separately below. Om's Pass 6 Bible and the eight-report
integrated reference are retained in Om's personal workspace, not copied into
this team repository. `R-nn` refers to that reference's reconciliation
ledger. A source plan or decision is **not** evidence that the feature exists.

The existing [jury strategy](JURY_STRATEGY.md), [source review](SOURCE_REVIEW.md),
[pitch](PITCH.md), [handoff](EVENT_HANDOFF.md), [design](DESIGN.md) and
[previous validation record](VALIDATION.md) remain available for comparison.
The original organizer/jury PDFs are not present in this checkout; verify
their actual wording and edition when they are available. Do not treat this
summary as a substitute for them.

**Status terms:** *existing* means present in the repository; *simulation only*
means fixture- or model-based, not physical proof; *proposed* means design or
backlog, not implemented; *open* means a decision or measurement is pending.
Automated tests do not establish physical B2 safety or learned-model accuracy.

## Topic decisions

| Topic | Team baseline and Om's plan | Reconciled disposition |
|---|---|---|
| Objective and tiers | ULTRON already demonstrates bounded patrol, evidence, history and conservative outcomes in a simulator. Pass 6 protects an autonomous Tier 1, then bounded Tier 2 and experimental Tier 3. | **Keep** ULTRON's working demo and the tier/fallback structure. Tier 1 on a physical robot is a future acceptance target, not a current capability. |
| Deployment order | ULTRON originally said sim-only; the current request permits periodic partial hardware tests before the complete system passes simulation. | **Adopt** the [simulation-to-hardware policy](SIM_TO_HARDWARE.md): full integrated deployment waits for full-system simulation acceptance and independent hardware checks; individually bounded, authorized hardware tests may occur earlier. No robot is contacted by this change. |
| Executive | [`supervisor.py`](../src/ultron/supervisor.py) selects deterministic simulation tasks; Om proposes a `py_trees_ros` executive and four ROS 2 Action servers (R-05, R-56). | **Keep** the existing supervisor for the sim. Proposed ROS action adapters can be added later behind contracts; do not claim that ROS Actions, Nav2 or a behavior tree already run. |
| Actuation | [`motion.py`](../src/ultron/motion.py) records bounded simulated command profiles and has an explicitly invoked B2 SDK transport. It is not an independent watchdog or an approved hardware supervisor. | **Extend later** with a verified onboard control chain, RC priority and independent stop/watchdog. Proposed chain: executive → ROS Actions → Nav2 → `/cmd_vel` → `twist_mux` → watchdog → verified B2 bridge. The physical E-stop is independent. A 500 ms stop is a **MUST TEST** target, not a measurement (R-40, R-41, R-73). |
| Mapping and coverage | The demo uses a static 2-D grid, obstacle inflation and A*; Om proposes saved-map localization, PatrolZone and keep-outs. | **Keep** the grid for simulation; add real localization and the verified starter stack later. Waypoint completion and simulated `MISSION_COMPLETE` do not establish searched-area coverage. |
| Mission modes | The demo records `AUTONOMOUS_SIMULATION` or an injected `DEGRADED_TELEOP_ASSISTED` case; Pass 6 describes setup, autonomous patrol and guarded operator recovery. | **Merge** the vocabulary as a proposed future contract. Real operator preemption, recovery and guarded resume are not implemented. |
| Discovery and pinning | The MuJoCo demo uses segmentation and fixture-key oracles, projected through simulated depth. Om specifies timestamped RGB/depth/CameraInfo/TF and at least three consistent observations. | **Keep** the oracle honestly labelled; **propose** a real sensor adapter and consistency-checked confirmation. The current three fixture-key sightings are not validation of a real detector, TF or pin accuracy. |
| Type and scope | The runtime uses six string types and per-type lists in [`inspection.py`](../src/ultron/inspection.py). Om specifies the same types, explicit aliases, visibility and condition as separate axes, and a conservative UNKNOWN type (R-07, R-76). | **Keep** the current strings and schema for compatibility. **Extend later** with validated aliases and a reviewed complete applicability/UNKNOWN policy. The present `UNKNOWN` template is not the proposed full component superset. |
| Registry and history | [`memory.py`](../src/ultron/memory.py) uses SQLite `missions`, `assets`, `world_objects`, and `inspections`, with JSON payloads and map/type-aware association within 0.65 m. Om proposes normalized event/view/check tables, foreign keys and stronger ambiguity checks (R-06, R-36). | **Keep** existing data and IDs; record the 0.65 m and proposed 1.0 m gates as different, uncalibrated designs. Do not silently change a live database. Design and test an explicit versioned migration, foreign-key enforcement and ambiguity behavior before adoption. |
| Missing assets | The registry marks `MISSING_EXPECTED`; the demo's `COVERAGE_INCOMPLETE` summarizes its benchmark. Om separates an asset state from mission coverage (R-34). | **Keep** those axes separate. A missing component on a seen ladder is `UNKNOWN`/`MANUAL_REVIEW`, not a site-coverage verdict. |
| Views and recovery | The demo takes an overview, close-up and at most one scripted side recovery. The prior candidate utility permitted a full ring for platform ladders. Om limits rear exploration to open-space A-frames needing rear evidence, and keeps it Tier 3 (R-01, R-44). | **Change** the simulator utility: full-ring candidates require `A_FRAME_STEPLADDER`, explicit `open_space_confirmed=True` and `rear_evidence_missing=True`; otherwise only its nominal fixed-axis candidates are returned. This is not a real-world clearance or line-of-sight proof. The demo still uses its existing one-retry path. |
| Pose validation | ULTRON's grid checks free cells and available A* paths. Om proposes full-footprint validation against an unmodified costmap and six distinct gates (R-25, R-26). | **Keep** grid checks in simulation; the merged six-gate order and real footprint validation remain **proposed**. Never clear target cells or reduce keep-outs to force close-up acceptance. Retain the rejected pose, reasons and evidence. |
| Tilt | No verified body-pose integration exists. Sources disagree on SDK/ROS posture APIs and limits (R-02, R-67). | **Keep disabled** until an approved interface, return-to-neutral, stability, timing and TF are tested; a normal overview/close-up remains the fallback. |
| Evidence and verdict | `verdict()` scans supplied failures first, then completeness. Model outputs remain uncalibrated; `evidence_gate()` forces model-based passes to review. Om requires hard-fail-first across all validated evidence and abstention on missing/conflicting data (R-31–R-33). | **Change** empty required scopes to `MANUAL_REVIEW`; preserve visible, threshold-meeting FAIL proposals for recognized components outside the type template. **Do not** count optional PASS as required evidence. Invalid model responses still route to review; full raw-evidence scanning, derived failures and cross-view conflict inference remain future work. |
| Evidence independence | The current simulator's simple check union does not require distinct angles or measure image sharpness. Om proposes independent views and a ≥25° separation (R-32, R-38). | **Do not claim implemented**. Add this to the real evidence adapter only after the method, data and thresholds are approved and tested. Repeated correlated views must not manufacture a pass. |
| Labels and OCR | Demo labels are scenario fixtures, evaluated against fixed date `2026-11-27`; the separate model reader proposes label status. Om proposes literal QR/OCR, explicit clock and sticker-schema verification. | **Keep** fixtures marked synthetic; real date parsing and label meaning remain **proposed**. Unreadable/ambiguous labels require review. |
| System One | No System One service or feature flag exists in ULTRON. Om proposes an offboard typed-decision experiment (R-04). | **Proposal only:** `DECISION_SHADOW_ENABLED` defaults off; after the hard-fail scan, a read-only allowed-action comparison is appended to audit and stops. Timeout/invalid response → `NO_RECOMMENDATION`. No path to motion, verdict or veto; the baseline is unchanged. |
| Simulation and replay | ULTRON renders MuJoCo RGB/depth/segmentation with kinematically placed base and oracle findings. Rerun/RViz are optional replay paths. Om distinguishes sensor mocks, MCAP replay and optional Isaac/Gazebo experiments (R-08, R-75). | **Keep** the working simulator; **propose** real-recording replay and interface mocks. ULTRON's MuJoCo app is not a validated Unitree DDS/SportClient simulator. No simulated test proves balance, slip, stopping, RF behavior, glare or real model performance. |
| Reporting and hash | ULTRON writes local HTML/JSON, evidence files and per-file SHA-256 hashes. Om proposes a UI, an unsigned PDF, canonical manifest hash and n.core-oriented export (R-24, R-39, R-59). | **Keep** the existing report; **propose** the other artifacts. The current per-file hash map is not yet a shared canonical manifest hash, a signed PDF, a live n.core API or a formal inspection certificate. |
| Data and model choice | ULTRON provides optional YOLOE/YOLO-World and Qwen adapters plus synthetic annotations. The research discusses alternative detector, OCR, reader and anomaly stacks. | **Keep** adapters as unverified options; pick and license-check models with held-out real data. Simulation defect metadata is not evidence that a defect is visible. Split data by physical ladder/session; do not publish without approval. |
| Standards and jury sources | Team source review cites an August 2022 DGUV edition; the research proposes March 2022 while explicitly leaving the PDF check open (R-80). Team notes cite the event scoring and deliverables. | **Open:** check actual organizer and DGUV documents and qualified interpretation. Preserve both source histories; do not invent an edition, compliance claim or legal duty. A proposed dataset publication is not automatically an organizer hand-in requirement. |
| Figures | Thirteen figures have specifications but no accepted renders. | **Keep pending.** No broken image embeds or claims that a generated image has been checked. Use the figure list in the Pass 7 Bible. |

## Small changes in this branch

1. Empty required inspection scope cannot become `AUTOMATED_PASS`; a supplied
   hard fail still wins.
2. Recognized non-template visible FAIL proposals that meet the existing
   model threshold are not discarded solely because of the required
   template. Unknown names and invalid responses remain rejected.
3. Full-ring simulation candidates are restricted as stated above; the
   runtime's usual overview/close-up and single side recovery are unchanged.
4. Team instructions and the deployment policy distinguish full deployment
   from authorized partial hardware experiments. Requirement/jury comparison
   documents are retained.

## Open decisions carried forward

These are not silently approved or implemented by this branch.

| ID | Open issue | Conservative working position |
|---|---|---|
| R-02 | B2 tilt API, bounds, timeout and link-loss response | Tilt off until verified; use neutral captures. |
| R-15 | Side-view ±50° versus 25–45° with an unstated reference axis | ±50° is a proposed generator value; measure alternatives. A ≤35 s reposition budget differs from a 45 s per-ladder timeout; neither is validated. |
| R-17 | Original C5 recovery-test fixture versus engine behavior | Keep missing components unknown; fix and review the test fixture, not evidence completeness. |
| R-25 | Close poses versus the padded footprint and target keep-out | Reject/hold unsafe or unknown candidates; compare stand-offs on the same unmodified costmap. |
| R-26 | Six-gate order and fallback | Proposed: zone, keep-out, behind-wall, footprint cost, clearance, valid path plus bounded approach; fallback is separate. |
| R-42 | Wi-Fi loss versus loss of safety signals | Safety-signal loss defaults to stop-and-hold; offboard link behavior requires a team decision, local recording and test. Wi-Fi loss alone is not automatically a human intervention. |
| R-44 | One versus two extra repositions | One maximum for the hackathon; a second is a later option. |
| R-52 | Reader and parent-box thresholds | Existing 0.85 is an uncalibrated prototype choice; proposed dual-reader and box thresholds require team selection and held-out evaluation. |
| R-79 | Approval of project-policy values | Proposed until the team records approval and validation. |
| R-80 | DGUV edition and applicable wording | Verify the actual publication; the current sources disagree. |
| R-81 | Conformal versus PatchCore calibration | Separate datasets and provenance; no validated calibration is claimed. |

## Integration sequence

1. Freeze the intended configuration and acceptance cases; run unit tests,
   rendered MuJoCo scenarios and failure injection. Record oracle and
   kinematic limitations.
2. Add versioned evidence/registry contracts and replay of authorized real
   recordings; preserve the existing database until an explicit migration is
   reviewed. Add measured orientation, reliable timestamps/TF and real
   detection before interpreting real-world observations.
3. Independently validate the on-robot safety chain. Bounded component-level
   tests may precede the complete simulation gate only under the policy;
   they are not full autonomous deployment.
4. Complete all selected system simulation gates, then satisfy real-world
   safety, perception and operator gates before attempting the integrated
   hardware solution. Keep the team requirement and jury documents unchanged
   for comparison and record any discrepancies.
