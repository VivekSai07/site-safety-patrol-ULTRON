# Organizer starter alignment — 27 November 2026

## Decision

Keep ULTRON as the inspection application. Adopt the organizer's `unitree-mujoco` as the SDK/DDS/ROS interface simulator alongside ULTRON's rendered mission simulator. Replacing ULTRON with the kit would remove mission, evidence, memory and reporting without adding perception or autonomous walking.

| Capability | ULTRON renderer | Organizer kit |
|---|---|---|
| B2 model | Official joints/inertias/collisions, primitive visuals | Upstream Unitree B2 model |
| Motion | Kinematic base placement, cosmetic joint cycling | Physics stepping, DDS joint commands, standing example |
| Walking/navigation | Simulated grid paths | No walking controller or sport service |
| Sensors/ladders | Rendered camera/depth/scene, oracle findings by default | No simulated camera or lidar |
| SDK/ROS | Optional unverified adapters | Pinned SDK, DDS bridge and Humble messages |
| Inspection/memory/protocol | Present with explicit simulation limits | Application layer must be built |

Neither simulator establishes real B2 walking, perception accuracy or full deployment readiness. Never transplant the kit's low-level standing script onto the physical robot. ULTRON camera output is not yet bridged into the organizer simulator.

## Source snapshots inspected locally

- [CTRL challenge repository](https://github.com/ctrl-rwth/ctrl-hackathon-2026) at `bd3247546f0c753a7535da1554bbbb1315e0bfa8`: challenge/nesseler READMEs, actual three-page ladder checklist and all 14 photos. The root README redirects detailed robot/simulation documentation and further data to a gated Hugging Face collection. `robots.md` and `simulation.md` are absent at this revision; their contents were not independently verified here.
- [SMAROBIX kit](https://github.com/smarobix/ctrl-hackathon-2026-kit) at `8f5de5806fcd432d339ca1dbb61a1f0f9ea1d6cd`: READMEs, Dockerfile, compose, DDS config, simulation/standing/environment scripts and lowstate subscriber.
- User-supplied current briefing: B2, Jetson Orin, Helios-32, D457, approximately 10 × 10 m, 6–8 ladders, aisle-facing labels around 80 cm. Calibration, streams, timestamp alignment, NIC and physical control settings still require verification. ULTRON's illustrative world is not a course replica.

The kit pins SDK2 Python `814556d15970dd2ecf1c9984e845ca02ab07e206`, Unitree MuJoCo `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`, and unitree_ros2 `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88`. B2 uses `unitree_go` LowState/LowCmd, twelve motors in FR/FL/RR/RL leg order. Simulation interface `lo` and DDS domain 1 are not physical-robot settings.

## Complementary simulator

Keep the organizer clone beside ULTRON, or set `ULTRON_ORGANIZER_KIT`. The wrapper checks the reviewed commit and selects B2; it leaves upstream sources unchanged. Inspect upstream `compose.yaml`, including host networking and container capabilities.

```bash
bash scripts/organizer-sim.sh inspect
bash scripts/organizer-sim.sh config
bash scripts/organizer-sim.sh start
# Viewer: http://localhost:6081/vnc.html
bash scripts/organizer-sim.sh watch
bash scripts/organizer-sim.sh shell
bash scripts/organizer-sim.sh stop
```

Compose configuration was checked locally. Container build/start and live DDS subscription were not tested because this execution environment denies Docker socket access. No physical connection was made. First kit acceptance: a standing B2 with observable lowstate and matching ROS messages. Walking requires an independently integrated controller and separate validation.

## Nesseler protocol and photos

The PDF is a BGI 694 sample form. Page 1 identifies inventory, location, type/material, rung count, length, manufacturer/type, purchase/withdrawal dates and responsible inspector. Pages 2–3 contain seven sections: rails; rungs/steps/platform; spreaders; fittings; feet/rollers; accessories; markings. The result offers usable / repair / withdraw, remarks, next inspection month/year and inspection date. Repeated “3. Prüfung” is a source-form column typo; ULTRON retains ordered history.

`export-nesseler` preserves all sections/criteria, attaches inspection-level evidence and retains screening flags. Each individual criterion stays `PLEASE_CHECK` pending evidence-backed human completion. Coarse component PASS does not establish every criterion. Map IDs need confirmation against physical inventory. Responsible inspector, date, next inspection and formal result stay pending. Historical sticker dates are not automatically current deadlines. Existing simulation reports/history remain available without a database migration.

```bash
PYTHONPATH=src python3 -m ultron export-nesseler runs/ultron-demo --out runs/ultron-demo/nesseler.json
```

The 14 reference photos show whole ladders, Layher inspection grids, euroline manufacturer/barcode labels, pictograms and round stickers. They demonstrate glare, oblique views and small text. They lack defect annotations and do not prove present condition. Confirm reuse rights before publication. Original PDFs/photos remain in the organizer clone, excluded from this public repository.

## Team baseline

Prioritize recall, evidence-backed defect/label flags, explicit uncertainty and the seven-section hand-off. Read aisle-facing labels before extra views. Treat inclination, pallet support, spreader condition and inspection status separately; depth-based inclination/support checks are not yet implemented or calibrated. Unknown, dirty and blank labels require review. VLM confidence is uncalibrated evidence quality, not probability of safe use.

Collect human-reviewed annotations grouped by physical ladder/session, reserving independent evaluation groups. Synthetic boxes support detector plumbing; scenario defect truth is not visible-defect annotation. Training commands do not imply fine-tuning was executed. DimOS adapters are not a validated B2 autonomy stack. Preserve the bounded overview/close-up/one-retry default. Experimental full rings are simulation-only for explicitly open A-frames with missing rear evidence, consistent with Pass 7.
