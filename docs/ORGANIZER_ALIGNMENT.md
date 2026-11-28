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

The original assessment checked Compose configuration only. A later isolated
run built the image and observed a standing B2, noVNC protocol traffic and
finite DDS/ROS samples; host-browser access to that stock viewer was not
verified. See [the verified probe and limitations](ORGANIZER_B2_SIM.md). No
physical connection was made. Walking requires an independently integrated
controller and separate validation. The upstream wrapper above uses host
namespaces; in a shared environment prefer the isolated team-side probes in
the linked guide.

## Nesseler protocol and photos

The PDF is a BGI 694 sample form. Page 1 concerns equipment and inspection context; pages 2–3 cover condition checks and human outcomes. A column heading repeats, but its cause is unverified. Keep inspection history ordered and compare any proposed mapping with the authorized source; the public synthetic example is neither the official form nor evidence of complete coverage.

The legacy `export-nesseler` path already present in main emits source-derived
criterion labels whose reuse rights remain unverified. Do not share its output
externally without separate approval and rights review. It attaches
inspection-level evidence and retains screening flags; each criterion remains
`PLEASE_CHECK` for human completion. Coarse component PASS does not establish
every criterion. Map IDs, dates and outcomes require qualified verification.
The public generic draft example is separate and does not claim complete
source-form coverage.

```bash
PYTHONPATH=src python3 -m ultron export-nesseler runs/ultron-demo --out runs/ultron-demo/nesseler.json
```

The 14 reference photos show whole ladders, Layher inspection grids, euroline manufacturer/barcode labels, pictograms and round stickers. They demonstrate glare, oblique views and small text. They lack defect annotations and do not prove present condition. Confirm reuse rights before publication. Original PDFs/photos remain in the organizer clone, excluded from this public repository.

## Team baseline

Prioritize recall, evidence-backed defect/label flags, explicit uncertainty and the seven-section hand-off. Read aisle-facing labels before extra views. Treat inclination, pallet support, spreader condition and inspection status separately; depth-based inclination/support checks are not yet implemented or calibrated. Unknown, dirty and blank labels require review. VLM confidence is uncalibrated evidence quality, not probability of safe use.

Collect human-reviewed annotations grouped by physical ladder/session, reserving independent evaluation groups. Synthetic boxes support detector plumbing; scenario defect truth is not visible-defect annotation. Training commands do not imply fine-tuning was executed. DimOS adapters are not a validated B2 autonomy stack. Preserve the bounded overview/close-up/one-retry default. Experimental full rings are simulation-only for explicitly open A-frames with missing rear evidence, consistent with Pass 7.
