# Source review and decision log

Update: the organizer's actual checklist, photos and simulator are now reviewed in [organizer alignment](ORGANIZER_ALIGNMENT.md). This supersedes the earlier “official template unknown” statements below. Original planning evidence is retained for comparison.

Reviewed 27 November 2026. Source PDFs remain unmodified under `docs/`; extracted text was read page by page for the organizer documents and teammate build bible, with the DGUV scope and inspection/checklist sections checked directly. Figures are illustrative planning material, not hardware verification. Sources have distinct authority: organizer facts; teammate proposals; manufacturer/library interfaces; DGUV guidance.

## Organizer evidence

| Source/page | What it establishes | Repository consequence |
|---|---|---|
| Kickoff Recap p.2; Slides p.9 | Code and real-hardware video ≤2 min required; slides optional; live pitch demo optional | Simulation is preparation/replay, never the required hardware evidence |
| Recap p.2; Slides p.10 | 30% works, 25% reliability, 20% technical quality, 15% creativity, 10% pitch; each scored 1–5 | Protect a working hardware baseline and repeatable evidence before advanced features |
| Slides p.17 | Every ladder assessed, automatic digital protocol, course demo | Memory, explicit coverage gaps and usable evidence reports are core |
| Recap p.2; Slides p.12 | Existing/open-source tools welcome; core built during weekend; prior work disclosed | Keep this as team preparation; no organizer contact or approval workflow is performed |
| Recap p.2; Slides pp.8–9 | Sunday 29 Nov noon hand-in, pitches 13:00 | Capture backup hardware video early; finish packaging before noon |

The supplied Discord thread establishes the planned Unitree B2, Jetson, Ubuntu 22.04 and Humble. The organizer PDFs do not verify detailed camera counts, exact models, calibration, Jetson variant, real SDK network settings or the inspection schema.

## Teammate build bible (Pass 6, 26 Nov)

- pp.2–3: protected autonomous Tier 1 is bounded patrol, discovery, persistent identity, autonomous approach, inspection/history, route resume and coverage. Teleop is a logged degraded mode.
- pp.4–5: offboard inference is the working proposal; detailed sensors/extrinsics, DDS domains, remote sequences and limits remain TO VERIFY. We have not hardcoded those as real B2 facts.
- pp.7–10: hard-fail first; overview/close-up baseline; at most one extra pose; blocked/unresolved observations require review. Tilt and orbit are later extensions.
- pp.11–14: rail depth avoids wall bleed-through; multi-type required components; map-scoped asset/event/mission separation; unknown/conflict preservation and scoped verdicts.
- pp.16–19: baseline gate Saturday 13:30, backup video Saturday 22:00, seeded tests and separate recall/false-pass/intervention measures. These are team planning targets, not organizer promises.
- pp.22–24: proposed Pydantic contracts and capture list guide this repo. The executable spike uses smaller JSON/SQLite contracts; it does not claim to implement every appendix field or a complete schema-validation layer.
- p.25: repeat patrol/history and an uncut hardware story matter more than an isolated AI screenshot.

Disagreements resolved: the original text-only demo routed expiry to review; the new ULTRON policy follows the bible and flags an explicitly overdue label. The legacy package remains separately labeled. A scoped AUTOMATED_PASS is retained for communication, with safe_to_use=false and human sign-off always pending. Dataset/Hugging Face and one-pager requirements in the challenge/team plan are retained as team deliverables; the general kickoff hand-in minimum is code plus hardware video.

## DGUV Information 208-016 (supplied August 2022 edition)

Pages 5–6 describe guidance for portable ladders/steps and exclude fixed ladders from this document. Page 43 addresses before-use checks, recurring visual/functional checks, risk-based intervals (at least annual inspection is recommended), inspector qualifications and numbered records. Page 44 describes withdrawing damaged ladders from use. Pages 46–47 provide the checklist: stiles, fixings, rungs, hinges, spreaders, hooks/guides, feet, contamination, locks, platforms, markings and accessories.

This supports a documentation/prescreening aid with persistent IDs, evidence and human completion. It does not support a robot certifying fastener tightness, locking function or structural load capacity from images. An inspection sticker alone proves neither current condition nor a full inspection. Do not infer a universal legal deadline from a fixed 365-day interval; the scenario uses an explicit due date only. The actual official nesseler template remains unknown.

The manufacturer's summary and detailed standard references in search results must not replace the actual document or qualified human interpretation. This repo makes no DGUV/BetrSichV compliance claim. Official publication: [DGUV 208-016](https://publikationen.dguv.de/widgets/pdf/download/article/450).

## Verified upstream interfaces

- [Unitree profile](https://github.com/unitreerobotics/.github/blob/main/profile/README.md) links the SDK/simulator ecosystem. [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) supplies B2 MJCF and a low-level simulation bridge. It does not provide a drop-in high-level walking/navigation service.
- Preserved B2 model source and blob SHA are in `src/ultron/assets/b2/PROVENANCE.md`; its BSD license is included.
- [DimOS README](https://github.com/dimensionalOS/dimos) documents Go2 simulation/navigation/memory. B2 support is not established by that README. Current inspected pyproject supports Python >=3.10.12,<3.13 and pins Rerun 0.32.0. Keep it separate from this host's Python 3.14.
- [DimOS memory architecture](https://github.com/dimensionalOS/dimos/blob/main/dimos/memory/architecture.md) and `store/sqlite.py` establish `SqliteStore(path=...)`, named streams, append(ts,pose,tags), and spatial/time queries. Our optional ingestion uses that API; runtime validation remains pending.
- [Rerun](https://rerun.io/) and its [Pinhole](https://rerun.io/docs/reference/types/archetypes/pinhole)/[Transform3D](https://rerun.io/docs/reference/types/archetypes/transform3d) references guide logging. This repo pins 0.32.0 to match the inspected DimOS environment; latest docs may describe newer versions. Runtime SDK verification remains pending.
