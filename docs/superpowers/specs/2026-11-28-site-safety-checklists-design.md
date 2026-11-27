# Site safety checklists beyond ladders — design

Date: 2026-11-28 · Owner: Vivek · Branch: `feature/site-safety-checklists` (from origin/main after the dashboard merge)

## Goal

Extend ULTRON from a ladder-only inspector to a site-safety platform by adding two
robot-observable checks from the nesseler material
([nesseler-data](https://github.com/ctrl-rwth/ctrl-hackathon-2026/tree/main/site-safety-patrol/nesseler-data)):

1. **First aid & fire protection** (`erste_hilfe_brandschutz.pdf`, BG BAU/AMS BAU):
   extinguishers, first-aid notice/kit, escape routes.
2. **PPE violations**: a person on site without helmet or hi-vis vest, framed as the
   "risk-assessment measures really in place" check (`wirksamkeitskontrolle`).

Demo surface is **both**: the MuJoCo patrol discovers the new assets and captures
evidence; the VLM assessment judges the frames; results reach the dashboard.
Constraint: **hours of runway, implemented by Vivek alone** — maximum reuse, checklists
as data, no new engines.

## Non-goals

- The other three nesseler checklists (scaffold, low-dust, subcontractor briefing) —
  out of scope; the pack format makes them future data-only additions.
- Person tracking, pose estimation, or a trained PPE detector. The mannequin check is
  VLM-judged from captured frames.
- The 24-view 360° coverage gate for the new assets (stays ladder-only).
- Automated passes: clean VLM proposals remain review-pending, exactly like ladders.
- Real-hardware claims. Everything here is simulation + photo evidence.

## Architecture

The ladder pipeline already has the right stages; each stage gains a data-driven
extension, no stage is replaced:

```
scenario JSON ──> scene.py/make_scene ──> patrol + oracle discovery ──> keyframe capture
      │                 (new: safety_assets)        (new: non-ladder kinds)      │
      └─> site_safety_v1 pack (new, mirrors inspection_v1) <──────────────────────┘
                     │ assessment.py request builder (new: pack parameter)
                     ▼
          VLM response → local validation → dashboard adapter
                                             (new: safety sweep events + violation,
                                              optional checklist-type extension)
```

### 1. Checklist pack `site_safety_v1` (pure data)

New directory `src/ultron/assets/site_safety_v1/` mirroring `inspection_v1` file-for-file:
`rubric.json` (version `ultron.site_safety.v1`), `response_schema.json`, `system.txt`,
`section_NN_*.txt` prompts. Sections and criteria (9 total, German labels + English
guidance, same `OK/DEFECT/UNKNOWN/NOT_CHECKED/N_A` states):

| # | key | criteria |
|---|-----|----------|
| 1 | `ext` (Feuerlöscher) | `ext.vorhanden` present at station · `ext.zugaenglich` unobstructed · `ext.pruefplakette` inspection tag readable |
| 2 | `firstaid` (Erste Hilfe) | `firstaid.aushang` notice posted · `firstaid.kasten` kit present & marked |
| 3 | `flucht` (Fluchtwege) | `flucht.kennzeichnung` green sign visible · `flucht.frei` route unobstructed |
| 4 | `psa` (PSA/PPE) | `psa.helm` every visible person wears a helmet · `psa.warnweste` hi-vis vest |

`assessment.py` change: the hardcoded `ASSETS = .../inspection_v1` becomes a
`pack` parameter (default `inspection_v1`; ladder behavior byte-identical).
`pyproject.toml` package-data adds `assets/site_safety_v1/*`.

### 2. Sim props and discovery

- Scenario JSON gains an optional `safety_assets` list next to `ladders`:
  `{"id": "EXT01", "kind": "extinguisher", "position": [x, y]}` with
  `kind ∈ {extinguisher, firstaid, escape_sign, person}`.
- `scene.py` gets `safety_assets()` defaults — one extinguisher, one first-aid box,
  one escape sign, and two `person` mannequins (one with helmet geom, one without,
  as a contrast pair) — and `make_scene()` renders them
  as primitives: red cylinder + bracket post (extinguisher), white box with green
  cross (first-aid), green plate on post (escape sign), capsule torso + head sphere
  (person; helmet = contrasting cap sphere).
- Oracle detection in `mujoco_sim.py` generalizes from `self.ladders` to all scene
  assets via the same named-geom pixel-count mechanism; detections carry `kind`.
- Patrol approaches a discovered safety asset like a ladder but captures only
  **2–4 keyframes** (no 360 gate). Asset identity flows through the existing
  memory path (`asset_group` keyed), so repeat patrols recognize them.

### 3. Assessment and dashboard

- One **site-safety sweep per patrol**: all safety-asset keyframes in a single
  `site_safety_v1` request; strict schema keeps every criterion present. Criteria
  with no applicable evidence come back `NOT_CHECKED`/`UNKNOWN`, never silently OK.
- `psa.*` = `DEFECT` additionally emits a **violation event** carrying the evidence
  photo ID.
- Dashboard (now merged in-repo under `src/ultron/dashboard/`): implement a minimal
  extension on this branch for Aravind to review — a `site_safety` sweep summary and
  a `violation` event type on the existing `Event` model/live feed. **Fallback** (if
  the extension is contested or breaks): publish only through the existing
  progress-event + photo-upload channels; ladder verdicts remain the sole
  `Inspection`. The existing duplicate rule (one verdict per asset/mission) is
  untouched: sweeps are events, not `Inspection` posts.
- `safe_to_use`, sign-off-pending and human-only (`person`-mode) semantics are
  unchanged.

## Error handling

- Prop absent / never discovered → affected criteria `NOT_CHECKED`; patrol completes.
- No API key → request body is built and saved locally, not sent (existing pattern).
- Unknown map position at publish time → integration error, no made-up origin
  (existing rule).
- Dashboard extension unavailable → events-only fallback path, demo still works.
- VLM returns malformed/incomplete JSON → local validation rejects; sweep stays
  unpublished with the raw response retained for debugging.

## Testing

- `tests/test_site_safety_pack.py`: pack loads, schema covers all 9 criteria,
  request builder produces a valid sweep body (mirrors `test_assessment.py`).
- `tests/test_mujoco_workflow.py` additions: new geoms exist in the scene; oracle
  reports the new kinds; mannequin-without-helmet is distinguishable from the
  helmeted one by geom naming.
- Dashboard extension: one test that a violation event round-trips the live bus
  (mirrors `test_dashboard.py` patterns).
- Baseline before work: 110 tests OK (33 skipped) with system python3.

## Build order (cut from the back if time collapses)

1. `site_safety_v1` pack + `assessment.py` pack parameter (~1–2 h)
2. Sim props + oracle generalization + patrol capture (~2–3 h, riskiest)
3. Sweep builder + events publishing + violation event (~1 h)
4. Dashboard extension + PR note for Aravind (~30 min)

Step 1 alone already demos on photos (photo-first fallback).
