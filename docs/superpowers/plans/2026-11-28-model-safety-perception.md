# Model-Backed Safety Perception Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The site-safety sweep confirms stations and judges criteria (incl. PPE violations) with YOLOE + Qwen3-VL when model backends are configured; the segmentation oracle remains only the explicit default.

**Architecture:** Humanize the mannequin props until Qwen3-VL reads them as persons; add `YoloSafetyDetector`/`QwenSafetyJudge` adapters beside the existing ladder adapters; give `RuntimePerception` two safety methods (`safety_confirm`, `safety_judge`); switch the mission sweep to those methods whenever a model backend exists.

**Tech Stack:** ultralytics (YOLOE open-vocab), transformers Qwen3-VL-2B-Instruct, existing `project_box` depth projection. GPU: RTX PRO 2000 8 GB.

## Global Constraints

- Oracle sweep behavior with default flags stays byte-identical; all existing tests stay green (`PYTHONPATH=src python3 -m unittest discover -s tests` → 121 OK baseline).
- Events must disclose sources: `ULTRALYTICS_MODEL`, `QWEN_VL_PROPOSAL`, `MODEL_PROPOSAL`, `SIMULATION_ORACLE`.
- No automated pass; model errors → `UNKNOWN`, never silent OK.
- Use system `python3`; render via `MUJOCO_GL=osmesa` (EGL wedged on this host).
- HF downloads need `HF_HUB_DISABLE_IMPLICIT_TOKEN=1` / `token=False` (stale OAuth token on this machine).

---

### Task 1: Humanized mannequins that Qwen reads as persons

**Files:**
- Modify: `src/ultron/scene.py` (person block in `make_scene`)
- Test: model-in-the-loop acceptance (manual loop, no unit test — geometry tuned against live Qwen); existing XML tests in `tests/test_site_safety_pack.py` must keep passing (`_head`, `_torso`, `_helmet` names unchanged)

**Interfaces:**
- Produces: person props with geoms `{key}_legs` (replaced by `{key}_leg_l`/`{key}_leg_r`), `{key}_torso`, `{key}_arm_l`, `{key}_arm_r`, `{key}_head`, optional `{key}_helmet`. `PRIMARY_GEOM['person']` stays `'head'`.

- [ ] **Step 1: Replace the person block in `make_scene`**

```python
        elif kind == 'person':
            shirt = ".95 .5 .05 1" if asset.get('vest') else ".25 .35 .6 1"   # hi-vis orange vs blue workwear
            for side, tag in ((-1, 'leg_l'), (1, 'leg_r')):
                box(body, f"{key}_{tag}", [side * .08, 0, .42], [.055, .07, .42], ".2 .22 .3 1")
            box(body, f"{key}_torso", [0, 0, 1.14], [.17, .1, .3], shirt)
            for side, tag in ((-1, 'arm_l'), (1, 'arm_r')):
                ET.SubElement(body, "geom", name=f"{key}_{tag}", type="capsule",
                              fromto=f"{side * .2} 0 1.38 {side * .3} 0 .92", size=".045", rgba=shirt)
            ET.SubElement(body, "geom", name=f"{key}_head", type="sphere",
                          pos="0 0 1.56", size=".1", rgba=".87 .72 .6 1")
            if asset.get('helmet'):
                ET.SubElement(body, "geom", name=f"{key}_helmet", type="sphere",
                              pos="0 0 1.63", size=".105", rgba=".95 .8 .05 1")
```

- [ ] **Step 2: Render both mannequins and judge with Qwen (acceptance loop)**

Render each mannequin head-on from 1.8 m with `Simulation` + `capture(save=True)`, then run Qwen3-VL-2B with the spike prompt. Acceptance: `person_visible=true` for BOTH; violator `helmet=NO, vest=NO`; compliant `helmet=YES, vest=YES`. If it fails, adjust proportions/colors (NOT scene truth leakage) and repeat. Record the final Qwen JSON in the task commit message.

- [ ] **Step 3: Keep existing tests green**

Run: `PYTHONPATH=src python3 -m unittest discover -s tests 2>&1 | tail -3`
Expected: OK (XML test checks `_head`/`_helmet` names — unchanged)

- [ ] **Step 4: Commit**

```bash
git add src/ultron/scene.py
git commit -m "Humanize mannequins so a VLM reads them as workers"
```

---

### Task 2: `YoloSafetyDetector` + `QwenSafetyJudge` adapters

**Files:**
- Modify: `src/ultron/perception/models.py`
- Modify: `src/ultron/site_safety.py` (kind→criteria map + YOLO class map)
- Test: `tests/test_model_safety.py` (new; mapping-only tests, CI-safe)

**Interfaces:**
- Produces in `site_safety.py`:
  ```python
  KIND_CRITERIA = {
      'extinguisher': ('ext.vorhanden', 'ext.zugaenglich', 'ext.pruefplakette'),
      'firstaid': ('firstaid.aushang', 'firstaid.kasten'),
      'escape_sign': ('flucht.kennzeichnung', 'flucht.frei'),
      'person': ('psa.helm', 'psa.warnweste'),
  }
  YOLO_CLASS_FOR_KIND = {'extinguisher': 'fire extinguisher', 'firstaid': 'first aid kit',
                         'escape_sign': 'exit sign', 'person': 'person'}
  SAFETY_CLASSES = ['fire extinguisher', 'first aid kit', 'exit sign', 'person',
                    'safety helmet', 'high visibility vest']
  ```
- Produces in `perception/models.py`:
  - `YoloSafetyDetector(weights, *, family='yoloe', device='cpu', allow_download=False)` with `.detect(image) -> [{'box_xyxy', 'confidence', 'label', 'source': 'ULTRALYTICS_MODEL', 'checkpoint'}]` — same checkpoint/offline rules as `YoloLadderDetector`, but `set_classes(SAFETY_CLASSES, model.get_text_pe(SAFETY_CLASSES))` and no label filter (all safety labels pass through).
  - `QwenSafetyJudge(model_path, *, allow_download=False, device_map='auto')` with `.judge(image_paths, kind) -> str` returning raw model text; prompt varies per kind and demands strict JSON with keys = `KIND_CRITERIA[kind]` plus `station_visible` and `reason`, values `YES/NO/UNKNOWN` where YES=criterion satisfied, NO=deficiency. Loading mirrors `QwenInspector`; generation `max_new_tokens=300`.
  - Module function `parse_judge_text(text, kind) -> dict` (pure, CI-testable): extracts the JSON object (strips ``` fences), maps YES→`OK`, NO→`DEFECT`, anything else→`UNKNOWN` per criterion, returns `{'states': {...}, 'station_visible': bool, 'reason': str}`; on any parse error returns all-`UNKNOWN` with `station_visible=False` and the error text as reason.

- [ ] **Step 1: Write failing CI-safe tests** (`tests/test_model_safety.py`)

```python
import unittest

from ultron.site_safety import KIND_CRITERIA, SAFETY_CLASSES, YOLO_CLASS_FOR_KIND
from ultron.perception.models import parse_judge_text


class JudgeParsingTests(unittest.TestCase):
    def test_kind_maps_cover_all_site_safety_criteria(self):
        from ultron.assessment import _criteria
        keys = sorted(k for kinds in KIND_CRITERIA.values() for k in kinds)
        self.assertEqual(keys, sorted(_criteria('site_safety_v1')))
        self.assertEqual(set(YOLO_CLASS_FOR_KIND), set(KIND_CRITERIA))
        self.assertTrue(set(YOLO_CLASS_FOR_KIND.values()) <= set(SAFETY_CLASSES))

    def test_parse_judge_text_maps_states_and_survives_garbage(self):
        text = '```json\n{"station_visible": true, "psa.helm": "NO", "psa.warnweste": "YES", "reason": "bare head"}\n```'
        result = parse_judge_text(text, 'person')
        self.assertTrue(result['station_visible'])
        self.assertEqual(result['states'], {'psa.helm': 'DEFECT', 'psa.warnweste': 'OK'})
        bad = parse_judge_text('not json at all', 'person')
        self.assertFalse(bad['station_visible'])
        self.assertEqual(set(bad['states'].values()), {'UNKNOWN'})


if __name__ == '__main__':
    unittest.main()
```

Run: `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_model_safety.py' -v` — Expected: FAIL (imports missing)

- [ ] **Step 2: Implement the maps, adapters, and parser as specified above.** The judge prompt template (one per kind, English, with the simulation disclosure) lives in `QwenSafetyJudge.PROMPTS: dict[kind, str]`; each prompt enumerates the exact JSON keys. Example for `person`:

```text
This is a rendered construction-site simulation with simplified shapes; figures are mannequin-style workers.
Look at the images. Answer ONLY with a JSON object with exactly these keys:
"station_visible" (true if a person-like figure is visible),
"psa.helm" ("YES" if every visible person wears a safety helmet, "NO" if any visible person has a bare head, "UNKNOWN" if not determinable),
"psa.warnweste" (same rule for a high-visibility vest),
"reason" (one short sentence).
Never claim anything outside the images.
```

- [ ] **Step 3: Tests pass + full suite green; commit**

```bash
PYTHONPATH=src python3 -m unittest discover -s tests 2>&1 | tail -3
git add src/ultron/perception/models.py src/ultron/site_safety.py tests/test_model_safety.py
git commit -m "Add YOLOE safety detector and Qwen safety judge adapters"
```

---

### Task 3: `RuntimePerception` safety methods + mission wiring

**Files:**
- Modify: `src/ultron/runtime_perception.py`
- Modify: `src/ultron/mission.py` (sweep block only)
- Test: `tests/test_model_safety.py` (extend with fake backends)

**Interfaces:**
- Produces on `RuntimePerception`:
  - `self.safety_detector` / `self.safety_judge`, built in `__init__` from the same flags (`detector=='yolo'` → `YoloSafetyDetector(weights, ...)`; `inspector=='qwen'` → `QwenSafetyJudge(qwen, ...)`).
  - `safety_confirm(rgb, depth, frame, kind, position) -> dict | None`: boxes from `safety_detector.detect` with `label == YOLO_CLASS_FOR_KIND[kind]`, projected via `project_box`; accept nearest within 1.0 m of `position[:2]`; returns `{'box_xyxy', 'confidence', 'position', 'source': 'ULTRALYTICS_MODEL'}`.
  - `safety_judge_station(image_paths, kind) -> dict`: `parse_judge_text(self.safety_judge.judge(image_paths, kind), kind)`; raises `RuntimeError` if no judge configured.
  - `model_safety_mode` property: `self.safety_detector is not None or self.safety_judge is not None`.
- Mission sweep in model mode (replaces the oracle confirm/violation logic when `perception.model_safety_mode`):
  1. burst-capture 2 frames as today;
  2. confirmation: `safety_confirm` on each burst frame (skip if no detector); else judge first frame — `station_visible` confirms with `source='QWEN_VL_PROPOSAL'`;
  3. judgment: `safety_judge_station([evidence jpg], kind)`; emit `SAFETY_STATION_JUDGED` with states+reason+`source='MODEL_PROPOSAL'`; emit `SAFETY_VIOLATION` for each `psa.*` criterion whose state is `DEFECT` (same event shape as oracle mode but `source='MODEL_PROPOSAL'`);
  4. nothing confirms → `SAFETY_CHECK_INCOMPLETE reason='NOT_DETECTED_BY_MODEL'`;
  5. oracle mode (no model backends) → existing code path unchanged.

- [ ] **Step 1: Extend `tests/test_model_safety.py` with fake-backend tests**

```python
class FakeDetector:
    def __init__(self, boxes): self.boxes = boxes
    def detect(self, image): return self.boxes


class FakeJudge:
    def __init__(self, text): self.text = text
    def judge(self, paths, kind): return self.text


class RuntimeSafetyTests(unittest.TestCase):
    def perception(self):
        from ultron.runtime_perception import RuntimePerception
        return RuntimePerception()  # oracle mode, no models

    def test_oracle_mode_has_no_model_safety(self):
        p = self.perception()
        self.assertFalse(p.model_safety_mode)
        with self.assertRaises(RuntimeError):
            p.safety_judge_station(['x.jpg'], 'person')

    def test_fake_judge_station_states_flow_through(self):
        p = self.perception()
        p.safety_judge = FakeJudge('{"station_visible": true, "psa.helm": "NO", "psa.warnweste": "NO", "reason": "r"}')
        result = p.safety_judge_station(['x.jpg'], 'person')
        self.assertEqual(result['states']['psa.helm'], 'DEFECT')
        self.assertTrue(p.model_safety_mode)
```

(`safety_confirm` needs depth/frame fixtures — cover the distance gate inside the mission integration test instead; unit-testing `project_box` again would restate existing coverage.)

- [ ] **Step 2: Implement `RuntimePerception` changes and the mission model-mode branch.** In `mission.py`, pass evidence paths: judge the saved burst JPEG (`output / frame['rgb']`). Keep the grid-aware approach + manifest writing untouched.

- [ ] **Step 3: CI suite green; commit**

```bash
PYTHONPATH=src python3 -m unittest discover -s tests 2>&1 | tail -3
git add src/ultron/runtime_perception.py src/ultron/mission.py tests/test_model_safety.py
git commit -m "Route safety sweep through model backends when configured"
```

---

### Task 4: Full model-backed demo + GIF + docs

**Files:**
- Modify: `docs/SITE_SAFETY.md` (model-mode section + new GIF)
- Create: `docs/images/ultron-safety-sweep-model.gif`
- Test: live run (gated integration; no new unit test)

- [ ] **Step 1: Run the demo in full model mode**

```bash
cd /home/vivek/ULTRON && HF_HUB_DISABLE_IMPLICIT_TOKEN=1 PYTHONPATH=src MUJOCO_GL=osmesa LP_NUM_THREADS=2 \
python3 -m ultron demo --out runs/model-safety --patrols 1 --safety \
  --detector yolo --weights weights/yoloe-11s-seg.pt \
  --inspector qwen --qwen Qwen/Qwen3-VL-2B-Instruct --allow-download
```

Verify in `runs/model-safety/events.jsonl`: `SAFETY_STATION_JUDGED` events with `source=MODEL_PROPOSAL`; `SAFETY_VIOLATION` for `person_nohelm` from the judge; honest `NOT_DETECTED_BY_MODEL` where neither model confirms.

- [ ] **Step 2: Build the GIF** (reuse the Task-6 GIF script pattern from the site-safety plan, 400 ms frames, milestone holds, banners showing the model source) and update `docs/SITE_SAFETY.md` with a "Model-backed mode" section documenting the demo command, `SAFETY_STATION_JUDGED`, and the measured YOLOE-blind/Qwen-capable spike results.

- [ ] **Step 3: Full suite green; commit**

```bash
PYTHONPATH=src python3 -m unittest discover -s tests 2>&1 | tail -3
git add docs/SITE_SAFETY.md docs/images/ultron-safety-sweep-model.gif
git commit -m "Demo model-backed safety sweep with Qwen-judged PPE violation"
```
