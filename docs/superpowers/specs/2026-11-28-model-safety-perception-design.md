# Model-backed safety sweep — historical design

> Historical teammate design, superseded where it conflicts with
> [Pass 7](../../bible/BUILD_BIBLE_PASS7.md) and
> [site-safety limits](../../SITE_SAFETY.md). Model confirmation uses
> pixels, but target kinds, positions and routing still use simulated scene
> truth. External uploads and hardware use require separate authorization.

Date: 2026-11-28 · Owner: Vivek · Branch: `feature/model-safety-perception` (from main after the site-safety merge)

## Goal

Replace the segmentation-oracle backend of the site-safety sweep with real
perception when model backends are configured: YOLOE open-vocabulary detection
for discovery confirmation and Qwen3-VL for pixel-based criteria judgment,
including PPE violations. The oracle remains only as the explicit default
backend for CI/no-GPU runs.

## Spike evidence (2026-11-28, RTX PRO 2000 8 GB, CUDA)

- YOLOE (`yoloe-11s-seg`, open-vocab classes) is **blind on primitive sim
  renders**: `NOTHING` on person/extinguisher frames, one 0.18-conf misread.
  It stays wired because it is the adapter that works on real imagery.
- Qwen3-VL-2B-Instruct **can judge PPE from pixels**: it read the helmeted
  mannequin as a person with helmet YES / vest YES, but read the bare
  mannequin as "a pillar with a sphere on top" — the violator lacks human
  geometry cues. Scene fidelity, not the model, is the blocker.

## Non-goals

- No training/fine-tuning; no new model families.
- No Unitree SDK/locomotion claims — ULTRON's base stays kinematic; real
  controller work happens in the organizer simulator (team agreement).
- No automated pass: model "all clear" is still review-pending; model DEFECT
  flags for human review.
- Ladder perception paths (`--detector yolo --inspector qwen` for ladders)
  are untouched.

## Components

1. **Mannequin fidelity** (`scene.py`): arms, two separated legs, shirt color
   on the no-vest mannequin, skin-tone head. Acceptance (model-in-the-loop):
   Qwen3-VL-2B reads BOTH mannequins as persons; violator judged helmet NO,
   vest NO; compliant one helmet YES, vest YES.
2. **`YoloSafetyDetector`** (`perception/models.py`): YOLOE/YOLO-World with
   safety vocabulary (`fire extinguisher`, `first aid kit`, `exit sign`,
   `person`, `safety helmet`, `high visibility vest`); returns labeled boxes
   with confidence, `source='ULTRALYTICS_MODEL'`. Checkpoint/offline rules
   mirror `YoloLadderDetector` (`--allow-download` gate, saved vocabulary).
3. **`QwenSafetyJudge`** (`perception/models.py`): per-station-kind prompt;
   returns strict JSON (`station_visible`, per-criterion `YES/NO/UNKNOWN`,
   `reason`). Loading mirrors `QwenInspector` (`local_files_only` unless
   `--allow-download`).
4. **`RuntimePerception` safety methods** (`runtime_perception.py`):
   - `safety_confirm(rgb, depth, frame, kind, position)` → detection dict or
     `None` (YOLO boxes of the matching class projected via `project_box`,
     accepted within 1.0 m of the station).
   - `safety_judge(image_paths, kind)` → `{criterion_key: 'OK'|'DEFECT'|'UNKNOWN',
     'reason': str, 'station_visible': bool}` via `QwenSafetyJudge`, mapped
     through a `kind → criteria` table in `site_safety.py`.
5. **Mission sweep wiring** (`mission.py`): when the run's `RuntimePerception`
   has a detector or inspector, the sweep uses models:
   - Confirmation: YOLO (`safety_confirm`) on each burst frame; if none and a
     judge exists, Qwen `station_visible` on the captured frame confirms with
     `source='QWEN_VL_PROPOSAL'`. Neither → `SAFETY_CHECK_INCOMPLETE`
     `reason='NOT_DETECTED_BY_MODEL'`.
   - Judgment: criteria states and `SAFETY_VIOLATION` come from Qwen's
     verdicts (`violation` emitted when `psa.helm`/`psa.warnweste` is DEFECT),
     `source='MODEL_PROPOSAL'`. The oracle's helmet/vest scene truth is not
     consulted in model mode.
   - Oracle backend (default, no model flags) behaves exactly as today.

## Demo command

    PYTHONPATH=src MUJOCO_GL=osmesa python3 -m ultron demo --out runs/model-safety \
      --patrols 1 --safety --detector yolo --weights weights/yoloe-11s-seg.pt \
      --inspector qwen --qwen Qwen/Qwen3-VL-2B-Instruct --allow-download

## Error handling

- Model import/checkpoint failures raise at startup (existing adapter rules).
- Per-frame model errors → criterion `UNKNOWN` with the error as reason;
  station stays unconfirmed rather than silently OK.
- Stale HuggingFace tokens: adapters pass `token=False` when downloading
  public checkpoints is explicitly allowed (observed 401 from an expired
  OAuth token during the spike).
- Events always disclose their source (`ULTRALYTICS_MODEL`,
  `QWEN_VL_PROPOSAL`, `MODEL_PROPOSAL`, or `SIMULATION_ORACLE`).

## Testing

- Unit tests inject fake detector/judge objects into `RuntimePerception`
  (CI-safe, no GPU): confirmation distance gate, kind→criteria mapping,
  violation emission from judged DEFECT, incomplete path when nothing
  confirms.
- One GPU/model integration test gated by `ULTRON_MODEL_TESTS=1` running the
  sweep with real YOLOE + Qwen.
- Mannequin acceptance is validated live during development (model-in-loop)
  and captured in the demo artifacts (new GIF with model-sourced banners).

## Build order (cut from the back)

1. Mannequin fidelity loop against live Qwen (riskiest)
2. Adapters: `YoloSafetyDetector`, `QwenSafetyJudge` (+ kind→criteria map)
3. `RuntimePerception` safety methods + mission wiring + unit tests
4. Full model-backed demo run, new GIF, docs update
