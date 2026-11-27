# Version 0.4 runtime paths and verification

The demo defaults to simulation-oracle discovery/findings. Optional YOLO discovery projects boxes through simulated depth and confirms position tracks; Qwen inspects same-target crops and retains raw responses. Model proposals always require review; no calibrated safe-use probability is produced. Missing packages/checkpoints fail explicitly, with no silent oracle fallback. Camera calibration and depth association remain simulation assumptions.

```bash
# Optional extras in a suitable isolated model environment; not ROS system Python.
python -m pip install -e '.[sim,perception-yolo,perception-vlm]'
PYTHONPATH=src python3 -m ultron demo --patrols 1 --detector yolo --weights /path/ladder.pt --detector-family custom --inspector qwen --qwen /path/qwen3-vl --out runs/model-demo
# Experimental, explicitly open A-frame with missing rear evidence only:
PYTHONPATH=src python3 -m ultron demo --patrols 1 --variant open-orbit --view-policy orbit --out runs/open-orbit
```

Offline YOLOE/YOLO-World requires a checkpoint with ladder vocabulary/embeddings already saved. A standard COCO detector does not gain a ladder class by renaming labels. `--allow-download` explicitly opts into upstream model downloads. Synthetic inspection labels/defects are not inferred unless the model inspector is selected; mixed oracle/model mode remains disclosed in each record. Eight captured viewpoints do not guarantee every surface or defect is visible.

## Dataset and training

```bash
PYTHONPATH=src python3 -m ultron generate-corpus --scenes 4 --views 2 --out runs/corpus
PYTHONPATH=src python3 -m ultron train-detector runs/corpus --weights /path/ladder.pt --out runs/train-plan --dry-run
# Actual training needs the optional dependency and local checkpoint:
PYTHONPATH=src python3 -m ultron train-detector runs/corpus --weights /path/ladder.pt --out runs/train --epochs 10
PYTHONPATH=src python3 -m ultron train-vlm /path/reviewed.jsonl --model /path/qwen3-vl --out runs/vlm-plan --dry-run
```

Corpus scenes vary geometry and views, with disjoint procedural asset groups across train/validation. Validation is synthetic-only and shares the generator. Boxes derive from visible segmentation; damage visibility stays UNANNOTATED. Detector validation checks actual image split paths, existing labels and asset-group leakage. Real recordings need reviewed annotations and separate physical/session groups. VLM JSONL rows require `reviewed: true`, `split`, `asset_group`, `images`, `required_components` and `label` matching the component-evidence schema. Review flags are operator assertions, not automatic quality certification.

DimOS `navigation.DimosNavigator` polls an injected navigation interface, cancels on timeout/operator inhibition and accepts supervisor goals. It is not instantiated by the simulation CLI, supplies no SLAM/exploration stack and establishes no real B2 transport. Its tests use injected interfaces. DimOS memory ingestion and Rerun remain separately installed optional adapters.

## Evidence from this environment

Executed: rendered mission regressions (blocked views, restricted open A-frame ring, return to base, two-patrol re-ID/missing asset), procedural corpus generation (four scenes, 21 train / 7 validation images), detector training-plan validation with `executed=false`, actual nesseler JSON export from a saved run, and organizer compose configuration validation. Recovery stash `489426f427e42cf93dfbc042a53ea1f9883fbb01` preserves work predating the remote reconciliation.

Not executed: YOLO/Qwen inference, actual detector/VLM training, real DimOS runtime/navigation, organizer container build/start/DDS, ROS/RViz or Rerun SDK runtime, physical robot connection. Missing optional dependencies/checkpoints, sandbox network/GPU restrictions and Docker socket permissions prevent those checks here. Tests of injected interfaces and synthetic mission behavior do not establish integrated system acceptance; the hardware gate remains in [SIM_TO_HARDWARE.md](SIM_TO_HARDWARE.md).
