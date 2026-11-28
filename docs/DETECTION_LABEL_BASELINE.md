# Offline inspection-label localization baseline

This is a **fixed yellow-card color/connected-component heuristic**, not a
trained model. It uses RGB pixels and the image IDs, filenames and dimensions
from a COCO manifest, not annotations, segmentation, MuJoCo or online APIs.
There is no OCR, semantic understanding or safety assessment. It will miss
most real labels and may detect unrelated yellow objects. Its synthetic
demonstration exercises the evaluator; it does not validate real perception,
inspection, robotic operation or safe use.

## Frozen settings and assumptions

The heuristic was specified before evaluation using the synthetic world's
yellow card design (`rgba=.96 .88 .25`), not fitted on held-out images or
annotations. It has no training dataset or checkpoint. It assumes sufficiently
large, visible yellow cards and similar lighting/rendering. Real materials,
color balance, reflections, occlusion, resolution and backgrounds violate
those assumptions; no generalization is claimed.

Convert each image to RGB; select pixels with R >= 100, G >= 90 and
100*B <= 65*min(R,G). Dilate by a 2-pixel square radius, form 8-connected
components, and box the original selected pixels in each component having
at least 9 selected pixels. This merges small gaps, including dark stripes,
but can also merge adjacent objects. No preprocessing varies per dataset.
Each retained component is category 2 (`inspection_label`) with a **constant
score 1.0**, solely an evaluator inclusion marker, not a learned, calibrated
or safety confidence; confidence sweeps are meaningless. Category 1 is
unassessed. Zero predictions on a negative image are valid; unrelated yellow
regions can produce false positives.

The predictor checks the exact category mapping, non-empty image list, unique
IDs/names, safe in-root paths and actual image dimensions. It does not validate
or use annotation contents: separately run `detection_eval`, which validates
the evaluation annotations. All intended images, including negatives, must be
listed, and visible labels must be completely and independently annotated;
otherwise counts can be misleading. Ground truth file and predictions remain
separate. Output creation is exclusive: an existing file is never overwritten.
Pillow and NumPy are required; these are optional simulation dependencies.

## Run on a synthetic dataset

First capture a dataset as described in [world usage](DETECTION_WORLD_USAGE.md).
From the team repository root, using an environment with Pillow and NumPy,
choose a **new** ignored `runs/` prediction path; never publish the dataset,
predictions or reports without approval and a rights/privacy review.

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m ultron.detection_label_baseline \
  --ground-truth runs/my-detection-world/instances.json \
  --image-root runs/my-detection-world \
  --out runs/label-baseline-example-NEW.json

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m ultron.detection_eval \
  --ground-truth runs/my-detection-world/instances.json \
  --predictions runs/label-baseline-example-NEW.json \
  --domain synthetic --prediction-kind detector \
  --prediction-source 'fixed yellow-card RGB connected-component heuristic v1; no training; score=1 inclusion marker' \
  --assessed-class inspection_label --confidence 0.5 --iou 0.5

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m unittest discover -s tests -p test_detection_label_baseline.py -v
```

The earlier run on `detection-world-1128-final` used the locally supplied
Sahil-derived hall and produced 19 predictions over five images; at IoU 0.5:
11 TP, 8 FP, 2 FN, precision 0.579, recall 0.846. These are not results for
the new independent default scene. This is a small synthetic demonstration,
not a held-out performance claim.
Split status was unverified, and ladder was unassessed. Results include
synthetic segmentation-derived ground truth only at the **evaluation** stage.

## Evaluate a real, independently annotated dataset

Use locally held, rights-cleared photos with a separately prepared COCO JSON
meeting `docs/DETECTION_EVALUATION.md`: the same exact two categories,
all image IDs/names/dimensions, and complete image-level annotations (including
negative images). Reserve physical ladder, site and session groups ahead of
any training or tuning and independently audit near-duplicates and privacy.
Keep the fixed settings unchanged. Replace the illustrative paths below with
your own existing dataset and a **new** output path:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m ultron.detection_label_baseline \
  --ground-truth runs/real-label-test/instances.json \
  --image-root runs/real-label-test \
  --out runs/real-label-test-predictions-NEW.json
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m ultron.detection_eval \
  --ground-truth runs/real-label-test/instances.json \
  --predictions runs/real-label-test-predictions-NEW.json \
  --domain real --prediction-kind detector \
  --prediction-source 'fixed yellow-card RGB connected-component heuristic v1; no training; score=1 inclusion marker' \
  --assessed-class inspection_label --confidence 0.5 --iou 0.5
```

No real-image evaluation is claimed here. The evaluator's `split_check` is
unverified without a supplied training manifest; disjoint declared groups
are only a mechanical check, not proof of independence. A no-training
heuristic still needs a frozen selection process and independently audited
dataset for a meaningful held-out claim.
