# Offline ladder and inspection-label detection evaluation

`ultron.detection_eval` compares **independently produced** detections with
image-level COCO ground truth. It is not a detector, OCR, an inspection
assessment, or a safety certification. It makes no model, API, network or
hardware calls. The simulated segmentation oracle is ground truth, not a
detector; never copy annotations into predictions or report fixture scores as
model performance. The tool cannot prove prediction provenance.

## Input and provenance

Supply a COCO JSON object with `images` (`id`, `file_name`, `width`, `height`),
`annotations` (`id`, `image_id`, `category_id`, `bbox`, `iscrowd: 0`) and
exactly these `categories`: `{"id": 1, "name": "ladder"}` and
`{"id": 2, "name": "inspection_label"}`. This includes the capture-world
format. Include negative images and every visible target that can be assessed;
incomplete annotations can turn correct detections into false positives.
This evaluator reads metadata only, not image pixels.

Predictions are a **separate JSON array**, with each object containing
`image_id`, `category_id`, `bbox` and `score`. Boxes are COCO pixel
`[x, y, width, height]`; ground truth and prediction boxes must be finite,
positive-area and entirely inside the image. Clip predictions upstream if
necessary and disclose this preprocessing. Scores must be finite numbers
from 0 through 1. IDs must be non-negative integers, image filenames and
IDs/annotation IDs unique, and prediction references must exist in the ground
truth. Crowd annotations are rejected, not silently counted or ignored.
Malformed inputs fail rather than being dropped.

Declare `--domain real` or `--domain synthetic`, `--prediction-kind detector`
or `fixture`, and a meaningful `--prediction-source` identifying the
checkpoint/version, inference configuration and any preprocessing, or
identifying a hand-authored fixture. `detector` is a user declaration, not a
verification. Do not mix domains in one report. A local fixture baseline is
only a test of the evaluator and **not** detector performance.

Declare each class the prediction process actually evaluated using
`--assessed-class`. An omitted class is explicitly `unassessed`, not assigned
zero recall. A ladder-only model is not a label detector; separately run a
real inspection-label predictor and assess that class before making any
label-detection performance claim. Predictions for undeclared classes are
rejected. If both classes were genuinely evaluated, supply both options.
The optional `ultron.detection_predict` command runs the existing offline
YOLO ladder adapter with a local, prepared checkpoint. It does not provide
inspection-label detection. Alternatively supply predictions from another
independently run model and record its settings.

## Held-out real data and leakage

Manually import permitted, independently annotated images in the same COCO
format. Keep physical ladder, site and capture-session identity across all
frames, including negatives and frames with multiple ladders. Reserve whole
groups before training/tuning, and run evaluation once on a held-out test
set after choices are frozen. Avoid near-duplicates across splits; review
annotation completeness, occlusions, negatives and privacy/usage rights.
No photos are downloaded or licensed by this tool. Obtain explicit approval
and complete rights/privacy review before using or publishing licensed or
private imagery.

For a machine check, add `group_ids` to each image in *both* train and test
COCO files, e.g. `["ladder:42", "session:2026-01", "site:west"]`. Include
**all** applicable physical ladders and sessions; `group_id` as a single
non-empty string is also accepted. Use consistent namespaced identifiers
across files. Pass `--training-ground-truth`: overlapping declared groups,
missing groups, and empty train/test sets are rejected. The result
`declared_groups_disjoint` only checks the submitted manifests; it cannot
verify that identities are correct, complete, or that near-duplicates or
other training/validation/tuning sets are absent. Independently audit all
splits and provenance. Without the option, `split_check` is `unverified` and
held-out independence must **not** be claimed from this report.

## Commands

From the team repository root, using existing local files at the example
paths (replace them with your own):

```bash
# Requires the optional perception-yolo dependencies and a local, prepared
# ladder checkpoint with a saved ladder vocabulary. No checkpoint is supplied.
# Use a new prediction path and preserve the printed checkpoint SHA-256.
PYTHONPATH=src python3 -m ultron.detection_predict \
  --ground-truth runs/my-detection-world/instances.json \
  --image-root runs/my-detection-world --weights /path/to/ladder.pt \
  --family world --device cpu --out runs/ladder-predictions.json

# Substitute the generated paths in the evaluation command below.
# Synthetic ladder-only detection; inspection_label will be unassessed.
PYTHONPATH=src python3 -m ultron.detection_eval \
  --ground-truth data/synthetic-test.json \
  --predictions data/synthetic-ladder-predictions.json \
  --domain synthetic --prediction-kind detector \
  --prediction-source 'local ladder checkpoint v1; inference config v1' \
  --assessed-class ladder --confidence 0.3 --confidence 0.5 --iou 0.5 --iou 0.75

# Independent held-out real data, both classes actually assessed.
PYTHONPATH=src python3 -m ultron.detection_eval \
  --ground-truth data/real-test.json --training-ground-truth data/real-train.json \
  --predictions data/real-predictions.json \
  --domain real --prediction-kind detector \
  --prediction-source 'local ladder and label checkpoints; inference config v1' \
  --assessed-class ladder --assessed-class inspection_label \
  --confidence 0.5 --iou 0.5

# Hand-authored predictions are only a fixture baseline.
PYTHONPATH=src python3 -m ultron.detection_eval \
  --ground-truth data/fixture-gt.json --predictions data/fixture-predictions.json \
  --domain synthetic --prediction-kind fixture --prediction-source 'hand-authored fixture' \
  --assessed-class ladder --assessed-class inspection_label

PYTHONPATH=src python3 -m unittest discover -s tests -p test_detection_eval.py -v
PYTHONPATH=src python3 -m unittest discover -s tests -p test_detection_predict.py -v
```

JSON report goes to stdout; errors go to stderr with nonzero exit status.
Input SHA-256 hashes and declared provenance support reproducibility.
Preserve the exact source files, predictor version/configuration, split
manifest and data rights separately; hashes do not authenticate provenance.
The optional predictor clips returned boxes to image bounds and rejects empty
or nonfinite boxes; its inference confidence floor is 0.15 in the existing
adapter. Evaluation thresholds below that floor cannot recover predictions.
Model loading and inference require a local checkpoint and installed
optional dependencies.

## Local smoke results, 2026-11-28

These are small, non-independent smoke samples, not validated or held-out
performance. At confidence 0.25 and IoU 0.5:

| Input | Class and method | TP | FP | FN | Limits |
|---|---|---:|---:|---:|---|
| Five historical Sahil-hall views | YOLOv8s-worldv2, `ladder` and `stepladder` vocabulary | 8 | 1 | 9 | One scene; 17 ground-truth boxes include trestles |
| Five historical Sahil-hall views | Fixed yellow-card pixel heuristic | 11 | 8 | 2 | Score is an inclusion marker, not confidence; no OCR |
| Four real public photos | Same YOLO-World ladder predictor | 2 | 0 | 2 | No verified negative images; approximate AI-drawn boxes; model training overlap unknown |

Real inspection labels were **unassessed**: the four photographs lack
confirmed inspection-label annotations. The input split was unverified, and
neither synthetic nor real numbers support safety or generalization claims.
The synthetic numbers are not results for the new independent default scene.
The [label baseline](DETECTION_LABEL_BASELINE.md) explains its fixed settings.
The reports and the photos' individual attribution and stated licenses are
recorded locally and excluded from publication. Do not publish the images or
generated artifacts without explicit authorization and rights/privacy review.

The local model is Ultralytics YOLOv8s-worldv2, CPU inference, saved prompts
`ladder, stepladder`, default 640-pixel preprocessing and confidence floor
0.15. Prepared checkpoint SHA-256:
`676446bbbe2f205a7159fc588914c5c794085a78595c2ed6c6f52fcddeb6250d`.
Weights and environment are local and are not part of this repository. Local
source records identify the Ultralytics package as AGPL-3.0 and the original
YOLO-World repository as GPL-3.0; verify applicable versions and all model,
data, CLIP and deployment licenses before redistribution or deployment.
These records do not grant rights.

The smoke inputs and reports are not included. The portable commands above
can be adapted for separately authorized local data and checkpoints; they do
not reproduce this specific smoke result without the corresponding inputs and
environment. Preserve required provenance and choose a new output path.

## Matching and interpretation

For each class, confidence and IoU threshold independently, include scores
at or above the confidence cutoff, sort by descending score (prediction
array order breaks ties), and greedily match each prediction to the highest
IoU unmatched annotation in the same image and class. Equal IoUs prefer the
lowest annotation ID. IoU at the threshold counts. An unmatched prediction
is FP; an unmatched annotation is FN; matches are TP. Duplicate detections
are therefore FP. Precision is TP/(TP+FP) and recall TP/(TP+FN); zero
denominators are JSON `null`, not 0 or 1. Empty predictions/images are
supported; an empty dataset provides no performance evidence.

Repeat `--confidence` and `--iou` to report their Cartesian product. Both
default to **proposed** 0.5, not a validated operating point or hardware
setting. Confidence accepts [0,1], IoU (0,1]. These are box-level counts,
not COCO mAP, object tracking, measured latency, inspection accuracy, or
physical performance. Synthetic results cannot establish real-world recall.
