# ULTRON dataset and label-first evidence plan

## Current priority: label-first simulated capture

The prioritized next baseline is an operator-positioned, simulation-only
capture of a selected ladder overview and readable manufacturer/inspection
label, followed by an evidence-linked draft mapped to the Nesseller
checklist as a later goal. The public draft schema is a minimal synthetic
example, not a Nesseller mapping. A rendered synthetic capture-to-draft probe
used HTTP requests, manual fixture transcription and a locally retained
source-specific catalog; browser controls were checked separately in
simulation, while real label/OCR performance remains untested. The legacy
simulator does not implement this workflow. The local catalog is excluded
from distribution pending rights review and is not the official form or a
completeness claim.
See the [validation record](VALIDATION.md) and the
[form and capture baseline](LABEL_FIRST_BASELINE.md).

Keep complete original images and metadata; crops are linked derivatives, not
independent views. Select additional views only to resolve missing, obscured
or unreadable visual fields; there is no measured minimum image count.
Fourteen unannotated source photos are not fourteen independent physical
ladders and do not establish camera or model performance. Tactile,
functional, structural, authorization and sign-off fields remain human
responsibilities. Real capture remains a future, explicitly authorized
component activity; this documentation does not authorize hardware access.

## Available now

Each new demo writes `annotations.jsonl`: visible segmentation bounding boxes, stable simulator asset group, ladder type, visible pixel count and scenario defect metadata. `export-dataset` creates a one-class YOLO detection layout, data.yaml, metadata and a dataset card. Empty frames become detector negatives.

This is a detection dataset starter. Scene truth that a ladder has a missing foot does not establish that the foot is visible in the image. Damage visibility is therefore UNANNOTATED; damage boxes/part masks and visual PASS/FAIL/UNKNOWN labels need separate annotation. Do not use scene-level defects as indiscriminate image-level damage labels. No fine-tuning job or performance improvement is claimed.

The export fills one explicitly selected split. It does not invent a validation set from neighboring frames of the same three ladders. Build independent scenarios and keep every physical ladder/session group within one split before evaluating generalization. The same synthetic geometry/template across splits is another limitation to disclose. Real held-out ladders must determine eventual acceptance.

```bash
PYTHONPATH=src python3 -m ultron demo --out runs/dataset-patrol
PYTHONPATH=src python3 -m ultron export-dataset runs/dataset-patrol --out runs/ladder-data --split train
```

## At the first real robot slot

Capture permitted RGB, aligned metric depth, intrinsics, actual frame transforms, poses, timestamps, map version and the hardware configuration. Keep source bag/frames intact; make crops/annotations derived artifacts. Record overview, label/base close-up and reachable side views for the same physical ladder under one stable human-verified ID.

Useful cases: intact versus missing foot cap; intact versus missing/bent rung; legible versus unreadable/overdue label; occluded feet; glare; ladder against wall; free-standing A-frame; neighboring ladders; non-ladder distractors. Stage defects only with the operator's approved test equipment and label them staged. Do not damage working ladders for data.

For each component record: physical ladder ID, component name, visible/occluded, observed state PASS/FAIL/UNKNOWN, defect category, source image/crop/box, view angle and annotator/reviewer. Preserve tactile-only UNKNOWN. Ask the qualified inspector to establish ground truth and explain label semantics; model answers are proposals, not training truth.

## Split and evaluation

Group by physical ladder and capture session/site; hold out complete ladders/sessions. Avoid training and evaluating on nearly identical adjacent frames. Version the annotation schema and review a sample for consistency before labeling in bulk.

Evaluate detection recall, false detections, defect-to-pass errors, human-review rate, completeness per component, useful recovery rate and runtime. Report dataset size and class imbalance. Zero false passes on three examples is not a safety guarantee; abstaining on every example is not useful inspection performance.

Use synthetic data for debugging, pretraining or augmentation. Validate domain transfer on real held-out captures before accepting a model. Keep data origin explicit, preserve privacy/rights, and publish only the approved subset. No data uploads are performed by the exporter.
