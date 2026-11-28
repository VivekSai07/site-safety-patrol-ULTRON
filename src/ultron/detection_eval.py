"""Offline, box-level evaluation of independently supplied COCO predictions."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


CLASSES = {1: "ladder", 2: "inspection_label"}


def _integer(value, where):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{where} must be a non-negative integer")
    return value


def _number(value, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{where} must be a finite number")
    try:
        if not math.isfinite(value):
            raise ValueError(f"{where} must be a finite number")
    except OverflowError as exc:
        raise ValueError(f"{where} must be a finite number") from exc
    return value


def _threshold(value, where, *, positive=False):
    value = _number(value, where)
    if value > 1 or value < 0 or (positive and value == 0):
        raise ValueError(f"{where} must be in {'(0, 1]' if positive else '[0, 1]'}")
    return value


def _box(value, image, where):
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError(f"{where} must be [x, y, width, height]")
    x, y, w, h = (_number(v, where) for v in value)
    if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > image["width"] or y + h > image["height"]:
        raise ValueError(f"{where} must have positive area and lie within the image")
    area = _number(w * h, f"{where} area")
    if area <= 0:
        raise ValueError(f"{where} area must be positive")
    return (x, y, w, h)


def _array(value, where):
    if not isinstance(value, list):
        raise ValueError(f"{where} must be an array")
    return value


def _object(value, where):
    if not isinstance(value, dict):
        raise ValueError(f"{where} must be an object")
    return value


def validate_ground_truth(data):
    """Validate the supported, non-crowd COCO subset and index its images."""
    data = _object(data, "ground truth")
    categories = {}
    for category in _array(data.get("categories"), "categories"):
        category = _object(category, "category")
        cid = _integer(category.get("id"), "category id")
        if cid in categories:
            raise ValueError(f"duplicate category id {cid}")
        categories[cid] = category.get("name")
    if categories != CLASSES:
        raise ValueError("categories must be 1: ladder and 2: inspection_label")
    images = {}
    filenames = set()
    for image in _array(data.get("images"), "images"):
        image = _object(image, "image")
        iid = _integer(image.get("id"), "image id")
        name = image.get("file_name")
        if iid in images:
            raise ValueError(f"duplicate image id {iid}")
        if not isinstance(name, str) or not name.strip() or name in filenames:
            raise ValueError("image file_name must be a unique non-empty string")
        for field in ("width", "height"):
            if _integer(image.get(field), f"image {field}") == 0:
                raise ValueError(f"image {field} must be positive")
        images[iid] = image
        filenames.add(name)
    annotations = []
    annotation_ids = set()
    for annotation in _array(data.get("annotations"), "annotations"):
        annotation = _object(annotation, "annotation")
        aid = _integer(annotation.get("id"), "annotation id")
        iid = _integer(annotation.get("image_id"), "annotation image_id")
        cid = _integer(annotation.get("category_id"), "annotation category_id")
        if aid in annotation_ids:
            raise ValueError(f"duplicate annotation id {aid}")
        if iid not in images or cid not in CLASSES:
            raise ValueError("annotation references an unknown image or category")
        if _integer(annotation.get("iscrowd"), "annotation iscrowd") != 0:
            raise ValueError("crowd annotations are not supported")
        annotations.append((aid, iid, cid, _box(annotation.get("bbox"), images[iid], "annotation bbox")))
        annotation_ids.add(aid)
    return images, annotations


def _groups(images):
    groups = set()
    for image in images.values():
        ids = image.get("group_ids")
        if ids is None:
            ids = [image.get("group_id")]
        if not isinstance(ids, list) or not ids or any(not isinstance(g, str) or not g.strip() for g in ids):
            raise ValueError("each image needs non-empty group_id or group_ids for split checking")
        groups.update(ids)
    return groups


def _iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    w = max(0, min(ax + aw, bx + bw) - max(ax, bx))
    h = max(0, min(ay + ah, by + bh) - max(ay, by))
    intersection = w * h
    # Scaling by the largest area avoids overflowing the union.
    area_a, area_b = aw * ah, bw * bh
    scale = max(area_a, area_b)
    return (intersection / scale) / (area_a / scale + area_b / scale - intersection / scale)


def evaluate(ground_truth, predictions, *, domain, prediction_kind, prediction_source,
             assessed_classes, confidence_thresholds=(0.5,), iou_thresholds=(0.5,),
             training_ground_truth=None):
    """Return deterministic per-class box counts; no inference is performed."""
    if domain not in ("real", "synthetic"):
        raise ValueError("domain must be real or synthetic")
    if prediction_kind not in ("detector", "fixture"):
        raise ValueError("prediction_kind must be detector or fixture")
    if not isinstance(prediction_source, str) or not prediction_source.strip():
        raise ValueError("prediction_source is required")
    assessed_classes = list(assessed_classes)
    if not assessed_classes or len(set(assessed_classes)) != len(assessed_classes) or any(
        name not in CLASSES.values() for name in assessed_classes
    ):
        raise ValueError("assessed_classes must contain unique supported class names")
    confidences = sorted(set(_threshold(v, "confidence threshold") for v in confidence_thresholds))
    ious = sorted(set(_threshold(v, "IoU threshold", positive=True) for v in iou_thresholds))
    if not confidences or not ious:
        raise ValueError("at least one confidence and IoU threshold is required")
    images, annotations = validate_ground_truth(ground_truth)
    split_status = "unverified"
    if training_ground_truth is not None:
        training_images, _ = validate_ground_truth(training_ground_truth)
        if not images or not training_images:
            raise ValueError("split verification requires non-empty evaluation and training images")
        overlap = _groups(images) & _groups(training_images)
        if overlap:
            raise ValueError("training/evaluation group leakage: " + ", ".join(sorted(overlap)))
        split_status = "declared_groups_disjoint"
    parsed_predictions = []
    for index, prediction in enumerate(_array(predictions, "predictions")):
        prediction = _object(prediction, f"prediction {index}")
        iid = _integer(prediction.get("image_id"), "prediction image_id")
        cid = _integer(prediction.get("category_id"), "prediction category_id")
        if iid not in images or cid not in CLASSES:
            raise ValueError("prediction references an unknown image or category")
        if CLASSES[cid] not in assessed_classes:
            raise ValueError("prediction category is not declared assessed")
        score = _threshold(prediction.get("score"), "prediction score")
        box = _box(prediction.get("bbox"), images[iid], "prediction bbox")
        parsed_predictions.append((index, iid, cid, box, score))

    results = []
    for cid, name in CLASSES.items():
        if name not in assessed_classes:
            results.append({"category_id": cid, "class": name, "status": "unassessed"})
            continue
        truth = sorted((a for a in annotations if a[2] == cid), key=lambda a: a[0])
        detections = sorted((p for p in parsed_predictions if p[2] == cid), key=lambda p: (-p[4], p[0]))
        for confidence in confidences:
            for iou in ious:
                matched = set()
                tp = fp = 0
                for _, iid, _, box, score in detections:
                    if score < confidence:
                        continue
                    candidates = [(_iou(box, a[3]), a[0]) for a in truth
                                  if a[1] == iid and a[0] not in matched]
                    candidates.sort(key=lambda pair: (-pair[0], pair[1]))
                    if candidates and candidates[0][0] >= iou:
                        matched.add(candidates[0][1])
                        tp += 1
                    else:
                        fp += 1
                fn = len(truth) - tp
                results.append({"category_id": cid, "class": name, "status": "assessed",
                                "confidence_threshold": confidence, "iou_threshold": iou,
                                "tp": tp, "fp": fp, "fn": fn,
                                "precision": tp / (tp + fp) if tp + fp else None,
                                "recall": tp / (tp + fn) if tp + fn else None})
    return {"dataset_domain": domain, "prediction_kind": prediction_kind,
            "prediction_source": prediction_source, "split_check": split_status,
            "images": len(images), "results": results}


def _json_file(path):
    def unique_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    content = Path(path).read_bytes()
    data = json.loads(content, object_pairs_hook=unique_pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f"invalid JSON number: {value}")))
    return data, hashlib.sha256(content).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", required=True, help="COCO ground truth JSON")
    parser.add_argument("--predictions", required=True, help="independent COCO prediction JSON array")
    parser.add_argument("--domain", required=True, choices=("real", "synthetic"))
    parser.add_argument("--prediction-kind", required=True, choices=("detector", "fixture"))
    parser.add_argument("--prediction-source", required=True, help="model/checkpoint provenance or fixture description")
    parser.add_argument("--assessed-class", required=True, action="append", choices=tuple(CLASSES.values()))
    parser.add_argument("--confidence", action="append", type=float, help="repeatable, proposed default 0.5")
    parser.add_argument("--iou", action="append", type=float, help="repeatable, proposed default 0.5")
    parser.add_argument("--training-ground-truth", help="optional COCO train JSON with group IDs")
    args = parser.parse_args(argv)
    try:
        if Path(args.ground_truth).resolve() == Path(args.predictions).resolve():
            raise ValueError("ground truth and predictions must be separate files")
        gt, gt_hash = _json_file(args.ground_truth)
        predictions, pred_hash = _json_file(args.predictions)
        training, training_hash = _json_file(args.training_ground_truth) if args.training_ground_truth else (None, None)
        report = evaluate(gt, predictions, domain=args.domain, prediction_kind=args.prediction_kind,
                          prediction_source=args.prediction_source, assessed_classes=args.assessed_class,
                          confidence_thresholds=args.confidence if args.confidence is not None else (0.5,),
                          iou_thresholds=args.iou if args.iou is not None else (0.5,),
                          training_ground_truth=training)
        report["input_sha256"] = {"ground_truth": gt_hash, "predictions": pred_hash}
        if training_hash:
            report["input_sha256"]["training_ground_truth"] = training_hash
        print(json.dumps(report, indent=2, allow_nan=False))
    except (ValueError, OSError, UnicodeError, RecursionError) as exc:
        print(f"detection evaluation error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
