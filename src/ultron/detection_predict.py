"""Generate offline ladder-box predictions using a local prepared checkpoint."""

import argparse
import hashlib
import json
import math
from pathlib import Path

from .detection_eval import validate_ground_truth
from .perception.models import YoloLadderDetector


def predict(ground_truth, image_root, detector):
    images, _ = validate_ground_truth(ground_truth)
    root = Path(image_root).resolve()
    predictions = []
    for image_id, image in sorted(images.items()):
        path = (root / image["file_name"]).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise ValueError(f"Image must be a file inside image root: {image['file_name']}")
        for detection in detector.detect(str(path)):
            coords = detection["box_xyxy"]
            score = float(detection["confidence"])
            if len(coords) != 4 or not all(math.isfinite(float(x)) for x in coords) or not math.isfinite(score):
                raise ValueError("Detector returned nonfinite or malformed predictions")
            if not 0 <= score <= 1:
                raise ValueError("Detector returned an invalid confidence")
            x1, y1, x2, y2 = (float(x) for x in coords)
            x1, x2 = max(0., x1), min(float(image["width"]), x2)
            y1, y2 = max(0., y1), min(float(image["height"]), y2)
            if x2 <= x1 or y2 <= y1:
                raise ValueError("Detector returned an empty or out-of-frame box")
            predictions.append({"image_id": image_id, "category_id": 1,
                                "bbox": [x1, y1, x2 - x1, y2 - y1], "score": score})
    return predictions


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", required=True)
    parser.add_argument("--image-root", required=True)
    parser.add_argument("--weights", required=True, help="Local prepared ladder checkpoint")
    parser.add_argument("--family", choices=("yoloe", "world", "custom"), default="world")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--out", required=True, help="New prediction JSON path; never overwritten")
    args = parser.parse_args(argv)
    checkpoint = Path(args.weights)
    if not checkpoint.is_file():
        parser.error("weights must be an existing local file")
    output = Path(args.out)
    if output.exists():
        parser.error("output already exists")
    try:
        ground_truth = json.loads(Path(args.ground_truth).read_text(encoding="utf-8"))
        detector = YoloLadderDetector(str(checkpoint), family=args.family, device=args.device,
                                      allow_download=False)
        predictions = predict(ground_truth, args.image_root, detector)
        with output.open("x", encoding="utf-8") as file:
            json.dump(predictions, file, indent=2, allow_nan=False)
            file.write("\n")
        hasher = hashlib.sha256()
        with checkpoint.open("rb") as file:
            for block in iter(lambda: file.read(1024 * 1024), b""):
                hasher.update(block)
        digest = hasher.hexdigest()
        print(f"Wrote {len(predictions)} ladder predictions to {output}")
        print(f"checkpoint_sha256={digest} family={args.family} device={args.device}; "
              "inspection_label is unassessed")
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        parser.exit(2, f"detection prediction error: {exc}\n")


if __name__ == "__main__":
    main()
