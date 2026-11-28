"""Offline fixed yellow-card pixel heuristic, not a trained label detector."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image

from .detection_eval import _json_file


def image_index(data):
    """Validate only the inference manifest; annotations are never consulted."""
    if not isinstance(data, dict) or not isinstance(data.get("categories"), list):
        raise ValueError("manifest must contain categories and images")
    categories = {}
    for category in data["categories"]:
        if not isinstance(category, dict):
            raise ValueError("category must be an object")
        cid = category.get("id")
        if isinstance(cid, bool) or not isinstance(cid, int) or cid in categories:
            raise ValueError("category ids must be unique integers")
        categories[cid] = category.get("name")
    if categories != {1: "ladder", 2: "inspection_label"}:
        raise ValueError("categories must be 1: ladder and 2: inspection_label")
    if not isinstance(data.get("images"), list) or not data["images"]:
        raise ValueError("images must be a non-empty array")
    result, names = {}, set()
    for image in data["images"]:
        if not isinstance(image, dict):
            raise ValueError("image must be an object")
        iid, name = image.get("id"), image.get("file_name")
        if isinstance(iid, bool) or not isinstance(iid, int) or iid < 0 or iid in result:
            raise ValueError("image ids must be unique non-negative integers")
        if not isinstance(name, str) or not name.strip() or name in names:
            raise ValueError("image file names must be unique non-empty strings")
        for field in ("width", "height"):
            n = image.get(field)
            if isinstance(n, bool) or not isinstance(n, int) or n <= 0:
                raise ValueError("image dimensions must be positive integers")
        result[iid] = image
        names.add(name)
    return result


def yellow_boxes(rgb):
    """Fixed RGB mask, 2-pixel square dilation, 8-connected components."""
    pixels = np.asarray(rgb, dtype=np.int16)
    if pixels.ndim != 3 or pixels.shape[2] != 3:
        raise ValueError("expected an RGB image")
    r, g, b = (pixels[:, :, i] for i in range(3))
    mask = (r >= 100) & (g >= 90) & (b * 100 <= np.minimum(r, g) * 65)
    height, width = mask.shape
    padded = np.pad(mask, 2)
    joined = np.zeros_like(mask)
    for dy in range(5):
        for dx in range(5):
            joined |= padded[dy:dy + height, dx:dx + width]
    visited = np.zeros_like(mask)
    boxes = []
    for y, x in zip(*np.nonzero(joined)):
        if visited[y, x]:
            continue
        visited[y, x] = True
        stack = [(y, x)]
        xs, ys = [], []
        while stack:
            cy, cx = stack.pop()
            if mask[cy, cx]:
                xs.append(cx)
                ys.append(cy)
            for ny in range(max(0, cy - 1), min(height, cy + 2)):
                for nx in range(max(0, cx - 1), min(width, cx + 2)):
                    if joined[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        stack.append((ny, nx))
        if len(xs) >= 9:
            boxes.append([int(min(xs)), int(min(ys)),
                          int(max(xs) - min(xs) + 1), int(max(ys) - min(ys) + 1)])
    return boxes


def predict(manifest, image_root):
    images = image_index(manifest)
    root = Path(image_root).resolve()
    if not root.is_dir():
        raise ValueError("image root must be a directory")
    predictions = []
    for iid, image in sorted(images.items()):
        path = (root / image["file_name"]).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise ValueError(f"image must be a file inside image root: {image['file_name']}")
        with Image.open(path) as source:
            if source.size != (image["width"], image["height"]):
                raise ValueError(f"image dimensions disagree with manifest: {image['file_name']}")
            boxes = yellow_boxes(source.convert("RGB"))
        for box in boxes:
            predictions.append({"image_id": iid, "category_id": 2, "bbox": box, "score": 1.0})
    return predictions


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", required=True, help="COCO manifest; annotation contents are ignored")
    parser.add_argument("--image-root", required=True)
    parser.add_argument("--out", required=True, help="New prediction JSON path; never overwritten")
    args = parser.parse_args(argv)
    try:
        manifest, _ = _json_file(args.ground_truth)
        predictions = predict(manifest, args.image_root)
        with Path(args.out).open("x", encoding="utf-8") as file:
            json.dump(predictions, file, indent=2, allow_nan=False)
            file.write("\n")
        print(f"Wrote {len(predictions)} yellow-card heuristic predictions to {args.out}; "
              "score=1 is a fixed inclusion marker, not confidence")
    except (OSError, ValueError, UnicodeError, RecursionError) as exc:
        parser.exit(2, f"label baseline error: {exc}\n")


if __name__ == "__main__":
    main()
