import contextlib
import io
import unittest
from unittest.mock import patch

try:
    import numpy as np
    from PIL import Image
except ImportError as exc:
    DEPENDENCY_ERROR = str(exc)
else:
    DEPENDENCY_ERROR = None
    from ultron.detection_label_baseline import image_index, main, predict, yellow_boxes


def manifest():
    return {
        "categories": [{"id": 1, "name": "ladder"}, {"id": 2, "name": "inspection_label"}],
        "images": [{"id": 8, "file_name": "photo.png", "width": 20, "height": 16}],
        "annotations": [{"arbitrary": "ignored"}],
    }


@unittest.skipIf(DEPENDENCY_ERROR is not None, f"optional image dependency unavailable: {DEPENDENCY_ERROR}")
class LabelBaselineTests(unittest.TestCase):
    def test_negative_gap_merge_and_disconnected_candidates(self):
        image = np.zeros((16, 20, 3), dtype=np.uint8)
        self.assertEqual(yellow_boxes(image), [])
        image[1:4, 1:4] = (220, 200, 30)
        image[1:4, 7:10] = (220, 200, 30)
        image[11:14, 15:18] = (220, 200, 30)
        image[14, 0] = (220, 200, 30)
        self.assertEqual(yellow_boxes(image), [[1, 1, 9, 3], [15, 11, 3, 3]])

    def test_inference_ignores_annotations_and_produces_coco(self):
        image = Image.new("RGB", (20, 16))
        for x in range(3, 6):
            for y in range(4, 7):
                image.putpixel((x, y), (220, 200, 30))
        expected = [{"image_id": 8, "category_id": 2, "bbox": [3, 4, 3, 3], "score": 1.0}]
        with patch("pathlib.Path.is_file", return_value=True), \
             patch("ultron.detection_label_baseline.Image.open", side_effect=lambda _: image.copy()):
            first = predict(manifest(), ".")
            other = manifest()
            other["annotations"] = "not consulted"
            self.assertEqual(first, expected)
            self.assertEqual(predict(other, "."), first)

    def test_manifest_validation(self):
        for edit in ("categories", "empty", "duplicate", "dimension"):
            bad = manifest()
            if edit == "categories":
                bad["categories"][1]["name"] = "wrong"
            elif edit == "empty":
                bad["images"] = []
            elif edit == "duplicate":
                bad["images"].append(dict(bad["images"][0]))
            else:
                bad["images"][0]["width"] = True
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                image_index(bad)

    def test_escape_and_dimension_mismatch_rejected(self):
        bad = manifest()
        bad["images"][0]["file_name"] = "../escape.png"
        with self.assertRaises(ValueError):
            predict(bad, ".")
        with patch("pathlib.Path.is_file", return_value=True), \
             patch("ultron.detection_label_baseline.Image.open", return_value=Image.new("RGB", (2, 2))):
            with self.assertRaises(ValueError):
                predict(manifest(), ".")

    def test_exclusive_output_does_not_clobber(self):
        with patch("ultron.detection_label_baseline._json_file", return_value=(manifest(), "hash")), \
             patch("ultron.detection_label_baseline.predict", return_value=[]), \
             patch("pathlib.Path.open", side_effect=FileExistsError("already exists")) as opened, \
             contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
            main(["--ground-truth", "input.json", "--image-root", ".", "--out", "existing.json"])
        self.assertEqual(exc.exception.code, 2)
        self.assertEqual(opened.call_args.args, ("x",))


if __name__ == "__main__":
    unittest.main()
