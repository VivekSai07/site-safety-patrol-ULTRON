import contextlib
import copy
import io
import json
import unittest
from unittest.mock import patch

from ultron.detection_eval import _json_file, evaluate, main


def ground_truth():
    return {
        "images": [
            {"id": 1, "file_name": "one.jpg", "width": 100, "height": 100, "group_ids": ["ladder:a", "session:a"]},
            {"id": 2, "file_name": "negative.jpg", "width": 100, "height": 100, "group_ids": ["session:b"]},
        ],
        "categories": [{"id": 1, "name": "ladder"}, {"id": 2, "name": "inspection_label"}],
        "annotations": [
            {"id": 1, "image_id": 1, "category_id": 1, "bbox": [10, 10, 20, 20], "iscrowd": 0},
            {"id": 2, "image_id": 1, "category_id": 2, "bbox": [10, 10, 5, 5], "iscrowd": 0},
        ],
    }


def prediction(image, category, box, score):
    return {"image_id": image, "category_id": category, "bbox": box, "score": score}


class DetectionEvaluationTests(unittest.TestCase):
    def evaluate(self, gt, predictions, **kwargs):
        options = dict(domain="synthetic", prediction_kind="fixture", prediction_source="hand-authored test",
                       assessed_classes=["ladder", "inspection_label"])
        options.update(kwargs)
        return evaluate(gt, predictions, **options)

    def test_one_to_one_thresholds_and_class_independence(self):
        predictions = [
            prediction(1, 1, [10, 10, 20, 20], .9),
            prediction(1, 1, [10, 10, 20, 20], .8),
            prediction(2, 1, [10, 10, 20, 20], .7),
            prediction(1, 2, [10, 10, 5, 5], .4),
        ]
        result = self.evaluate(ground_truth(), predictions, confidence_thresholds=[.5, .3])
        a, b, c, d = result["results"]
        self.assertEqual((a["class"], a["tp"], a["fp"], a["fn"]), ("ladder", 1, 2, 0))
        self.assertAlmostEqual(a["precision"], 1 / 3)
        self.assertEqual((b["tp"], b["fp"], b["fn"]), (1, 2, 0))
        self.assertEqual((c["class"], c["tp"], c["fp"], c["fn"]), ("inspection_label", 1, 0, 0))
        self.assertEqual((d["tp"], d["fp"], d["fn"]), (0, 0, 1))
        self.assertIsNone(d["precision"])

    def test_iou_boundaries_and_empty_denominators(self):
        gt = ground_truth()
        gt["annotations"] = [gt["annotations"][0]]
        report = self.evaluate(gt, [prediction(1, 1, [10, 10, 10, 20], .5)],
                               iou_thresholds=[.5, .51])
        self.assertEqual((report["results"][0]["tp"], report["results"][1]["fp"]), (1, 1))
        self.assertIsNone(report["results"][2]["precision"])
        self.assertIsNone(report["results"][2]["recall"])
        self.assertEqual(report["results"][1]["recall"], 0)

    def test_score_order_tie_order_and_best_unmatched(self):
        gt = ground_truth()
        gt["annotations"] = [
            {"id": 1, "image_id": 1, "category_id": 1, "bbox": [0, 0, 10, 10], "iscrowd": 0},
            {"id": 2, "image_id": 1, "category_id": 1, "bbox": [5, 0, 10, 10], "iscrowd": 0},
        ]
        predictions = [prediction(1, 1, [0, 0, 10, 10], .8),
                       prediction(1, 1, [0, 0, 15, 10], .8)]
        row = self.evaluate(gt, predictions, iou_thresholds=[.3])["results"][0]
        self.assertEqual((row["tp"], row["fp"], row["fn"]), (2, 0, 0))
        self.assertEqual(self.evaluate(gt, predictions), self.evaluate(gt, predictions))

    def test_unassessed_label_is_not_zero_recall(self):
        result = self.evaluate(ground_truth(), [], assessed_classes=["ladder"])
        self.assertEqual(result["results"][1], {"category_id": 2, "class": "inspection_label", "status": "unassessed"})
        with self.assertRaisesRegex(ValueError, "not declared assessed"):
            self.evaluate(ground_truth(), [prediction(1, 2, [10, 10, 5, 5], .9)],
                          assessed_classes=["ladder"])

    def test_split_group_check_and_unverified(self):
        train = copy.deepcopy(ground_truth())
        with self.assertRaisesRegex(ValueError, "group leakage"):
            self.evaluate(ground_truth(), [], domain="real", training_ground_truth=train)
        for image in train["images"]:
            image["group_ids"] = ["train:" + str(image["id"])]
        self.assertEqual(self.evaluate(ground_truth(), [], domain="real",
                                       training_ground_truth=train)["split_check"], "declared_groups_disjoint")
        del train["images"][0]["group_ids"]
        with self.assertRaisesRegex(ValueError, "group_id"):
            self.evaluate(ground_truth(), [], training_ground_truth=train)
        self.assertEqual(self.evaluate(ground_truth(), [])["split_check"], "unverified")

    def test_invalid_inputs_rejected(self):
        gt = ground_truth()
        invalid = [
            prediction(99, 1, [1, 1, 1, 1], .5),
            prediction(1, 3, [1, 1, 1, 1], .5),
            prediction(1, 1, [1, 1, 0, 1], .5),
            prediction(1, 1, [99, 1, 2, 1], .5),
            prediction(1, 1, [1, 1, 1e-200, 1e-200], .5),
            prediction(1, 1, [1, 1, 1, 1], float("nan")),
            prediction(1, 1, [1, 1, 1, 1], True),
            prediction(1, 1, [1, 1, 1, 1], 1.1),
        ]
        for pred in invalid:
            with self.subTest(pred=pred), self.assertRaises(ValueError):
                self.evaluate(gt, [pred])
        for field, value in (("iscrowd", 1), ("id", gt["annotations"][1]["id"])):
            bad = copy.deepcopy(gt)
            bad["annotations"][0][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.evaluate(bad, [])
        with self.assertRaises(ValueError):
            self.evaluate(gt, [], iou_thresholds=[0])
        with self.assertRaises(ValueError):
            self.evaluate(gt, [], confidence_thresholds=[float("inf")])

    def test_cli_report_and_errors(self):
        args = ["--ground-truth", "gt.json", "--predictions", "pred.json", "--domain", "synthetic",
                "--prediction-kind", "fixture", "--prediction-source", "manual fixture",
                "--assessed-class", "ladder"]
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch("ultron.detection_eval._json_file", side_effect=[(ground_truth(), "a"), ([], "b")]), \
                contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            self.assertEqual(main(args), 0)
        self.assertEqual(json.loads(stdout.getvalue())["input_sha256"]["predictions"], "b")
        with contextlib.redirect_stderr(stderr):
            self.assertEqual(main(args[:3] + ["gt.json"] + args[4:]), 2)
        self.assertIn("separate files", stderr.getvalue())

    def test_strict_json_rejects_duplicate_keys_and_nonfinite_values(self):
        for content in (b'{"x": 1, "x": 2}', b'[NaN]', b'[Infinity]'):
            with self.subTest(content=content), \
                    patch("pathlib.Path.read_bytes", return_value=content), \
                    self.assertRaises(ValueError):
                _json_file("unused.json")


if __name__ == "__main__":
    unittest.main()
