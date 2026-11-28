import tempfile
import unittest
from pathlib import Path

from ultron.detection_predict import predict


def dataset(filename):
    return {
        "categories": [{"id": 1, "name": "ladder"}, {"id": 2, "name": "inspection_label"}],
        "images": [{"id": 1, "file_name": filename, "width": 100, "height": 80}],
        "annotations": [],
    }


class Detector:
    def __init__(self, detections):
        self.detections = detections

    def detect(self, path):
        return self.detections


class DetectionPredictTests(unittest.TestCase):
    def test_clips_predictions_without_using_annotations(self):
        with tempfile.TemporaryDirectory() as root:
            Path(root, "image.png").touch()
            got = predict(dataset("image.png"), root, Detector([
                {"box_xyxy": [-2, 5, 110, 30], "confidence": .8}
            ]))
        self.assertEqual(got, [{"image_id": 1, "category_id": 1,
                                "bbox": [0., 5., 100., 25.], "score": .8}])

    def test_rejects_bad_boxes_and_path_escape(self):
        with tempfile.TemporaryDirectory() as root:
            Path(root, "image.png").touch()
            for box in ([101, 2, 110, 4], [1, 2, float("nan"), 4]):
                with self.subTest(box=box), self.assertRaises(ValueError):
                    predict(dataset("image.png"), root, Detector([
                        {"box_xyxy": box, "confidence": .5}
                    ]))
            with self.assertRaises(ValueError):
                predict(dataset("../image.png"), root, Detector([]))


if __name__ == "__main__":
    unittest.main()
