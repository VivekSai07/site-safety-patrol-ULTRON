import json
import hashlib
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch
import uuid
import xml.etree.ElementTree as ET

try:
    import mujoco
    import numpy as np
    from PIL import Image
except ImportError as exc:
    DEPENDENCY_ERROR = str(exc)
else:
    DEPENDENCY_ERROR = None
    from ultron.detection_world import example_scene, world
    from ultron.detection_world.world import (
        _camera, _initial_data, _instances, _rle, build_xml, capture, visible_instances,
    )


def decode(rle):
    values = []
    for index, count in enumerate(rle["counts"]):
        values.extend([index % 2] * count)
    return np.array(values, dtype=np.uint8).reshape(rle["size"], order="F")


@unittest.skipIf(DEPENDENCY_ERROR is not None, f"optional simulation dependency unavailable: {DEPENDENCY_ERROR}")
class DetectionWorldTests(unittest.TestCase):
    def setUp(self):
        self.environment = patch.dict(os.environ, {"ULTRON_HALL_SCENE": ""})
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def test_independent_example_and_instance_groups(self):
        original_xml, ladders = example_scene.build_hall()
        self.assertEqual(len(ladders), 7)
        self.assertEqual(len({ladder["key"] for ladder in ladders}), 7)
        augmented_xml = build_xml()
        original_root = ET.fromstring(original_xml)
        augmented_root = ET.fromstring(augmented_xml)
        for ladder in ladders:
            name = ladder["key"]
            original = original_root.find(f"./worldbody/body[@name='{name}']")
            augmented = augmented_root.find(f"./worldbody/body[@name='{name}']")
            self.assertEqual(original.attrib, augmented.attrib)
            for geom in original.findall("geom"):
                self.assertEqual(geom.attrib, augmented.find(f"geom[@name='{geom.get('name')}']").attrib)
        model = mujoco.MjModel.from_xml_string(augmented_xml)
        self.assertGreaterEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link"), 0)
        self.assertGreater(model.njnt, 1)
        np.testing.assert_allclose(_initial_data(model).qpos[:3], [1.6, 1.5, .65])
        groups = _instances(model)
        for index in range(1, 8):
            self.assertIn((f"ladder_{index:02}", 1), groups)
            self.assertEqual(len(groups[(f"label_{index:02}", 2)]), 4)
        support = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "support_ladder_03_support_wall")
        self.assertGreaterEqual(support, 0)
        self.assertNotIn(support, groups[("ladder_03", 1)])

    def test_external_hall_requires_the_pinned_source(self):
        with tempfile.TemporaryDirectory() as directory:
            source = b"from ultron.detection_world.example_scene import BASE_HEIGHT_M, build_hall\n"
            for name in ("hall.py", "hall.py.local"):
                with self.subTest(name=name):
                    path = Path(directory) / name
                    path.write_bytes(source)
                    with patch.dict(os.environ, {"ULTRON_HALL_SCENE": str(path)}):
                        with self.assertRaisesRegex(ValueError, "does not match"):
                            build_xml()
                        with patch.object(world, "SAHIL_HALL_SHA256", hashlib.sha256(source).hexdigest()):
                            model = mujoco.MjModel.from_xml_string(build_xml())
                            self.assertGreater(model.njnt, 1)
            world._load_external_scene.cache_clear()

    def test_visible_pixels_and_hidden_instances(self):
        typ = int(mujoco.mjtObj.mjOBJ_GEOM)
        seg = np.full((4, 5, 2), -1, dtype=int)
        seg[1:3, 1:4] = [3, typ]
        seg[1, 2] = [4, typ]  # occluding label removes a ladder pixel
        objects = visible_instances(seg, {("ladder_01", 1): [3], ("label_01", 2): [4],
                                          ("ladder_hidden", 1): [5]})
        self.assertEqual(len(objects), 2)
        ladder = next(x for x in objects if x["category_id"] == 1)
        self.assertEqual(ladder["bbox"], [1, 1, 3, 2])
        self.assertEqual(ladder["area"], 5)
        self.assertEqual(decode(ladder["segmentation"])[1, 2], 0)
        self.assertEqual(_rle(np.ones((2, 2), dtype=bool))["counts"], [0, 4])

    def test_camera_validation(self):
        model = mujoco.MjModel.from_xml_string(build_xml())
        data = mujoco.MjData(model)
        for pos, target in [((0, 1, 1), (2, 2, 1)), ((1, 1, 1), (1, 1, 1)),
                            ((1, 1, 1), (1, 1, 2)), ((float("nan"), 1, 1), (2, 2, 1))]:
            with self.subTest(pos=pos, target=target), self.assertRaises(ValueError):
                _camera(model, data, pos, target)

    @unittest.skipUnless(os.environ.get("ULTRON_RENDER_TESTS") == "1", "requires rendering")
    def test_render_dataset_and_reproducibility(self):
        base = Path("runs") / ("detection-world-test-" + uuid.uuid4().hex)
        try:
            first = capture(base / "one", width=320, height=240)
            second = capture(base / "two", width=320, height=240)
            dataset = json.loads((first / "instances.json").read_text())
            self.assertEqual(dataset, json.loads((second / "instances.json").read_text()))
            self.assertEqual(dataset["info"]["scene_id"], example_scene.SCENE_ID)
            self.assertTrue(all(image["group_id"] == example_scene.SCENE_ID
                                for image in dataset["images"]))
            self.assertEqual([x["id"] for x in dataset["categories"]], [1, 2])
            self.assertEqual(len(dataset["images"]), 5)
            self.assertEqual({a["category_id"] for a in dataset["annotations"]}, {1, 2})
            annotated_images = {annotation["image_id"] for annotation in dataset["annotations"]}
            self.assertTrue(any(image["id"] not in annotated_images for image in dataset["images"]))
            for image in dataset["images"]:
                with Image.open(first / image["file_name"]) as im:
                    self.assertEqual(im.size, (320, 240))
                self.assertEqual((first / image["file_name"]).read_bytes(),
                                 (second / image["file_name"]).read_bytes())
            for annotation in dataset["annotations"]:
                mask = decode(annotation["segmentation"])
                ys, xs = np.nonzero(mask)
                self.assertEqual(annotation["area"], int(mask.sum()))
                self.assertEqual(annotation["bbox"],
                                 [int(xs.min()), int(ys.min()), int(xs.max() - xs.min() + 1),
                                  int(ys.max() - ys.min() + 1)])
                self.assertEqual(annotation["iscrowd"], 0)
            with self.assertRaises(FileExistsError):
                capture(first, width=320, height=240)
        finally:
            shutil.rmtree(base, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
