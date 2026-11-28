"""Spatial memory: appearance re-identification and queryable place memory."""
import json
import math
from pathlib import Path
import tempfile
import unittest
import unittest.mock

try:
    import numpy as np
    from PIL import Image
except ImportError:
    np = None

ROOT = Path(__file__).resolve().parents[1]
WEIGHTS = ROOT / "clip_b32_datacomp.ts"
LADDER_IMAGES = sorted((ROOT / "datasets/nesseler_photoreal_v2/images").glob("S01-01-*-a000-*.png"))


def img(angle, size=8):
    """An image whose fake embedding points `angle` degrees from the reference."""
    return np.full((size, size, 3), angle, dtype=np.uint8)


class FakeEmbedder:
    """Deterministic: the first pixel value is the embedding's angle in degrees.

    cosine(img(a), img(b)) == cos(a - b), so similarity is fully controllable:
    0 deg apart = 1.0 (same), 45 deg = 0.707 (undecided), 90 deg = 0.0 (different).
    Texts that are digit strings embed like images of that angle.
    """

    def _vector(self, degrees):
        angle = math.radians(float(degrees))
        vector = np.zeros(512, dtype=np.float32)
        vector[0], vector[1] = math.cos(angle), math.sin(angle)
        return vector

    def embed_image(self, image):
        return self._vector(int(np.asarray(image).flat[0]))

    def embed_text(self, text):
        return self._vector(int(text) if text.strip().isdigit() else 0)


@unittest.skipIf(np is None, "numpy and Pillow required")
class TestEmbedderLoading(unittest.TestCase):
    def test_load_embedder_degrades_to_none(self):
        from ultron.spatial import embedder
        with unittest.mock.patch.object(embedder, "ClipEmbedder", side_effect=RuntimeError("no model")):
            self.assertIsNone(embedder.load_embedder())


@unittest.skipIf(np is None, "numpy and Pillow required")
class TestRealEmbedder(unittest.TestCase):
    """Loads the real CLIP TorchScript; skipped wherever torch or the weights are absent."""

    @unittest.skipUnless(WEIGHTS.is_file() and len(LADDER_IMAGES) >= 2, "CLIP weights or fixtures absent")
    def test_same_ladder_views_beat_noise(self):
        try:
            import torch  # noqa: F401
        except ImportError:
            self.skipTest("torch not installed")
        from ultron.spatial.embedder import load_embedder
        embedder = load_embedder(str(WEIGHTS))
        self.assertIsNotNone(embedder)
        view_a = embedder.embed_image(Image.open(LADDER_IMAGES[0]).convert("RGB"))
        view_b = embedder.embed_image(Image.open(LADDER_IMAGES[1]).convert("RGB"))
        rng = np.random.default_rng(0)
        noise = embedder.embed_image(rng.integers(0, 255, (224, 224, 3), dtype=np.uint8))
        self.assertAlmostEqual(float(np.linalg.norm(view_a)), 1.0, places=3)
        self.assertGreater(float(view_a @ view_b), float(view_a @ noise))
        text = embedder.embed_text("a ladder on a construction site")
        self.assertEqual(text.shape, (512,))
        self.assertGreater(float(view_a @ text), float(noise @ text))


@unittest.skipIf(np is None, "numpy and Pillow required")
class TestObjectStore(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)

    def memory(self, embedder="fake", **config):
        from ultron.spatial.memory import SpatialConfig, SpatialMemory
        return SpatialMemory(self.root, embedder=FakeEmbedder() if embedder == "fake" else embedder,
                             config=SpatialConfig(**config))

    def test_register_creates_object_with_embedding(self):
        memory = self.memory()
        memory.register_sighting("obj-1", img(0), (1.0, 2.0), "ladder")
        record = memory.objects["obj-1"]
        self.assertEqual(record["kind"], "ladder")
        self.assertEqual(record["position"], [1.0, 2.0])
        self.assertEqual(len(record["embedding_ids"]), 1)
        self.assertIsNone(record["inspected"])

    def test_position_follows_sightings_with_ema(self):
        memory = self.memory()
        memory.register_sighting("obj-1", img(0), (0.0, 0.0), "ladder")
        memory.register_sighting("obj-1", img(0), (1.0, 0.0), "ladder")
        self.assertAlmostEqual(memory.objects["obj-1"]["position"][0], 0.3)
        self.assertAlmostEqual(memory.objects["obj-1"]["position"][1], 0.0)

    def test_near_duplicate_sample_not_stored(self):
        memory = self.memory()
        memory.register_sighting("obj-1", img(0), (0.0, 0.0), "ladder")
        memory.register_sighting("obj-1", img(10), (0.0, 0.0), "ladder")   # cos 10 deg = 0.985 >= 0.95
        self.assertEqual(len(memory.objects["obj-1"]["embedding_ids"]), 1)
        memory.register_sighting("obj-1", img(30), (0.0, 0.0), "ladder")   # cos 30 deg = 0.866 < 0.95
        self.assertEqual(len(memory.objects["obj-1"]["embedding_ids"]), 2)

    def test_samples_capped_oldest_dropped(self):
        memory = self.memory()
        for angle in range(0, 9 * 20, 20):  # nine diverse views
            memory.register_sighting("obj-1", img(angle), (0.0, 0.0), "ladder")
        ids = memory.objects["obj-1"]["embedding_ids"]
        self.assertEqual(len(ids), 8)
        self.assertEqual(ids[0], "e000002")  # the first sample made way

    def test_persistence_round_trip(self):
        self.memory().register_sighting("obj-1", img(0), (1.0, 2.0), "ladder")
        reloaded = self.memory()
        self.assertEqual(reloaded.objects["obj-1"]["position"], [1.0, 2.0])
        key = reloaded.objects["obj-1"]["embedding_ids"][0]
        self.assertAlmostEqual(float(np.linalg.norm(reloaded._embeddings[key])), 1.0, places=5)

    def test_corrupt_store_moved_aside(self):
        (self.root / "objects.json").write_text("{not json")
        memory = self.memory()
        self.assertEqual(memory.objects, {})
        self.assertTrue(list(self.root.glob("objects.json.corrupt-*")))

    def test_without_embedder_positions_still_tracked(self):
        memory = self.memory(embedder=None)
        memory.register_sighting("obj-1", img(0), (1.0, 2.0), "ladder")
        self.assertEqual(memory.objects["obj-1"]["embedding_ids"], [])
        self.assertEqual(memory.objects["obj-1"]["position"], [1.0, 2.0])
