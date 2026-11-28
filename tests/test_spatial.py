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
