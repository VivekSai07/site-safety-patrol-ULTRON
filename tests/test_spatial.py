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


@unittest.skipIf(np is None, "numpy and Pillow required")
class TestReId(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        from ultron.spatial.memory import SpatialMemory
        self.memory = SpatialMemory(Path(self._tmp.name), embedder=FakeEmbedder())
        self.memory.register_sighting("ladder-1", img(0), (0.0, 0.0), "ladder")

    def test_same_object_recognised_across_position_drift(self):
        matches = self.memory.advise(img(5), (2.0, 0.0), "ladder")  # 2 m off, nearly identical look
        self.assertEqual(matches[0][0], "ladder-1")
        self.assertEqual(self.memory.classify(matches[0][1]), "same")

    def test_adjacent_lookalike_kept_distinct(self):
        matches = self.memory.advise(img(90), (0.5, 0.0), "ladder")  # 0.5 m away, looks different
        self.assertEqual(self.memory.classify(matches[0][1]), "different")

    def test_mid_similarity_is_undecided(self):
        matches = self.memory.advise(img(45), (0.0, 0.0), "ladder")  # cos 45 deg = 0.707
        self.assertEqual(self.memory.classify(matches[0][1]), "undecided")

    def test_geometry_gate_and_kind_filter(self):
        self.assertEqual(self.memory.advise(img(0), (10.0, 0.0), "ladder"), [])      # beyond 3 m
        self.assertEqual(self.memory.advise(img(0), (0.0, 0.0), "extinguisher"), []) # other kind
        self.assertEqual(self.memory.neighbours((1.0, 0.0), "ladder"), ["ladder-1"])

    def test_explicit_candidates_override_the_gate(self):
        matches = self.memory.advise(img(0), (10.0, 0.0), "ladder", candidate_ids=["ladder-1"])
        self.assertEqual(matches[0][0], "ladder-1")

    def test_without_embedder_advice_is_empty(self):
        from ultron.spatial.memory import SpatialMemory
        silent = SpatialMemory(Path(self._tmp.name), embedder=None)
        self.assertEqual(silent.advise(img(0), (0.0, 0.0), "ladder"), [])

    def test_cross_score_between_stored_objects(self):
        self.memory.register_sighting("ladder-2", img(5), (0.5, 0.0), "ladder")
        self.assertGreaterEqual(self.memory.cross_score("ladder-1", "ladder-2"), 0.99)
        self.memory.register_sighting("bare", None, (5.0, 5.0), "ladder")
        self.assertIsNone(self.memory.cross_score("ladder-1", "bare"))

    def test_inspection_guard_survives_reload(self):
        self.assertIsNone(self.memory.was_inspected("ladder-1"))
        self.memory.mark_inspected("ladder-1", "HALL-1")
        from ultron.spatial.memory import SpatialMemory
        reloaded = SpatialMemory(Path(self._tmp.name), embedder=FakeEmbedder())
        self.assertEqual(reloaded.was_inspected("ladder-1"), "HALL-1")


@unittest.skipIf(np is None, "numpy and Pillow required")
class TestPlaceMemory(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        from ultron.spatial.memory import SpatialMemory
        self.memory = SpatialMemory(self.root, embedder=FakeEmbedder())

    def test_sampling_by_distance_turn_and_time(self):
        self.assertTrue(self.memory.remember_place(img(0, 64), (0.0, 0.0, 0.0), stamp=100.0))
        # too soon, even though it moved far
        self.assertFalse(self.memory.remember_place(img(0, 64), (5.0, 0.0, 0.0), stamp=100.5))
        # late enough but barely moved and barely turned
        self.assertFalse(self.memory.remember_place(img(0, 64), (0.5, 0.0, 0.2), stamp=102.0))
        # moved a metre
        self.assertTrue(self.memory.remember_place(img(0, 64), (1.6, 0.0, 0.2), stamp=103.0))
        # turned on the spot by 90 degrees (wrap-around across pi must count the short way)
        self.assertTrue(self.memory.remember_place(img(0, 64), (1.6, 0.0, 0.2 + math.pi / 2), stamp=104.0))
        self.assertEqual(len(self.memory.places), 3)
        self.assertTrue((self.root / self.memory.places[0]["thumbnail"]).is_file())

    def test_no_pose_no_place(self):
        self.assertFalse(self.memory.remember_place(img(0, 64), None, stamp=100.0))

    def test_sampling_state_survives_reload(self):
        self.memory.remember_place(img(0, 64), (0.0, 0.0, 0.0), stamp=100.0)
        from ultron.spatial.memory import SpatialMemory
        reloaded = SpatialMemory(self.root, embedder=FakeEmbedder())
        self.assertFalse(reloaded.remember_place(img(0, 64), (0.1, 0.0, 0.0), stamp=200.0))
        self.assertEqual(len(reloaded.places), 1)

    def test_place_without_embedder_keeps_pose_and_thumbnail(self):
        from ultron.spatial.memory import SpatialMemory
        silent = SpatialMemory(self.root, embedder=None)
        self.assertTrue(silent.remember_place(img(0, 64), (0.0, 0.0, 0.0), stamp=100.0))
        self.assertIsNone(silent.places[0]["embedding"])
        self.assertTrue((self.root / silent.places[0]["thumbnail"]).is_file())


@unittest.skipIf(np is None, "numpy and Pillow required")
class TestTextQuery(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        from ultron.spatial.memory import SpatialMemory
        self.memory = SpatialMemory(Path(self._tmp.name), embedder=FakeEmbedder())
        self.memory.register_sighting("ladder-1", img(0), (1.0, 1.0), "ladder")
        self.memory.register_sighting("ext-1", img(90), (5.0, 5.0), "extinguisher")
        self.memory.remember_place(img(10, 64), (2.0, 0.0, 0.0), stamp=100.0)

    def test_best_match_first_with_positions(self):
        hits = self.memory.query_text("0", limit=3)
        self.assertEqual([h["type"] for h in hits], ["object", "place", "object"])
        self.assertEqual(hits[0]["id"], "ladder-1")
        self.assertEqual(hits[0]["position"], [1.0, 1.0])
        self.assertEqual(hits[1]["position"], [2.0, 0.0])
        self.assertEqual(hits[2]["id"], "ext-1")

    def test_limit_respected(self):
        self.assertEqual(len(self.memory.query_text("0", limit=1)), 1)

    def test_no_embedder_no_hits(self):
        from ultron.spatial.memory import SpatialMemory
        silent = SpatialMemory(Path(self._tmp.name), embedder=None)
        self.assertEqual(silent.query_text("0"), [])


@unittest.skipIf(np is None, "numpy and Pillow required")
class TestAdvisorHook(unittest.TestCase):
    def test_spatial_memory_is_an_association_advisor(self):
        from ultron.spatial import AssociationAdvisor, SpatialConfig, SpatialMemory, load_embedder  # noqa: F401
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIsInstance(SpatialMemory(tmp), AssociationAdvisor)


@unittest.skipIf(np is None, "numpy and Pillow required")
class TestCli(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        root = Path(self._tmp.name)
        self.store = root / "memory"
        self.workspace = root / "workspace"
        (self.workspace / "keyframes").mkdir(parents=True)
        # Two stored objects that look identical and stand 1 m apart: this morning's bug.
        (self.workspace / "objects.json").write_text(json.dumps([
            {"id": "ladder-1", "kind": "ladder", "position": [0.0, 0.0]},
            {"id": "ladder-2", "kind": "ladder", "position": [1.0, 0.0]},
            {"id": "ext-1", "kind": "extinguisher", "position": [4.0, 0.0]},
        ]))
        keyframes = []
        for name, value in (("ladder-1", 0), ("ladder-2", 2), ("ext-1", 90)):
            file = f"keyframes/{name}.jpg"
            Image.fromarray(img(value, 64)).save(self.workspace / file)
            keyframes.append({"object_id": name, "file": file, "box": [250, 250, 750, 750],
                              "pose": [float(value), 0.0, 0.0], "stamp": 100.0 + value})
        (self.workspace / "keyframes.json").write_text(json.dumps(keyframes))

    def run_cli(self, *argv):
        import contextlib, io
        from ultron.spatial.__main__ import main
        output = io.StringIO()
        with unittest.mock.patch("ultron.spatial.__main__.load_embedder", return_value=FakeEmbedder()):
            with contextlib.redirect_stdout(output):
                main(["--store", str(self.store), *argv])
        return output.getvalue()

    def test_ingest_builds_memory_from_workspace(self):
        out = self.run_cli("ingest", "--data", str(self.workspace))
        self.assertIn("3 objects", out)
        from ultron.spatial.memory import SpatialMemory
        memory = SpatialMemory(self.store, embedder=FakeEmbedder())
        self.assertEqual(memory.objects["ladder-1"]["position"], [0.0, 0.0])
        self.assertEqual(len(memory.objects["ladder-1"]["embedding_ids"]), 1)
        self.assertGreaterEqual(len(memory.places), 1)

    def test_audit_flags_the_double_detection(self):
        self.run_cli("ingest", "--data", str(self.workspace))
        out = self.run_cli("audit")
        self.assertIn("ladder-1", out)
        self.assertIn("ladder-2", out)
        self.assertNotIn("ext-1", out)

    def test_query_prints_best_hits(self):
        self.run_cli("ingest", "--data", str(self.workspace))
        out = self.run_cli("query", "90", "--limit", "2")
        self.assertIn("ext-1", out.splitlines()[0])

    def test_status_counts(self):
        self.run_cli("ingest", "--data", str(self.workspace))
        self.assertIn("3 objects", self.run_cli("status"))
