# Spatial Memory with Appearance Re-Identification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A standalone `ultron.spatial` package that gives every patrolled object a visual identity (CLIP re-ID, stops the same ladder being inspected twice) plus a text-queryable place memory, exposed through an `AssociationAdvisor` hook for the `pipeline/association` branch.

**Architecture:** Advisory sidecar inspired by DimOS spatial memory (Apache-2.0, github.com/dimensionalOS/dimos). `embedder.py` wraps the CLIP B/32 TorchScript that ultralytics ships (`clip_b32_datacomp.ts` — both image and text encoders; note the repo's `mobileclip_blt.ts` only scripts the *text* path and cannot embed images, verified by loading it). `memory.py` keeps per-object embedding samples and place frames in JSON+npz, saved after every change. `hook.py` is the only import surface the association branch needs. A CLI ingests an existing pipeline workspace, answers text queries, and audits for duplicate objects.

**Tech Stack:** Python 3.10+, numpy, Pillow, torch + ultralytics (optional `[spatial]` extra; everything degrades to geometry-only without them), unittest (repo convention — CI runs `python -m unittest discover -s tests -v` without torch, so every test touching torch must skip cleanly).

**Spec:** `docs/superpowers/specs/2026-11-28-spatial-memory-design.md` (branch `feature/spatial-memory`).

## Global Constraints

- Branch: `feature/spatial-memory` (off `main`). Never touch `src/ultron/pipeline/*` — the `pipeline/association` branch owns those files overnight; our integration point is `hook.py` only.
- Tests: unittest style, in `tests/test_spatial.py`, guard `numpy`/`PIL` imports like `tests/test_pipeline.py` does; tests must pass in CI (no torch, no GPU, no weights file).
- Thresholds (from spec, all in `SpatialConfig`): gate 3.0 m; same ≥ 0.80; different ≤ 0.60; max 8 samples/object; diversity < 0.95; position EMA α = 0.3; places at ≥ 1 m or ≥ 45°, ≥ 1 s apart.
- All state saved after every change with atomic tmp+replace writes (repo convention).
- Embedding vectors: unit-norm `np.float32[512]`.
- No DimOS code vendored; no ChromaDB; no new pip packages beyond the `spatial` extra (`ultralytics`, `torch`, `numpy`, `Pillow` — all already installed on the GPU machine).
- Every commit message ends with the trailer: `Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>`
- Run tests as: `python3 -m unittest tests.test_spatial -v` from the repo root (the repo installs with `pip install -e`, already done; if imports fail, prefix `PYTHONPATH=src`).

---

### Task 1: Package and embedder

**Files:**
- Create: `src/ultron/spatial/__init__.py`
- Create: `src/ultron/spatial/embedder.py`
- Modify: `.gitignore` (the 289 MB `clip_b32_datacomp.ts` must never be committed)
- Modify: `docs/superpowers/specs/2026-11-28-spatial-memory-design.md` (embedding-model correction)
- Test: `tests/test_spatial.py`

**Interfaces:**
- Consumes: nothing from this plan.
- Produces: `ClipEmbedder(weight="clip_b32_datacomp.ts", device=None)` with `embed_image(image: PIL.Image | np.ndarray) -> np.float32[512]` and `embed_text(text: str) -> np.float32[512]` (both unit norm); `load_embedder(weight=..., device=None) -> ClipEmbedder | None` (None + warning on any failure). Module constants `WEIGHT = "clip_b32_datacomp.ts"`, `DIM = 512`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_spatial.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: ERROR — `ModuleNotFoundError: No module named 'ultron.spatial'`

- [ ] **Step 3: Write the implementation**

Create `src/ultron/spatial/__init__.py`:

```python
"""Spatial memory: object appearance re-identification and queryable place memory.

Advisory sidecar inspired by DimOS spatial memory (Apache-2.0,
github.com/dimensionalOS/dimos); no DimOS code is vendored.
"""
```

Create `src/ultron/spatial/embedder.py`:

```python
"""Image and text embeddings from the CLIP TorchScript model ultralytics ships for YOLOE.

`clip_b32_datacomp.ts` exports both encoders in one file. The repo's
`mobileclip_blt.ts` (fetched for YOLOE text prompts) only scripts the text
path, so it cannot re-identify objects by appearance. `load_embedder`
returns None when torch, ultralytics or the weights are missing; callers
then fall back to geometry-only behaviour.
"""
import logging

import numpy as np

logger = logging.getLogger(__name__)

WEIGHT = "clip_b32_datacomp.ts"
DIM = 512


class ClipEmbedder:
    """CLIP B/32 (datacomp) image and text encoder; vectors are unit-norm float32[512]."""

    def __init__(self, weight=WEIGHT, device=None):
        import torch
        from ultralytics.nn.text_model import MobileCLIPTS

        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.model = MobileCLIPTS(device=self.device, weight=weight)
        if not hasattr(self.model, "image_preprocess"):
            raise ValueError(f"{weight} does not export an image encoder")

    def embed_image(self, image):
        from PIL import Image

        if not isinstance(image, Image.Image):
            image = Image.fromarray(np.asarray(image, dtype=np.uint8))
        tensor = self.model.image_preprocess(image.convert("RGB")).unsqueeze(0).to(self.device)
        return np.asarray(self.model.encode_image(tensor)[0].cpu(), dtype=np.float32)

    def embed_text(self, text):
        return np.asarray(self.model.encode_text(self.model.tokenize([text]))[0].cpu(), dtype=np.float32)


def load_embedder(weight=WEIGHT, device=None):
    """The embedder, or None (geometry-only degradation) when the stack is unavailable."""
    try:
        return ClipEmbedder(weight=weight, device=device)
    except Exception as error:  # any missing dependency, file or GPU trouble
        logger.warning("spatial memory runs without appearance matching: %s", error)
        return None
```

Notes for the implementer:
- `MobileCLIPTS` (in `ultralytics.nn.text_model`) calls `attempt_download_asset(weight)`, so passing an absolute path to an existing file is honoured without a download, and the bare name downloads once into the working directory. Its `encode_image`/`encode_text` are wrapped in `smart_inference_mode` and the TorchScript normalises outputs — do not re-wrap in `no_grad` or re-normalise.
- On CUDA the wrapper runs the model in fp16; `.cpu()` + `dtype=np.float32` in our code makes the output dtype uniform.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: `test_load_embedder_degrades_to_none` PASS; `test_same_ladder_views_beat_noise` PASS (machine has torch + weights) or SKIP elsewhere. Record the two real cosines printed by adding a temporary `print` only if debugging — do not leave prints in.

- [ ] **Step 5: Ignore the downloaded weights**

In `.gitignore`, directly under the existing `mobileclip*.ts` line (line 35), add:

```gitignore
clip_*.ts
```

Verify: `git check-ignore clip_b32_datacomp.ts` prints the filename.

- [ ] **Step 6: Amend the spec's embedding-model paragraph**

In `docs/superpowers/specs/2026-11-28-spatial-memory-design.md`, replace the "## Embedding model" paragraph body with:

```markdown
Reuse the ultralytics CLIP TorchScript assets. The repo's `mobileclip_blt.ts`
(fetched for YOLOE text prompts) only scripts the text encoder — verified by
loading it — so the embedder uses `clip_b32_datacomp.ts` (CLIP B/32, 289 MB,
one-time download from ultralytics' asset store, cached in the repo root),
which exports image and text encoders in a shared 512-d space. No new pip
dependencies: the `spatial` extra is
`["ultralytics>=8.3.0", "torch>=2.2", "numpy>=1.24", "Pillow>=9"]`,
all already installed on the GPU machine.
```

Also in the spec's "## Place memory" section, replace the CLI line so it matches the implementation:

```markdown
CLI: `python -m ultron.spatial --store runs/sites/ctrl-hall/spatial query "fire extinguisher"`
prints ranked hits with `(x, y)`.
```

- [ ] **Step 7: Commit**

```bash
cd ~/ULTRON
git add src/ultron/spatial/ tests/test_spatial.py .gitignore docs/superpowers/specs/2026-11-28-spatial-memory-design.md
git commit -m "Embed images and text with the CLIP TorchScript YOLOE already uses

mobileclip_blt.ts only scripts the text path, so the image-capable
clip_b32_datacomp.ts asset is used instead; load_embedder degrades to
None so everything stays geometry-only without torch.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

### Task 2: Object store with diversity-capped samples

**Files:**
- Create: `src/ultron/spatial/memory.py`
- Test: `tests/test_spatial.py` (append)

**Interfaces:**
- Consumes: `FakeEmbedder` and `img()` from Task 1's test file.
- Produces: `SpatialConfig` dataclass (fields and defaults exactly as in Global Constraints plus `place_*` fields used by Task 4); `SpatialMemory(root, embedder=None, config=None)` with attributes `root: Path`, `objects: dict[str, dict]`, `places: list[dict]`, `config`, and methods `register_sighting(object_id, crop, position, kind) -> None`, internal `_embed(image) -> np.ndarray | None`, `_save() -> None`. Object record dict keys: `kind`, `position` (list `[x, y]` or None), `embedding_ids` (list of `"e%06d"` keys into `_embeddings`), `inspected` (None or `{"mission": str, "stamp": float}`), `last_seen` (float).

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_spatial.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: the new tests ERROR with `ModuleNotFoundError: No module named 'ultron.spatial.memory'`; Task 1 tests still pass.

- [ ] **Step 3: Write the implementation**

Create `src/ultron/spatial/memory.py`:

```python
"""The spatial memory itself: who was seen where, how they look, what was inspected.

Geometry gates candidates, appearance decides; state is JSON + npz written
after every change so a crash loses at most the frame in hand.
"""
from dataclasses import dataclass
import json
import logging
import math
from pathlib import Path
import time

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class SpatialConfig:
    gate_radius: float = 3.0           # m: geometry gate for candidate objects
    same_threshold: float = 0.80       # cosine at/above which a sighting is the stored object
    different_threshold: float = 0.60  # cosine at/below which it is another object
    max_samples: int = 8               # appearance samples kept per object
    diversity_threshold: float = 0.95  # a new sample this close to a stored one is redundant
    position_alpha: float = 0.3        # EMA weight of a new sighting's position
    place_distance: float = 1.0        # m moved before a new place frame
    place_angle: float = math.pi / 4   # rad turned before a new place frame
    place_interval: float = 1.0        # s between place frames


def _unit(vector):
    vector = np.asarray(vector, dtype=np.float32)
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm else vector


def _move_aside(path, error):
    aside = path.with_name(f"{path.name}.corrupt-{time.time_ns()}")
    path.replace(aside)
    logger.warning("unreadable %s moved to %s: %s", path.name, aside.name, error)


class SpatialMemory:
    """Per-site object appearance store and place memory. Knows no pipeline types:
    images are PIL or numpy, positions are (x, y), kinds are strings."""

    def __init__(self, root, embedder=None, config=None):
        self.root = Path(root)
        (self.root / "places").mkdir(parents=True, exist_ok=True)
        self.embedder = embedder
        self.config = config or SpatialConfig()
        self.objects = self._load_json("objects.json") or {}
        self.places = self._load_json("places.json") or []
        self._embeddings = self._load_npz("embeddings.npz")
        self._place_embeddings = self._load_npz("place_embeddings.npz")
        self._count = max((int(key[1:]) for key in self._embeddings), default=0)
        last = self.places[-1] if self.places else None
        self._last_place = (*last["pose"], last["stamp"]) if last else None

    # persistence ----------------------------------------------------------
    def _load_json(self, name):
        path = self.root / name
        if not path.is_file():
            return None
        try:
            return json.loads(path.read_text())
        except (json.JSONDecodeError, OSError) as error:
            _move_aside(path, error)
            return None

    def _load_npz(self, name):
        path = self.root / name
        if not path.is_file():
            return {}
        try:
            with np.load(path) as data:
                return {key: data[key] for key in data.files}
        except Exception as error:
            _move_aside(path, error)
            return {}

    def _save(self):
        for name, payload in (("objects.json", self.objects), ("places.json", self.places)):
            temporary = self.root / (name + ".tmp")
            temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=1))
            temporary.replace(self.root / name)
        for name, vectors in (("embeddings.npz", self._embeddings),
                              ("place_embeddings.npz", self._place_embeddings)):
            temporary = self.root / ("tmp-" + name)
            np.savez(temporary, **vectors)
            temporary.replace(self.root / name)

    # objects ----------------------------------------------------------------
    def _embed(self, image):
        if self.embedder is None or image is None:
            return None
        try:
            return _unit(self.embedder.embed_image(image))
        except Exception as error:  # one bad frame must not end the patrol
            logger.warning("embedding failed: %s", error)
            return None

    def register_sighting(self, object_id, crop, position, kind):
        """Fold one sighting of a known object into the memory."""
        record = self.objects.setdefault(object_id, {
            "kind": kind, "position": None, "embedding_ids": [], "inspected": None, "last_seen": 0.0})
        record["kind"] = kind
        record["last_seen"] = time.time()
        if position is not None:
            x, y = float(position[0]), float(position[1])
            if record["position"] is None:
                record["position"] = [x, y]
            else:
                alpha = self.config.position_alpha
                record["position"] = [(1 - alpha) * record["position"][0] + alpha * x,
                                      (1 - alpha) * record["position"][1] + alpha * y]
        embedding = self._embed(crop)
        if embedding is not None:
            samples = [self._embeddings[key] for key in record["embedding_ids"]]
            closest = max((float(embedding @ sample) for sample in samples), default=-1.0)
            if closest < self.config.diversity_threshold:  # a genuinely new viewpoint
                if len(samples) >= self.config.max_samples:
                    self._embeddings.pop(record["embedding_ids"].pop(0), None)
                self._count += 1
                key = f"e{self._count:06d}"
                self._embeddings[key] = embedding
                record["embedding_ids"].append(key)
        self._save()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: all PASS (real-embedder test may SKIP off the GPU machine).

- [ ] **Step 5: Commit**

```bash
cd ~/ULTRON
git add src/ultron/spatial/memory.py tests/test_spatial.py
git commit -m "Remember each object's position and up to eight diverse appearance samples

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

### Task 3: Re-ID advice and the inspection guard

**Files:**
- Modify: `src/ultron/spatial/memory.py` (append methods to `SpatialMemory`)
- Test: `tests/test_spatial.py` (append)

**Interfaces:**
- Consumes: `SpatialMemory` internals from Task 2.
- Produces: `advise(crop, position, kind, candidate_ids=None) -> list[tuple[str, float]]` (best first; `[]` when appearance has no say); `neighbours(position, kind=None) -> list[str]`; `classify(score: float) -> str` (`"same"`/`"different"`/`"undecided"`); `cross_score(id_a, id_b) -> float | None`; `mark_inspected(object_id, mission_id) -> None`; `was_inspected(object_id) -> str | None`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_spatial.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: new tests ERROR with `AttributeError: 'SpatialMemory' object has no attribute 'advise'` (and similar).

- [ ] **Step 3: Write the implementation**

Append to the `SpatialMemory` class in `src/ultron/spatial/memory.py`:

```python
    # re-identification -------------------------------------------------------
    def advise(self, crop, position, kind, candidate_ids=None):
        """Appearance scores for candidate objects, best first.

        [] means appearance has no say (no embedder, no candidates, or no samples):
        the caller keeps its purely geometric behaviour. Candidates default to the
        stored objects of this kind within `gate_radius` of the sighting.
        """
        embedding = self._embed(crop)
        if embedding is None:
            return []
        if candidate_ids is None:
            candidate_ids = self.neighbours(position, kind)
        scored = []
        for object_id in candidate_ids:
            record = self.objects.get(object_id)
            samples = [self._embeddings[key] for key in (record or {}).get("embedding_ids", [])
                       if key in self._embeddings]
            if samples:
                scored.append((object_id, max(float(embedding @ sample) for sample in samples)))
        return sorted(scored, key=lambda pair: -pair[1])

    def neighbours(self, position, kind=None):
        """Stored objects (of `kind`, if given) within the geometry gate of `position`."""
        if position is None:
            return []
        return [object_id for object_id, record in self.objects.items()
                if (kind is None or record["kind"] == kind) and record["position"] is not None
                and math.dist(record["position"], position) <= self.config.gate_radius]

    def classify(self, score):
        """'same' / 'different' / 'undecided' for one advise() score.

        Conservative on purpose: a wrong 'same' loses an inspection, a wrong
        'different' only duplicates one, so the 'same' threshold is the strict one.
        """
        if score >= self.config.same_threshold:
            return "same"
        if score <= self.config.different_threshold:
            return "different"
        return "undecided"

    def cross_score(self, id_a, id_b):
        """Best appearance match between two stored objects; None when either has no samples."""
        samples_a = [self._embeddings[k] for k in self.objects.get(id_a, {}).get("embedding_ids", [])]
        samples_b = [self._embeddings[k] for k in self.objects.get(id_b, {}).get("embedding_ids", [])]
        if not samples_a or not samples_b:
            return None
        return max(float(a @ b) for a in samples_a for b in samples_b)

    # inspections --------------------------------------------------------------
    def mark_inspected(self, object_id, mission_id):
        record = self.objects.get(object_id)
        if record is not None:
            record["inspected"] = {"mission": mission_id, "stamp": time.time()}
            self._save()

    def was_inspected(self, object_id):
        """The mission that inspected this object, or None."""
        inspected = self.objects.get(object_id, {}).get("inspected")
        return inspected["mission"] if inspected else None
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
cd ~/ULTRON
git add src/ultron/spatial/memory.py tests/test_spatial.py
git commit -m "Decide same, different or undecided by appearance inside the geometry gate

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

### Task 4: Place memory

**Files:**
- Modify: `src/ultron/spatial/memory.py` (append)
- Test: `tests/test_spatial.py` (append)

**Interfaces:**
- Consumes: `SpatialMemory` from Tasks 2-3.
- Produces: `remember_place(image, pose, stamp=None, mission_id="") -> bool` (True when stored). Place record dict keys: `pose` (`[x, y, yaw]`), `stamp`, `mission`, `thumbnail` (relative path or None), `embedding` (`"p%05d"` key or None).

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_spatial.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: new tests ERROR with `AttributeError: ... no attribute 'remember_place'`.

- [ ] **Step 3: Write the implementation**

Append to `SpatialMemory` in `src/ultron/spatial/memory.py`:

```python
    # places ---------------------------------------------------------------------
    def remember_place(self, image, pose, stamp=None, mission_id=""):
        """Store a patrol frame when the robot has moved or turned enough; True when stored.

        DimOS samples places by distance and time; a patrol also pans in place,
        so a large enough turn counts as movement too.
        """
        if pose is None:
            return False
        stamp = time.time() if stamp is None else float(stamp)
        x, y = float(pose[0]), float(pose[1])
        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        if self._last_place is not None:
            last_x, last_y, last_yaw, last_stamp = self._last_place
            turned = abs((yaw - last_yaw + math.pi) % (2 * math.pi) - math.pi)
            if stamp - last_stamp < self.config.place_interval:
                return False
            if (math.dist((x, y), (last_x, last_y)) < self.config.place_distance
                    and turned < self.config.place_angle):
                return False
        index = len(self.places)
        thumbnail = self._thumbnail(image, f"places/{index:05d}.jpg")
        embedding = self._embed(image)
        key = f"p{index:05d}" if embedding is not None else None
        if key:
            self._place_embeddings[key] = embedding
        self.places.append({"pose": [x, y, yaw], "stamp": stamp, "mission": mission_id,
                            "thumbnail": thumbnail, "embedding": key})
        self._last_place = (x, y, yaw, stamp)
        self._save()
        return True

    def _thumbnail(self, image, name, max_side=320):
        if image is None:
            return None
        try:
            from PIL import Image
            if not isinstance(image, Image.Image):
                image = Image.fromarray(np.asarray(image, dtype=np.uint8))
            small = image.convert("RGB").copy()
            small.thumbnail((max_side, max_side))
            small.save(self.root / name, "JPEG", quality=80)
            return name
        except Exception as error:
            logger.warning("thumbnail failed: %s", error)
            return None
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
cd ~/ULTRON
git add src/ultron/spatial/memory.py tests/test_spatial.py
git commit -m "Remember a place frame each metre, quarter turn or longer pause

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

### Task 5: Text query over objects and places

**Files:**
- Modify: `src/ultron/spatial/memory.py` (append)
- Test: `tests/test_spatial.py` (append)

**Interfaces:**
- Consumes: object and place stores from Tasks 2-4.
- Produces: `query_text(text, limit=5) -> list[dict]`, hits sorted best first. Object hit: `{"type": "object", "id", "kind", "score", "position"}`. Place hit: `{"type": "place", "id" (thumbnail path), "score", "position" ([x, y]), "thumbnail"}`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_spatial.py` (FakeEmbedder embeds digit strings like images of that angle):

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: new tests ERROR with `AttributeError: ... no attribute 'query_text'`.

- [ ] **Step 3: Write the implementation**

Append to `SpatialMemory` in `src/ultron/spatial/memory.py`:

```python
    # queries ----------------------------------------------------------------------
    def query_text(self, text, limit=5):
        """Where was something like `text` seen? Objects and places ranked together."""
        if self.embedder is None:
            return []
        try:
            query = _unit(self.embedder.embed_text(text))
        except Exception as error:
            logger.warning("text embedding failed: %s", error)
            return []
        hits = []
        for object_id, record in self.objects.items():
            samples = [self._embeddings[key] for key in record["embedding_ids"] if key in self._embeddings]
            if samples:
                hits.append({"type": "object", "id": object_id, "kind": record["kind"],
                             "score": max(float(query @ sample) for sample in samples),
                             "position": record["position"]})
        for place in self.places:
            key = place.get("embedding")
            if key and key in self._place_embeddings:
                hits.append({"type": "place", "id": place["thumbnail"],
                             "score": float(query @ self._place_embeddings[key]),
                             "position": place["pose"][:2], "thumbnail": place["thumbnail"]})
        return sorted(hits, key=lambda hit: -hit["score"])[:limit]
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
cd ~/ULTRON
git add src/ultron/spatial/memory.py tests/test_spatial.py
git commit -m "Answer where something was seen: rank objects and places against a text query

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

### Task 6: Advisor hook for the association branch

**Files:**
- Create: `src/ultron/spatial/hook.py`
- Modify: `src/ultron/spatial/__init__.py`
- Test: `tests/test_spatial.py` (append)

**Interfaces:**
- Consumes: `SpatialMemory` method signatures from Tasks 2-3.
- Produces: `AssociationAdvisor` runtime-checkable Protocol with exactly `advise(crop, position, kind, candidate_ids=None)`, `register_sighting(object_id, crop, position, kind)`, `mark_inspected(object_id, mission_id)`, `was_inspected(object_id)`. Package exports: `from ultron.spatial import SpatialMemory, SpatialConfig, AssociationAdvisor, load_embedder`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_spatial.py`:

```python
@unittest.skipIf(np is None, "numpy and Pillow required")
class TestAdvisorHook(unittest.TestCase):
    def test_spatial_memory_is_an_association_advisor(self):
        from ultron.spatial import AssociationAdvisor, SpatialConfig, SpatialMemory, load_embedder  # noqa: F401
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIsInstance(SpatialMemory(tmp), AssociationAdvisor)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial.TestAdvisorHook -v`
Expected: ERROR — `ImportError: cannot import name 'AssociationAdvisor'`.

- [ ] **Step 3: Write the implementation**

Create `src/ultron/spatial/hook.py`:

```python
"""The import surface for the association pipeline: appearance advice at its decision points.

`objects.py` on the association branch calls `advise()` where it asks "is this
sighting a known object?" and "are these two the same?", interprets the scores
with `classify()`, and otherwise keeps its geometric behaviour. `was_inspected`
lets the inspection stage skip objects a mission already inspected.
"""
from typing import Protocol, runtime_checkable


@runtime_checkable
class AssociationAdvisor(Protocol):
    def advise(self, crop, position, kind, candidate_ids=None):
        """Appearance scores (object_id, cosine) for the caller's candidates, best
        first. An empty list means appearance has no say."""
        ...

    def register_sighting(self, object_id, crop, position, kind):
        """Fold one sighting of a (possibly new) object into the memory."""
        ...

    def mark_inspected(self, object_id, mission_id): ...

    def was_inspected(self, object_id):
        """The mission that inspected this object, or None."""
        ...
```

Replace `src/ultron/spatial/__init__.py` content with:

```python
"""Spatial memory: object appearance re-identification and queryable place memory.

Advisory sidecar inspired by DimOS spatial memory (Apache-2.0,
github.com/dimensionalOS/dimos); no DimOS code is vendored.
"""
from .embedder import load_embedder
from .hook import AssociationAdvisor
from .memory import SpatialConfig, SpatialMemory

__all__ = ["AssociationAdvisor", "SpatialConfig", "SpatialMemory", "load_embedder"]
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
cd ~/ULTRON
git add src/ultron/spatial/hook.py src/ultron/spatial/__init__.py tests/test_spatial.py
git commit -m "Name the advisor hook the association pipeline builds against

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

### Task 7: CLI — ingest a workspace, query, audit duplicates

**Files:**
- Create: `src/ultron/spatial/__main__.py`
- Modify: `pyproject.toml` (add the `spatial` extra)
- Test: `tests/test_spatial.py` (append)

**Interfaces:**
- Consumes: `SpatialMemory`, `load_embedder` from earlier tasks; the pipeline workspace format (read-only): `objects.json` is a JSON *list* of dicts with `id`, `kind`, `position`; `keyframes.json` is a JSON list with `object_id`, `file` (JPEG path relative to the workspace), `box` (`[x0, y0, x1, y1]` in thousandths of the image), `pose` (`[x, y, yaw]` or null), `stamp`.
- Produces: `python -m ultron.spatial --store DIR {ingest,query,audit,status}`; `main(argv=None)` for tests.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_spatial.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial.TestCli -v`
Expected: ERROR — `ModuleNotFoundError: No module named 'ultron.spatial.__main__'`.

- [ ] **Step 3: Write the implementation**

Create `src/ultron/spatial/__main__.py`:

```python
"""Spatial memory from the command line: build it from a patrol workspace, ask it questions.

    python -m ultron.spatial --store runs/spatial ingest --data runs/pipeline
    python -m ultron.spatial --store runs/spatial query "wooden step ladder" --limit 3
    python -m ultron.spatial --store runs/spatial audit    # one object stored twice?
    python -m ultron.spatial --store runs/spatial status
"""
import argparse
import json
import math
from pathlib import Path

from .embedder import load_embedder
from .memory import SpatialMemory


def _memory(args, need_embedder):
    embedder = load_embedder()
    if need_embedder and embedder is None:
        raise SystemExit("appearance model unavailable; pip install -e '.[spatial]' and retry")
    return SpatialMemory(args.store, embedder=embedder)


def ingest(args):
    """Feed every keyframe of an existing pipeline workspace into the memory."""
    from PIL import Image

    memory = _memory(args, need_embedder=True)
    workspace = Path(args.data)
    objects = {record["id"]: record for record in json.loads((workspace / "objects.json").read_text())}
    for keyframe in json.loads((workspace / "keyframes.json").read_text()):
        record = objects.get(keyframe["object_id"])
        if record is None:
            continue
        image = Image.open(workspace / keyframe["file"]).convert("RGB")
        width, height = image.size
        x0, y0, x1, y1 = keyframe["box"]
        crop = image.crop((x0 * width // 1000, y0 * height // 1000,
                           x1 * width // 1000, y1 * height // 1000))
        memory.register_sighting(record["id"], crop, record.get("position"), record["kind"])
        if keyframe.get("pose"):
            memory.remember_place(image, keyframe["pose"], stamp=keyframe.get("stamp"),
                                  mission_id=args.mission)
    print(f"{len(memory.objects)} objects, {len(memory.places)} places remembered in {memory.root}")


def query(args):
    memory = _memory(args, need_embedder=True)
    hits = memory.query_text(args.text, limit=args.limit)
    if not hits:
        print("nothing remembered matches")
    for hit in hits:
        where = ", ".join(f"{value:.1f}" for value in (hit.get("position") or []))
        label = hit.get("kind") or hit.get("thumbnail") or ""
        print(f"{hit['score']:.3f}  {hit['type']:6}  {hit['id']}  {label}  ({where})")


def audit(args):
    """Stored object pairs that look like one object recorded twice."""
    memory = _memory(args, need_embedder=False)
    identifiers = sorted(memory.objects)
    suspects = 0
    for index, id_a in enumerate(identifiers):
        for id_b in identifiers[index + 1:]:
            record_a, record_b = memory.objects[id_a], memory.objects[id_b]
            if record_a["kind"] != record_b["kind"]:
                continue
            if not record_a["position"] or not record_b["position"]:
                continue
            if math.dist(record_a["position"], record_b["position"]) > memory.config.gate_radius:
                continue
            score = memory.cross_score(id_a, id_b)
            if score is not None and score >= memory.config.same_threshold:
                print(f"likely one object stored twice: {id_a} and {id_b} (cosine {score:.3f})")
                suspects += 1
    if not suspects:
        print("no likely duplicates")


def status(args):
    memory = _memory(args, need_embedder=False)
    inspected = sum(1 for record in memory.objects.values() if record["inspected"])
    print(f"{len(memory.objects)} objects ({inspected} inspected), {len(memory.places)} places")
    for object_id, record in sorted(memory.objects.items()):
        where = ", ".join(f"{value:.1f}" for value in (record["position"] or []))
        done = record["inspected"]["mission"] if record["inspected"] else "-"
        print(f"  {object_id}  {record['kind']}  ({where})  samples={len(record['embedding_ids'])}  inspected={done}")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m ultron.spatial",
                                     description="Build and query the patrol's spatial memory.")
    parser.add_argument("--store", default="runs/spatial", help="memory folder (default runs/spatial)")
    commands = parser.add_subparsers(dest="command", required=True)
    command = commands.add_parser("ingest", help="feed a pipeline workspace into the memory")
    command.add_argument("--data", default="runs/pipeline", help="pipeline workspace folder")
    command.add_argument("--mission", default="", help="mission id recorded with the places")
    command.set_defaults(run=ingest)
    command = commands.add_parser("query", help="ask where something was seen")
    command.add_argument("text")
    command.add_argument("--limit", type=int, default=5)
    command.set_defaults(run=query)
    command = commands.add_parser("audit", help="find objects that are likely one object stored twice")
    command.set_defaults(run=audit)
    command = commands.add_parser("status", help="what the memory holds")
    command.set_defaults(run=status)
    arguments = parser.parse_args(argv)
    arguments.run(arguments)


if __name__ == "__main__":
    main()
```

Add to `pyproject.toml` under `[project.optional-dependencies]`, after the `perception-vlm` line:

```toml
# Appearance re-ID and place memory; reuses the CLIP TorchScript ultralytics downloads.
spatial = ["ultralytics>=8.3.0", "torch>=2.2", "numpy>=1.24", "Pillow>=9"]
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ~/ULTRON && python3 -m unittest tests.test_spatial -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
cd ~/ULTRON
git add src/ultron/spatial/__main__.py pyproject.toml tests/test_spatial.py
git commit -m "Build the memory from a patrol workspace; query it and audit for doubles

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

### Task 8: Documentation, full suite, real-model smoke run

**Files:**
- Create: `docs/SPATIAL_MEMORY.md`
- Test: whole suite + manual smoke run

**Interfaces:**
- Consumes: everything above.
- Produces: the handoff document Kaan reads to wire `advise()` into `objects.py`.

- [ ] **Step 1: Write the documentation**

Create `docs/SPATIAL_MEMORY.md`:

```markdown
# Spatial memory (`ultron.spatial`)

Appearance re-identification and place memory for the patrol, inspired by
DimOS spatial memory (Apache-2.0, github.com/dimensionalOS/dimos; concept
only, no code vendored). It exists to stop one ladder being stored — and
inspected — twice, and to answer "where did you see X?".

## Why appearance

The association pipeline matches sightings to objects by geometry. That
fails two ways: pose drift turns one ladder into two objects, and two
similar ladders close together collapse into one. The memory adds the
missing signal: CLIP image embeddings per object (up to 8 viewpoint-diverse
samples), compared by cosine.

**Decision rule** (`SpatialMemory.advise` + `classify`): geometry gates the
candidates (same kind within 3 m); appearance decides — cosine >= 0.80 is the
same object, <= 0.60 is a different one, in between is undecided and the
caller keeps its geometric behaviour. Thresholds are first guesses in
`SpatialConfig`; calibrate against the hall recording.

## Model

`clip_b32_datacomp.ts` (CLIP B/32, ultralytics asset, one-time 289 MB
download, cached in the repo root). The repo's `mobileclip_blt.ts` only
scripts the *text* encoder and cannot embed images. Without torch or the
weights everything degrades to geometry-only: `load_embedder()` returns
None, `advise()` returns [], places keep pose + thumbnail.

## For the association branch (Kaan)

`from ultron.spatial import AssociationAdvisor, SpatialMemory, load_embedder`

- Construct per site map: `SpatialMemory(sites_dir / map_id / "spatial", embedder=load_embedder())`.
- In `objects.py`, where a sighting is matched: pass your geometric
  candidates to `advise(crop, position, kind, candidate_ids=[...])`; a
  `classify(score) == "same"` top hit justifies matching across drift, a
  "different" verdict on all candidates justifies a new object, undecided
  changes nothing.
- After the match: `register_sighting(object_id, crop, position, kind)`.
- Before re-inspecting: `was_inspected(object_id)`; after a protocol:
  `mark_inspected(object_id, mission_id)`.
- The crop is the detection box cut from the frame (thousandths -> pixels).

## CLI

    pip install -e '.[spatial]'
    python -m ultron.spatial --store runs/spatial ingest --data runs/pipeline
    python -m ultron.spatial --store runs/spatial status
    python -m ultron.spatial --store runs/spatial audit          # one object stored twice?
    python -m ultron.spatial --store runs/spatial query "wooden step ladder"

`ingest` replays an existing pipeline workspace (objects.json + keyframes)
into the memory, so the morning's recording can be audited for the
double-detection immediately. `audit` works offline from stored vectors.

## State

`<store>/objects.json`, `embeddings.npz`, `places.json`,
`place_embeddings.npz`, `places/*.jpg` — JSON + npz, written atomically
after every change. Corrupt files are moved aside with a warning, never
fatal. Design: `docs/superpowers/specs/2026-11-28-spatial-memory-design.md`.
```

- [ ] **Step 2: Run the full test suite**

Run: `cd ~/ULTRON && python3 -m unittest discover -s tests -v 2>&1 | tail -20`
Expected: everything passes (pre-existing skips allowed); no new failures vs `git stash`-free baseline. If unrelated tests already failed on `main`, note them and move on.

- [ ] **Step 3: Real-model smoke run**

Build a tiny real workspace from the repo's ladder renders and run the CLI end to end with the real CLIP (GPU machine only):

```bash
cd ~/ULTRON && python3 - <<'EOF'
import json
from pathlib import Path
workspace = Path("runs/spatial-smoke/workspace"); (workspace / "keyframes").mkdir(parents=True, exist_ok=True)
images = sorted(Path("datasets/nesseler_photoreal_v2/images").glob("S01-01-*-a000-*.png"))[:2]
objects, keyframes = [], []
for index, source in enumerate(images):
    name = f"ladder-{index + 1}"
    file = f"keyframes/{name}.jpg"
    from PIL import Image
    Image.open(source).convert("RGB").save(workspace / file)
    objects.append({"id": name, "kind": "ladder", "position": [float(index), 0.0]})
    keyframes.append({"object_id": name, "file": file, "box": [0, 0, 1000, 1000],
                      "pose": [float(index), 0.0, 0.0], "stamp": 100.0 + index})
(workspace / "objects.json").write_text(json.dumps(objects))
(workspace / "keyframes.json").write_text(json.dumps(keyframes))
EOF
python3 -m ultron.spatial --store runs/spatial-smoke/memory ingest --data runs/spatial-smoke/workspace
python3 -m ultron.spatial --store runs/spatial-smoke/memory audit
python3 -m ultron.spatial --store runs/spatial-smoke/memory query "a ladder" --limit 3
rm -rf runs/spatial-smoke
```

Expected: ingest reports 2 objects; **audit flags ladder-1 and ladder-2 as likely one object stored twice** (two views of the same rendered ladder 1 m apart — the exact morning failure, caught); query ranks ladders with plausible scores. If audit does not flag them, record the printed cosine in the commit message and flag the thresholds for calibration — do not silently retune.

- [ ] **Step 4: Commit and push the branch**

```bash
cd ~/ULTRON
git add docs/SPATIAL_MEMORY.md
git commit -m "Document the spatial memory: decision rule, hook wiring, CLI demo

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
git push -u origin feature/spatial-memory
```
