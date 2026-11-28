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
