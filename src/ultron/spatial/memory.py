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
