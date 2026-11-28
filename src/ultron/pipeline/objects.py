"""The object map: every ladder, extinguisher, hydrant and first-aid kit found so far.

Saved to `objects.json` after every change, so the survey can stop at any moment and a
later run (or the inspection stage) picks up exactly where it was.
"""
import json
import math
from pathlib import Path
import threading

from .catalog import KINDS
from .types import SiteObject

TRACK_JUMP = 3.0   # radii: a tracked object that moves this far in the map is a tracker mix-up
MERGE_REACH = 2.0  # radii: a weak object this close to a strong one of its kind is the same thing
RELATIVELY_WEAK = 0.25  # an object with under this share of a neighbour's sightings is its fragment
AMBIGUOUS = object()
SAME_SIGHTING_GAP = 6.0  # seconds: without positions, detections this close in time are one object


class ObjectMap:
    def __init__(self, root):
        self.path = Path(root) / "objects.json"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        saved = json.loads(self.path.read_text()) if self.path.is_file() else []
        self._objects = {item["id"]: SiteObject(**item) for item in saved}

    def _save(self):
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps([o.to_json() for o in self._objects.values()], ensure_ascii=False, indent=1))
        temporary.replace(self.path)

    def all(self):
        return list(self._objects.values())

    def get(self, object_id):
        return self._objects.get(object_id)

    def pending(self, min_sightings=3):
        """Objects found but not yet inspected, in the order they were found.

        A single sighting is not trusted: detectors produce one-off false hits.
        """
        return [o for o in self._objects.values() if o.status == "FOUND" and o.sightings >= min_sightings]

    def _match(self, kind, position, stamp, exclude):
        same_kind = [o for o in self._objects.values() if o.kind == kind and o.id not in exclude]
        if position is None:
            recent = [o for o in same_kind if o.position is None and 0 <= stamp - o.last_seen <= SAME_SIGHTING_GAP]
            return max(recent, key=lambda o: o.last_seen, default=None)
        near = sorted((o for o in same_kind if o.position and math.dist(o.position, position) <= KINDS[kind].same_object_radius),
                      key=lambda o: math.dist(o.position, position))
        if len(near) > 1 and near[1].id in near[0].apart:
            return AMBIGUOUS  # two different objects stand within reach: position cannot decide
        return near[0] if near else None

    def observe(self, sighting, stamp, exclude=(), known=None):
        """The object this sighting belongs to: a known one nearby, else a new one. None if it
        lies between two known objects and nothing says which of them it is.

        `exclude` holds ids already matched in the same frame: two boxes in one image
        are two objects, however close together they stand. `known` is the object the
        detector's tracker says this is; it is believed unless the position contradicts it.
        """
        kind, position = sighting.detection.kind, sighting.position
        with self._lock:
            found = self._objects.get(known) if known not in exclude else None
            if found and position and found.position and (
                    math.dist(found.position, position) > TRACK_JUMP * KINDS[kind].same_object_radius):
                found = None  # the tracker swapped identities
            found = found or self._match(kind, position, stamp, exclude)
            if found is AMBIGUOUS:
                return None
            if found is None:
                number = sum(o.kind == kind for o in self._objects.values()) + 1
                found = SiteObject(f"{KINDS[kind].prefix}-{number:03}", kind, first_seen=stamp)
                self._objects[found.id] = found
            # An assumed range is a guess: it may seed a position but never drags a measured one.
            if position is not None and not (sighting.approximate and found.sightings):
                n = found.sightings
                found.position = list(position) if found.position is None else [
                    round((found.position[i] * n + position[i]) / (n + 1), 3) for i in range(2)]
            found.sightings += 1
            found.last_seen = stamp
            self._save()
            return found

    def seen_together(self, ids):
        """Objects of one frame are certainly different from each other: remember that."""
        with self._lock:
            for object_id in ids:
                found = self._objects[object_id]
                found.apart = sorted(set(found.apart) | (set(ids) - {object_id}))
            self._save()

    def consolidate(self, min_sightings=3):
        """Fold weakly seen objects into a well-seen one of the same kind standing next to them.

        A detector that loses and regains an object, or boxes only part of it, leaves such
        fragments. Objects once seen in the same frame are never merged. Returns
        {removed id: id it was merged into}.
        """
        merged = {}
        with self._lock:
            by_strength = sorted((o for o in self._objects.values() if o.position), key=lambda o: o.sightings)
            for weak in by_strength:
                radius = KINDS[weak.kind].same_object_radius
                others = [o for o in self._objects.values() if o.kind == weak.kind and o.id != weak.id and o.position
                          and o.sightings >= max(min_sightings, weak.sightings)]
                # Never merge what was seen side by side, unless it sits right on top of the other:
                # then it was a second box on a part of the same object.
                near = [o for o in others if math.dist(o.position, weak.position) <= (
                    radius if o.id in weak.apart else MERGE_REACH * radius)]
                near = [o for o in near if weak.sightings < max(min_sightings, RELATIVELY_WEAK * o.sightings)]
                if near:
                    target = min(near, key=lambda o: math.dist(o.position, weak.position))
                    target.sightings += weak.sightings
                    merged[weak.id] = target.id
                    del self._objects[weak.id]
            for found in self._objects.values():
                found.apart = [i for i in found.apart if i in self._objects]
            self._save()
        return merged

    def relabel(self, object_id, kind):
        """The object turned out to be of another kind. Returns the id it now has: that of a
        known object of the right kind at the same place, else a fresh id of the right kind."""
        with self._lock:
            wrong = self._objects.pop(object_id)
            near = [o for o in self._objects.values() if o.kind == kind and o.position and wrong.position
                    and math.dist(o.position, wrong.position) <= KINDS[kind].same_object_radius]
            if near:
                target = min(near, key=lambda o: math.dist(o.position, wrong.position))
                target.sightings += wrong.sightings
            else:
                number = sum(o.kind == kind for o in self._objects.values()) + 1
                while f"{KINDS[kind].prefix}-{number:03}" in self._objects:
                    number += 1
                target = SiteObject(f"{KINDS[kind].prefix}-{number:03}", kind, wrong.position, wrong.sightings,
                                    wrong.first_seen, wrong.last_seen, verified=True)
                self._objects[target.id] = target
            for found in self._objects.values():
                found.apart = [i for i in found.apart if i != object_id]
            self._save()
            return target.id

    def update(self, object_id, **changes):
        with self._lock:
            found = self._objects[object_id]
            for key, value in changes.items():
                setattr(found, key, value)
            self._save()
            return found
