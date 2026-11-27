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

    def pending(self):
        """Objects found but not yet inspected, in the order they were found."""
        return [o for o in self._objects.values() if o.status == "FOUND"]

    def _match(self, kind, position, stamp):
        same_kind = [o for o in self._objects.values() if o.kind == kind]
        if position is None:
            recent = [o for o in same_kind if o.position is None and 0 <= stamp - o.last_seen <= SAME_SIGHTING_GAP]
            return max(recent, key=lambda o: o.last_seen, default=None)
        near = [o for o in same_kind if o.position and math.dist(o.position, position) <= KINDS[kind].same_object_radius]
        return min(near, key=lambda o: math.dist(o.position, position), default=None)

    def observe(self, sighting, stamp):
        """The object this sighting belongs to: a known one nearby, else a new one."""
        kind, position = sighting.detection.kind, sighting.position
        with self._lock:
            found = self._match(kind, position, stamp)
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

    def update(self, object_id, **changes):
        with self._lock:
            found = self._objects[object_id]
            for key, value in changes.items():
                setattr(found, key, value)
            self._save()
            return found
