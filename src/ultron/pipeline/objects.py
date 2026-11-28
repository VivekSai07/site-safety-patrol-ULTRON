"""The object map: every ladder, extinguisher, hydrant and first-aid kit found so far.

Saved to `objects.json` after every change, so the survey can stop at any moment and a
later run (or the inspection stage) picks up exactly where it was.
"""
import json
import math
from pathlib import Path
import threading

import numpy as np

from .catalog import KINDS
from .lift import CELL
from .types import SiteObject

TRACK_JUMP = 3.0   # radii: a tracked object that moves this far in the map is a tracker mix-up
MERGE_REACH = 2.0  # radii: a weak object this close to a strong one of its kind is the same thing
RELATIVELY_WEAK = 0.25  # an object with under this share of a neighbour's sightings is its fragment
AMBIGUOUS = object()
SAME_SIGHTING_GAP = 6.0  # seconds: without positions, detections this close in time are one object
# Footprints. On the hall recording the footprints of one wall box seen on the robot's two passes
# lie up to 0.32 m apart (pose and extrinsics error), while the two ladders, 3 m apart, are 1.6 m
# apart edge to edge: a margin of 0.4 m joins the first and keeps the second apart.
REPEATED = 2        # sightings a cell needs before it belongs to an object's footprint
MARGIN = 4          # cells around a footprint that still count as touching it
MIN_SHARE = 0.2     # of a sighting's cells that must touch a footprint for the sighting to be that object
ON_TOP = 0.8        # of a fragment's cells inside a neighbour's footprint: a part of it, even if seen apart


def _cell(key):
    i, j = key.split(",")
    return int(i), int(j)


def footprint(site_object):
    """The floor cells an object was seen covering more than once (all while it has one sighting)."""
    need = REPEATED if site_object.sightings >= REPEATED else 1
    return np.array([_cell(k) for k, n in site_object.cells.items() if n >= need], dtype=int).reshape(-1, 2)


def touching(cells, area, margin=MARGIN):
    """Share of `cells` lying within `margin` cells of `area` (both arrays of [i, j])."""
    cells = np.asarray(cells, dtype=int).reshape(-1, 2)
    if not len(cells) or not len(area):
        return 0.0
    gap = np.abs(cells[:, None, :] - area[None, :, :]).max(axis=2).min(axis=1)
    return float((gap <= margin).mean())


def _centre(site_object):
    cells = footprint(site_object)
    return [round(float(v), 3) for v in (cells.mean(axis=0) + 0.5) * CELL] if len(cells) else site_object.position


def _join(target, other):
    """Add `other`'s sightings and footprint to `target`."""
    target.sightings += other.sightings
    for key, count in other.cells.items():
        target.cells[key] = target.cells.get(key, 0) + count
    if target.cells:
        target.position = _centre(target)


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

    def _new_id(self, kind):
        number = sum(o.kind == kind for o in self._objects.values()) + 1
        while f"{KINDS[kind].prefix}-{number:03}" in self._objects:
            number += 1  # after a merge the count no longer gives a free number
        return f"{KINDS[kind].prefix}-{number:03}"

    def _match_footprint(self, kind, sighting, exclude):
        """The object whose footprint the sighting's cells touch most; objects without a
        footprint yet are matched by the old radius rule."""
        radius = KINDS[kind].same_object_radius
        scored = []
        for o in self._objects.values():
            if o.kind != kind or o.id in exclude:
                continue
            area = footprint(o)
            if len(area):
                share = touching(sighting.cells, area)
            else:
                share = 1.0 if o.position and math.dist(o.position, sighting.position) <= radius else 0.0
            if share >= MIN_SHARE:
                scored.append((share, o))
        scored.sort(key=lambda item: -item[0])
        if len(scored) > 1 and scored[1][1].id in scored[0][1].apart and scored[1][0] * 2 > scored[0][0]:
            return AMBIGUOUS  # touches two objects known to be different about equally
        return scored[0][1] if scored else None

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
            if not found:
                found = (self._match_footprint(kind, sighting, exclude) if sighting.cells
                         else self._match(kind, position, stamp, exclude))
            if found is AMBIGUOUS:
                return None
            if found is None:
                found = SiteObject(self._new_id(kind), kind, first_seen=stamp)
                self._objects[found.id] = found
            found.sightings += 1
            if sighting.cells:
                for i, j in sighting.cells:
                    key = f"{i},{j}"
                    found.cells[key] = found.cells.get(key, 0) + 1
                found.position = _centre(found)
            # An assumed range is a guess: it may seed a position but never drags a measured one.
            # Once an object has a footprint, its position is the footprint's middle.
            elif position is not None and not found.cells and not (sighting.approximate and found.sightings > 1):
                n = found.sightings - 1
                found.position = list(position) if found.position is None else [
                    round((found.position[i] * n + position[i]) / (n + 1), 3) for i in range(2)]
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
                cells = footprint(weak)

                def beside(o):
                    area = footprint(o)
                    if len(cells) and len(area):  # footprints decide where both objects have one
                        return touching(cells, area, 1) >= ON_TOP if o.id in weak.apart else touching(cells, area) >= MIN_SHARE
                    return math.dist(o.position, weak.position) <= (radius if o.id in weak.apart else MERGE_REACH * radius)
                near = [o for o in others if beside(o)]
                near = [o for o in near if weak.sightings < max(min_sightings, RELATIVELY_WEAK * o.sightings)]
                if near:
                    target = min(near, key=lambda o: math.dist(o.position, weak.position))
                    _join(target, weak)
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
            cells = footprint(wrong)
            near = [o for o in self._objects.values() if o.kind == kind and o.position and wrong.position and (
                touching(cells, footprint(o)) >= MIN_SHARE if len(cells) and o.cells
                else math.dist(o.position, wrong.position) <= KINDS[kind].same_object_radius)]
            if near:
                target = min(near, key=lambda o: math.dist(o.position, wrong.position))
                _join(target, wrong)
            else:
                target = SiteObject(self._new_id(kind), kind, wrong.position, wrong.sightings,
                                    wrong.first_seen, wrong.last_seen, verified=True, cells=dict(wrong.cells))
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
