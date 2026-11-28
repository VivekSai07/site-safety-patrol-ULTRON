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
REPEATED = 2        # sightings a cell needs before it belongs to an object's footprint ...
# ... and this share of all the object's sightings: otherwise rare stray cells pile up over many
# patrols (on the recording the A-frame grew from 1.2 x 2.1 m to 2.2 x 3.8 m over six replays; with
# the share it stays 1.2-1.3 x 2.0-2.1 m)
REPEATED_SHARE = 0.05
MARGIN = 4          # cells around a footprint that still count as touching it
MIN_SHARE = 0.2     # of a sighting's cells that must touch a footprint for the sighting to be that object
# Two objects known to be different, standing closer than MARGIN (identical ladders 0.3 m apart),
# both touch a sighting of either. Seen together from further away, each also picks up a few cells
# of the other. The sighting then belongs to the one that holds the cells under it strongly.
CLEARLY_MORE = 2.0  # times the other's support
MIN_SUPPORT = 0.2   # of the object's strongest cell, on average under the sighting
ON_TOP = 0.8        # of a fragment's cells inside a neighbour's footprint: a part of it, even if seen apart


def _cell(key):
    i, j = key.split(",")
    return int(i), int(j)


def footprint(site_object):
    """The floor cells an object was seen covering more than once (all while it has one sighting)."""
    need = max(REPEATED, REPEATED_SHARE * site_object.sightings) if site_object.sightings >= REPEATED else 1
    return np.array([_cell(k) for k, n in site_object.cells.items() if n >= need], dtype=int).reshape(-1, 2)


def touching(cells, area, margin=MARGIN):
    """Share of `cells` lying within `margin` cells of `area` (both arrays of [i, j])."""
    cells = np.asarray(cells, dtype=int).reshape(-1, 2)
    if not len(cells) or not len(area):
        return 0.0
    gap = np.abs(cells[:, None, :] - area[None, :, :]).max(axis=2).min(axis=1)
    return float((gap <= margin).mean())


def support(cells, site_object):
    """How strongly `site_object`'s footprint holds `cells`: for each, the count of its strongest
    cell at most one cell away, as a share of the object's strongest cell; averaged."""
    counts = site_object.cells
    if not counts or not len(cells):
        return 0.0
    top = max(counts.values())
    near = [max(counts.get(f"{i + di},{j + dj}", 0) for di in (-1, 0, 1) for dj in (-1, 0, 1)) for i, j in cells]
    return float(np.mean(near)) / top


def offset(cells, area):
    """Mean shift in metres from each of `cells` to the nearest cell of `area`: how far a view of
    an object lies from where its stored footprint says, whichever face of it the view shows."""
    cells = np.asarray(cells, dtype=int).reshape(-1, 2)
    gaps = cells[:, None, :] - area[None, :, :]
    nearest = gaps[np.arange(len(cells)), (gaps ** 2).sum(axis=2).argmin(axis=1)]
    return nearest.mean(axis=0) * CELL


def _centre(site_object):
    cells = footprint(site_object)
    return [round(float(v), 3) for v in (cells.mean(axis=0) + 0.5) * CELL] if len(cells) else site_object.position


def _join(target, other):
    """Add `other`'s sightings and footprint to `target`."""
    target.sightings += other.sightings
    target.views += other.views
    target.detected += other.detected
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

    def pending(self, min_sightings=3, again=()):
        """Objects found but not yet inspected, in the order they were found, and those in `again`
        that an earlier patrol inspected (or could not reach).

        A single sighting is not trusted: detectors produce one-off false hits.
        """
        return [o for o in self._objects.values() if o.sightings >= min_sightings and (
            o.status == "FOUND" or o.status in ("INSPECTED", "UNREACHABLE") and o.id in again)]

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
            # Touches two objects known to be different about equally: the one it lies on, if clear.
            cells = np.asarray(sighting.cells, dtype=int).reshape(-1, 2)
            held = sorted(((support(cells, o), o) for _, o in scored[:2]), key=lambda item: -item[0])
            if held[0][0] >= MIN_SUPPORT and held[0][0] >= CLEARLY_MORE * held[1][0]:
                return held[0][1]
            return AMBIGUOUS
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

    def observe(self, sighting, stamp, exclude=(), known=None, create=True, frozen=()):
        """The object this sighting belongs to: a known one nearby, else a new one (unless not
        `create`). None if it lies between two known objects and nothing says which of them it is.
        Objects in `frozen` count the sighting but keep their footprint and position.

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
                if not create:
                    return None
                found = SiteObject(self._new_id(kind), kind, first_seen=stamp)
                self._objects[found.id] = found
            found.sightings += 1
            if sighting.zband and found.id not in frozen:
                n = found.sightings - 1  # running mean over the sightings that measured it
                found.zband = list(sighting.zband) if not found.zband else [
                    round((found.zband[i] * n + sighting.zband[i]) / (n + 1), 3) for i in range(2)]
            if found.id in frozen:
                pass
            elif sighting.cells:
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

    def beside(self, site_object):
        """Objects of the same kind that `site_object` may be a fragment of: their footprints
        touch (or, without footprints, they stand within reach). Of objects seen in the same frame
        only one lying right on top of the other counts: a second box on a part of the same thing."""
        if not site_object.position:
            return []
        radius, cells = KINDS[site_object.kind].same_object_radius, footprint(site_object)

        def touches(o):
            area = footprint(o)
            if len(cells) and len(area):  # footprints decide where both objects have one
                return (touching(cells, area, 1) >= ON_TOP if o.id in site_object.apart
                        else touching(cells, area) >= MIN_SHARE)
            return math.dist(o.position, site_object.position) <= (
                radius if o.id in site_object.apart else MERGE_REACH * radius)
        return [o for o in self._objects.values() if o.kind == site_object.kind and o.id != site_object.id
                and o.position and o.status not in ("REJECTED", "MOVED") and touches(o)]

    def fragment_of(self, weak, min_sightings=3):
        """The well-seen neighbour (see `beside`) that `weak` is a fragment of, or None: a detector
        that loses and regains an object, or boxes only part of it, leaves objects with far fewer
        sightings than the one they stand beside."""
        near = [o for o in self.beside(weak) if o.sightings >= max(min_sightings, weak.sightings)
                and weak.sightings < max(min_sightings, RELATIVELY_WEAK * o.sightings)]
        return min(near, key=lambda o: math.dist(o.position, weak.position), default=None)

    def consolidate(self, min_sightings=3, keep=()):
        """Fold fragments into the objects they belong to. Returns {removed id: id it was merged into}.

        Ids in `keep` (already announced to someone) survive a merge: if a kept fragment folds
        into an object nobody has heard of, the merged object takes the kept id.
        """
        merged = {}
        with self._lock:
            for weak in sorted(self._objects.values(), key=lambda o: o.sightings):
                if weak.id not in self._objects:
                    continue
                target = self.fragment_of(weak, min_sightings)
                if target is None:
                    continue
                _join(target, weak)
                target.first_seen = min(target.first_seen, weak.first_seen)
                target.verified = target.verified or weak.verified
                del self._objects[weak.id]
                removed = weak.id
                if weak.id in keep and target.id not in keep:
                    removed = target.id
                    del self._objects[target.id]
                    target.id = weak.id
                    self._objects[target.id] = target
                    for found in self._objects.values():
                        found.apart = sorted({target.id if i == removed else i for i in found.apart} - {found.id})
                merged = {old: (target.id if new == removed else new) for old, new in merged.items()}
                merged[removed] = target.id
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
                                    wrong.first_seen, wrong.last_seen, verified=True, cells=dict(wrong.cells),
                                    views=wrong.views, detected=wrong.detected)
                self._objects[target.id] = target
            for found in self._objects.values():
                found.apart = [i for i in found.apart if i != object_id]
            self._save()
            return target.id

    def remove(self, object_id):
        with self._lock:
            self._objects.pop(object_id, None)
            for found in self._objects.values():
                found.apart = [i for i in found.apart if i != object_id]
            self._save()

    def count_views(self, in_view, detected):
        """Objects (ids) whose place a frame had in clear view, and which of them it detected."""
        with self._lock:
            for object_id in in_view:
                found = self._objects[object_id]
                found.views += 1
                found.detected += object_id in detected
            if in_view:
                self._save()

    def update(self, object_id, **changes):
        with self._lock:
            found = self._objects[object_id]
            for key, value in changes.items():
                setattr(found, key, value)
            self._save()
            return found
