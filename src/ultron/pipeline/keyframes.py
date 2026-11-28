"""Keyframes: the few frames per object worth keeping, chosen by geometry rather than by time.

Each object has cells (view range x viewing direction). A cell keeps its best frames by a
quality score, so memory is bounded by objects x cells x `per_cell`, however long the patrol.
The index is saved after every change. Improving the score is the place to add occlusion,
part visibility or a DAAAM-style joint selection across objects.
"""
import json
from pathlib import Path
import threading

from .catalog import KINDS
from .types import Keyframe

FULL_AREA = 0.25      # share of the image at which a larger box no longer scores higher
SHARP_ENOUGH = 100.0  # Laplacian variance at which a frame gets half the sharpness credit


def view_name(kind, distance):
    """Which catalog view a frame taken from `distance` serves; unknown range counts as overview."""
    for view in KINDS[kind].views:
        if distance is not None and view.min_range <= distance < view.max_range:
            return view.name
    return KINDS[kind].views[0].name if distance is None else None


def quality(keyframe):
    x0, y0, x1, y1 = keyframe.box
    area = (x1 - x0) * (y1 - y0) / 1e6
    return keyframe.score * min(1.0, area / FULL_AREA) * keyframe.sharpness / (keyframe.sharpness + SHARP_ENOUGH)


class KeyframeStore:
    def __init__(self, root, per_cell=2):
        self.root = Path(root)
        (self.root / "keyframes").mkdir(parents=True, exist_ok=True)
        self.index = self.root / "keyframes.json"
        self.per_cell = per_cell
        self._lock = threading.RLock()
        saved = json.loads(self.index.read_text()) if self.index.is_file() else []
        self._frames = [Keyframe(**item) for item in saved]
        self._count = max((int(k.id.split("-")[1]) for k in self._frames), default=0)

    def _save(self):
        temporary = self.index.with_suffix(".tmp")
        temporary.write_text(json.dumps([k.to_json() for k in self._frames], indent=1))
        temporary.replace(self.index)

    def for_object(self, object_id):
        return [k for k in self._frames if k.object_id == object_id]

    def drop(self, object_id):
        """Forget an object's keyframes (it was merged into another or discarded)."""
        with self._lock:
            for keyframe in self.for_object(object_id):
                (self.root / keyframe.file).unlink(missing_ok=True)
                self._frames.remove(keyframe)
            self._save()

    def reassign(self, object_id, new_id):
        """Hand an object's keyframes to another id (the object was relabelled)."""
        with self._lock:
            for keyframe in self.for_object(object_id):
                keyframe.object_id = new_id
            self._save()

    def best(self, object_id):
        """The single best frame of an object, or None."""
        return max(self.for_object(object_id), key=quality, default=None)

    def offer(self, frame, site_object, sighting, sharpness, any_range=False):
        """Keep this frame if it beats what its cell already holds. Returns the Keyframe or None.

        A frame too close or too far for any view is kept only with `any_range`: for an object
        with no other frame (one the vision-model sweep found), so that it can be checked at all.
        It serves no view: the robot is still sent to take them."""
        view = view_name(site_object.kind, sighting.distance)
        if view is None and not any_range:
            return None
        with self._lock:
            candidate = Keyframe(f"KF-{self._count + 1:05}", site_object.id, "", frame.stamp, sighting.detection.box,
                                 list(frame.pose) if frame.pose else None, sighting.distance,
                                 sighting.side(frame.pose), round(sharpness, 1), sighting.detection.score)
            cell = [k for k in self.for_object(site_object.id)
                    if k.side == candidate.side and view_name(site_object.kind, k.distance) == view]
            if len(cell) >= self.per_cell:
                worst = min(cell, key=quality)
                if quality(candidate) <= quality(worst):
                    return None
                (self.root / worst.file).unlink(missing_ok=True)
                self._frames.remove(worst)
            self._count += 1
            candidate.file = f"keyframes/{candidate.id}.jpg"
            frame.image.save(self.root / candidate.file, "JPEG", quality=90)
            self._frames.append(candidate)
            self._save()
            return candidate

    def select(self, site_object, budget=6):
        """Frames to hand to the form reader: the best of every cell first (overviews leading),
        then the runners-up, until the budget is spent."""
        cells = {}
        for keyframe in self.for_object(site_object.id):
            cells.setdefault((view_name(site_object.kind, keyframe.distance), keyframe.side), []).append(keyframe)
        order = [view.name for view in KINDS[site_object.kind].views]
        ranked = []
        for (view, _), members in cells.items():
            for rank, keyframe in enumerate(sorted(members, key=quality, reverse=True)):
                ranked.append((rank, order.index(view) if view in order else len(order), -quality(keyframe),
                               keyframe.id, keyframe))
        return [entry[-1] for entry in sorted(ranked)[:budget]]
