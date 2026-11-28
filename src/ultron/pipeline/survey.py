"""Stage 1, survey: turn the camera stream into an object map and keyframes.

Feed it frames at any rate. It keeps no state of its own beyond the rate limit: objects
and keyframes are on disk after every frame, so pausing is simply not feeding it.
"""
import copy
import math

from PIL import Image

from .anchor import Anchoring
from .catalog import KINDS
from .lift import lift
from .moved import look_through
from .objects import MIN_SHARE, footprint, offset, touching
from .verify import NONE


EDGE = 10               # thousandths: a box this close to the top or bottom of the image is cut off
VERIFY_VIEWS = 3        # keyframes of an object shown to the vision model, one call each
SAME_BOX_OVERLAP = 0.5  # boxes of one kind overlapping this much (intersection over union) are one object
PART_OF_BOX = 0.7       # a box of one kind lying this much inside a better one is a part of it
WIDE = 0.5              # metres: objects larger than this look different from every side
EMPTY_VIEWS = 3         # views from places this far apart (below) that saw a stored object's place empty
EMPTY_SPREAD = 0.3      # metres
# The depth camera's error grows with the square of the range. With the D457's noise (as fitted
# to the real camera, simulated yard) sightings up to 12.5 m land inside the object's footprint;
# from 15 m on 0.25-0.9 m off (90th percentile), more than footprints are matched within. Such
# sightings split objects, and their frames are useless for inspection: they are not used.
MAX_RANGE = 12.0        # metres
ANNOUNCE = 3            # sightings before an object is announced: single boxes are often false hits or parts


def overlap(a, b):
    """How much two boxes are the same thing: intersection over union, raised when one
    lies inside the other so that a contained part counts as a repeat."""
    width = min(a[2], b[2]) - max(a[0], b[0])
    height = min(a[3], b[3]) - max(a[1], b[1])
    if width <= 0 or height <= 0:
        return 0.0
    area_a, area_b = (a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1])
    shared = width * height
    return max(shared / (area_a + area_b - shared), (shared / min(area_a, area_b)) * SAME_BOX_OVERLAP / PART_OF_BOX)


def distinct(detections):
    """Best-scoring detections first, dropping repeats of the same thing."""
    kept = []
    for detection in sorted(detections, key=lambda d: d.score, reverse=True):
        if not any(k.kind == detection.kind and overlap(k.box, detection.box) > SAME_BOX_OVERLAP for k in kept):
            kept.append(detection)
    return kept


class Survey:
    def __init__(self, detector, objects, keyframes, store, calibration=None, interval=1.0, min_sharpness=0.0,
                 verifier=None, map_id=None):
        self.detector, self.objects, self.keyframes, self.store = detector, objects, keyframes, store
        self.verifier = verifier            # optional second opinion from the vision model, one call per object
        self.calibration = calibration
        self.interval = interval            # seconds between processed frames
        self.min_sharpness = min_sharpness  # frames blurrier than this are skipped
        self._last = None
        self._tracks = {}  # detector track id -> object id, for this run of the detector
        self.map_id = map_id  # the object map is shared by all runs in this map's frame
        self._warned = False
        # On a shared map, objects stored by earlier patrols are landmarks this one is lined up with.
        stored = objects.all() if map_id else []
        self.anchoring = Anchoring(stored) if stored else None
        self._empty = {}  # stored object id -> robot positions from which its place looked empty
        self.created = set()  # objects this patrol created: provisional until it is lined up
        # Objects announced (`object_found`) in this workspace: resuming does not announce them twice.
        self.announced = {e["object"] for e in store.events(limit=None) if e["kind"] == "object_found"}
        self.seen_now = {}  # object id -> sightings in this run: a patrol reports what it saw itself

    def feed(self, frame, force=False):
        """Process one frame. Returns the objects seen in it (empty if skipped or nothing found)."""
        if not force and self._last is not None and 0 <= frame.stamp - self._last < self.interval:
            return []
        sharpness = frame.sharpness()
        if not force and sharpness < self.min_sharpness:
            return []
        self._last = frame.stamp
        self.calibration = self.calibration or frame.camera  # live sources bring their own camera model
        if self.map_id and getattr(frame, "pose_frame", None) == "odometry" and not self._warned:
            self._warned = True  # objects from earlier runs line up only if this run started where they did
            self.store.emit("localisation_unconfirmed", reason="pose from odometry, not localisation in the map",
                            map_id=self.map_id)
        if self.anchoring and self.anchoring.transform and frame.pose:
            frame = copy.copy(frame)
            frame.pose = self.anchoring.to_map(frame.pose)  # this run's drift against the stored objects
        try:
            detections = distinct(self.detector.detect(frame))
        except Exception as error:  # a failing model must not end the patrol
            self.store.emit("error", stage="detect", message=str(error))
            return []
        seen = []
        # Detections the tracker already knows go first, so a newcomer cannot take their object.
        for detection in sorted(detections, key=lambda d: d.track not in self._tracks):
            sighting = lift(frame, detection, self.calibration)
            if sighting.distance and not sighting.approximate and sighting.distance > MAX_RANGE:
                continue
            low, high = KINDS[detection.kind].size
            # A box cut off by the image edge shows only part of the object: it may be any size or larger.
            cut_off = detection.box[1] <= EDGE or detection.box[3] >= 1000 - EDGE
            if sighting.height is not None and not ((cut_off or low <= sighting.height) and sighting.height <= high):
                continue  # wrong size for its kind: a look-alike
            measured = sighting.position is not None and not sighting.approximate
            create, landmark = True, None
            if self.anchoring:
                if self.anchoring.state == Anchoring.SUSPECT:
                    continue  # the robot is not where it thinks it is: change nothing
                # Until the patrol is lined up, a sighting near a stored object of its kind may be
                # that object, displaced: it creates nothing, and tells where the object appears.
                landmark = self.anchoring.near(detection.kind, sighting.position) if measured else None
                create = self.anchoring.state == Anchoring.ANCHORED or landmark is None
            # Until then, the stored objects are not moved by positions that may be off.
            frozen = self.anchoring.stored if self.anchoring and self.anchoring.state != Anchoring.ANCHORED else ()
            # Two boxes in one frame are two objects; an object seen moved elsewhere is not here.
            exclude = {o.id for o in seen} | set(self.anchoring.displaced if self.anchoring else ())
            site_object = self.objects.observe(sighting, frame.stamp, exclude=exclude,
                                               known=self._tracks.get(detection.track), create=create, frozen=frozen)
            if self.anchoring and measured:
                landmark = site_object.id if site_object and site_object.id in self.anchoring.stored else (
                    None if site_object else landmark)
                if landmark:
                    self._landmark(landmark, sighting)
                if self.anchoring.state == Anchoring.SUSPECT:
                    # Found off just now: what this frame made from its pose was discarded with the rest.
                    seen = [o for o in seen if self.objects.get(o.id)]
                    continue
            if site_object is None:
                continue  # between two known objects and no way to tell which, or waiting to line up
            if self.anchoring and site_object.sightings == 1 and site_object.id not in self.anchoring.stored:
                self.created.add(site_object.id)
            if site_object.status == "MOVED":  # it is back (or was never gone)
                site_object = self.objects.update(site_object.id, status="FOUND")
                self._empty.pop(site_object.id, None)
                self.store.emit("object_back", object=site_object.id, position=site_object.position)
            if detection.track is not None:
                self._tracks[detection.track] = site_object.id
            self.keyframes.offer(frame, site_object, sighting, sharpness)
            self.seen_now[site_object.id] = self.seen_now.get(site_object.id, 0) + 1
            seen.append(site_object)
        for kind in {o.kind for o in seen}:
            together = [o.id for o in seen if o.kind == kind]
            if len(together) > 1:
                self.objects.seen_together(together)
        self.check_moved(frame, seen)
        self.announce()
        self.store.save_live(frame)
        best = max(detections, key=lambda d: d.score, default=None)
        known = sorted({o.id for o in seen} & self.announced)
        self.store.emit("look", frame=frame.name, ladder=any(d.kind == "ladder" for d in detections),
                        view=None, box=best.box if best else None, pose=frame.pose,
                        note=", ".join(known) or ("nothing confirmed yet" if seen else "nothing found"))
        return seen

    def check_moved(self, frame, seen):
        """Stored objects whose place this frame looks right through, from a trusted pose.

        Only a patrol lined up with the stored objects can say a place is empty; one view is not
        enough, nor are views from one spot. An object detected in this patrol is not gone."""
        if not (self.anchoring and self.anchoring.state == Anchoring.ANCHORED and frame.pose):
            return
        here = {o.id for o in seen}
        for site_object in self.objects.all():
            if (site_object.id not in self.anchoring.stored or site_object.id in here
                    or self.seen_now.get(site_object.id) or site_object.status not in ("FOUND", "INSPECTED")):
                continue
            verdict = look_through(frame, self.calibration, site_object)
            if verdict == "occupied":
                self._empty.pop(site_object.id, None)
            elif verdict == "empty":
                places = self._empty.setdefault(site_object.id, [])
                if all(math.dist(p, frame.pose[:2]) >= EMPTY_SPREAD for p in places):
                    places.append(tuple(frame.pose[:2]))
                if len(places) >= EMPTY_VIEWS:
                    self.objects.update(site_object.id, status="MOVED")
                    self.store.emit("object_moved", object=site_object.id, object_kind=site_object.kind,
                                    position=site_object.position, views=len(places))

    def _landmark(self, object_id, sighting):
        """A stored object was seen (positions in the stored map's frame, as corrected so far).

        A compact object is where its view's middle is. A wide one is placed by how far the view's
        cells lie from its stored footprint, so a ladder seen from one side is not mistaken for a
        shifted ladder."""
        stored = self.objects.get(object_id)
        area = footprint(stored) if stored and stored.position else []
        position = sighting.position
        if (KINDS[stored.kind].extent > WIDE and sighting.cells and len(area)
                and touching(sighting.cells, area) >= MIN_SHARE):
            dx, dy = offset(sighting.cells, area)
            position = (self.anchoring.stored[object_id][1][0] + dx, self.anchoring.stored[object_id][1][1] + dy)
        changed = self.anchoring.add(object_id, self.anchoring.to_run(position))
        for moved, seen_at in list(self.anchoring.displaced.items()):
            if self.objects.get(moved).status != "MOVED":  # seen elsewhere: not a duplicate, a move
                self.objects.update(moved, status="MOVED")
                self.store.emit("object_moved", object=moved, object_kind=self.objects.get(moved).kind,
                                position=self.objects.get(moved).position, seen_at=[round(v, 2) for v in seen_at])
        if changed == Anchoring.SUSPECT:
            self.store.emit("localisation_suspect", map_id=self.map_id, **self.anchoring.details)
            # Created from positions that were off: nothing of this patrol is kept (none was announced).
            discarded = sorted(i for i in self.created if self.objects.get(i))
            for created in discarded:
                self.objects.remove(created)
                self.keyframes.drop(created)
                self._tracks = {t: i for t, i in self._tracks.items() if i != created}
            for created in discarded:
                self.seen_now.pop(created, None)
            self.created.clear()
            if discarded:
                self.store.emit("objects_discarded", objects=discarded, reason="localisation suspect")
        elif changed == Anchoring.ANCHORED:
            self.store.emit("localisation_anchored", map_id=self.map_id, **self.anchoring.details)

    def announce(self, min_sightings=ANNOUNCE, settled=False):
        """Report objects seen often enough (or confirmed by the vision model).

        While the survey runs, an object standing beside another of its kind waits: it may be a
        fragment, and whether it is one only shows once both are well seen. `settled` (after
        fragments were merged) reports whatever is left."""
        lined_up = not self.anchoring or self.anchoring.state == Anchoring.ANCHORED or (
            settled and self.anchoring.state == Anchoring.UNANCHORED)
        if not lined_up:
            # Poses not yet checked against the stored objects (or found off): new objects may be
            # misplaced, and a stored one may have been taken for another of its kind nearby.
            return
        for site_object in self.objects.all():
            now = self.seen_now.get(site_object.id, 0)
            if (site_object.id in self.announced or site_object.status == "REJECTED" or not now
                    or not (now >= min_sightings or site_object.verified)
                    or not settled and self.objects.beside(site_object)):
                continue
            self.announced.add(site_object.id)
            self.store.emit("object_found", object=site_object.id, object_kind=site_object.kind,
                            position=site_object.position, sightings=site_object.sightings)

    def _replace(self, old, new):
        self._tracks = {track: new if i == old else i for track, i in self._tracks.items()}
        if old in self.seen_now:
            self.seen_now[new] = self.seen_now.get(new, 0) + self.seen_now.pop(old)

    def tidy(self, min_sightings=3):
        """Merge fragments into the objects they belong to, then have the vision model check
        what is left. Call when a survey pass ends, or now and then during a long one."""
        if self.anchoring and self.anchoring.state == Anchoring.SUSPECT:
            self.announce(min_sightings, settled=True)
            return {}  # positions of this patrol are not to be trusted: merge and relabel nothing
        merged = self.objects.consolidate(min_sightings, keep=self.announced)
        for removed, target in merged.items():
            self.keyframes.reassign(removed, target)
            self._replace(removed, target)
            self.store.emit("object_merged", object=removed, into=target, announced=removed in self.announced)
            self.announced.discard(removed)
        self.announce(min_sightings, settled=True)
        self.verify(min_sightings)
        self.announce(min_sightings, settled=True)  # a confirmed object counts as found whatever its sightings
        return merged

    def verify(self, min_sightings=3):
        """Ask the vision model about every well-seen object it has not judged yet."""
        if self.verifier is None:
            return
        for site_object in self.objects.pending(min_sightings):
            views = self.keyframes.select(site_object, budget=VERIFY_VIEWS)
            if site_object.verified or not views:
                continue
            opinions, heights = [], []
            for view in views:
                with Image.open(self.keyframes.root / view.file) as image:
                    # Boxes cut off by the image edge show only a part: their height says nothing.
                    whole = EDGE < view.box[1] and view.box[3] < 1000 - EDGE
                    height = None
                    if self.calibration and view.distance and whole:
                        height = (view.box[3] - view.box[1]) / 1000 * image.height / self.calibration.fy * view.distance
                        heights.append(height)
                    opinion = self.verifier.classify(image.convert("RGB"), view.box, view.id, height)
                if opinion is not None:
                    opinions.append(opinion)
            if not opinions:
                continue  # model unreachable: the object stays unverified and is tried again next time
            kinds = [kind for kind, _ in opinions]
            kind = max(set(kinds), key=kinds.count)
            remark = next(text for answer, text in opinions if answer == kind)
            # One glance must not overturn the detector: a different verdict needs a majority of
            # at least two views. Anything less leaves the object as it is, unverified.
            # Nor may a new kind contradict the measured size: a 0.8 m cabinet is not an alarm box.
            fits = kind == NONE or not heights or all(
                KINDS[kind].size[0] <= height <= KINDS[kind].size[1] for height in heights)
            if kind != site_object.kind and (kinds.count(kind) < 2 or kinds.count(kind) * 2 <= len(kinds) or not fits):
                self.store.emit("object_unclear", object=site_object.id, opinions=kinds)
                continue
            if kind == site_object.kind:
                self.objects.update(site_object.id, verified=True)
                self.store.emit("object_verified", object=site_object.id, remark=remark)
            elif kind == NONE:
                self.objects.update(site_object.id, status="REJECTED", result={"reason": remark})
                self.keyframes.drop(site_object.id)
                self.store.emit("object_rejected", object=site_object.id, was=site_object.kind, remark=remark)
            else:
                new_id = self.objects.relabel(site_object.id, kind)
                self.keyframes.reassign(site_object.id, new_id)
                self._replace(site_object.id, new_id)
                if site_object.id in self.announced:  # the dashboard learns the new id from this event
                    self.announced.add(new_id)
                self.store.emit("object_relabelled", object=site_object.id, was=site_object.kind,
                                now=kind, into=new_id, remark=remark)

    def run(self, source):
        try:
            for frame in source:
                self.feed(frame)
        finally:
            self.tidy()
