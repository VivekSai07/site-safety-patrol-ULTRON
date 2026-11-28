"""Stage 1, survey: turn the camera stream into an object map and keyframes.

Feed it frames at any rate. It keeps no state of its own beyond the rate limit: objects
and keyframes are on disk after every frame, so pausing is simply not feeding it.
"""
import copy
import math

import numpy as np

from PIL import Image

from .anchor import Anchoring
from .catalog import KINDS
from .lift import lift, lying
from .moved import look_through
from .objects import AMBIGUOUS, MIN_SHARE, footprint, offset, touching
from .sweep import rectangle
from .types import Sighting
from .verify import NONE


EDGE = 10               # thousandths: a box this close to the top or bottom of the image is cut off
VERIFY_VIEWS = 3        # keyframes of an object shown to the vision model, one call each
SAME_BOX_OVERLAP = 0.5  # boxes of one kind overlapping this much (intersection over union) are one object
PART_OF_BOX = 0.7       # a box of one kind lying this much inside a better one is a part of it
WIDE = 0.5              # metres: objects larger than this look different from every side
EMPTY_VIEWS = 3         # views from places this far apart (below) that saw a stored object's place empty
EMPTY_SPREAD = 0.3      # metres
# An object in front of a wall (a wall box, a ladder leaning on the wall) cannot be looked through:
# where it stood the wall is at its distance, so its place never looks empty. That it is gone shows
# as not being detected where it would be in clear view. A detector misses things, and how often
# depends on the side it looks from (a leaning ladder seen edge-on): a look counts only from a side
# the object was found from in most clear views before. (Simulated yard: present leaning ladders
# went unfound for 9 looks from 3 places; the detector found one ladder in 1 of 57 clear views.)
RELIABLE = 0.4          # share of its earlier clear views from a side an object was detected in ...
MIN_VIEWS = 5           # ... over at least this many looks from that side
# Frames from one spot are one look, whatever the frame rate: a view counts as a new one only after
# the robot moved or turned this much.
LOOK_STEP = (0.2, math.radians(10))  # metres, radians
# Looks in this patrol with its place in clear view and no detection: as many as make missing a
# present object every time unlikely (below CHANCE, by each side's rate, were looks independent) ...
CHANCE = 0.02
MIN_LOOKS = 4           # ... and at least this many
MISSED_PLACES = 2       # ... from places this far apart
MISSED_SPREAD = 0.5     # metres
# The depth camera's error grows with the square of the range. With the D457's noise (as fitted
# to the real camera, simulated yard) sightings up to 12.5 m land inside the object's footprint;
# from 15 m on 0.25-0.9 m off (90th percentile), more than footprints are matched within. Such
# sightings split objects, and their frames are useless for inspection: they are not used.
MAX_RANGE = 12.0        # metres
ANNOUNCE = 3            # sightings before an object is announced: single boxes are often false hits or parts
# The vision-model sweep (`sweep.py`) adds an object only where the fast detector has none: a sweep
# box overlapping one of the fast detector's of its kind in the same frame is that object; one of
# another kind covering much the same box is a dispute about the kind, which verification settles.
# (Another kind's box merely inside it is not: on the hall recording the hose cabinet's box lies
# inside the leaning ladder's.) A new object needs this many sweep frames placing it.
SWEEP_OVERLAP = 0.3
SWEEP_DISPUTE = 0.5     # intersection over union
# Nor is a sweep object lying on the footprint of a mapped object of another kind new: it is that
# object under another name (hall recording: an "extinguisher" in the hose cabinet).
SWEEP_ON_OTHER = 0.5    # share of its floor cells within a cell of the other object's footprint
SWEEP_AGREE = 2


def new_look(last, pose):
    """Has the robot moved or turned enough since `last` (x, y, yaw) for `pose` to be another look?"""
    return (math.dist(last[:2], pose[:2]) >= LOOK_STEP[0]
            or abs(math.remainder(pose[2] - last[2], math.tau)) >= LOOK_STEP[1])


def reliable_sides(site_object):
    """{side: share of its clear views the detector found it in} for the sides it was found from
    reliably: from those, not finding it in a later patrol counts toward its being gone."""
    return {int(k): d / v for k, (v, d) in site_object.sides.items() if v >= MIN_VIEWS and d >= RELIABLE * v}


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


def iou(a, b):
    width = min(a[2], b[2]) - max(a[0], b[0])
    height = min(a[3], b[3]) - max(a[1], b[1])
    if width <= 0 or height <= 0:
        return 0.0
    shared = width * height
    return shared / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - shared)


def plausible(sighting):
    """Can the detector be right? Not if the object is too far away to place, or the wrong size
    for its kind (a look-alike). A box cut off by the image edge may show any part of it."""
    if sighting.distance and not sighting.approximate and sighting.distance > MAX_RANGE:
        return False
    box, (low, high) = sighting.detection.box, KINDS[sighting.detection.kind].size
    cut_off = box[1] <= EDGE or box[3] >= 1000 - EDGE
    return sighting.height is None or (cut_off or low <= sighting.height) and sighting.height <= high


def same_place(a, b):
    """Do two sightings of one kind place the object at the same spot?"""
    if a.cells and b.cells:
        return touching(a.cells, np.asarray(b.cells, dtype=int).reshape(-1, 2)) >= MIN_SHARE
    return math.dist(a.position, b.position) <= KINDS[a.detection.kind].same_object_radius


def on(sighting, site_object):
    """Does the sighting lie on the mapped object (its footprint; its place, without one)?"""
    if sighting.cells and site_object.cells:
        return touching(sighting.cells, footprint(site_object), 1) >= SWEEP_ON_OTHER
    return bool(site_object.position) and (
        math.dist(sighting.position, site_object.position) <= KINDS[site_object.kind].same_object_radius)


def distinct(detections):
    """Best-scoring detections first, dropping repeats of the same thing."""
    kept = []
    for detection in sorted(detections, key=lambda d: d.score, reverse=True):
        if not any(k.kind == detection.kind and overlap(k.box, detection.box) > SAME_BOX_OVERLAP for k in kept):
            kept.append(detection)
    return kept


class Survey:
    def __init__(self, detector, objects, keyframes, store, calibration=None, interval=1.0, min_sharpness=0.0,
                 verifier=None, map_id=None, sweep=None):
        self.detector, self.objects, self.keyframes, self.store = detector, objects, keyframes, store
        self.verifier = verifier            # optional second opinion from the vision model, one call per object
        self.sweep = sweep                  # optional vision-model sweep for objects the detector misses
        self.swept = set()                  # objects the sweep added
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
        # Sides each stored object was found reliably from: not finding it there says something.
        self.reliable = {o.id: reliable_sides(o) for o in stored if reliable_sides(o)}  # id -> {side: rate}
        self._looks = {}  # object id -> [pose its current look began at, side, found in it]
        self._missed = {}  # stored object id -> [looks without a detection, log chance of that, places, last pose]
        self.created = set()  # objects this patrol created: provisional until it is lined up
        # Objects announced (`object_found`) in this workspace: resuming does not announce them twice.
        self.announced = {e["object"] for e in store.events(limit=None) if e["kind"] == "object_found"}
        self.seen_now = {}  # object id -> sightings in this run: a patrol reports what it saw itself
        self.inside = None  # inside(x, y): the room's fence (gate.Fence.contains); sightings outside are dropped
        self.outside = 0    # sightings dropped for lying outside it
        self.doubted = {}   # reason -> detections dropped as seen on or through glass

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
        raw = frame  # pose in this run's frame: the sweep places it with the final line-up
        if self.anchoring and self.anchoring.transform and frame.pose:
            frame = copy.copy(frame)
            frame.pose = self.anchoring.to_map(frame.pose)  # this run's drift against the stored objects
        try:
            detections = distinct(self.detector.detect(frame))
        except Exception as error:  # a failing model must not end the patrol
            self.store.emit("error", stage="detect", message=str(error))
            return []
        if self.sweep and not (self.anchoring and self.anchoring.state == Anchoring.SUSPECT):
            self.sweep.offer(raw, detections)
        seen = []
        # Detections the tracker already knows go first, so a newcomer cannot take their object.
        for detection in sorted(detections, key=lambda d: d.track not in self._tracks):
            sighting = lift(frame, detection, self.calibration)
            if getattr(sighting, "doubt", None):  # on or through glass (lift.doubt); YOLO drops these itself
                self.doubted[sighting.doubt] = self.doubted.get(sighting.doubt, 0) + 1
                if self.doubted[sighting.doubt] in (1, 10, 100, 1000):
                    self.store.emit("detection_doubted", reason=sighting.doubt, object_kind=detection.kind,
                                    count=self.doubted[sighting.doubt])
                continue
            if not plausible(sighting):  # too far to place, or the wrong size for its kind
                continue
            if self.inside and sighting.position is not None and not sighting.approximate \
                    and not self.inside(*sighting.position):
                self.outside += 1
                continue  # beyond the room's fence: a reflection, or something seen through the glass
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
        """Count the objects whose place this frame has in clear view, and judge stored ones not
        detected in this patrol: is their place empty, or are they reliably found yet missed?

        Only a patrol lined up with the stored objects can say a place is empty; one view is not
        enough, nor are views from one spot. An object detected in this patrol is not gone."""
        if not (frame.pose and self.calibration) or self.anchoring and self.anchoring.state != Anchoring.ANCHORED:
            return
        here = {o.id for o in seen}
        in_view = {}
        for site_object in self.objects.all():
            if site_object.status not in ("FOUND", "INSPECTED"):
                continue
            verdict = look_through(frame, self.calibration, site_object)
            if verdict is None:
                continue  # out of view, too near or far, or hidden: says nothing
            side = Sighting(None, position=site_object.position).side(frame.pose)
            # Counted per look, not per frame: a look found it if any of its frames did.
            hit, look = site_object.id in here, self._looks.get(site_object.id)
            if look is None or new_look(look[0], frame.pose):
                self._looks[site_object.id] = [tuple(frame.pose), side, hit]
                in_view[site_object.id] = (side, 1, int(hit))
            elif hit and not look[2]:
                look[2] = True
                in_view[site_object.id] = (look[1], 0, 1)
            if (not self.anchoring or site_object.id not in self.anchoring.stored or site_object.id in here
                    or self.seen_now.get(site_object.id)):
                continue
            if verdict == "occupied":
                self._empty.pop(site_object.id, None)
            else:
                places = self._empty.setdefault(site_object.id, [])
                if all(math.dist(p, frame.pose[:2]) >= EMPTY_SPREAD for p in places):
                    places.append(tuple(frame.pose[:2]))
                if len(places) >= EMPTY_VIEWS:
                    self._gone(site_object, "place seen empty", len(places))
                    continue
            rate = self.reliable.get(site_object.id, {}).get(side)
            if rate is not None:
                missed = self._missed.setdefault(site_object.id, [0, 0.0, [], None])
                if missed[3] is None or new_look(missed[3], frame.pose):
                    missed[0], missed[1], missed[3] = missed[0] + 1, missed[1] + math.log(1 - min(rate, 0.95)), tuple(frame.pose)
                if all(math.dist(p, frame.pose[:2]) >= MISSED_SPREAD for p in missed[2]):
                    missed[2].append(tuple(frame.pose[:2]))
        self.objects.count_views(in_view)

    def judge_missed(self):
        """Stored objects found reliably before, in clear view often enough in this patrol and never
        detected: gone. Judged when the survey settles, after fragments were merged: on the hall
        recording at 16 frames a second, the ladder's sightings went for 14 looks to a fragment that
        only then joined it."""
        if not (self.anchoring and self.anchoring.state == Anchoring.ANCHORED):
            return
        for object_id, (looks, chance, places, _) in self._missed.items():
            site_object = self.objects.get(object_id)
            if (site_object and site_object.status in ("FOUND", "INSPECTED") and not self.seen_now.get(object_id)
                    and looks >= MIN_LOOKS and chance <= math.log(CHANCE) and len(places) >= MISSED_PLACES):
                self._gone(site_object, "not detected in clear view", looks)

    def _gone(self, site_object, reason, views):
        self.objects.update(site_object.id, status="MOVED")
        self.store.emit("object_moved", object=site_object.id, object_kind=site_object.kind,
                        position=site_object.position, reason=reason, views=views)

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
            self.apply_sweep()  # ends it without placing anything
            self.announce(min_sightings, settled=True)
            return {}  # positions of this patrol are not to be trusted: merge and relabel nothing
        merged = self.objects.consolidate(min_sightings, keep=self.announced)
        for removed, target in merged.items():
            self.keyframes.reassign(removed, target)
            self._replace(removed, target)
            self.store.emit("object_merged", object=removed, into=target, announced=removed in self.announced)
            self.announced.discard(removed)
        self.apply_sweep()
        self.judge_missed()
        self.announce(min_sightings, settled=True)
        self.verify(min_sightings)
        self.announce(min_sightings, settled=True)  # a confirmed object counts as found whatever its sightings
        return merged

    def apply_sweep(self):
        """Take in the vision-model sweep's answers (waiting for calls still running). Ends the sweep.

        Its boxes are placed like the detector's. One on an object the map has makes that object seen
        in this patrol (so a stored object the detector missed is not taken for gone); one where the
        fast detector boxed something in the same frame is skipped. A new object needs SWEEP_AGREE
        sweep frames placing it at one spot; it is announced once verification confirms it."""
        if self.sweep is None:
            return
        sweep, self.sweep = self.sweep, None
        if self.anchoring and self.anchoring.state == Anchoring.SUSPECT:
            sweep.cancel()
            self.store.emit("vision_sweep", skipped="localisation suspect")
            return
        answered, failed = sweep.results()
        confirmed, new = set(), []  # map objects the sweep saw; (frame, sighting) of objects not on the map
        for frame, fast, found in answered:
            if self.anchoring and self.anchoring.transform:
                frame = copy.copy(frame)
                frame.pose = self.anchoring.to_map(frame.pose)
            for detection in distinct(found):
                if any(overlap(detection.box, d.box) > SWEEP_OVERLAP if d.kind == detection.kind
                       else iou(detection.box, d.box) > SWEEP_DISPUTE for d in fast):
                    continue
                sighting = lift(frame, rectangle(detection), self.calibration)
                if sighting.position is None or sighting.approximate or not plausible(sighting):
                    continue
                if getattr(sighting, "doubt", None) or self.inside and not self.inside(*sighting.position):
                    continue  # on or through glass, or beyond the fence: the model saw a reflection
                match = self.objects.find(sighting)
                if match is AMBIGUOUS:
                    continue
                if match is not None:
                    if match.status in ("FOUND", "INSPECTED"):
                        confirmed.add(match.id)
                    continue
                if any(o.kind != detection.kind and o.status in ("FOUND", "INSPECTED") and on(sighting, o)
                       for o in self.objects.all()):
                    continue
                if (self.anchoring and self.anchoring.state != Anchoring.ANCHORED
                        and self.anchoring.near(detection.kind, sighting.position)):
                    continue  # may be a stored object, displaced by poses not yet lined up
                new.append((frame, sighting))
        for object_id in confirmed:
            self.seen_now[object_id] = self.seen_now.get(object_id, 0) + 1
        groups = []  # one list of (frame, sighting) per object, at most one sighting per frame
        for frame, sighting in new:
            group = next((g for g in groups if g[0][1].detection.kind == sighting.detection.kind
                          and all(f is not frame for f, _ in g) and any(same_place(sighting, s) for _, s in g)), None)
            if group is None:
                groups.append([(frame, sighting)])
            else:
                group.append((frame, sighting))
        added = []
        # Better supported first: of two kinds the model gave one thing in different frames, the
        # more frequent wins and the other lies on it.
        for group in sorted(groups, key=len, reverse=True):
            if len(group) < SWEEP_AGREE or any(
                    o.kind != group[0][1].detection.kind and on(group[0][1], o) for o in map(self.objects.get, added)):
                continue
            site_object = None
            for frame, sighting in group:
                found = self.objects.observe(sighting, frame.stamp, known=site_object and site_object.id,
                                             create=site_object is None)
                if found is None or site_object and found.id != site_object.id:
                    continue
                site_object = found
                self.keyframes.offer(frame, site_object, sighting, frame.sharpness(), any_range=True)
                self.seen_now[site_object.id] = self.seen_now.get(site_object.id, 0) + 1
            if site_object and site_object.id not in added:  # two groups may join one object
                self.swept.add(site_object.id)
                if self.anchoring:
                    self.created.add(site_object.id)
                added.append(site_object.id)
        # Seen in one sweep frame only: not added, but worth a look by whoever watches the patrol.
        single = [{"kind": g[0][1].detection.kind, "position": [round(v, 2) for v in g[0][1].position]}
                  for g in groups if len(g) < SWEEP_AGREE]
        self.store.emit("vision_sweep", frames=len(answered), failed=failed, confirmed=sorted(confirmed),
                        added=added, unconfirmed=single)

    def verify(self, min_sightings=3):
        """Ask the vision model about every well-seen object it has not judged yet, and about those
        the sweep added."""
        if self.verifier is None:
            return
        swept = [self.objects.get(i) for i in sorted(self.swept) if self.objects.get(i)]
        swept = [o for o in swept if o.status == "FOUND" and o.sightings < min_sightings]
        for site_object in self.objects.pending(min_sightings) + swept:
            views = self.verify_views(site_object)
            if site_object.verified or not views:
                continue
            opinions = [self.verifier.classify(*view) for view in views]
            self.judge(site_object, [o for o in opinions if o is not None], [view[3] for view in views])

    def verify_views(self, site_object):
        """The keyframes the vision model is shown, loaded into memory (a better frame may replace
        the file meanwhile): [(image, box, keyframe id, measured height or None)]."""
        views = []
        flat = lying(site_object)  # a ladder on the floor is 0.1 m "tall": that would tell the model it is none
        for view in self.keyframes.select(site_object, budget=VERIFY_VIEWS):
            with Image.open(self.keyframes.root / view.file) as image:
                # Boxes cut off by the image edge show only a part: their height says nothing.
                whole = EDGE < view.box[1] and view.box[3] < 1000 - EDGE
                height = None
                if self.calibration and view.distance and whole and not flat:
                    height = (view.box[3] - view.box[1]) / 1000 * image.height / self.calibration.fy * view.distance
                views.append((image.convert("RGB"), view.box, view.id, height))
        return views

    def judge(self, site_object, opinions, heights):
        """Apply the vision model's opinions [(kind, remark)] of one object. Returns the object's id
        afterwards (a new one when relabelled), or None when it was rejected."""
        heights = [height for height in heights if height is not None]
        if not opinions:
            return site_object.id  # model unreachable: the object stays unverified and is tried again next time
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
            return site_object.id
        if kind == site_object.kind:
            self.objects.update(site_object.id, verified=True)
            self.store.emit("object_verified", object=site_object.id, remark=remark)
            return site_object.id
        if kind == NONE:
            self.objects.update(site_object.id, status="REJECTED", result={"reason": remark})
            self.keyframes.drop(site_object.id)
            self.store.emit("object_rejected", object=site_object.id, was=site_object.kind, remark=remark)
            return None
        new_id = self.objects.relabel(site_object.id, kind)
        self.keyframes.reassign(site_object.id, new_id)
        self._replace(site_object.id, new_id)
        if site_object.id in self.announced:  # the dashboard learns the new id from this event
            self.announced.add(new_id)
        self.store.emit("object_relabelled", object=site_object.id, was=site_object.kind,
                        now=kind, into=new_id, remark=remark)
        return new_id

    def run(self, source):
        try:
            for frame in source:
                self.feed(frame)
        finally:
            self.tidy()
