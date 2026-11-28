"""Stage 1, survey: turn the camera stream into an object map and keyframes.

Feed it frames at any rate. It keeps no state of its own beyond the rate limit: objects
and keyframes are on disk after every frame, so pausing is simply not feeding it.
"""
from .catalog import KINDS
from .lift import lift


SAME_BOX_OVERLAP = 0.5  # boxes of one kind overlapping this much (intersection over union) are one object
PART_OF_BOX = 0.7       # a box of one kind lying this much inside a better one is a part of it


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
    def __init__(self, detector, objects, keyframes, store, calibration=None, interval=1.0, min_sharpness=0.0):
        self.detector, self.objects, self.keyframes, self.store = detector, objects, keyframes, store
        self.calibration = calibration
        self.interval = interval            # seconds between processed frames
        self.min_sharpness = min_sharpness  # frames blurrier than this are skipped
        self._last = None
        self._tracks = {}  # detector track id -> object id, for this run of the detector

    def feed(self, frame, force=False):
        """Process one frame. Returns the objects seen in it (empty if skipped or nothing found)."""
        if not force and self._last is not None and 0 <= frame.stamp - self._last < self.interval:
            return []
        sharpness = frame.sharpness()
        if not force and sharpness < self.min_sharpness:
            return []
        self._last = frame.stamp
        try:
            detections = distinct(self.detector.detect(frame))
        except Exception as error:  # a failing model must not end the patrol
            self.store.emit("error", stage="detect", message=str(error))
            return []
        seen = []
        # Detections the tracker already knows go first, so a newcomer cannot take their object.
        for detection in sorted(detections, key=lambda d: d.track not in self._tracks):
            sighting = lift(frame, detection, self.calibration)
            low, high = KINDS[detection.kind].size
            if sighting.height is not None and not low <= sighting.height <= high:
                continue  # wrong size for its kind: a look-alike
            site_object = self.objects.observe(sighting, frame.stamp, exclude={o.id for o in seen},
                                               known=self._tracks.get(detection.track))
            if site_object is None:
                continue  # between two known objects and no way to tell which: wait for a clearer frame
            if detection.track is not None:
                self._tracks[detection.track] = site_object.id
            keyframe = self.keyframes.offer(frame, site_object, sighting, sharpness)
            if site_object.sightings == 1:
                self.store.emit("object_found", object=site_object.id, object_kind=site_object.kind,
                                position=site_object.position, approximate=sighting.approximate)
            seen.append(site_object)
        for kind in {o.kind for o in seen}:
            together = [o.id for o in seen if o.kind == kind]
            if len(together) > 1:
                self.objects.seen_together(together)
        self.store.save_live(frame)
        best = max(detections, key=lambda d: d.score, default=None)
        self.store.emit("look", frame=frame.name, ladder=any(d.kind == "ladder" for d in detections),
                        view=None, box=best.box if best else None, pose=frame.pose,
                        note=", ".join(sorted({o.id for o in seen})) or "nothing found")
        return seen

    def tidy(self, min_sightings=3):
        """Merge fragments into the objects they belong to. Call when a survey pass ends."""
        merged = self.objects.consolidate(min_sightings)
        for removed, target in merged.items():
            self.keyframes.drop(removed)
            self._tracks = {track: target if i == removed else i for track, i in self._tracks.items()}
            self.store.emit("object_merged", object=removed, into=target)
        return merged

    def run(self, source):
        try:
            for frame in source:
                self.feed(frame)
        finally:
            self.tidy()
