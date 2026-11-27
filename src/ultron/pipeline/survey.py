"""Stage 1, survey: turn the camera stream into an object map and keyframes.

Feed it frames at any rate. It keeps no state of its own beyond the rate limit: objects
and keyframes are on disk after every frame, so pausing is simply not feeding it.
"""
from .lift import lift


class Survey:
    def __init__(self, detector, objects, keyframes, store, calibration=None, interval=1.0, min_sharpness=0.0):
        self.detector, self.objects, self.keyframes, self.store = detector, objects, keyframes, store
        self.calibration = calibration
        self.interval = interval            # seconds between processed frames
        self.min_sharpness = min_sharpness  # frames blurrier than this are skipped
        self._last = None

    def feed(self, frame, force=False):
        """Process one frame. Returns the objects seen in it (empty if skipped or nothing found)."""
        if not force and self._last is not None and 0 <= frame.stamp - self._last < self.interval:
            return []
        sharpness = frame.sharpness()
        if not force and sharpness < self.min_sharpness:
            return []
        self._last = frame.stamp
        try:
            detections = self.detector.detect(frame)
        except Exception as error:  # a failing model must not end the patrol
            self.store.emit("error", stage="detect", message=str(error))
            return []
        seen = []
        for detection in detections:
            sighting = lift(frame, detection, self.calibration)
            site_object = self.objects.observe(sighting, frame.stamp)
            keyframe = self.keyframes.offer(frame, site_object, sighting, sharpness)
            if site_object.sightings == 1:
                self.store.emit("object_found", object=site_object.id, object_kind=site_object.kind,
                                position=site_object.position, approximate=sighting.approximate)
            seen.append(site_object)
        self.store.save_live(frame)
        best = max(detections, key=lambda d: d.score, default=None)
        self.store.emit("look", frame=frame.name, ladder=any(d.kind == "ladder" for d in detections),
                        view=None, box=best.box if best else None, pose=frame.pose,
                        note=", ".join(sorted({o.id for o in seen})) or "nothing found")
        return seen

    def run(self, source):
        for frame in source:
            self.feed(frame)
