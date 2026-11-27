"""From a camera stream to finished protocols.

Every `interval` seconds one frame gets a quick look. While a ladder is in view its
frames are collected; when it leaves the view, the sharpest overview, detail and label
frames go to the full reading and one inspection column is written.
"""
import threading
import uuid

from .assess import assess, clean_box
from .lidar import locate
from .vlm import VlmError

VIEW_ORDER = ("overview", "detail", "label")


class Patrol:
    def __init__(self, vlm, store, *, interval=2.0, min_sharpness=0.0, gap=2, per_view=2, max_views=12,
                 calibration=None, mission=None):
        self.vlm, self.store = vlm, store
        self.interval = interval            # seconds between quick looks
        self.min_sharpness = min_sharpness  # frames blurrier than this are skipped
        self.gap = gap                      # quick looks without a ladder that end a sighting
        self.per_view = per_view            # frames kept per view kind for the full reading
        self.max_views = max_views          # a sighting is closed after this many ladder frames
        self.calibration = calibration
        self.mission = mission or "PATROL-" + uuid.uuid4().hex[:6]
        self._views, self._misses, self._last_look, self._id_text = [], 0, None, None
        self._workers = []

    def feed(self, frame):
        """Offer a frame from the stream. Returns the quick-look result, or None if skipped."""
        if self._last_look is not None and 0 <= frame.stamp - self._last_look < self.interval:
            return None
        sharpness = frame.sharpness()
        if sharpness < self.min_sharpness:
            return None
        self._last_look = frame.stamp
        try:
            look = self.vlm.triage(frame)
        except VlmError as error:
            self.store.emit("error", stage="triage", message=str(error))
            return None
        look = dict(look, box=clean_box(look.get("box")), sharpness=round(sharpness, 1))
        self.store.save_live(frame)
        seen = bool(look.get("ladder_present")) and look.get("view") in VIEW_ORDER
        self.store.emit("look", frame=frame.name, ladder=seen, view=look.get("view"), box=look["box"],
                        note=look.get("note"), pose=frame.pose)
        if not seen:
            self._misses += 1
            if self._views and self._misses >= self.gap:
                self.finish()
            return look
        marking = look.get("id_text")
        if self._views and marking and self._id_text and marking != self._id_text:
            self.finish()  # a different marking: the next ladder is already in view
        self._id_text = marking or self._id_text
        self._misses = 0
        self._views.append((frame, look))
        if len(self._views) >= self.max_views:
            self.finish()
        return look

    def finish(self):
        """Close the current sighting and read the ladder in the background."""
        views, self._views, self._misses, self._id_text = self._views, [], 0, None
        if not views:
            return
        worker = threading.Thread(target=self._inspect, args=(views,), daemon=True)
        self._workers.append(worker)
        worker.start()

    def wait(self):
        """Block until every started reading has been written."""
        for worker in self._workers:
            worker.join()
        self._workers.clear()

    def _select(self, views):
        chosen = []
        for kind in VIEW_ORDER:
            of_kind = [v for v in views if v[1]["view"] == kind]
            chosen += sorted(of_kind, key=lambda v: v[1]["sharpness"], reverse=True)[:self.per_view]
        return chosen

    def _position(self, views):
        """Lidar range to the ladder where available, else where the robot stood."""
        located = [v for v in views if v[0].pose]
        if not located:
            return None
        frame, look = max(located, key=lambda v: v[1]["view"] == "overview")
        if self.calibration and frame.points is not None and look["box"]:
            position = locate(frame.points, look["box"], self.calibration, frame.pose)
            if position:
                return position
        return frame.pose[:2]

    def _inspect(self, views):
        chosen = self._select(views)
        frames = [frame for frame, _ in chosen]
        self.store.emit("inspecting", frames=[frame.name for frame in frames])
        try:
            answer = self.vlm.inspect(frames)
            inspection = assess(answer, len(frames))
            self.store.record(inspection, frames, position=self._position(views), mission=self.mission,
                              model=getattr(self.vlm, "model", None))
        except (VlmError, OSError, ValueError) as error:
            self.store.emit("error", stage="inspection", message=str(error),
                            frames=[frame.name for frame in frames])

    def run(self, source):
        """Consume a frame source until it ends, then finish any open sighting."""
        try:
            for frame in source:
                self.feed(frame)
        finally:
            self.finish()
            self.wait()
