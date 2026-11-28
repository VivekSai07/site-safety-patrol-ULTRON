"""Vision-model sweep: a second detector for what the fast one missed.

The fast detector (YOLOE) runs on every frame and finds most objects. Where it scores an object
low (a ladder seen edge-on, an object of a look it was not prompted for), the object never enters
the map, and nothing says so. The sweep keeps a few whole frames taken from distinct places and
directions, asks the vision model for every catalogued object in each (one call per frame, in the
background while the patrol goes on), and hands the answers to the survey when it settles.

The survey adds an object from the sweep only where the fast detector has nothing, and only when
two sweep frames agree on it (see `Survey.apply_sweep`).
"""
from concurrent.futures import ThreadPoolExecutor, wait
import math

from .types import Detection

STEP = 1.5                   # metres moved ...
TURN = math.radians(45)      # ... or radians turned since every kept frame: a new place to look from
MAX_FRAMES = 40              # frames kept per patrol: memory and vision-model calls are bounded
WORKERS = 2                  # vision-model calls in flight at once
WAIT = 180.0                 # seconds the survey waits at its end for calls still running


def rectangle(detection):
    """The detection with its box as outline: lets the box be placed like a segmented object.
    (Hall recording: a box placed this way lands on the object segmentation finds for 100% of the
    wall boxes and 88% of the ladders, median 1-2 cm from it.)"""
    x0, y0, x1, y1 = detection.box
    return Detection(detection.kind, detection.box, detection.score, None, [[x0, y0], [x1, y0], [x1, y1], [x0, y1]])


class Sweep:
    def __init__(self, detector, max_frames=MAX_FRAMES, step=STEP, turn=TURN, workers=WORKERS, wait=WAIT):
        self.detector = detector     # anything with detect(frame): the vision model (`VlmDetector`)
        self.max_frames, self.step, self.turn, self.wait = max_frames, step, turn, wait
        self.taken = []              # [(frame, fast detector's detections in it, future answer)]
        self._pool = ThreadPoolExecutor(workers, thread_name_prefix="sweep")

    def distinct(self, pose):
        return all(math.dist(pose[:2], f.pose[:2]) >= self.step
                   or abs(math.remainder(pose[2] - f.pose[2], math.tau)) >= self.turn for f, _, _ in self.taken)

    def offer(self, frame, detections):
        """Keep `frame` (as the survey placed it) if it looks from somewhere new. Returns True if kept."""
        if (len(self.taken) >= self.max_frames or frame.pose is None
                or frame.depth is None and frame.points is None or not self.distinct(frame.pose)):
            return False
        self.taken.append((frame, list(detections), self._pool.submit(self.detector.detect, frame)))
        return True

    def results(self):
        """[(frame, fast detections, vision-model detections)] for the calls that answered, and the
        number that failed or did not answer in time. Ends the sweep."""
        done, _ = wait([future for _, _, future in self.taken], timeout=self.wait)
        answered, failed = [], 0
        for frame, fast, future in self.taken:
            if future in done and future.exception() is None:
                answered.append((frame, fast, future.result()))
            else:
                failed += 1
        self._pool.shutdown(wait=False, cancel_futures=True)
        self.taken = []
        return answered, failed
