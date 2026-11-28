"""Online patrol: each object is checked and its form read while the robot walks on.

    frame ─▶ survey (detect, lift, object map, keyframes)        main thread, every frame
               │ object_found
               ▼
             COLLECTING ── views complete / left behind / time budget / operator "n" ──▶ flush
               │  the vision model checks the object early (relabel, reject)            │
               │                                                                         ▼
             focus: the nearest object still collecting gets the                   READING (worker thread)
             view requests; the others queue                                             │
                                                                                         ▼
             DONE ◀──────────────────────────────── sheet / checklist saved, dashboard told
               │ seen again later with views it lacked at the first read
               └──▶ COLLECTING (read once more, at most `max_reads` times)

Only the vision model calls run on the worker (`Verifier.classify`, `Inspection.ask`), on frames
loaded into memory when the job is made; every write to the object map, keyframes and ladder
store stays on the main thread. Which object comes next is decided by geometry and time, never
by a model's answer: model output carries no motion authority.

What the robot does meanwhile is up to whoever drives it. With `--navigator passive` (a person
teleoperates, or a recording plays) the focus and its next missing view are announced as events
(`focus`); with `dashboard` or `contract` the view goals are sent off-thread, so the survey keeps
reading frames while the robot walks.

Lidar discovery plugs in as `candidates(frame) -> [(x, y), ...]`: map positions of things worth a
look. A candidate no object explains is looked at (`look_candidate`) once nothing is collecting.
No source of candidates exists yet; see docs/ONLINE.md.
"""
from concurrent.futures import ThreadPoolExecutor, wait
from dataclasses import dataclass, field
import math
import queue
import sys
import threading

from .coverage import missing_views
from .types import ViewRequest
from ..protocol.vlm import VlmError

COLLECTING, CHECKING, READING, DONE = "collecting", "checking", "reading", "done"
LOOK_RANGE = 2.5          # metres from a candidate a look goal stands
CANDIDATE_MERGE = 1.0     # metres: candidates closer than this are one
EXPLAINED = 1.5           # metres: an object this near a candidate explains it
LOOKED_AT = (1.0, 5.0)    # range at which a candidate in front of the camera counts as looked at ...
LOOK_CONE = math.radians(35)  # ... within this angle of the heading
DISMISS_LOOKS = 5         # frames that looked at a candidate and found nothing there: dismissed


@dataclass
class Track:
    object_id: str
    since: float                 # stamp the patrol started collecting it (again)
    state: str = COLLECTING
    checked: bool = False        # the vision model has given its opinion of the kind (or could not)
    reads: int = 0
    read_at: float = None        # stamp of the last flush
    missing_at_read: int = None  # missing views at the last flush
    failures: int = 0
    retry_after: float = 0.0
    reason: str = ""             # why it was flushed last


@dataclass
class Candidate:
    id: str
    position: tuple
    hits: int = 1
    looks: int = 0
    state: str = "open"          # open | explained | dismissed
    explained_by: str = None


class OnlinePatrol:
    def __init__(self, survey, inspection, store, navigator=None, is_free=None, candidates=None,
                 leave_seconds=6.0, leave_distance=3.5, budget_seconds=120.0, max_reads=2, max_failures=2,
                 commands=None, log=print):
        self.survey, self.inspection, self.store = survey, inspection, store
        self.objects, self.keyframes = survey.objects, survey.keyframes
        self.navigator = navigator if navigator is not None and getattr(navigator, "moves", False) else None
        self.is_free = is_free
        self.find_candidates = candidates
        self.leave_seconds = leave_seconds      # not seen for this long ...
        self.leave_distance = leave_distance    # ... and at least this far away: the robot has left it
        self.budget_seconds = budget_seconds    # longest an object is collected before it is read anyway
        self.max_reads, self.max_failures = max_reads, max_failures
        self.commands = commands                # queue of operator lines ("n" = next, "q" = stop), or None
        self.log = log
        self.mission = inspection.mission
        self.tracks = {}
        self.candidates = []
        self.jobs = []                          # [(future, kind, object id, payload)]
        self.worker = ThreadPoolExecutor(1, thread_name_prefix="ultron-vlm")
        self.driver = ThreadPoolExecutor(1, thread_name_prefix="ultron-nav") if self.navigator else None
        self.goal = None                        # (future, ViewRequest) while the navigator is under way
        self.tried = {}                         # object id -> {(view, side)} goals that failed
        self.focus = None
        self.announced_focus = None
        self.stamp, self.pose = None, None
        self.stopped = False
        self._next = False                      # the operator asked to move on from the focus
        self._force = False                     # feed the next frame whatever the interval: a goal was reached
        self.verify = survey.verifier is not None

    # --- the loop ----------------------------------------------------------------------------
    def run(self, frames):
        try:
            for frame in frames:
                self.step(frame)
                if self.stopped:
                    break
        except KeyboardInterrupt:
            self.log("Online patrol stopped by Ctrl-C: reading what is left. Ctrl-C again aborts.")
        self.finish()

    def step(self, frame):
        seen = self.survey.feed(frame, force=self._force)
        self._force = False
        self.stamp = frame.stamp
        self.pose = frame.pose or self.pose
        self._operator()
        self._apply_results()
        self._take_new()
        for track in list(self.tracks.values()):
            self._advance(track)
        if self.find_candidates:
            self._candidates(frame)
        self._steer()
        return seen

    def finish(self):
        """Wait for the navigator and the reader, then let the ordinary inspection settle the survey
        (merge fragments, judge missing objects, announce the rest) and inspect what is left: objects
        beside one of their kind are announced only then, and none should fare worse than offline."""
        if self.goal:
            self.goal[0].result()
            self.goal = None
        while self.jobs:
            wait([job[0] for job in self.jobs])
            self._apply_results()  # a read applied here may reopen nothing: the stream is over
        self.worker.shutdown()
        if self.driver:
            self.driver.shutdown()
        self.store.emit("stage", name="inspection", objects=len(self.objects.all()),
                        read_online=sorted(t.object_id for t in self.tracks.values() if t.reads))
        self.inspection.run()

    # --- objects -----------------------------------------------------------------------------
    def _take_new(self):
        for object_id in sorted(self.survey.announced):
            site_object = self.objects.get(object_id)
            if object_id in self.tracks or site_object is None:
                continue
            if site_object.status in ("REJECTED", "MOVED") or site_object.result.get("mission") == self.mission:
                continue
            self.tracks[object_id] = Track(object_id, self.stamp, checked=not self.verify or site_object.verified)

    def _advance(self, track):
        site_object = self.objects.get(track.object_id)
        if site_object is None or site_object.status in ("REJECTED", "MOVED"):
            if track.state in (COLLECTING, DONE):
                track.state = DONE
                track.reads = self.max_reads  # nothing more to do with it
            return
        if track.state == DONE:
            self._maybe_reopen(track, site_object)
            return
        if track.state != COLLECTING or self.stamp < track.retry_after:
            return
        if not track.checked:
            views = self.survey.verify_views(site_object)
            if views:
                track.state = CHECKING
                self._submit("check", track.object_id, [view[3] for view in views],
                             lambda: [self.survey.verifier.classify(*view) for view in views])
            return
        reason = self._ready(track, site_object)
        if reason:
            self._flush(track, site_object, reason)

    def _ready(self, track, site_object):
        """Why the robot is done with this object, or None while it should keep collecting."""
        frames = self.keyframes.for_object(site_object.id)
        if not frames:
            return None
        if not missing_views(site_object, frames):
            return "views complete"
        if self._next and self.focus == track.object_id:
            self._next = False
            return "operator"
        unseen = self.stamp - site_object.last_seen >= self.leave_seconds
        far = self._distance(site_object.position)
        if unseen and (far is None or far >= self.leave_distance):
            return "left behind"
        if self.stamp - track.since >= self.budget_seconds:
            return "time budget"
        return None

    def _flush(self, track, site_object, reason):
        job = self.inspection.prepare(site_object)
        if job is None:
            return
        missing = missing_views(site_object, self.keyframes.for_object(site_object.id))
        track.state, track.reason = READING, reason
        track.read_at, track.missing_at_read = self.stamp, len(missing)
        self.store.emit("object_ready", object=site_object.id, object_kind=site_object.kind, reason=reason,
                        frames=len(job["frames"]), missing=[r.reason for r in missing], read=track.reads + 1)
        self._submit("read", site_object.id, (job, [r.reason for r in missing]), lambda: self.inspection.ask(job))

    def _maybe_reopen(self, track, site_object):
        """Back with an object already read, with views it lacked then: read it again."""
        if track.reads >= self.max_reads or track.read_at is None or site_object.last_seen <= track.read_at:
            return
        missing = len(missing_views(site_object, self.keyframes.for_object(site_object.id)))
        if missing < track.missing_at_read:
            track.state, track.since = COLLECTING, self.stamp
            self.store.emit("object_reopened", object=site_object.id, missing=missing, was=track.missing_at_read)

    # --- the worker --------------------------------------------------------------------------
    def _submit(self, kind, object_id, payload, call):
        self.jobs.append((self.worker.submit(call), kind, object_id, payload))

    def _apply_results(self):
        waiting = []
        for job in self.jobs:
            (waiting.append(job) if not job[0].done() else self._apply(*job))
        self.jobs = waiting

    def _apply(self, future, kind, object_id, payload):
        track = self.tracks.get(object_id)
        site_object = self.objects.get(object_id)
        if track is None:
            return
        if site_object is None:  # merged away meanwhile
            track.state = DONE
            return
        try:
            answer = future.result()
        except (VlmError, OSError, ValueError) as error:
            track.failures += 1
            self.store.emit("error", stage="online " + kind, object=object_id, message=str(error))
            if kind == "check":
                track.checked = True  # unreachable model: go on without its opinion, as the survey does
            track.state = COLLECTING if track.failures < self.max_failures else DONE
            track.retry_after = self.stamp + 10.0
            return
        if kind == "check":
            track.checked = True
            track.state = COLLECTING
            new_id = self.survey.judge(site_object, [o for o in answer if o is not None], payload)
            if new_id is None:
                track.state = DONE
            elif new_id != object_id:
                del self.tracks[object_id]
                track.object_id = new_id
                self.tracks[new_id] = track
                self.focus = new_id if self.focus == object_id else self.focus
            return
        job, missing = payload
        self.inspection.finish(site_object, job, answer, missing)
        track.state, track.reads = DONE, track.reads + 1
        if self.focus == object_id:
            self.focus = None

    # --- where to go -------------------------------------------------------------------------
    def _distance(self, position):
        if position is None or self.pose is None:
            return None
        return math.hypot(position[0] - self.pose[0], position[1] - self.pose[1])

    def _steer(self):
        collecting = [t for t in self.tracks.values() if t.state in (COLLECTING, CHECKING)]
        if collecting:
            def order(track):
                distance = self._distance(self.objects.get(track.object_id).position)
                return (distance is None, distance or 0.0, track.since)
            self.focus = min(collecting, key=order).object_id
        else:
            self.focus = None
        request = self._next_view() if self.focus else self._look_goal()
        key = (self.focus, request and (request.view, request.side, request.object_id))
        if key != self.announced_focus:
            self.announced_focus = key
            if self.focus or request:
                self.store.emit("focus", object=self.focus, queue=[t.object_id for t in collecting if t.object_id != self.focus],
                                reading=[t.object_id for t in self.tracks.values() if t.state == READING],
                                goal=[request.x, request.y, request.yaw] if request else None,
                                view=request.view if request else None, reason=request.reason if request else "")
        self._drive(request)

    def _next_view(self):
        site_object = self.objects.get(self.focus)
        requests = missing_views(site_object, self.keyframes.for_object(self.focus), self.is_free,
                                 self.tried.get(self.focus, set()))
        return requests[0] if requests else None

    def _drive(self, request):
        if not self.navigator:
            return
        if self.goal and self.goal[0].done():
            future, sent = self.goal
            self.goal = None
            if future.exception() is None and future.result():
                self._force = True  # standing at the view: take the next frame whatever the interval
            else:
                self.tried.setdefault(sent.object_id, set()).add((sent.view, sent.side))
                self.store.emit("view_failed", object=sent.object_id, view=sent.view, side=sent.side)
        if self.goal or request is None:
            return
        self.store.emit("view_requested", object=request.object_id, view=request.view, side=request.side,
                        goal=[request.x, request.y, request.yaw], reason=request.reason)
        self.goal = (self.driver.submit(self.navigator.go_to, request), request)

    # --- operator ----------------------------------------------------------------------------
    def _operator(self):
        while self.commands is not None:
            try:
                line = self.commands.get_nowait().strip().lower()
            except queue.Empty:
                return
            if line in ("n", "next"):
                self._next = True
                self.log(f"operator: done with {self.focus or 'nothing'}")
            elif line in ("q", "quit", "stop"):
                self.stopped = True

    # --- lidar candidates --------------------------------------------------------------------
    def _candidates(self, frame):
        for x, y in self.find_candidates(frame) or ():
            near = min(self.candidates, key=lambda c: math.dist(c.position, (x, y)), default=None)
            if near and math.dist(near.position, (x, y)) < CANDIDATE_MERGE:
                near.hits += 1
                continue
            self.candidates.append(Candidate(f"CAND-{len(self.candidates) + 1:03}", (x, y)))
            self.store.emit("candidate", candidate=self.candidates[-1].id, position=[round(x, 2), round(y, 2)])
        live = [o for o in self.objects.all() if o.position and o.status not in ("REJECTED", "MOVED")]
        for candidate in self.candidates:
            if candidate.state != "open":
                continue
            near = min(live, key=lambda o: math.dist(o.position, candidate.position), default=None)
            if near and math.dist(near.position, candidate.position) <= EXPLAINED:
                candidate.state, candidate.explained_by = "explained", near.id
                self.store.emit("candidate_explained", candidate=candidate.id, object=near.id)
            elif self._in_view(candidate.position):
                candidate.looks += 1
                if candidate.looks >= DISMISS_LOOKS:
                    candidate.state = "dismissed"
                    self.store.emit("candidate_dismissed", candidate=candidate.id, looks=candidate.looks)

    def _in_view(self, position):
        distance = self._distance(position)
        if distance is None or not LOOKED_AT[0] <= distance <= LOOKED_AT[1]:
            return False
        bearing = math.atan2(position[1] - self.pose[1], position[0] - self.pose[0])
        return abs((bearing - self.pose[2] + math.pi) % (2 * math.pi) - math.pi) <= LOOK_CONE

    def _look_goal(self):
        """Stand LOOK_RANGE before the nearest open candidate, on the robot's side, facing it."""
        open_ = [c for c in self.candidates if c.state == "open"]
        if not open_ or self.pose is None:
            return None
        target = min(open_, key=lambda c: self._distance(c.position))
        bearing = math.atan2(self.pose[1] - target.position[1], self.pose[0] - target.position[0])
        x = target.position[0] + LOOK_RANGE * math.cos(bearing)
        y = target.position[1] + LOOK_RANGE * math.sin(bearing)
        if self.is_free and not self.is_free(x, y):
            return None
        return ViewRequest(target.id, "look", 0, round(x, 2), round(y, 2), round(bearing + math.pi, 3),
                           f"lidar candidate {target.id}: nothing detected there yet")


def stdin_commands(log=print):
    """Operator lines from the terminal on a background thread: "n" (done with the focus), "q" (stop)."""
    if not sys.stdin or not sys.stdin.isatty():
        return None
    lines = queue.Queue()

    def read():
        for line in sys.stdin:
            lines.put(line)
    threading.Thread(target=read, daemon=True).start()
    log("Online patrol: type n + Enter to finish the current object, q + Enter to stop.")
    return lines
