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
store stays on the main thread. When the robot is done with an object, and which one is next, is
decided by geometry and time. The model's check can remove an object from the queue (rejected) or
rename it (relabelled), as it does offline; its form answers never steer anything.

A second read of an object (it came back with views it lacked) never clears what an earlier read
found: the worse status stays and earlier results are kept in `result["earlier"]`.

What the robot does meanwhile is up to whoever drives it. With `--navigator passive` (a person
teleoperates, or a recording plays) the focus and its next missing view are announced as events
(`focus`); with `dashboard` or `contract` the view goals are sent off-thread, so the survey keeps
reading frames while the robot walks.

Lidar discovery plugs in as `candidates(frame) -> [(x, y), ...]`: map positions of things worth a
look. A candidate no object explains is looked at (`look_candidate`) once nothing is collecting.
`discover.LidarCandidates` is one (upright, ladder-sized blobs); only `sim:PATH,lidar=1` frames
carry the cloud it needs so far, see docs/ONLINE.md.
"""
from concurrent.futures import FIRST_COMPLETED, Future, ThreadPoolExecutor, wait
import itertools
from dataclasses import dataclass, field
import math
import queue
import sys
import threading
import time

from .coverage import missing_views
from .keyframes import view_name
from .navigate import BUSY, PREEMPTED, UNAVAILABLE
from .types import ViewRequest

COLLECTING, CHECKING, READING, DONE = "collecting", "checking", "reading", "done"
LOOK_RANGE = 2.5          # metres from a candidate a look goal stands
CANDIDATE_MERGE = 1.0     # metres: candidates closer than this are one
EXPLAINED = 1.5           # metres: an object this near a candidate explains it
LOOKED_AT = (1.0, 5.0)    # range at which a candidate in front of the camera counts as looked at ...
LOOK_CONE = math.radians(35)  # ... within this angle of the heading
DISMISS_LOOKS = 5         # looks (at least LOOK_GAP apart) at a candidate that found nothing: dismissed
LOOK_GAP = 1.0            # seconds
SEVERITY = {"aussondern": 4, "reparatur": 3, "MANGEL": 3, "OFFEN": 2, "io": 1, "IN_ORDNUNG": 1}
CANDIDATE_HITS = 2        # clouds a candidate must show up in before it counts (sim yard: 28 of 40 did)


class Worker:
    """One thread for the vision model; quick object checks go before long form reads, so a new
    object is confirmed while earlier ones are still being read (one model serves one call at a time)."""
    CHECK, READ = 0, 1

    def __init__(self):
        self._queue, self._order = queue.PriorityQueue(), itertools.count()
        self._thread = threading.Thread(target=self._serve, name="ultron-vlm", daemon=True)
        self._thread.start()

    def submit(self, call, priority=READ):
        future = Future()
        self._queue.put((priority, next(self._order), future, call))
        return future

    def pending(self, priority=READ):
        return sum(1 for item in list(self._queue.queue) if item[0] == priority)

    def _serve(self):
        while True:
            _, _, future, call = self._queue.get()
            if call is None:
                return
            if future.set_running_or_notify_cancel():
                try:
                    future.set_result(call())
                except BaseException as error:  # handed to the main thread with the result
                    future.set_exception(error)

    def shutdown(self):
        self._queue.put((99, next(self._order), Future(), None))
        self._thread.join()


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
    cells: int = 0               # (view, side) cells its keyframes fill ...
    progress_at: float = None    # ... and when that last grew


@dataclass
class Candidate:
    id: str
    position: tuple
    hits: int = 1
    looks: int = 0
    last_look: float = None
    state: str = "new"           # new (seen once) | open | explained | dismissed
    explained_by: str = None


class OnlinePatrol:
    def __init__(self, survey, inspection, store, navigator=None, is_free=None, candidates=None,
                 leave_seconds=6.0, leave_distance=3.5, budget_seconds=120.0, max_reads=2, max_failures=2,
                 stall_seconds=20.0, exploration=None, pause_radius=4.0, pause_cap=90.0, operator_grace=10.0,
                 obstacles=None, commands=None, log=print, clock=time.monotonic):
        self.survey, self.inspection, self.store = survey, inspection, store
        self.objects, self.keyframes = survey.objects, survey.keyframes
        self.navigator = navigator if navigator is not None and getattr(navigator, "moves", False) else None
        self.is_free = is_free
        self.find_candidates = candidates
        self.leave_seconds = leave_seconds      # not seen for this long ...
        self.leave_distance = leave_distance    # ... and at least this far away: the robot has left it
        self.budget_seconds = budget_seconds    # longest an object is collected before it is read anyway
        self.max_reads, self.max_failures = max_reads, max_failures
        # In a tight room the robot may never get far from an object: it is done once its views stop growing.
        self.stall_seconds = stall_seconds
        self.exploration = exploration if self.navigator else None  # navigate.Exploration, or None
        self.pause_radius = pause_radius        # pause the explorer for objects this near the robot ...
        self.pause_cap = pause_cap              # ... for at most this long (wall clock)
        self.operator_grace = operator_grace    # after an operator goal, wait this long before sending one
        self.publish_obstacles = obstacles      # callable(cells): known footprints to the navigation stack
        self.clock = clock
        self.commands = commands                # queue of operator lines ("n" = next, "q" = stop), or None
        self.log = log
        self.mission = inspection.mission
        self.tracks = {}
        self.candidates = []
        self.jobs = []                          # [(future, kind, object id, payload)]
        self.worker = Worker()
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
        self.hold_until = -math.inf             # (clock) no goal is sent before this
        self.held = None                        # why goals are held, as last announced
        self.pause = None                       # {"since", "renewed", "objects"} while the pipeline holds a pause
        self.paused_for = set()                 # objects that already had their pause this patrol
        self._pause_retry = -math.inf
        self._explore_state, self._explore_checked = None, -math.inf
        self._obstacles_sent, self._obstacles_at = None, -math.inf

    # --- the loop ----------------------------------------------------------------------------
    def run(self, frames):
        try:
            for frame in frames:
                self.step(frame)
                if self.stopped:
                    break
        except KeyboardInterrupt:
            self.log("Online patrol stopped by Ctrl-C: reading what is left. Ctrl-C again aborts.")
        finally:  # on any exit, an error included: hand exploration back, read what was found
            self.finish()

    def step(self, frame):
        seen = self.survey.feed(frame, force=self._force)
        self._force = False
        self.stamp = frame.stamp
        self.pose = frame.pose or self.pose
        if hasattr(self.is_free, "robot"):
            self.is_free.robot = self.pose  # the gate checks goals are reachable from here
        self._operator()
        self._apply_results()
        self._take_new()
        for track in list(self.tracks.values()):
            self._advance(track)
        if self.find_candidates:
            self._candidates(frame)
        self._steer()
        self._explore()
        self._obstacles()
        return seen

    def finish(self):
        """Wait for the navigator and the reader, then let the ordinary inspection settle the survey
        (merge fragments, judge missing objects, announce the rest) and inspect what is left: objects
        beside one of their kind are announced only then, and none should fare worse than offline."""
        if self.pause:
            self._end_pause("patrol ended")
        if self.goal:
            wait([self.goal[0]])  # a navigator that failed must not cost the inspection of what was found
            self.goal = None
        while self.jobs:  # each result goes out (dashboard included) as soon as it is in
            wait([job[0] for job in self.jobs], return_when=FIRST_COMPLETED)
            self._apply_results()
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
        cells = len({(view_name(site_object.kind, k.distance), k.side) for k in self.keyframes.for_object(site_object.id)})
        if track.progress_at is None or cells > track.cells:
            track.cells, track.progress_at = cells, self.stamp
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
        if self.stamp - track.progress_at >= self.stall_seconds:
            return "no new views"
        if (self.navigator and self.focus == track.object_id and self.clock() >= self.hold_until
                and getattr(self.is_free, "informed", lambda: True)()
                and self._may_drive() and self._next_view() is None and not self.goal):
            return "no reachable view left"
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
        if self.worker.pending(Worker.READ) > 1:
            return  # the reader is behind: first reads of other objects come first
        missing = len(missing_views(site_object, self.keyframes.for_object(site_object.id)))
        if missing < track.missing_at_read:
            track.state, track.since = COLLECTING, self.stamp
            self.store.emit("object_reopened", object=site_object.id, missing=missing, was=track.missing_at_read)

    # --- the worker --------------------------------------------------------------------------
    def _submit(self, kind, object_id, payload, call):
        priority = Worker.CHECK if kind == "check" else Worker.READ
        self.jobs.append((self.worker.submit(call, priority), kind, object_id, payload))

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
        except Exception as error:  # an unreachable model or a malformed answer fails this object, not the walk
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
        earlier = dict(site_object.result) if track.reads and site_object.result.get("mission") == self.mission else None
        try:
            self.inspection.finish(site_object, job, answer, missing)
        except Exception as error:
            self.store.emit("error", stage="online read", object=object_id, message=str(error))
        if earlier:
            self._keep_earlier(object_id, earlier)
        track.state, track.reads = DONE, track.reads + 1
        if self.focus == object_id:
            self.focus = None

    def _keep_earlier(self, object_id, earlier):
        """A later read must not clear what an earlier one found: keep the worse status, and the
        earlier result itself (its sheet entry and evidence stay in the ladder store as they were)."""
        result = dict(self.objects.get(object_id).result)
        previous = earlier.pop("earlier", [])
        result["earlier"] = previous + [earlier]
        if SEVERITY.get(earlier.get("status"), 0) > SEVERITY.get(result.get("status"), 0):
            result["status"], result["status_from"] = earlier["status"], "earlier read"
        self.objects.update(object_id, result=result)

    # --- where to go -------------------------------------------------------------------------
    def _distance(self, position):
        if position is None or self.pose is None:
            return None
        return math.hypot(position[0] - self.pose[0], position[1] - self.pose[1])

    def _steer(self):
        collecting = [t for t in self.tracks.values() if t.state in (COLLECTING, CHECKING)]
        if collecting:
            def order(track):
                found = self.objects.get(track.object_id)
                distance = self._distance(found.position) if found else None
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
        if site_object is None:
            return None
        requests = missing_views(site_object, self.keyframes.for_object(self.focus), self.is_free,
                                 self.tried.get(self.focus, set()))
        return requests[0] if requests else None

    def _drive(self, request):
        if not self.navigator:
            return
        if self.goal and self.goal[0].done():
            future, sent = self.goal
            self.goal = None
            outcome = getattr(self.navigator, "outcome", None)
            if future.exception() is None and outcome in (BUSY, PREEMPTED, UNAVAILABLE):
                # Says nothing about the place: someone else drives, or there is no navigation.
                self._hold(outcome, getattr(self.navigator, "detail", ""))
            else:
                # Reached or not, this spot was tried: reached without the view (hidden, not detected
                # from there) it is not asked for again.
                self.tried.setdefault(sent.object_id, set()).add((sent.view, sent.side))
                if future.exception() is None and future.result():
                    self._force = True  # standing at the view: take the next frame whatever the interval
                else:
                    self.store.emit("view_failed", object=sent.object_id, view=sent.view, side=sent.side)
        if self.goal or request is None or self.clock() < self.hold_until or not self._may_drive():
            return
        if hasattr(self.navigator, "idle") and not self.navigator.idle():
            self._hold(PREEMPTED, "operator goal underway")
            return
        if self.held:
            self.store.emit("navigation_resumed", was=self.held)
            self.held = None
        self.store.emit("view_requested", object=request.object_id, view=request.view, side=request.side,
                        goal=[request.x, request.y, request.yaw], reason=request.reason)
        self.goal = (self.driver.submit(self.navigator.go_to, request), request)

    def _hold(self, why, detail=""):
        grace = {BUSY: 3.0, PREEMPTED: self.operator_grace, UNAVAILABLE: 10.0}.get(why, 5.0)
        self.hold_until = self.clock() + grace
        if self.held != why:
            self.held = why
            self.store.emit("navigation_held", reason=why, detail=detail, seconds=grace)

    # --- exploration: pause for an object's views, resume after ---------------------------------
    def _exploration_state(self):
        if self.exploration is None:
            return None
        if self.clock() - self._explore_checked >= 1.0:
            self._explore_checked = self.clock()
            self._explore_state = self.exploration.state()
        return self._explore_state

    def _may_drive(self):
        """The explorer is not driving: idle, mapped, failed, or paused."""
        state = self._exploration_state()
        return state is None or state.get("state") in ("IDLE", "MAPPED", "FAILED", "PAUSED")

    def _explore(self):
        if self.exploration is None:
            return
        now, state = self.clock(), self._exploration_state()
        current = state.get("state") if state else None
        if self.pause:
            if state is not None and state.get("by") == "operator":
                self._pause_taken()  # the operator took the pause over: it is theirs to resume
                return
            if state is not None and current not in ("PAUSING", "PAUSED") and now - self.pause["since"] > 3.0:
                self.store.emit("exploration_pause_ended", reason=f"exploration is {current}", objects=sorted(self.pause["objects"]))
                self.pause = None  # resumed or finished elsewhere (operator, lease run out)
                return
            open_ = [o for o in self.pause["objects"] if o in self.tracks and self.tracks[o].state in (COLLECTING, CHECKING)]
            if not open_:
                self._end_pause("views collected")
            elif now - self.pause["since"] >= self.pause_cap:
                self._end_pause("time cap")
            elif now - self.pause["renewed"] >= self.exploration.lease_s / 3:
                self.pause["renewed"] = now
                ok, text = self.exploration.renew()
                if not ok and "operator" in text:
                    self._pause_taken()
                elif not ok:
                    self.store.emit("error", stage="exploration renew", message=text)
            return
        if current != "EXPLORING" or not self.focus or self.focus in self.paused_for or now < self._pause_retry:
            return
        focus = self.objects.get(self.focus)
        distance = self._distance(focus.position) if focus else None
        if distance is None or distance > self.pause_radius or self._next_view() is None:
            return
        batch = {t.object_id for t in self.tracks.values() if t.state in (COLLECTING, CHECKING)
                 and t.object_id not in self.paused_for
                 and (self._distance((self.objects.get(t.object_id) or focus).position) or 0) <= self.pause_radius + 2}
        ok, text = self.exploration.pause(reason="views of " + ", ".join(sorted(batch)))
        if not ok:
            self._pause_retry = now + 5.0
            self.store.emit("error", stage="exploration pause", message=text)
            return
        self.pause = {"since": now, "renewed": now, "objects": batch}
        self.paused_for |= batch
        self._explore_checked = -math.inf  # look again at once: PAUSING -> PAUSED
        self.store.emit("exploration_paused", objects=sorted(batch))

    def _pause_taken(self):
        self.store.emit("exploration_pause_taken", by="operator", objects=sorted(self.pause["objects"]))
        self.pause = None

    def _end_pause(self, reason):
        ok, text = self.exploration.resume()
        if not ok and "operator" in text:
            return self._pause_taken()  # not ours to resume any more
        self.store.emit("exploration_resumed", reason=reason, objects=sorted(self.pause["objects"]),
                        ok=ok, detail=text)
        self.pause = None
        self._explore_checked = -math.inf

    def _obstacles(self):
        """Known footprints to the navigation stack (via the dashboard), every 2 s when they change."""
        if self.publish_obstacles is None or self.clock() - self._obstacles_at < 2.0:
            return
        self._obstacles_at = self.clock()
        from .gate import obstacle_cells
        cells = obstacle_cells(self.objects)
        if cells != self._obstacles_sent:
            self._obstacles_sent = cells
            self.publish_obstacles(cells)

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
                if near.state == "new" and near.hits >= CANDIDATE_HITS:
                    near.state = "open"
                    self.store.emit("candidate", candidate=near.id, position=list(near.position), hits=near.hits)
                continue
            self.candidates.append(Candidate(f"CAND-{len(self.candidates) + 1:03}", (x, y),
                                             state="open" if CANDIDATE_HITS <= 1 else "new"))
        live = [o for o in self.objects.all() if o.position and o.status not in ("REJECTED", "MOVED")]
        for candidate in self.candidates:
            if candidate.state not in ("new", "open"):
                continue
            near = min(live, key=lambda o: math.dist(o.position, candidate.position), default=None)
            if near and math.dist(near.position, candidate.position) <= EXPLAINED:
                candidate.state, candidate.explained_by = "explained", near.id
                self.store.emit("candidate_explained", candidate=candidate.id, object=near.id)
            elif (candidate.state == "open" and self._in_view(candidate.position)
                  and (candidate.last_look is None or self.stamp - candidate.last_look >= LOOK_GAP)):
                candidate.looks, candidate.last_look = candidate.looks + 1, self.stamp
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
