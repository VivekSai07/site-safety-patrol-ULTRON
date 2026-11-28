"""Stage 2, inspection: for each found object, get the views its form needs, then fill the form.

Ladders go through the nesseler Kontrollblatt reader (`ultron.protocol`) and on to the dashboard.
Other kinds get their checklist rows answered and saved next to the object.
"""
import json
import time

from . import checklists
from .catalog import KINDS
from .coverage import missing_views
from .navigate import BUSY, PREEMPTED, UNAVAILABLE, Passive
from ..protocol import sections
from ..protocol.assess import assess
from ..protocol.frames import Frame
from ..protocol.store import LIGHT_COLOURS, evidence_crop
from ..protocol.vlm import VlmError


class Inspection:
    def __init__(self, objects, keyframes, vlm, store, survey=None, navigator=None, camera=None,
                 is_free=None, max_extra_views=4, max_attempts=8, min_sightings=3, mission=None, by_section=True):
        self.objects, self.keyframes, self.vlm, self.store = objects, keyframes, vlm, store
        self.survey = survey                  # used to process frames captured at requested views
        self.navigator = navigator or Passive()
        self.camera = camera                  # callable returning the current Frame, or None
        self.is_free = is_free                # is_free(x, y) from the occupancy map, or None
        self.max_extra_views = max_extra_views  # extra frames taken per object
        self.max_attempts = max_attempts        # navigation goals tried per object, reached or not
        self.min_sightings = min_sightings      # objects seen less often are not inspected
        self.mission = mission
        self.by_section = by_section            # read the ladder form section by section (seven focused calls)
        self.max_hold = 30.0                    # seconds per object to wait while someone else drives
        self.sleep = time.sleep

    def run(self):
        if self.survey:
            self.survey.tidy(self.min_sightings)
        # On a map shared by patrols, an object inspected in an earlier mission is inspected again
        # once this mission has seen it itself (announced it).
        again = {o.id for o in self.objects.all() if o.result.get("mission") != self.mission} & (
            self.survey.announced if self.survey and self.mission else set())
        for site_object in self.objects.pending(self.min_sightings, again):
            self.inspect(site_object)

    def collect_views(self, site_object):
        """Ask the navigator for missing views until none is missing, none is reachable or the budget is spent."""
        tried, taken, held = set(), 0, 0.0
        while getattr(self.navigator, "moves", True) and taken < self.max_extra_views and len(tried) < self.max_attempts:
            requests = missing_views(site_object, self.keyframes.for_object(site_object.id), self.is_free, tried)
            if not requests:
                break
            request = requests[0]
            tried.add((request.view, request.side))
            self.store.emit("view_requested", object=site_object.id, view=request.view, side=request.side,
                            goal=[request.x, request.y, request.yaw], reason=request.reason)
            if not self.navigator.go_to(request):
                outcome = getattr(self.navigator, "outcome", None)
                if outcome == UNAVAILABLE:
                    self.store.emit("navigation_held", reason=outcome, detail=getattr(self.navigator, "detail", ""))
                    break  # no navigation: the views stay missing, the form is read from what there is
                if outcome in (BUSY, PREEMPTED):
                    # The explorer or the operator drives: wait and ask again, the place is not at fault.
                    if held >= self.max_hold:
                        break  # still not ours: read the form from what there is
                    tried.discard((request.view, request.side))
                    self.store.emit("navigation_held", reason=outcome, detail=getattr(self.navigator, "detail", ""))
                    self.sleep(5.0)
                    held += 5.0
                    continue
                self.store.emit("view_failed", object=site_object.id, view=request.view, side=request.side)
                continue
            if self.camera and self.survey:
                self.survey.feed(self.camera(), force=True)
                taken += 1
        return [r.reason for r in missing_views(site_object, self.keyframes.for_object(site_object.id))]

    def inspect(self, site_object):
        still_missing = self.collect_views(site_object)
        job = self.prepare(site_object)
        if job is None:
            return self.objects.update(site_object.id, status="UNREACHABLE",
                                       result={"reason": "no usable keyframe", "mission": self.mission})
        try:
            answer = self.ask(job)
        except (VlmError, OSError, ValueError) as error:
            self.store.emit("error", stage="inspection", object=site_object.id, message=str(error))
            return site_object
        return self.finish(site_object, job, answer, still_missing)

    def prepare(self, site_object):
        """What the form reader is shown, loaded into memory so it can be read on another thread
        while the survey goes on: {"kind", "frames", "details", "focal"}, or None without keyframes."""
        chosen = self.keyframes.select(site_object)
        if not chosen:
            return None
        frames = [Frame.from_file(self.keyframes.root / k.file, name=k.id, stamp=k.stamp,
                                  pose=tuple(k.pose) if k.pose else None) for k in chosen]
        self.store.emit("inspecting", frames=[f.name for f in frames], object=site_object.id)
        # Pixels per metre each frame resolves on the object: decides which rows it can support.
        fy = self.survey.calibration.fy if self.survey and self.survey.calibration else None
        return {"kind": site_object.kind, "frames": frames, "focal": fy,
                "details": [fy / k.distance if fy and k.distance else None for k in chosen]}

    def ask(self, job):
        """The vision model's answer for a prepared job. Only model calls: safe off the main thread."""
        frames, details = job["frames"], job["details"]
        if KINDS[job["kind"]].checklist == "ladder":
            if self.by_section:
                return sections.read(self.vlm, frames, details, job["focal"])
            return self.vlm.inspect(frames)
        return self.vlm.ask(checklists.prompt(job["kind"], len(frames)), frames, "checklist",
                            checklists.ANSWER_SCHEMA, quick=False)

    def finish(self, site_object, job, answer, still_missing):
        """Assess the answer, save the sheet or checklist, report it, mark the object inspected."""
        try:
            if KINDS[job["kind"]].checklist == "ladder":
                result = self._ladder(site_object, job["frames"], job["details"], answer)
            else:
                result = self._checklist(site_object, job["frames"], job["details"], answer)
        except (OSError, ValueError) as error:
            self.store.emit("error", stage="inspection", object=site_object.id, message=str(error))
            return site_object
        result["missing_views"] = still_missing
        result["mission"] = self.mission
        return self.objects.update(site_object.id, status="INSPECTED", result=result)

    def _ladder(self, site_object, frames, details, answer):
        inspection = assess(answer, len(frames), details=details)
        sheet = self.store.record(inspection, frames, position=site_object.position, mission=self.mission,
                                  model=getattr(self.vlm, "model", None), ladder_id=site_object.id)
        last = sheet["inspections"][-1]
        return {"form": "ladder", "status": last["result"], "light": last["light"], "open": last["needs_view"],
                "sheet": f"ladders/{sheet['id']}/sheet.json"}

    def _checklist(self, site_object, frames, details, answer):
        kind = site_object.kind
        result = checklists.assess(kind, answer, len(frames), details=details, verified=site_object.verified)
        folder = self.store.root / "objects" / site_object.id
        folder.mkdir(parents=True, exist_ok=True)
        for index, frame in enumerate(frames):
            frame.image.save(folder / f"photo-{index}.jpg", quality=88)
        for row_id, row in result["rows"].items():
            row["evidence"] = None
            if row["photo"] is None:
                continue
            row["evidence"] = f"photo-{row['photo']}.jpg"
            if row["box"]:
                colour = LIGHT_COLOURS["red" if row["answer"] == checklists.NO else "green"]
                row["evidence"] = f"beleg-{row_id}.jpg"
                evidence_crop(frames[row["photo"]].image, row["box"], colour).save(folder / row["evidence"], quality=88)
        result.update(object=site_object.id, position=site_object.position, mission=self.mission)
        (folder / "checklist.json").write_text(json.dumps(result, ensure_ascii=False, indent=1))
        self.store.emit("checklist", object=site_object.id, object_kind=kind, status=result["status"],
                        open=result["open"], position=site_object.position)
        return {"form": kind, "status": result["status"], "open": result["open"],
                "sheet": f"objects/{site_object.id}/checklist.json"}
