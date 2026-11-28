"""Forward the patrol to the team dashboard (`ultron.dashboard`) over its HTTP API.

The store stays the record on the robot side; this only translates each event into the
dashboard's contract: mission start/end, live camera frame and pose, events, evidence
photos and one inspection per ladder. A dashboard that is down never stops the patrol.
"""
import json
import re
import time
import urllib.error
import urllib.request

from ultron.dashboard.checklist import GROUPS

from . import form
from .assess import FLAG_LABELS, FLAG_NO_STICKER, FLAG_OVERDUE

CAMERA = "front_rgb"

# Both sides list the paper form top to bottom, so criteria pair up by position.
GROUP_OF_SECTION = {number: group[0] for (number, _, _), group in zip(form.SECTIONS, GROUPS)}
ITEM_KEY = {criterion.id: f"{group[0]}.{item[0]}"
            for criterion, (group, item) in zip(form.CRITERIA, ((g, i) for g in GROUPS for i in g[5]))}
if len(ITEM_KEY) != sum(len(g[5]) for g in GROUPS) or any(
        ITEM_KEY[c.id].split(".")[0] != GROUP_OF_SECTION[c.section] for c in form.CRITERIA):
    raise ImportError("ultron.protocol.form and ultron.dashboard.checklist no longer describe the same form")

# Pipeline events passed on to the dashboard's event list, under these type names.
FORWARDED = {"object_found": "OBJECT_FOUND", "object_merged": "OBJECT_MERGED", "object_verified": "OBJECT_VERIFIED",
             "object_relabelled": "OBJECT_RELABELLED", "object_rejected": "OBJECT_REJECTED",
             "object_unclear": "OBJECT_UNCLEAR", "view_requested": "VIEW_REQUESTED", "view_failed": "VIEW_FAILED",
             "checklist": "CHECKLIST_DONE", "stage": "STAGE", "localisation_suspect": "LOCALISATION_SUSPECT",
             "localisation_anchored": "LOCALISATION_ANCHORED", "object_moved": "OBJECT_MOVED", "object_back": "OBJECT_BACK"}
# Events about objects known from earlier patrols, forwarded whether or not this one announced them.
ABOUT_STORED = {"object_moved", "object_back"}

ITEM_RESULT = {form.OK: "OK", form.DEFECT: "DEFECT", form.NOT_VISIBLE: "UNKNOWN",
               form.HUMAN_CHECK: "NOT_CHECKED", form.NOT_APPLICABLE: "N_A"}


def _item_result(entry):
    # A suspected defect must hold the verdict back, so it may not look like "a person's job anyway".
    return "UNKNOWN" if entry["suspected"] else ITEM_RESULT[entry["status"]]


def _group_result(results):
    for outcome in ("DEFECT", "UNKNOWN", "OK", "NOT_CHECKED"):
        if outcome in results:
            return outcome
    return "N_A"


def _length_m(text):
    match = re.search(r"\d+(?:[.,]\d+)?", str(text or ""))
    value = float(match.group().replace(",", ".")) if match else 0.0
    return value if 0 < value < 30 else None


def inspection_payload(sheet, inspection, mission_id, mission_time):
    """One inspection column in the dashboard's `Inspection` contract, plus the photo files to upload."""
    items = {ITEM_KEY[cid]: _item_result(entry) for cid, entry in inspection["criteria"].items()}
    checklist = dict(items)
    for number, group in GROUP_OF_SECTION.items():
        checklist[group] = _group_result({items[ITEM_KEY[c.id]] for c in form.CRITERIA if c.section == number})

    sticker = inspection["next_inspection"]
    sticker_status = "OVERDUE" if FLAG_OVERDUE in inspection["flags"] else (
        "UNREADABLE" if FLAG_NO_STICKER in inspection["flags"] else "VALID")
    unknown = sorted(key for key, result in items.items() if result == "UNKNOWN")
    if "DEFECT" in items.values() or sticker_status == "OVERDUE":
        status = "FLAGGED"
    elif unknown or sticker_status == "UNREADABLE":
        status = "MANUAL_REVIEW"
    else:
        status = "AUTOMATED_PASS"

    prefix = f"{sheet['id']}-{inspection['number']:02}-"

    def photo_id(file):
        return prefix + file.rsplit(".", 1)[0]

    photos = [{"id": photo_id(p["file"]), "camera": CAMERA, "caption": f"Aufnahme {p['source']}".strip()}
              for p in inspection["photos"]]
    item_photos = {}
    for criterion in form.CRITERIA:
        entry = inspection["criteria"][criterion.id]
        if not entry.get("evidence"):
            continue
        if entry["evidence"].startswith("beleg-"):
            photos.append({"id": photo_id(entry["evidence"]), "camera": CAMERA,
                           "caption": f"{criterion.id} {criterion.label}: {entry['remark']}"})
        item_photos[ITEM_KEY[criterion.id]] = [photo_id(entry["evidence"])]

    if (sticker.get("evidence") or "").startswith("beleg-"):
        photos.append({"id": photo_id(sticker["evidence"]), "camera": CAMERA,
                       "caption": f"Prüfplakette: {sticker.get('text') or ''}".strip()})

    master = sheet["master"]
    # The dashboard never lets the robot tick "Leiter i. O.", so the proposed result is not repeated here.
    reasons = [*(FLAG_LABELS[flag] for flag in inspection["flags"]), inspection["remarks"]]
    payload = {
        "mission_id": mission_id, "asset_id": sheet["id"], "ladder_type": master.get("aufstiegsart") or "Sonstige",
        "status": status, "reason": ". ".join(r.rstrip(".") for r in reasons if r) + "." if any(reasons) else "Keine Bemerkung.",
        "time": round(mission_time, 1), "checklist": checklist, "photos": photos, "item_photos": item_photos,
        "missing": unknown, "label_status": sticker_status,
        # Without localisation the position is unknown; the contract needs three numbers.
        "position": [*(sheet.get("position") or [0.0, 0.0]), 0.0],
        "ladder": {
            "inventory_no": master.get("inventar_nummer"), "ascent_type": master.get("aufstiegsart"),
            "material": master.get("werkstoff"),
            "rungs": master.get("anzahl_sprossen") if (master.get("anzahl_sprossen") or 0) >= 1 else None,
            "length_m": _length_m(master.get("leiterlaenge")), "manufacturer": master.get("hersteller"),
            "article_no": master.get("artikel_typ_nummer"),
            "sticker_due": f"{sticker['year']}-{sticker['month'] or 12:02}" if sticker["year"] else None,
            "sticker_status": sticker_status,
        },
    }
    return payload, [(photo["id"], photo["id"][len(prefix):] + ".jpg") for photo in photos]


def http_post(url):
    """POST helper for a dashboard at `url`: returns (status, body text)."""
    def post(path, body=None, raw=None):
        data = raw if raw is not None else json.dumps(body or {}).encode()
        request = urllib.request.Request(url.rstrip("/") + path, data, method="POST", headers={
            "Content-Type": "image/jpeg" if raw is not None else "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, response.read().decode(errors="replace")
        except urllib.error.HTTPError as error:
            return error.code, error.read().decode(errors="replace")
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            return 0, str(error)
    return post


class Publisher:
    """Subscribe `on_event` to a store; everything the patrol does then shows up on the dashboard."""

    def __init__(self, store, post, mission_id, site_id, log=print):
        self.store, self.post, self.log = store, post, log
        self.mission_id, self.site_id = mission_id, site_id
        self.started = time.time()
        # Objects the dashboard has been told about. Events about any other object (a fragment
        # that is folded into its neighbour later, a one-off box) are kept off the dashboard.
        self.announced = {e["object"] for e in store.events(limit=None) if e["kind"] == "object_found"}

    def _send(self, path, body=None, raw=None, accept=(200, 201, 204)):
        status, text = self.post(path, body, raw)
        if status not in accept:
            self.log(f"dashboard {path}: HTTP {status} {text[:300]}")
        return status in accept

    def _elapsed(self):
        return round(time.time() - self.started, 1)

    def start(self):
        # 409: the mission exists already, for example after a restart of the reader.
        return self._send("/api/missions", {"mission_id": self.mission_id, "site_id": self.site_id,
                                            "map_version": "", "synthetic": False}, accept=(201, 409))

    def end(self, status="MISSION_COMPLETE"):
        ladders = sum(any(i["mission"] == self.mission_id for i in s["inspections"]) for s in self.store.ladders())
        return self._send(f"/api/missions/{self.mission_id}/end", {"status": status, "summary": {"found": ladders}})

    def _event(self, kind, asset_id=None, **data):
        self._send("/api/events", {"mission_id": self.mission_id, "type": kind, "time": self._elapsed(),
                                   "asset_id": asset_id, "data": data})

    def on_event(self, event):
        kind = event["kind"]
        if kind == "look":
            live = self.store.root / "live.jpg"
            if live.is_file():
                self._send(f"/api/missions/{self.mission_id}/frames/{CAMERA}", raw=live.read_bytes())
            if event.get("pose"):
                x, y, yaw = event["pose"]
                self._send(f"/api/missions/{self.mission_id}/pose",
                           {"time": self._elapsed(), "x": x, "y": y, "yaw": yaw,
                            "state": "LADDER_IN_VIEW" if event["ladder"] else "PATROL"})
            if event["ladder"]:
                self._event("LADDER_IN_VIEW", view=event.get("view"), note=event.get("note"))
        elif kind == "inspecting":
            self._event("INSPECTION_STARTED", frames=event["frames"])
        elif kind == "error":
            self._event("READER_ERROR", stage=event.get("stage"), message=event.get("message"))
        elif kind == "protocol":
            self.publish(event["ladder"], event["number"])
        elif kind in FORWARDED:
            # Objects other than ladders have no record type on the dashboard yet: they appear
            # in its event list, with their position where known.
            subject = event.get("object")
            if kind == "object_found":
                self.announced.add(subject)
            elif kind.startswith("object_") and kind not in ABOUT_STORED and subject not in self.announced:
                return  # never announced: nothing on the dashboard to update
            if kind == "object_merged":
                self.announced.discard(subject)
            elif kind == "object_relabelled":
                self.announced.add(event.get("into"))
            data = {k: v for k, v in event.items() if k not in ("time", "kind", "object", "announced")}
            self._event(FORWARDED[kind], subject, **data)

    def publish(self, ladder_id, number):
        sheet = self.store.ladder(ladder_id)
        inspection = next(i for i in sheet["inspections"] if i["number"] == number)
        payload, files = inspection_payload(sheet, inspection, self.mission_id, self._elapsed())
        folder = self.store.root / "ladders" / ladder_id / inspection["folder"]
        for photo_id, file in files:
            self._send(f"/api/missions/{self.mission_id}/photos/{photo_id}", raw=(folder / file).read_bytes())
        if sheet.get("position"):  # the dashboard pins confirmed ladders on its map
            self._event("LADDER_CONFIRMED", ladder_id, position=payload["position"], ladder_type=payload["ladder_type"])
        ok = self._send("/api/inspections", payload)
        self.log(f"dashboard: {ladder_id} {payload['status']}" + ("" if ok else " NOT accepted"))
        return ok
