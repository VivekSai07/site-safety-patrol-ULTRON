"""Ladder sheets on disk: one folder per ladder, one evidence photo per checked point.

    <root>/ladders/LAD-001/sheet.json
    <root>/ladders/LAD-001/pruefung-01/photo-0.jpg      source frames
    <root>/ladders/LAD-001/pruefung-01/beleg-1.1.jpg    evidence crop for criterion 1.1
    <root>/events.jsonl                                 everything that happened, in order

Plain JSON and JPEG, so the protocol stays readable without this code.
"""
from datetime import datetime
import json
import math
from pathlib import Path
import re
import threading

from PIL import ImageDraw

from . import form
from .assess import traffic_light

PHOTO_SIDE = 1600
CROP_PADDING = 0.25      # of the box size, added on every side
CROP_MIN_SHARE = 0.25    # a crop is never smaller than this share of the photo
SAME_PLACE_RADIUS = 1.5  # metres: a ladder seen this close to a known one is the same ladder
LIGHT_COLOURS = {"red": (220, 38, 38), "amber": (217, 119, 6), "green": (22, 163, 74)}


def _pixels(box, size):
    width, height = size
    return [box[0] * width // 1000, box[1] * height // 1000, box[2] * width // 1000, box[3] * height // 1000]


def evidence_crop(image, box, colour):
    """The boxed region with some context around it and the box drawn in."""
    x0, y0, x1, y1 = _pixels(box, image.size)
    pad_x = max((x1 - x0) * CROP_PADDING, (image.width * CROP_MIN_SHARE - (x1 - x0)) / 2)
    pad_y = max((y1 - y0) * CROP_PADDING, (image.height * CROP_MIN_SHARE - (y1 - y0)) / 2)
    left, top = max(0, int(x0 - pad_x)), max(0, int(y0 - pad_y))
    crop = image.crop((left, top, min(image.width, int(x1 + pad_x)), min(image.height, int(y1 + pad_y))))
    ImageDraw.Draw(crop).rectangle((x0 - left, y0 - top, x1 - left, y1 - top), outline=colour,
                                   width=max(3, crop.width // 150))
    return crop


def identity_key(text):
    """Comparable form of an inventory marking: "Nr. 1" and "NR 1" are the same ladder."""
    return re.sub(r"[^a-z0-9]", "", str(text or "").lower())


class Store:
    def __init__(self, root):
        self.root = Path(root)
        (self.root / "ladders").mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._listeners = []

    # --- events -----------------------------------------------------------------
    def subscribe(self, callback):
        self._listeners.append(callback)

    def unsubscribe(self, callback):
        if callback in self._listeners:
            self._listeners.remove(callback)

    def emit(self, kind, **data):
        event = {"time": datetime.now().isoformat(timespec="seconds"), "kind": kind, **data}
        with self._lock:
            with open(self.root / "events.jsonl", "a") as log:
                log.write(json.dumps(event, ensure_ascii=False) + "\n")
        for callback in list(self._listeners):
            callback(event)
        return event

    def events(self, limit=50):
        path = self.root / "events.jsonl"
        if not path.is_file():
            return []
        lines = path.read_text().splitlines()
        return [json.loads(line) for line in (lines if limit is None else lines[-limit:])]

    def save_live(self, frame):
        """The frame the robot is looking at right now, for the dashboard."""
        image = frame.image.copy()
        image.thumbnail((800, 800))
        temporary = self.root / "live.tmp"
        image.save(temporary, "JPEG", quality=80)
        temporary.replace(self.root / "live.jpg")

    # --- reading ----------------------------------------------------------------
    def ladders(self):
        sheets = sorted((self.root / "ladders").glob("*/sheet.json"))
        return [json.loads(path.read_text()) for path in sheets]

    def ladder(self, ladder_id):
        path = self.root / "ladders" / ladder_id / "sheet.json"
        return json.loads(path.read_text()) if re.fullmatch(r"LAD-\d+", ladder_id) and path.is_file() else None

    def _save(self, sheet):
        path = self.root / "ladders" / sheet["id"] / "sheet.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(sheet, ensure_ascii=False, indent=1))
        temporary.replace(path)

    # --- identity ---------------------------------------------------------------
    def find(self, inventory=None, position=None):
        """A known ladder with the same marking, else the only known ladder at the same place."""
        sheets = self.ladders()
        key = identity_key(inventory)
        if key:
            for sheet in sheets:
                if identity_key(sheet["master"].get("inventar_nummer")) == key:
                    return sheet
        if position:
            near = [s for s in sheets if s.get("position") and math.hypot(
                s["position"][0] - position[0], s["position"][1] - position[1]) <= SAME_PLACE_RADIUS]
            # Two candidates would be a guess: record a new ladder and let a person merge.
            if len(near) == 1 and not (key and identity_key(near[0]["master"].get("inventar_nummer"))):
                return near[0]
        return None

    # --- writing ----------------------------------------------------------------
    def record(self, inspection, frames, position=None, mission=None, model=None, ladder_id=None):
        """Append one inspection column to the right ladder's sheet and save its evidence.

        `ladder_id` names the sheet when the caller already knows which ladder this is
        (the pipeline's object map); otherwise it is found by marking or position.
        """
        with self._lock:
            master = inspection.pop("master")
            sheet = self.ladder(ladder_id) if ladder_id else self.find(master.get("inventar_nummer"), position)
            if sheet is None:
                sheet = {"id": ladder_id or f"LAD-{len(self.ladders()) + 1:03}", "master": dict.fromkeys(
                    (key for key, _ in form.MASTER_DATA)), "position": None, "inspections": []}
                sheet["master"]["naechste_pruefung"] = "Siehe Prüfplakette"
            for key, value in master.items():  # the label does not change: keep what was read first
                if value is not None and sheet["master"].get(key) is None:
                    sheet["master"][key] = value
            if position:
                sheet["position"] = [round(float(v), 2) for v in position[:2]]

            now = datetime.now()
            number = len(sheet["inspections"]) + 1
            folder = f"pruefung-{number:02}"
            target = self.root / "ladders" / sheet["id"] / folder
            target.mkdir(parents=True, exist_ok=True)
            inspection.update(number=number, time=now.isoformat(timespec="seconds"),
                              date=now.strftime("%d.%m.%Y"), mission=mission, model=model,
                              folder=folder, light=traffic_light(inspection),
                              result_label=form.RESULTS[inspection["result"]], signoff=None)
            inspection["photos"] = self._save_evidence(inspection, frames, target)
            sheet["inspections"].append(inspection)
            sheet["light"] = inspection["light"]
            sheet["last_seen"] = inspection["time"]
            self._save(sheet)
        self.emit("protocol", ladder=sheet["id"], number=number, result=inspection["result"],
                  light=inspection["light"], flags=inspection["flags"], coverage=inspection["coverage"],
                  needs_view=inspection["needs_view"])
        return sheet

    def _save_evidence(self, inspection, frames, target):
        photos = []
        for index, frame in enumerate(frames):
            image = frame.image.copy()
            image.thumbnail((PHOTO_SIDE, PHOTO_SIDE))
            image.save(target / f"photo-{index}.jpg", quality=88)
            photos.append({"file": f"photo-{index}.jpg", "source": frame.name, "pose": frame.pose,
                           "time": datetime.fromtimestamp(frame.stamp).isoformat(timespec="seconds")})
        entries = list(inspection["criteria"].items()) + [("plakette", inspection["next_inspection"])]
        for name, entry in entries:
            entry["evidence"] = None
            if entry.get("photo") is None:
                continue
            entry["evidence"] = photos[entry["photo"]]["file"]  # whole photo unless a region is given
            if entry.get("box"):
                colour = LIGHT_COLOURS["red" if entry.get("status") == form.DEFECT else "green"]
                crop = evidence_crop(frames[entry["photo"]].image, entry["box"], colour)
                crop.thumbnail((PHOTO_SIDE // 2, PHOTO_SIDE // 2))
                entry["evidence"] = f"beleg-{name}.jpg"
                crop.save(target / entry["evidence"], quality=88)
        return photos

    def sign_off(self, ladder_id, number, inspector, result=None, remark=""):
        """A qualified person confirms (or overrides) the robot's proposal."""
        with self._lock:
            sheet = self.ladder(ladder_id)
            inspection = next((i for i in (sheet or {}).get("inspections", []) if i["number"] == number), None)
            if inspection is None or not str(inspector).strip() or result not in (None, *form.RESULTS):
                raise ValueError("Unknown inspection, missing inspector name or invalid result")
            inspection["signoff"] = {"inspector": str(inspector).strip(), "remark": str(remark).strip(),
                                     "time": datetime.now().isoformat(timespec="seconds"),
                                     "result": result or inspection["result"],
                                     "robot_result": inspection["result"]}
            sheet["master"]["sachkundiger"] = inspection["signoff"]["inspector"]
            self._save(sheet)
        self.emit("signoff", ladder=ladder_id, number=number, **inspection["signoff"])
        return sheet
