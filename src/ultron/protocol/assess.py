"""Turn a model reading into one inspection column of the Kontrollblatt.

Pure rules, no I/O. The model proposes per-criterion observations; the verdict in
section 8 is derived here, deterministically, so it can be explained and tested.
"""
from datetime import date

from . import form

MIN_CONFIDENCE = 0.5
MIN_COVERAGE = 0.8  # share of visual criteria that must be seen before "nothing found" counts
FLAG_OVERDUE = "pruefung_ueberfaellig"
FLAG_NO_STICKER = "pruefplakette_nicht_lesbar"
FLAG_LABELS = {
    FLAG_OVERDUE: "Prüfung überfällig",
    FLAG_NO_STICKER: "Prüfplakette fehlt oder nicht lesbar",
}


def clean_box(box):
    """A valid [x0, y0, x1, y1] box in thousandths, or None."""
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        return None
    try:
        x0, y0, x1, y1 = (min(1000, max(0, int(v))) for v in box)
    except (TypeError, ValueError):
        return None
    return [x0, y0, x1, y1] if x1 - x0 >= 5 and y1 - y0 >= 5 else None


def _photo_index(value, photo_count):
    return value if isinstance(value, int) and not isinstance(value, bool) and 0 <= value < photo_count else None


def _resolved(details, photo, needed):
    """Whether photo number `photo` shows enough detail. Unknown detail is not held against it."""
    return details is None or photo is None or details[photo] is None or details[photo] >= needed


def _criterion(criterion, reading, ladder_type, material, photo_count, details=None):
    if not form.applies(criterion, ladder_type, material):
        return {"status": form.NOT_APPLICABLE, "severity": "none", "remark": "", "confidence": 1.0,
                "suspected": False, "photo": None, "box": None}
    reading = reading or {}
    status = reading.get("status") if reading.get("status") in (
        form.OK, form.DEFECT, form.NOT_VISIBLE, form.NOT_APPLICABLE) else form.NOT_VISIBLE
    severity = reading.get("severity") if reading.get("severity") in ("minor", "critical") else "minor"
    remark = str(reading.get("remark") or "").strip()
    try:
        confidence = min(1.0, max(0.0, float(reading.get("confidence"))))
    except (TypeError, ValueError):
        confidence = 0.0
    photo = _photo_index(reading.get("image"), photo_count)
    suspected = False

    if status in (form.OK, form.DEFECT) and photo is None:
        status, remark = form.NOT_VISIBLE, (remark + " (ohne Fotobeleg)").strip()
    elif status == form.OK and not _resolved(details, photo, criterion.detail):
        # "Nothing wrong" from a photo too coarse to show it is not a finding. A defect the
        # model reports from afar is kept: a false alarm costs a look, a missed one does not.
        status, remark = form.NOT_VISIBLE, "Aufnahme zu grob für dieses Kriterium, Nahaufnahme nötig."
    elif status == form.DEFECT and confidence < MIN_CONFIDENCE:
        status, remark, suspected = form.HUMAN_CHECK, ("Verdacht: " + remark).strip(), True
    elif status == form.OK and confidence < MIN_CONFIDENCE:
        status = form.NOT_VISIBLE
    if status in (form.OK, form.NOT_VISIBLE) and criterion.mode == form.HANDS_ON:
        # Looking intact is not the same as working: a person has to operate it.
        status = form.HUMAN_CHECK
    return {"status": status, "severity": severity if status == form.DEFECT else "none", "remark": remark,
            "confidence": round(confidence, 2), "suspected": suspected, "photo": photo,
            "box": clean_box(reading.get("box")) if photo is not None else None}


def _next_inspection(sticker, photo_count, today, details=None):
    sticker = sticker or {}
    month, year = sticker.get("next_month"), sticker.get("next_year")
    photo = _photo_index(sticker.get("image"), photo_count)
    readable = (bool(sticker.get("present")) and isinstance(year, int) and 2000 <= year <= 2100
                and (month is None or (isinstance(month, int) and 1 <= month <= 12))
                and photo is not None and _resolved(details, photo, form.READING))
    result = {"present": bool(sticker.get("present")), "month": month if readable else None,
              "year": year if readable else None, "text": sticker.get("text"), "photo": photo,
              "box": clean_box(sticker.get("box")) if photo is not None else None, "overdue": None}
    if readable:
        # Without a month the sticker is valid to the end of its year.
        result["overdue"] = (year, month or 12) < (today.year, today.month)
    return result


def assess(answer, photo_count, today=None, details=None):
    """One inspection column: per-criterion states, sticker reading and the section 8 verdict.

    `details` gives each photo's resolution on the ladder in pixels per metre (None where
    unknown). Rows, the sticker date and label text are only accepted from photos fine enough.
    """
    today = today or date.today()
    ladder = answer.get("ladder") or {}
    ladder_type = ladder.get("aufstiegsart") if ladder.get("aufstiegsart") in form.LADDER_TYPES else None
    material = ladder.get("werkstoff") if ladder.get("werkstoff") in form.MATERIALS else None
    readings = {}
    for reading in answer.get("criteria") or []:
        if isinstance(reading, dict) and reading.get("id") in form.BY_ID:
            readings.setdefault(reading["id"], reading)
    criteria = {c.id: _criterion(c, readings.get(c.id), ladder_type, material, photo_count, details)
                for c in form.CRITERIA}
    # Text can only be read off a close-up of the label; without one, whatever the model
    # "read" is discarded. The ladder's type and material are judged from its shape and kept.
    label_readable = details is None or any(d is None or d >= form.READING for d in details)

    defects = [i for i, c in criteria.items() if c["status"] == form.DEFECT]
    if any(criteria[i]["severity"] == "critical" for i in defects):
        result = form.RESULT_REMOVE
    elif defects:
        result = form.RESULT_REPAIR
    else:
        result = form.RESULT_OK

    next_inspection = _next_inspection(answer.get("plakette"), photo_count, today, details)
    flags = []
    if next_inspection["overdue"]:
        flags.append(FLAG_OVERDUE)
    elif next_inspection["overdue"] is None:
        flags.append(FLAG_NO_STICKER)

    visual = [c.id for c in form.CRITERIA
              if c.mode == form.VISUAL and criteria[c.id]["status"] != form.NOT_APPLICABLE]
    seen = [i for i in visual if criteria[i]["status"] in (form.OK, form.DEFECT)]
    return {
        "criteria": criteria,
        "result": result,
        "defects": defects,
        "flags": flags,
        # Still open after this pass: another camera view could settle `needs_view`,
        # only a person can settle `needs_human`.
        "needs_view": [i for i, c in criteria.items() if c["status"] == form.NOT_VISIBLE],
        "needs_human": [i for i, c in criteria.items() if c["status"] == form.HUMAN_CHECK],
        "coverage": round(len(seen) / len(visual), 2) if visual else 0.0,
        "next_inspection": next_inspection,
        "remarks": str(answer.get("summary") or "").strip(),
        "master": {
            "aufstiegsart": ladder_type, "werkstoff": material,
            "anzahl_sprossen": ladder.get("anzahl_sprossen") if (
                isinstance(ladder.get("anzahl_sprossen"), int) and label_readable) else None,
            **{key: (str(ladder[key]).strip() or None) if ladder.get(key) is not None and label_readable else None
               for key in ("leiterlaenge", "hersteller", "artikel_typ_nummer", "inventar_nummer")},
        },
    }


def traffic_light(inspection):
    """red: take out of use. amber: repair, overdue or suspected defect.
    green: nothing found. grey: nothing found, but too little of the ladder was seen to say so."""
    if inspection["result"] == form.RESULT_REMOVE:
        return "red"
    suspected = any(c["suspected"] for c in inspection["criteria"].values())
    if inspection["result"] == form.RESULT_REPAIR or FLAG_OVERDUE in inspection["flags"] or suspected:
        return "amber"
    return "green" if inspection["coverage"] >= MIN_COVERAGE else "grey"
