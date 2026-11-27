"""Digital Kontrollblatt: verdict rules, evidence files, stream handling and the live API."""
from datetime import date
from pathlib import Path
import tempfile
import unittest

try:
    import numpy as np
    from PIL import Image
except ImportError:  # the simulation core runs without these
    raise unittest.SkipTest("Pillow and numpy are required for the protocol package")

from ultron.protocol import form
from ultron.protocol.assess import FLAG_NO_STICKER, FLAG_OVERDUE, assess, clean_box, traffic_light
from ultron.protocol.frames import Frame
from ultron.protocol.lidar import Calibration, locate
from ultron.protocol.patrol import Patrol
from ultron.protocol.store import Store
from ultron.protocol.vlm import INSPECTION_SCHEMA, VlmError, inspection_prompt, parse_json

TODAY = date(2026, 11, 27)


def reading(overrides=None, ladder=None, sticker=None):
    """A model answer in which every criterion is visibly fine, except `overrides`."""
    criteria = []
    for criterion in form.CRITERIA:
        entry = {"id": criterion.id, "status": "ok", "severity": "none", "remark": "unauffällig",
                 "image": 0, "box": [100, 100, 600, 600], "confidence": 0.9}
        entry.update((overrides or {}).get(criterion.id, {}))
        criteria.append(entry)
    return {"ladder": {"aufstiegsart": "Stehleiter", "werkstoff": "Aluminium", "anzahl_sprossen": 6,
                       "leiterlaenge": None, "hersteller": "Layher", "artikel_typ_nummer": None,
                       "inventar_nummer": "Nr. 1", **(ladder or {})},
            "plakette": sticker or {"present": True, "next_month": 6, "next_year": 2027, "text": "06/2027",
                                    "image": 0, "box": [10, 10, 100, 100]},
            "criteria": criteria, "summary": "ok"}


def frame(name, stamp=0.0, pose=None):
    noise = np.random.default_rng(0).integers(0, 255, (120, 160, 3), dtype=np.uint8)
    return Frame(Image.fromarray(noise), name=name, stamp=stamp, pose=pose)


class FakeVlm:
    """Sees a ladder in frames whose name starts with "ladder"; the marking follows the name."""
    model = "fake"

    def __init__(self, answer=None):
        self.answer, self.inspected = answer or reading(), []

    def triage(self, frame):
        seen = frame.name.startswith("ladder")
        return {"ladder_present": seen, "view": "overview" if seen else "none",
                "box": [200, 100, 800, 900] if seen else None,
                "id_text": frame.name.split("-")[1] if seen and "-" in frame.name else None, "note": ""}

    def inspect(self, frames):
        self.inspected.append([f.name for f in frames])
        return self.answer


class FormTest(unittest.TestCase):
    def test_form_matches_the_paper_sheet(self):
        per_section = {number: sum(c.section == number for c in form.CRITERIA) for number, _, _ in form.SECTIONS}
        self.assertEqual(per_section, {"1": 5, "2": 5, "3": 3, "4": 5, "5": 3, "6": 1, "7": 1})
        self.assertEqual(len(form.MASTER_DATA), 12)
        self.assertEqual(len(form.RESULTS), 3)

    def test_prompt_and_schema_cover_every_criterion(self):
        prompt = inspection_prompt(2)
        self.assertTrue(all(c.id in prompt and c.label in prompt for c in form.CRITERIA))
        ids = INSPECTION_SCHEMA["properties"]["criteria"]["items"]["properties"]["id"]["enum"]
        self.assertEqual(ids, [c.id for c in form.CRITERIA])


class AssessTest(unittest.TestCase):
    def test_clean_ladder_is_ok_but_function_checks_stay_with_the_human(self):
        result = assess(reading(), 1, TODAY)
        self.assertEqual(result["result"], form.RESULT_OK)
        self.assertEqual(result["flags"], [])
        self.assertEqual(result["coverage"], 1.0)
        self.assertEqual(result["needs_human"], ["3.2", "4.3", "4.5", "5.3"])
        self.assertEqual(result["criteria"]["1.5"]["status"], form.NOT_APPLICABLE)  # wood only
        self.assertEqual(traffic_light(result), "green")

    def test_minor_defect_means_repair_and_critical_means_remove(self):
        minor = assess(reading({"5.2": {"status": "defect", "severity": "minor"}}), 1, TODAY)
        self.assertEqual((minor["result"], traffic_light(minor)), (form.RESULT_REPAIR, "amber"))
        critical = assess(reading({"5.2": {"status": "defect", "severity": "minor"},
                                   "2.2": {"status": "defect", "severity": "critical"}}), 1, TODAY)
        self.assertEqual((critical["result"], traffic_light(critical)), (form.RESULT_REMOVE, "red"))
        self.assertEqual(critical["defects"], ["2.2", "5.2"])

    def test_visible_defect_on_a_function_criterion_still_counts(self):
        result = assess(reading({"3.2": {"status": "defect", "severity": "critical"}}), 1, TODAY)
        self.assertEqual(result["result"], form.RESULT_REMOVE)

    def test_unsure_defect_goes_to_the_human_instead_of_a_verdict(self):
        result = assess(reading({"1.2": {"status": "defect", "severity": "critical", "confidence": 0.3}}), 1, TODAY)
        self.assertEqual(result["result"], form.RESULT_OK)
        self.assertEqual(result["criteria"]["1.2"]["status"], form.HUMAN_CHECK)
        self.assertEqual(traffic_light(result), "amber")

    def test_claims_without_a_photo_are_not_accepted(self):
        result = assess(reading({"1.1": {"image": None}, "2.1": {"image": 7}}), 1, TODAY)
        self.assertEqual(result["criteria"]["1.1"]["status"], form.NOT_VISIBLE)
        self.assertEqual(result["criteria"]["2.1"]["status"], form.NOT_VISIBLE)
        self.assertEqual(result["needs_view"], ["1.1", "2.1"])
        self.assertLess(result["coverage"], 1.0)

    def test_nothing_found_on_a_half_seen_ladder_is_not_green(self):
        hidden = {c.id: {"status": "not_visible", "image": None} for c in form.CRITERIA if c.section in "45"}
        result = assess(reading(hidden), 1, TODAY)
        self.assertEqual((result["result"], traffic_light(result)), (form.RESULT_OK, "grey"))

    def test_missing_and_malformed_model_output_is_never_a_pass(self):
        result = assess({"criteria": [{"id": "1.1", "status": "great"}, "junk", {"id": "9.9"}]}, 1, TODAY)
        self.assertEqual(result["coverage"], 0.0)
        self.assertTrue(all(c["status"] in (form.NOT_VISIBLE, form.HUMAN_CHECK) for c in result["criteria"].values()))
        self.assertEqual(result["flags"], [FLAG_NO_STICKER])

    def test_leaning_ladder_has_no_spreader_rows(self):
        result = assess(reading(ladder={"aufstiegsart": "Anlegeleiter"}), 1, TODAY)
        self.assertEqual({result["criteria"][i]["status"] for i in ("3.1", "3.2", "3.3")}, {form.NOT_APPLICABLE})

    def test_sticker_dates(self):
        def flags(**sticker):
            return assess(reading(sticker={"present": True, "text": "", "image": 0, "box": None, **sticker}), 1, TODAY)["flags"]
        self.assertEqual(flags(next_month=10, next_year=2026), [FLAG_OVERDUE])
        self.assertEqual(flags(next_month=11, next_year=2026), [])  # due this month: still valid
        self.assertEqual(flags(next_month=None, next_year=2026), [])  # year only: valid to December
        self.assertEqual(flags(next_month=None, next_year=2025), [FLAG_OVERDUE])
        self.assertEqual(flags(next_month=None, next_year=None), [FLAG_NO_STICKER])
        self.assertEqual(flags(next_month=13, next_year=2027), [FLAG_NO_STICKER])

    def test_boxes(self):
        self.assertEqual(clean_box([-5, 10, 2000, 500]), [0, 10, 1000, 500])
        for bad in (None, [1, 2, 3], [500, 500, 500, 900], ["a", 1, 2, 3], "box"):
            self.assertIsNone(clean_box(bad))

    def test_parse_json(self):
        self.assertEqual(parse_json('```json\n{"a": 1}\n```'), {"a": 1})
        for bad in ("sorry", "[1]", ""):
            with self.assertRaises(VlmError):
                parse_json(bad)


class StoreTest(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.store = Store(self.folder.name)

    def record(self, answer=None, **kwargs):
        return self.store.record(assess(answer or reading(), 1, TODAY), [frame("a.jpg")], **kwargs)

    def test_every_assessed_point_has_an_evidence_photo_on_disk(self):
        sheet = self.record()
        inspection = sheet["inspections"][0]
        folder = Path(self.folder.name, "ladders", sheet["id"], inspection["folder"])
        for criterion_id, entry in inspection["criteria"].items():
            if entry["status"] == form.NOT_APPLICABLE:
                self.assertIsNone(entry["evidence"])
            else:
                self.assertTrue((folder / entry["evidence"]).is_file(), criterion_id)
        self.assertTrue((folder / inspection["next_inspection"]["evidence"]).is_file())
        self.assertEqual(sheet["master"]["naechste_pruefung"], "Siehe Prüfplakette")

    def test_same_marking_fills_the_next_column_of_the_same_sheet(self):
        self.record()
        sheet = self.record(reading({"5.1": {"status": "defect", "severity": "critical"}},
                                    ladder={"inventar_nummer": "NR 1", "hersteller": "Other"}))
        self.assertEqual((sheet["id"], len(sheet["inspections"])), ("LAD-001", 2))
        self.assertEqual(sheet["light"], "red")
        self.assertEqual(sheet["master"]["hersteller"], "Layher")  # first reading of the label is kept
        other = self.record(reading(ladder={"inventar_nummer": "Nr. 2"}))
        self.assertEqual(other["id"], "LAD-002")

    def test_unmarked_ladders_are_matched_by_position(self):
        unmarked = reading(ladder={"inventar_nummer": None})
        self.assertEqual(self.record(unmarked, position=(2.0, 3.0))["id"], "LAD-001")
        self.assertEqual(self.record(unmarked, position=(2.5, 3.2))["id"], "LAD-001")
        self.assertEqual(self.record(unmarked, position=(9.0, 3.0))["id"], "LAD-002")
        self.assertEqual(self.record(unmarked)["id"], "LAD-003")  # no marking, no position: new ladder

    def test_sign_off_can_override_and_is_logged(self):
        self.record()
        sheet = self.store.sign_off("LAD-001", 1, "M. Muster", form.RESULT_REPAIR)
        signoff = sheet["inspections"][0]["signoff"]
        self.assertEqual((signoff["result"], signoff["robot_result"]), (form.RESULT_REPAIR, form.RESULT_OK))
        self.assertEqual(self.store.events()[-1]["kind"], "signoff")
        for bad in (("LAD-001", 9, "x", None), ("LAD-001", 1, " ", None), ("LAD-001", 1, "x", "fine"), ("../x", 1, "x", None)):
            with self.assertRaises(ValueError):
                self.store.sign_off(*bad)


class PatrolTest(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.store, self.vlm = Store(self.folder.name), FakeVlm()

    def run_stream(self, names, **options):
        patrol = Patrol(self.vlm, self.store, interval=1.0, **options)
        patrol.run(frame(name, stamp=float(i)) for i, name in enumerate(names))
        return patrol

    def test_one_protocol_per_sighting(self):
        self.run_stream(["floor", "ladder-A", "ladder-A", "floor", "floor", "floor", "ladder-B", "floor", "floor"])
        self.assertEqual(self.vlm.inspected, [["ladder-A", "ladder-A"], ["ladder-B"]])

    def test_a_single_miss_does_not_split_a_sighting(self):
        self.run_stream(["ladder-A", "floor", "ladder-A", "floor", "floor"])
        self.assertEqual(len(self.vlm.inspected), 1)

    def test_a_new_marking_starts_a_new_sighting(self):
        self.run_stream(["ladder-A", "ladder-A", "ladder-B"])
        self.assertEqual(self.vlm.inspected, [["ladder-A", "ladder-A"], ["ladder-B"]])

    def test_open_sighting_is_written_when_the_stream_ends(self):
        self.run_stream(["ladder-A"])
        self.assertEqual(len(self.store.ladders()), 1)

    def test_frames_faster_than_the_interval_are_skipped(self):
        patrol = Patrol(self.vlm, self.store, interval=1.0)
        looks = [patrol.feed(frame("ladder-A", stamp=s)) for s in (0.0, 0.2, 0.9, 1.0)]
        self.assertEqual([look is not None for look in looks], [True, False, False, True])

    def test_blurry_frames_are_skipped(self):
        patrol = Patrol(self.vlm, self.store, min_sharpness=50)
        self.assertIsNone(patrol.feed(Frame(Image.new("RGB", (160, 120), "grey"), name="ladder-A")))
        self.assertIsNotNone(patrol.feed(frame("ladder-A")))

    def test_model_failure_is_logged_and_the_patrol_continues(self):
        def broken(frames):
            raise VlmError("offline")
        self.vlm.inspect = broken
        self.run_stream(["ladder-A", "floor", "floor", "ladder-B"])
        errors = [e for e in self.store.events() if e["kind"] == "error"]
        self.assertEqual(len(errors), 2)
        self.assertEqual(self.store.ladders(), [])


class LidarTest(unittest.TestCase):
    def test_ladder_in_front_of_a_wall_is_located_at_the_ladder(self):
        # Lidar frame: x forward, y left, z up. Camera optical frame: z forward, x right, y down.
        calibration = Calibration([500, 500, 320, 240], [640, 480],
                                  [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
        rng = np.random.default_rng(1)
        ladder = np.column_stack([np.full(200, 2.0), rng.uniform(-0.2, 0.2, 200), rng.uniform(-0.4, 0.4, 200)])
        wall = np.column_stack([np.full(600, 5.0), rng.uniform(-3, 3, 600), rng.uniform(-1, 1, 600)])
        points = np.vstack([ladder, wall])
        x, y = locate(points, [300, 100, 700, 900], calibration)
        self.assertAlmostEqual(x, 2.0, delta=0.05)
        self.assertAlmostEqual(y, 0.0, delta=0.15)
        map_x, map_y = locate(points, [300, 100, 700, 900], calibration, pose=(10.0, 5.0, np.pi / 2))
        self.assertAlmostEqual(map_x, 10.0, delta=0.15)
        self.assertAlmostEqual(map_y, 7.0, delta=0.05)
        self.assertIsNone(locate(points, [0, 0, 20, 20], calibration))


class PublishTest(unittest.TestCase):
    """The reader's output must pass the dashboard's own contract, checked against the real app."""

    def setUp(self):
        try:
            from fastapi.testclient import TestClient
            from ultron.dashboard.app import create_app
        except ImportError:
            self.skipTest("dashboard extra not installed")
        from ultron.protocol.publish import Publisher
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.store = Store(Path(self.folder.name, "data"))
        self.client = TestClient(create_app(media_dir=Path(self.folder.name, "media")))
        self.client.__enter__()
        self.addCleanup(self.client.__exit__, None, None, None)
        self.log = []

        def post(path, body=None, raw=None):
            response = (self.client.post(path, content=raw, headers={"Content-Type": "image/jpeg"})
                        if raw is not None else self.client.post(path, json=body or {}))
            return response.status_code, response.text

        self.publisher = Publisher(self.store, post, "PATROL-1", "site-a", log=self.log.append)
        self.assertTrue(self.publisher.start())
        self.store.subscribe(self.publisher.on_event)

    def record(self, answer=None, **kwargs):
        return self.store.record(assess(answer or reading(), 1, TODAY), [frame("a.jpg")], mission="PATROL-1", **kwargs)

    def published(self):
        return self.client.get("/api/missions/PATROL-1").json()["inspections"]

    def test_checklist_keys_pair_up_with_the_dashboard(self):
        from ultron.dashboard.checklist import ITEM_KEYS
        from ultron.protocol.publish import ITEM_KEY
        self.assertEqual([ITEM_KEY[c.id] for c in form.CRITERIA], ITEM_KEYS)
        self.assertEqual((ITEM_KEY["2.4"], ITEM_KEY["5.3"]), ("sprossen.verbindung", "fuesse.funktion"))

    def test_clean_ladder_arrives_with_a_photo_per_assessed_item(self):
        self.record(position=(2.0, 3.0))
        [record] = self.published()
        self.assertEqual((record["asset_id"], record["status"]), ("LAD-001", "AUTOMATED_PASS"))
        self.assertEqual(record["checklist"]["holme.verformung"], "OK")
        self.assertEqual(record["checklist"]["spreiz.funktion"], "NOT_CHECKED")
        self.assertEqual(record["checklist"]["holme.schutz"], "N_A")
        self.assertEqual(record["ladder"]["sticker_due"], "2027-06")
        self.assertEqual(record["position"], [2.0, 3.0, 0.0])
        for key, result in record["checklist"].items():
            if "." in key and result != "N_A":
                [photo] = record["item_photos"][key]
                self.assertEqual(self.client.get(f"/media/PATROL-1/{photo}.jpg").status_code, 200, key)
        self.assertFalse([line for line in self.log if "HTTP" in line])

    def test_verdicts_follow_the_dashboard_rules(self):
        cases = [
            (reading({"2.2": {"status": "defect", "severity": "critical"}}), "FLAGGED"),
            (reading(sticker={"present": True, "next_month": 1, "next_year": 2026, "text": "", "image": 0, "box": None}), "FLAGGED"),
            (reading({"5.1": {"status": "not_visible", "image": None}}), "MANUAL_REVIEW"),
            (reading({"1.2": {"status": "defect", "severity": "critical", "confidence": 0.2}}), "MANUAL_REVIEW"),
            (reading(sticker={"present": False, "next_month": None, "next_year": None, "text": None, "image": None, "box": None}), "MANUAL_REVIEW"),
        ]
        for number, (answer, expected) in enumerate(cases, 1):
            answer["ladder"]["inventar_nummer"] = f"Nr. {number}"
            self.record(answer)
        self.assertEqual([r["status"] for r in self.published()], [expected for _, expected in cases])
        self.assertFalse([line for line in self.log if "HTTP" in line])

    def test_live_frames_events_and_mission_end_reach_the_dashboard(self):
        patrol = Patrol(FakeVlm(), self.store, interval=0, mission="PATROL-1")
        patrol.run(frame(name, stamp=float(i), pose=(1.0, 2.0, 0.0)) for i, name in enumerate(["ladder-A", "floor", "floor"]))
        self.publisher.end()
        live = self.client.get("/api/live").json()
        self.assertEqual(live["mission"]["status"], "MISSION_COMPLETE")
        self.assertEqual(live["mission"]["summary"], {"found": 1})
        self.assertEqual(live["pose"]["x"], 1.0)
        self.assertIn("front_rgb", live["cameras"])
        self.assertTrue({"LADDER_IN_VIEW", "INSPECTION_STARTED", "LADDER_CONFIRMED"} <= {e["type"] for e in live["events"]})
        self.assertEqual(self.client.get("/api/missions/PATROL-1/export.csv").status_code, 200)

    def test_a_dashboard_that_is_down_does_not_stop_the_reader(self):
        self.publisher.post = lambda path, body=None, raw=None: (0, "connection refused")
        sheet = self.record()
        self.assertEqual(len(sheet["inspections"]), 1)
        self.assertTrue(any("NOT accepted" in line for line in self.log))


if __name__ == "__main__":
    unittest.main()
