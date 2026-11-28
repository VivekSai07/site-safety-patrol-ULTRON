"""Site patrol pipeline: survey into an object map, keyframes, view planning, inspection."""
from datetime import date
import math
from pathlib import Path
import tempfile
import unittest

try:
    import numpy as np
    from PIL import Image
except ImportError:
    raise unittest.SkipTest("Pillow and numpy are required for the pipeline package")

from ultron.pipeline import checklists
from ultron.pipeline.catalog import KINDS
from ultron.pipeline.coverage import missing_views
from ultron.pipeline.detect import ReplayDetector, VlmDetector
from ultron.pipeline.inspection import Inspection
from ultron.pipeline.keyframes import KeyframeStore, view_name
from ultron.pipeline.lift import ASSUMED_RANGE, lift
from ultron.pipeline.navigate import DashboardNavigator, Passive
from ultron.pipeline.objects import ObjectMap
from ultron.pipeline.survey import Survey
from ultron.pipeline.types import Detection, ViewRequest
from ultron.protocol.frames import Frame
from ultron.protocol.lidar import Calibration
from ultron.protocol.store import Store
from tests.test_protocol import reading

# Camera looking along the robot's +x; lidar frame = robot frame (x forward, y left, z up).
CALIBRATION = Calibration([500, 500, 320, 240], [640, 480], [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
CENTRE = [400, 200, 600, 800]


def frame(name, pose=None, stamp=0.0, depth=None, seed=0):
    noise = np.random.default_rng(seed).integers(0, 255, (480, 640, 3), dtype=np.uint8)
    return Frame(Image.fromarray(noise), name=name, stamp=stamp, pose=pose,
                 depth=None if depth is None else np.full((480, 640), depth, dtype=np.float32))


def facing(target, side, distance):
    """Robot pose `distance` metres from `target` on 45-degree step `side`, looking at it."""
    angle = side * math.pi / 4
    x, y = target[0] + distance * math.cos(angle), target[1] + distance * math.sin(angle)
    return (x, y, math.atan2(target[1] - y, target[0] - x))


class SeesEverything:
    """Detector that finds `kind` in the middle of every frame whose name starts with the kind."""

    def detect(self, frame):
        return [Detection(kind, CENTRE, 0.9) for kind in KINDS if frame.name.startswith(kind)]


class FormVlm:
    model = "fake"

    def __init__(self, checklist_answer=None):
        self.checklist_answer, self.calls = checklist_answer, []

    def inspect(self, frames):
        self.calls.append(("ladder", [f.name for f in frames]))
        return reading()

    def ask(self, prompt, frames, name, schema, quick=True):
        self.calls.append((name, [f.name for f in frames]))
        return self.checklist_answer


class Workspace(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.store = Store(self.root)
        self.objects, self.keyframes = ObjectMap(self.root), KeyframeStore(self.root)
        self.survey = Survey(SeesEverything(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)

    def see(self, kind, target, side, distance, stamp=0.0):
        return self.survey.feed(frame(f"{kind}-{side}-{distance}", facing(target, side, distance), stamp, depth=distance))


class LiftTest(unittest.TestCase):
    def test_depth_places_the_object_in_front_of_the_robot(self):
        sighting = lift(frame("a", (1.0, 2.0, math.pi / 2), depth=3.0), Detection("ladder", CENTRE), CALIBRATION)
        self.assertAlmostEqual(sighting.position[0], 1.0, delta=0.1)
        self.assertAlmostEqual(sighting.position[1], 5.0, delta=0.1)
        self.assertEqual((round(sighting.distance, 1), sighting.approximate), (3.0, False))
        self.assertEqual(sighting.side((1.0, 2.0, math.pi / 2)), 6)  # seen from the -y side

    def test_an_object_on_the_right_of_the_image_is_on_the_robots_right(self):
        sighting = lift(frame("a", (0.0, 0.0, 0.0), depth=2.0), Detection("ladder", [800, 200, 1000, 800]), CALIBRATION)
        self.assertLess(sighting.position[1], -0.5)

    def test_without_range_the_position_is_marked_approximate(self):
        sighting = lift(frame("a", (0.0, 0.0, 0.0)), Detection("ladder", CENTRE), CALIBRATION)
        self.assertEqual((sighting.distance, sighting.approximate), (ASSUMED_RANGE, True))

    def test_without_pose_only_the_detection_is_kept(self):
        self.assertIsNone(lift(frame("a"), Detection("ladder", CENTRE), CALIBRATION).position)


class DepthRegistrationTest(unittest.TestCase):
    def test_depth_is_reprojected_into_the_colour_camera(self):
        from ultron.protocol.bag import register_depth
        depth = np.zeros((480, 848), dtype=np.float32)
        depth[200:280, 400:450] = 2.0  # a patch 2 m away, near the depth camera's centre
        registered = register_depth(depth, (431.0, 431.0, 425.9, 241.0), (643.6, 642.9, 643.5, 362.0), (1280, 720),
                                    np.eye(3), np.array([-0.059, 0.0, 0.0]))
        self.assertEqual(registered.shape, (360, 640))
        rows, cols = np.nonzero(registered)
        self.assertTrue(np.allclose(registered[rows, cols], 2.0, atol=1e-3))
        # Expected colour pixel of the patch centre: ray through the depth pixel, shifted 59 mm, re-projected.
        x = (425 - 425.9) / 431.0 * 2.0 - 0.059
        self.assertAlmostEqual(cols.mean() * 2, 643.6 * x / 2.0 + 643.5, delta=4)
        self.assertAlmostEqual(rows.mean() * 2, 642.9 * ((240 - 241.0) / 431.0) + 362.0, delta=4)

    def test_the_nearer_surface_wins_where_two_project_to_one_pixel(self):
        from ultron.protocol.bag import register_depth
        depth = np.full((4, 4), 3.0, dtype=np.float32)
        depth[1, 1] = 1.0
        registered = register_depth(depth, (1.0, 1.0, 2.0, 2.0), (1.0, 1.0, 2.0, 2.0), (4, 4), np.eye(3), np.zeros(3), scale=4)
        self.assertEqual(registered.tolist(), [[1.0]])


class SurveyTest(Workspace):
    def test_repeated_sightings_become_one_object_per_place_and_kind(self):
        for stamp, side in enumerate((0, 1, 7)):
            self.see("ladder", (5.0, 5.0), side, 3.0, stamp)
        self.see("ladder", (12.0, 5.0), 0, 3.0, 5)
        self.see("fire_extinguisher", (5.0, 5.2), 0, 3.0, 6)  # same place, other kind
        self.assertEqual([(o.id, o.sightings) for o in self.objects.all()],
                         [("LAD-001", 3), ("LAD-002", 1), ("FEX-001", 1)])
        self.assertAlmostEqual(self.objects.get("LAD-001").position[0], 5.0, delta=0.15)
        self.assertEqual([e["object"] for e in self.store.events() if e["kind"] == "object_found"],
                         ["LAD-001", "LAD-002", "FEX-001"])

    def test_two_boxes_in_one_frame_are_two_objects_even_when_close(self):
        class TwoLadders:
            def detect(self, frame):
                return [Detection("ladder", [300, 200, 480, 800], 0.9), Detection("ladder", [520, 200, 700, 800], 0.8)]
        survey = Survey(TwoLadders(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        for stamp in range(3):
            survey.feed(frame("pair", (0.0, 0.0, 0.0), float(stamp), depth=3.0))
        self.assertEqual([(o.id, o.sightings) for o in self.objects.all()], [("LAD-001", 3), ("LAD-002", 3)])
        self.assertLess(math.dist(self.objects.get("LAD-001").position, self.objects.get("LAD-002").position), 1.5)

    def test_overlapping_boxes_of_one_kind_are_one_detection(self):
        class Stutter:
            def detect(self, frame):
                return [Detection("fire_extinguisher", [400, 300, 500, 600], 0.6),
                        Detection("fire_extinguisher", [405, 310, 510, 610], 0.9),
                        Detection("ladder", [400, 300, 500, 600], 0.8)]
        seen = Survey(Stutter(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0).feed(
            frame("x", (0.0, 0.0, 0.0), depth=3.0))
        self.assertEqual([o.id for o in seen], ["FEX-001", "LAD-001"])

    def test_a_stopped_survey_resumes_from_disk(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        resumed = Survey(SeesEverything(), ObjectMap(self.root), KeyframeStore(self.root), self.store, CALIBRATION, interval=0)
        resumed.feed(frame("ladder-again", facing((5.0, 5.0), 1, 3.0), 9.0, depth=3.0))
        self.assertEqual([(o.id, o.sightings) for o in resumed.objects.all()], [("LAD-001", 2)])
        self.assertEqual(len(resumed.keyframes.for_object("LAD-001")), 2)
        self.assertEqual(len({k.id for k in resumed.keyframes.for_object("LAD-001")}), 2)

    def test_without_poses_objects_are_split_by_time(self):
        survey = Survey(SeesEverything(), self.objects, self.keyframes, self.store, interval=0)
        for stamp in (0.0, 2.0, 4.0, 30.0):
            survey.feed(frame("ladder", stamp=stamp))
        self.assertEqual([(o.id, o.sightings, o.position) for o in self.objects.all()],
                         [("LAD-001", 3, None), ("LAD-002", 1, None)])

    def test_frames_inside_the_interval_are_skipped_and_a_failing_detector_is_logged(self):
        survey = Survey(SeesEverything(), self.objects, self.keyframes, self.store, interval=1.0)
        self.assertTrue(survey.feed(frame("ladder", stamp=0.0)))
        self.assertEqual(survey.feed(frame("ladder", stamp=0.5)), [])

        class Broken:
            def detect(self, frame):
                raise RuntimeError("model crashed")
        Survey(Broken(), self.objects, self.keyframes, self.store, interval=0).feed(frame("ladder"))
        self.assertEqual(self.store.events()[-1]["message"], "model crashed")


class KeyframeTest(Workspace):
    def test_memory_is_bounded_per_view_and_side(self):
        for stamp in range(10):
            self.survey.feed(frame("ladder", facing((5.0, 5.0), 0, 3.0), float(stamp), depth=3.0, seed=stamp))
        kept = self.keyframes.for_object("LAD-001")
        self.assertEqual(len(kept), self.keyframes.per_cell)
        self.assertEqual(len(list((self.root / "keyframes").glob("*.jpg"))), self.keyframes.per_cell)

    def test_sharper_frames_replace_blurrier_ones(self):
        blurry = Frame(Image.new("RGB", (640, 480), "grey"), name="ladder", pose=facing((5.0, 5.0), 0, 3.0),
                       depth=np.full((480, 640), 3.0, dtype=np.float32))
        for stamp in range(2):
            blurry.stamp = float(stamp)
            self.survey.feed(blurry)
        self.see("ladder", (5.0, 5.0), 0, 3.0, 5.0)
        self.assertEqual(max(k.sharpness for k in self.keyframes.for_object("LAD-001")) > 100, True)

    def test_selection_gives_one_frame_per_cell_overviews_first(self):
        self.see("ladder", (5.0, 5.0), 0, 1.2, 0.0)
        self.see("ladder", (5.0, 5.0), 0, 3.0, 1.0)
        self.see("ladder", (5.0, 5.0), 1, 3.0, 2.0)
        chosen = self.keyframes.select(self.objects.get("LAD-001"))
        self.assertEqual([view_name("ladder", k.distance) for k in chosen], ["overview", "overview", "close"])
        self.assertEqual(self.keyframes.select(self.objects.get("LAD-001"), budget=1)[0].distance > 2, True)

    def test_views_out_of_range_are_not_kept(self):
        self.see("ladder", (5.0, 5.0), 0, 9.0)
        self.assertEqual(self.keyframes.for_object("LAD-001"), [])


class CoverageTest(Workspace):
    def requests(self, object_id, **kwargs):
        return missing_views(self.objects.get(object_id), self.keyframes.for_object(object_id), **kwargs)

    def test_a_ladder_seen_once_needs_a_close_view_and_two_more_sides(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        requests = self.requests("LAD-001")
        self.assertEqual([(r.view, r.side) for r in requests], [("close", 0), ("overview", 1), ("overview", 7)])
        close = requests[0]
        self.assertAlmostEqual(math.dist((close.x, close.y), (5.0, 5.0)), 1.3, delta=0.2)
        self.assertAlmostEqual(close.yaw, math.pi, delta=0.05)  # standing at +x, looking back at the ladder

    def test_wall_equipment_needs_one_side_only(self):
        self.see("fire_extinguisher", (5.0, 5.0), 2, 3.0)
        self.assertEqual([(r.view, r.side) for r in self.requests("FEX-001")], [("close", 2)])

    def test_nothing_is_missing_once_every_view_and_side_is_seen(self):
        for stamp, (side, distance) in enumerate(((0, 3.0), (0, 1.2), (1, 3.0), (7, 3.0))):
            self.see("ladder", (5.0, 5.0), side, distance, stamp)
        self.assertEqual(self.requests("LAD-001"), [])

    def test_blocked_goals_are_dropped_and_unplaced_objects_request_nothing(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        self.assertEqual([(r.view, r.side) for r in self.requests("LAD-001", is_free=lambda x, y: y <= 5.5)],
                         [("close", 0), ("overview", 7), ("overview", 6)])
        self.assertEqual([(r.view, r.side) for r in self.requests("LAD-001", exclude={("close", 0), ("overview", 1)})],
                         [("overview", 7), ("overview", 2)])
        Survey(SeesEverything(), self.objects, self.keyframes, self.store, interval=0).feed(frame("first_aid_kit"))
        self.assertEqual(self.requests("AID-001"), [])


class Teleport:
    """Navigator for tests: the robot is simply at the requested pose afterwards."""

    moves = True

    def __init__(self, refuse=()):
        self.pose, self.refuse, self.goals = None, set(refuse), []

    def go_to(self, request):
        self.goals.append((request.view, request.side))
        if (request.view, request.side) in self.refuse:
            return False
        self.pose = (request.x, request.y, request.yaw)
        return True


class InspectionTest(Workspace):
    def inspection(self, navigator, vlm, target, kind="ladder"):
        counter = iter(range(100))

        def camera():
            pose = navigator.pose
            return frame(f"{kind}-view-{next(counter)}", pose, 100.0, depth=math.dist(pose[:2], target), seed=7)

        return Inspection(self.objects, self.keyframes, vlm, self.store, self.survey, navigator, camera,
                          min_sightings=1, mission="PATROL-1")

    def test_single_sightings_are_not_inspected_by_default(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        self.assertEqual(self.objects.pending(), [])
        self.see("ladder", (5.0, 5.0), 0, 3.0, stamp=1.0)
        self.assertEqual([o.id for o in self.objects.pending()], ["LAD-001"])

    def test_ladder_gets_its_missing_views_then_its_kontrollblatt(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        navigator, vlm = Teleport(), FormVlm()
        self.inspection(navigator, vlm, (5.0, 5.0)).run()
        self.assertEqual(navigator.goals[0], ("close", 0))
        self.assertEqual(len({side for _, side in navigator.goals[1:]}), 2)
        ladder = self.objects.get("LAD-001")
        self.assertEqual((ladder.status, ladder.result["form"], ladder.result["missing_views"]), ("INSPECTED", "ladder", []))
        self.assertEqual(len(vlm.calls[0][1]), 4)  # three overview sides and the close-up
        sheet = self.store.ladder("LAD-001")       # same id in the Kontrollblatt store and the object map
        self.assertEqual((len(sheet["inspections"]), sheet["position"]), (1, [round(v, 2) for v in ladder.position]))
        self.assertEqual(self.objects.pending(), [])

    def test_passive_patrol_still_fills_the_form_and_reports_what_is_missing(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        self.inspection(Passive(), FormVlm(), (5.0, 5.0)).run()
        ladder = self.objects.get("LAD-001")
        self.assertEqual(ladder.status, "INSPECTED")
        self.assertEqual(len(ladder.result["missing_views"]), 3)
        self.assertFalse([e for e in self.store.events() if e["kind"] == "view_requested"])

    def test_attempts_per_object_are_bounded_when_no_goal_is_reachable(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        navigator = Teleport(refuse={(view, side) for view in ("overview", "close") for side in range(8)})
        self.inspection(navigator, FormVlm(), (5.0, 5.0)).run()
        self.assertEqual(len(navigator.goals), 6)
        self.assertEqual(self.objects.get("LAD-001").status, "INSPECTED")

    def test_an_unreachable_view_is_skipped_and_the_others_are_still_taken(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        navigator = Teleport(refuse={("overview", 1)})
        self.inspection(navigator, FormVlm(), (5.0, 5.0)).run()
        self.assertIn(("overview", 1), navigator.goals)
        self.assertEqual(len({k.side for k in self.keyframes.for_object("LAD-001")}), 3)  # another side instead
        self.assertEqual(self.objects.get("LAD-001").result["missing_views"], [])

    def test_extinguisher_checklist_is_answered_with_evidence(self):
        self.see("fire_extinguisher", (5.0, 5.0), 2, 3.0)
        answer = {"rows": [
            {"id": "BS-1", "visible": True, "fulfilled": True, "next_month": None, "next_year": None,
             "remark": "Feuerlöscher an der Wand", "image": 0, "box": [300, 200, 700, 900], "confidence": 0.9},
            {"id": "BS-2", "visible": True, "fulfilled": None, "next_month": 3, "next_year": 2020,
             "remark": "Plakette 03/2020", "image": 1, "box": [400, 400, 600, 600], "confidence": 0.8}],
            "summary": "Prüffrist abgelaufen."}
        navigator, vlm = Teleport(), FormVlm(answer)
        self.inspection(navigator, vlm, (5.0, 5.0), "fire_extinguisher").run()
        self.assertEqual(navigator.goals, [("close", 2)])
        result = self.objects.get("FEX-001").result
        self.assertEqual((result["status"], result["open"]), ("MANGEL", []))
        folder = self.root / "objects" / "FEX-001"
        self.assertTrue((folder / "checklist.json").is_file() and (folder / "beleg-BS-2.jpg").is_file())
        self.assertEqual(self.store.events()[-1]["kind"], "checklist")

    def test_an_object_without_keyframes_is_marked_unreachable(self):
        self.see("ladder", (5.0, 5.0), 0, 9.0)  # too far for any view
        self.inspection(Passive(), FormVlm(), (5.0, 5.0)).run()
        self.assertEqual(self.objects.get("LAD-001").status, "UNREACHABLE")


class ChecklistRulesTest(unittest.TestCase):
    def row(self, **overrides):
        base = {"id": "BS-2", "visible": True, "fulfilled": None, "next_month": 6, "next_year": 2027,
                "remark": "", "image": 0, "box": None, "confidence": 0.9}
        return {**base, **overrides}

    def answer(self, kind, *rows):
        return checklists.assess(kind, {"rows": list(rows), "summary": ""}, 1, date(2026, 11, 28))

    def test_expiry_is_decided_by_the_date_not_by_the_model(self):
        present = self.row(id="BS-1", fulfilled=True)
        self.assertEqual(self.answer("fire_extinguisher", present, self.row())["status"], "IN_ORDNUNG")
        self.assertEqual(self.answer("fire_extinguisher", present, self.row(next_year=2026, next_month=10))["status"], "MANGEL")
        unread = self.answer("fire_extinguisher", present, self.row(next_year=None, next_month=None))
        self.assertEqual((unread["status"], unread["open"]), ("OFFEN", ["BS-2"]))

    def test_no_photo_low_confidence_or_missing_rows_stay_open(self):
        for bad in ({"image": None}, {"confidence": 0.2}, {"visible": False}, {"fulfilled": None}):
            result = self.answer("first_aid_kit", self.row(**{"id": "EH-1", "fulfilled": True, **bad}))
            self.assertEqual(result["rows"]["EH-1"]["answer"], "offen", bad)
        self.assertEqual(self.answer("first_aid_kit")["open"], ["EH-1"])
        self.assertEqual(self.answer("first_aid_kit", self.row(id="EH-1", fulfilled=False))["status"], "MANGEL")

    def test_prompt_quotes_every_row(self):
        for kind, rows in checklists.CHECKLISTS.items():
            self.assertTrue(all(row.text in checklists.prompt(kind, 2) for row in rows))


class DetectorTest(unittest.TestCase):
    def test_vlm_detector_keeps_only_confident_catalogued_boxes(self):
        class Vlm:
            def ask(self, prompt, frames, name, schema, quick=True):
                return {"objects": [{"kind": "ladder", "box": [100, 100, 500, 900], "confidence": 0.9},
                                    {"kind": "ladder", "box": [100, 100, 500, 900], "confidence": 0.2},
                                    {"kind": "crane", "box": [1, 1, 900, 900], "confidence": 0.9},
                                    {"kind": "first_aid_kit", "box": [5, 5, 5, 5], "confidence": 0.9}, "junk"]}
        found = VlmDetector(Vlm()).detect(frame("a"))
        self.assertEqual([(d.kind, d.score) for d in found], [("ladder", 0.9)])

    def test_replay_detector(self):
        detector = ReplayDetector({"a": [{"kind": "ladder", "box": CENTRE}]})
        self.assertEqual((len(detector.detect(frame("a"))), detector.detect(frame("b"))), (1, []))


class NavigatorTest(unittest.TestCase):
    REQUEST = ViewRequest("LAD-001", "close", 0, 3.0, 1.0, 1.57)

    def navigator(self, statuses, post_status=201):
        replies = iter(statuses)

        def call(method, path, body=None):
            if method == "POST":
                self.posted = body
                return post_status, {"seq": 4}
            return 200, next(replies)

        return DashboardNavigator(call, timeout=5, poll=1, sleep=lambda seconds: None)

    def test_goal_is_posted_and_polled_until_it_ends(self):
        accepted = {"seq": 4, "status": "ACCEPTED"}
        self.assertTrue(self.navigator([accepted, accepted, {"seq": 4, "status": "SUCCEEDED"}]).go_to(self.REQUEST))
        self.assertEqual(self.posted, {"x": 3.0, "y": 1.0, "yaw": 1.57})
        self.assertFalse(self.navigator([{"seq": 4, "status": "FAILED"}]).go_to(self.REQUEST))

    def test_replaced_goal_timeout_and_disabled_goals_all_fail(self):
        self.assertFalse(self.navigator([{"seq": 5, "status": "SENT"}]).go_to(self.REQUEST))
        self.assertFalse(self.navigator([{"seq": 4, "status": "ACCEPTED"}] * 10).go_to(self.REQUEST))
        self.assertFalse(self.navigator([], post_status=403).go_to(self.REQUEST))

    def test_against_the_real_dashboard(self):
        try:
            from fastapi.testclient import TestClient
            from ultron.dashboard.app import create_app
        except ImportError:
            self.skipTest("dashboard extra not installed")
        with tempfile.TemporaryDirectory() as media, TestClient(create_app(media_dir=media, allow_goals=True)) as client:
            def call(method, path, body=None):
                response = client.request(method, path, json=body)
                return response.status_code, response.json()

            def bridge_reports_arrival(seconds):  # what dashboard_bridge.py does when Nav2 succeeds
                client.post("/api/goal/status", json={"seq": client.get("/api/goal").json()["seq"], "status": "SUCCEEDED"})

            self.assertTrue(DashboardNavigator(call, timeout=5, poll=1, sleep=bridge_reports_arrival).go_to(self.REQUEST))


if __name__ == "__main__":
    unittest.main()
