"""Site patrol pipeline: survey into an object map, keyframes, view planning, inspection."""
from datetime import date
import math
from pathlib import Path
import tempfile
import unittest
import unittest.mock

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


REAL_HEIGHT = {"ladder": 1.7, "fire_extinguisher": 0.5, "fire_hydrant": 0.8, "fire_alarm": 0.36, "first_aid_kit": 0.3}


def sized_box(kind, distance):
    """A centred box as tall as an object of this kind looks from `distance` metres."""
    half = min(480.0, REAL_HEIGHT[kind] * CALIBRATION.fy / distance) / 480 * 500
    return [400, round(500 - half), 600, round(500 + half)]


class SeesEverything:
    """Detector that finds `kind` in the middle of every frame whose name starts with the kind."""

    def detect(self, frame):
        distance = float(frame.depth[240, 320]) if frame.depth is not None else 3.0
        return [Detection(kind, sized_box(kind, distance), 0.9) for kind in KINDS if frame.name.startswith(kind)]


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

    def test_range_comes_from_inside_the_outline_not_from_the_background_in_the_box(self):
        depth = np.full((480, 640), 6.0, dtype=np.float32)   # wall behind
        depth[100:400, 300:320] = 2.0                        # a thin rail
        seen = Frame(Image.new("RGB", (640, 480)), name="a", pose=(0.0, 0.0, 0.0), depth=depth)
        rail = [[469, 208], [500, 208], [500, 833], [469, 833]]
        with_outline = lift(seen, Detection("ladder", [300, 100, 700, 900], outline=rail), CALIBRATION)
        box_only = lift(seen, Detection("ladder", [300, 100, 700, 900]), CALIBRATION)
        self.assertAlmostEqual(with_outline.distance, 2.0, delta=0.05)
        self.assertAlmostEqual(box_only.distance, 6.0, delta=0.05)

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
                        Detection("ladder", [400, 100, 500, 900], 0.8)]
        seen = Survey(Stutter(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0).feed(
            frame("x", (0.0, 0.0, 0.0), depth=1.5))
        self.assertEqual([o.id for o in seen], ["FEX-001", "LAD-001"])

    def test_a_box_inside_a_better_box_of_its_kind_is_a_part_of_it(self):
        from ultron.pipeline.survey import distinct
        whole, part = Detection("ladder", [300, 100, 700, 900], 0.9), Detection("ladder", [320, 150, 480, 850], 0.7)
        beside = Detection("ladder", [720, 100, 900, 900], 0.8)
        self.assertEqual(distinct([part, whole, beside]), [whole, beside])

    def test_a_tracked_object_keeps_its_identity_next_to_another(self):
        """Two ladders 0.5 m apart: position cannot tell them apart, the detector's track ids can."""
        class Tracked:
            def detect(self, frame):
                if frame.name == "both":
                    return [Detection("ladder", [300, 200, 480, 800], 0.9, track=1),
                            Detection("ladder", [520, 200, 700, 800], 0.9, track=2)]
                # Later only the second is in view, boxed where the first used to be.
                return [Detection("ladder", [300, 200, 480, 800], 0.9, track=2)]
        survey = Survey(Tracked(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        survey.feed(frame("both", (0.0, 0.0, 0.0), 0.0, depth=3.0))
        for stamp in range(1, 4):
            survey.feed(frame("one", (0.0, 0.0, 0.0), float(stamp), depth=3.0))
        self.assertEqual([(o.id, o.sightings) for o in self.objects.all()], [("LAD-001", 1), ("LAD-002", 4)])
        self.assertEqual(self.objects.get("LAD-001").apart, ["LAD-002"])

    def test_a_track_that_jumps_across_the_map_is_not_believed(self):
        class Swapped:
            def detect(self, frame):
                return [Detection("ladder", CENTRE, 0.9, track=7)]
        survey = Survey(Swapped(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        survey.feed(frame("a", facing((5.0, 5.0), 0, 3.0), 0.0, depth=3.0))
        survey.feed(frame("b", facing((15.0, 5.0), 0, 3.0), 1.0, depth=3.0))
        self.assertEqual(len(self.objects.all()), 2)

    def test_fragments_are_merged_into_the_object_beside_them(self):
        for stamp in range(4):
            self.see("ladder", (5.0, 5.0), 0, 3.0, float(stamp))
        self.see("ladder", (5.0, 6.2), 0, 3.0, 10.0)   # a lone partial box 1.2 m off
        self.see("ladder", (12.0, 5.0), 0, 3.0, 11.0)  # a lone box with nothing near it
        self.assertEqual(len(self.keyframes.for_object("LAD-002")), 1)
        self.assertEqual(self.survey.tidy(), {"LAD-002": "LAD-001"})
        self.assertEqual([(o.id, o.sightings) for o in self.objects.all()], [("LAD-001", 5), ("LAD-003", 1)])
        self.assertEqual(self.keyframes.for_object("LAD-002"), [])
        self.assertEqual(self.store.events()[-1]["kind"], "object_merged")

    def test_a_look_alike_of_the_wrong_size_is_ignored(self):
        """A 36 cm alarm box four metres away was detected as a fire extinguisher on the real robot."""
        class Small:
            def detect(self, frame):
                return [Detection("fire_extinguisher", [480, 450, 520, 545], 0.6),   # about 0.36 m tall at 4 m
                        Detection("fire_alarm", [480, 450, 520, 545], 0.5)]
        seen = Survey(Small(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0).feed(
            frame("x", (0.0, 0.0, 0.0), depth=4.0))
        self.assertEqual([o.kind for o in seen], ["fire_alarm"])

    def test_a_box_between_two_known_neighbours_waits_for_a_clearer_frame(self):
        class Pair:
            def detect(self, frame):
                if frame.name == "both":
                    return [Detection("ladder", [300, 200, 480, 800], 0.9, track=1),
                            Detection("ladder", [520, 200, 700, 800], 0.9, track=2)]
                return [Detection("ladder", [410, 200, 590, 800], 0.9, track=3)]  # new track, midway
        survey = Survey(Pair(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        survey.feed(frame("both", (0.0, 0.0, 0.0), 0.0, depth=3.0))
        self.assertEqual(survey.feed(frame("between", (0.0, 0.0, 0.0), 1.0, depth=3.0)), [])
        self.assertEqual([o.sightings for o in self.objects.all()], [1, 1])

    def test_a_part_boxed_beside_the_whole_is_folded_back_in(self):
        class Halves:
            def detect(self, frame):
                whole = [Detection("ladder", [300, 200, 600, 800], 0.9, track=1)]
                return whole + ([Detection("ladder", [560, 200, 640, 800], 0.6, track=2)] if frame.stamp < 2 else [])
        survey = Survey(Halves(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        for stamp in range(20):
            survey.feed(frame("x", (0.0, 0.0, 0.0), float(stamp), depth=3.0))
        self.assertEqual(survey.tidy(), {"LAD-002": "LAD-001"})

    def test_objects_seen_in_one_frame_are_never_merged(self):
        class Pair:
            def detect(self, frame):
                boxes = [Detection("ladder", [300, 200, 480, 800], 0.9, track=1)]
                return boxes + ([Detection("ladder", [520, 200, 700, 800], 0.8, track=2)] if frame.name == "both" else [])
        survey = Survey(Pair(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        survey.feed(frame("both", (0.0, 0.0, 0.0), 0.0, depth=3.0))
        for stamp in range(1, 4):
            survey.feed(frame("one", (0.0, 0.0, 0.0), float(stamp), depth=3.0))
        self.assertEqual(survey.tidy(), {})
        self.assertEqual(len(self.objects.all()), 2)

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


class Opinion:
    """Vision model stand-in: answers by the frame name it is shown, as recorded per keyframe id."""

    def __init__(self, answers, default="none"):
        self.answers, self.default, self.asked = answers, default, []

    def ask(self, prompt, frames, name, schema, quick=True):
        self.asked.append(frames[0].name)
        self.prompts = getattr(self, "prompts", []) + [prompt]
        kind = self.answers.get(frames[0].name, self.default)
        if kind == "offline":
            from ultron.protocol.vlm import VlmError
            raise VlmError("offline")
        return {"kind": kind, "confidence": 0.9, "remark": "looks like " + kind}


class VerifyTest(Workspace):
    def verified_survey(self, answers, default="none"):
        from ultron.pipeline.verify import Verifier
        self.opinion = Opinion(answers, default)
        self.survey.verifier = Verifier(self.opinion)

    def see_often(self, kind, target, times=3):
        """Seen from three directions, so three different keyframes exist to vote on."""
        for stamp in range(times):
            self.see(kind, target, (0, 1, 7)[stamp % 3], 3.0, float(stamp))

    def test_one_dissenting_view_does_not_overturn_the_detector(self):
        self.see_often("fire_hydrant", (5.0, 5.0))
        ids = [k.id for k in self.keyframes.select(self.objects.get("HYD-001"), budget=3)]
        self.verified_survey({ids[0]: "fire_extinguisher"}, default="fire_hydrant")
        self.survey.tidy()
        self.assertEqual([(o.id, o.verified) for o in self.objects.all()], [("HYD-001", True)])

    def test_a_split_vote_changes_nothing_and_says_so(self):
        self.see_often("fire_hydrant", (5.0, 5.0))
        ids = [k.id for k in self.keyframes.select(self.objects.get("HYD-001"), budget=3)]
        self.verified_survey({ids[0]: "fire_extinguisher", ids[1]: "fire_alarm", ids[2]: "none"})
        self.survey.tidy()
        self.assertEqual([(o.id, o.status, o.verified) for o in self.objects.all()], [("HYD-001", "FOUND", False)])
        self.assertEqual(self.store.events()[-1]["kind"], "object_unclear")

    def test_an_agreed_object_is_marked_verified_and_judged_only_once(self):
        self.verified_survey({}, default="ladder")
        self.see_often("ladder", (5.0, 5.0))
        self.survey.tidy()
        self.survey.tidy()
        self.assertTrue(self.objects.get("LAD-001").verified)
        self.assertEqual(len(self.opinion.asked), 3)

    def test_a_relabel_that_contradicts_the_measured_size_is_refused(self):
        """On the real robot the model called the 0.8 m hose cabinet an alarm box in two of three views."""
        self.see_often("fire_hydrant", (5.0, 5.0))
        self.verified_survey({}, default="fire_alarm")
        self.survey.tidy()
        self.assertEqual([(o.id, o.kind, o.verified) for o in self.objects.all()], [("HYD-001", "fire_hydrant", False)])
        self.assertEqual(self.store.events()[-1]["kind"], "object_unclear")
        self.assertIn("about 0.80 m tall", self.opinion.prompts[0])

    def test_a_look_alike_is_relabelled_under_the_right_kind(self):
        """A 0.8 m red box may be a hose cabinet or an extinguisher box: here the model decides."""
        self.verified_survey({}, default="fire_extinguisher")
        self.see_often("fire_hydrant", (5.0, 5.0))
        keyframes = len(self.keyframes.for_object("HYD-001"))
        self.survey.tidy()
        self.assertEqual([(o.id, o.kind, o.sightings, o.verified) for o in self.objects.all()],
                         [("FEX-001", "fire_extinguisher", 3, True)])
        self.assertEqual(len(self.keyframes.for_object("FEX-001")), keyframes)
        self.assertEqual(self.store.events()[-1]["kind"], "object_relabelled")

    def test_a_relabelled_object_joins_the_known_one_at_that_place(self):
        self.see_often("fire_extinguisher", (5.0, 5.0))
        self.see_often("fire_hydrant", (5.1, 5.0))
        self.verified_survey({}, default="fire_extinguisher")
        self.survey.tidy()
        self.assertEqual([(o.id, o.sightings) for o in self.objects.all()], [("FEX-001", 6)])

    def test_something_that_is_nothing_is_rejected_and_never_inspected(self):
        self.verified_survey({})
        self.see_often("ladder", (5.0, 5.0))
        self.survey.tidy()
        self.assertEqual(self.objects.get("LAD-001").status, "REJECTED")
        self.assertEqual((self.objects.pending(), self.keyframes.for_object("LAD-001")), ([], []))

    def test_an_unsure_or_unreachable_model_confirms_nothing(self):
        from ultron.pipeline.verify import Verifier

        class Unsure:
            def ask(self, prompt, frames, name, schema, quick=True):
                return {"kind": "ladder", "confidence": 0.3, "remark": "maybe"}
        self.assertEqual(Verifier(Unsure()).classify(Image.new("RGB", (64, 64)), CENTRE)[0], "none")
        self.verified_survey({}, default="offline")
        self.see_often("ladder", (5.0, 5.0))
        self.survey.tidy()
        ladder = self.objects.get("LAD-001")
        self.assertEqual((ladder.status, ladder.verified), ("FOUND", False))

    def test_rarely_seen_objects_are_not_sent_to_the_model(self):
        self.verified_survey({}, default="ladder")
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        self.survey.tidy()
        self.assertEqual(self.opinion.asked, [])


class CalibrateTest(unittest.TestCase):
    """A detector whose prompts behave like the real one did: wording decides what is found."""

    class WordingDetector:
        # prompt -> boxes it produces per frame: (box, score, what the thing really is)
        BEHAVIOUR = {
            "red fire extinguisher": [([100, 400, 200, 700], 0.9, "fire_extinguisher"), ([600, 300, 650, 400], 0.5, "fire_alarm")],
            "fire extinguisher": [([100, 400, 200, 700], 0.2, "fire_extinguisher")],
            "fire extinguisher standing on the floor": [],
        }

        def use(self, prompts):
            self.prompts = prompts

        def detect(self, frame):
            return [Detection("fire_extinguisher", box, score) for box, score, _ in
                    self.BEHAVIOUR.get(self.prompts["fire_extinguisher"]["prompt"], [])
                    if score >= self.prompts["fire_extinguisher"]["min_score"]]

    class Judge:
        """Knows what each region really is."""

        def classify(self, image, box, name):
            return ("fire_extinguisher" if box[0] < 300 else "fire_alarm"), ""

    def run_calibration(self, kinds=("fire_extinguisher",)):
        from ultron.pipeline.calibrate import calibrate
        detector, lines = self.WordingDetector(), []
        frames = [frame(f"f{i}") for i in range(4)]
        return calibrate(frames, detector, self.Judge(), kinds=kinds, log=lines.append), detector

    def test_the_wording_that_finds_most_confirmed_boxes_wins(self):
        report, detector = self.run_calibration()
        chosen = report["fire_extinguisher"]
        self.assertEqual(chosen["prompt"], "red fire extinguisher")
        self.assertEqual([t["kept"] for t in chosen["tried"]], [4, 0, 0])
        self.assertEqual(chosen["tried"][0]["precision"], 1.0)
        self.assertEqual(detector.prompts["fire_extinguisher"]["prompt"], "red fire extinguisher")

    def test_one_wrong_judgement_does_not_throw_away_a_good_prompt(self):
        from ultron.pipeline.calibrate import _threshold
        judged = [(0.95, True), (0.9, False), (0.8, True), (0.7, True), (0.6, True), (0.3, False)]
        self.assertEqual(_threshold(judged), 0.57)
        self.assertIsNone(_threshold([(0.9, False), (0.5, False)]))

    def test_the_threshold_keeps_confirmed_boxes_and_drops_the_refuted_ones_below(self):
        report, _ = self.run_calibration()
        self.assertTrue(0.5 < report["fire_extinguisher"]["min_score"] <= 0.9)

    def test_a_kind_never_seen_keeps_its_default(self):
        from ultron.pipeline.catalog import DEFAULT_PROMPTS
        from ultron.pipeline.calibrate import calibrate

        class Blind:
            def use(self, prompts):
                pass

            def detect(self, frame):
                return []
        report = calibrate([frame("a")], Blind(), self.Judge(), kinds=["ladder"], log=lambda line: None)
        self.assertEqual((report["ladder"]["prompt"], report["ladder"]["calibrated"]),
                         (DEFAULT_PROMPTS["ladder"]["prompt"], False))

    def test_saved_prompts_are_picked_up_by_the_detector_setup(self):
        from ultron.pipeline.calibrate import save
        from ultron.pipeline.detect import load_prompts
        with tempfile.TemporaryDirectory() as folder:
            self.assertEqual(load_prompts(folder)["ladder"]["prompt"], "ladder")
            report, _ = self.run_calibration()
            save(report, folder)
            loaded = load_prompts(folder)
            self.assertEqual(loaded["fire_extinguisher"]["prompt"], "red fire extinguisher")
            self.assertEqual(set(loaded["fire_extinguisher"]), {"prompt", "min_score"})
            self.assertEqual(loaded["ladder"]["prompt"], "ladder")


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
        self.assertEqual([(r.view, r.side) for r in requests],
                         [("close", 0), ("detail", 0), ("overview", 1), ("overview", 7)])
        close = requests[0]
        self.assertAlmostEqual(math.dist((close.x, close.y), (5.0, 5.0)), 1.4, delta=0.1)
        self.assertAlmostEqual(math.dist((requests[1].x, requests[1].y), (5.0, 5.0)), 0.4, delta=0.05)
        self.assertAlmostEqual(close.yaw, math.pi, delta=0.05)  # standing at +x, looking back at the ladder

    def test_wall_equipment_needs_one_side_only(self):
        self.see("fire_extinguisher", (5.0, 5.0), 2, 3.0)
        self.assertEqual([(r.view, r.side) for r in self.requests("FEX-001")], [("close", 2), ("detail", 2)])

    def test_nothing_is_missing_once_every_view_and_side_is_seen(self):
        for stamp, (side, distance) in enumerate(((0, 3.0), (0, 1.2), (0, 0.4), (1, 3.0), (7, 3.0))):
            self.see("ladder", (5.0, 5.0), side, distance, stamp)
        self.assertEqual(self.requests("LAD-001"), [])

    def test_blocked_goals_are_dropped_and_unplaced_objects_request_nothing(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        self.assertEqual([(r.view, r.side) for r in self.requests("LAD-001", is_free=lambda x, y: y <= 5.5)],
                         [("close", 0), ("detail", 0), ("overview", 7), ("overview", 6)])
        self.assertEqual([(r.view, r.side) for r in self.requests("LAD-001", exclude={("close", 0), ("overview", 1)})],
                         [("detail", 0), ("overview", 7), ("overview", 2)])
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
                          min_sightings=1, mission="PATROL-1", by_section=False)

    def test_rarely_seen_objects_are_not_inspected_by_default(self):
        for stamp in range(2):
            self.see("ladder", (5.0, 5.0), 0, 3.0, stamp=float(stamp))
        self.assertEqual(self.objects.pending(), [])
        self.see("ladder", (5.0, 5.0), 0, 3.0, stamp=2.0)
        self.assertEqual([o.id for o in self.objects.pending()], ["LAD-001"])

    def test_ladder_gets_its_missing_views_then_its_kontrollblatt(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        navigator, vlm = Teleport(), FormVlm()
        self.inspection(navigator, vlm, (5.0, 5.0)).run()
        self.assertEqual(navigator.goals[:2], [("close", 0), ("detail", 0)])
        self.assertEqual(len({side for _, side in navigator.goals[2:]}), 2)
        ladder = self.objects.get("LAD-001")
        self.assertEqual((ladder.status, ladder.result["form"], ladder.result["missing_views"]), ("INSPECTED", "ladder", []))
        self.assertEqual(len(vlm.calls[0][1]), 5)  # three overview sides, the close-up and the detail view
        sheet = self.store.ladder("LAD-001")       # same id in the Kontrollblatt store and the object map
        self.assertEqual((len(sheet["inspections"]), sheet["position"]), (1, [round(v, 2) for v in ladder.position]))
        self.assertEqual(self.objects.pending(), [])

    def test_passive_patrol_still_fills_the_form_and_reports_what_is_missing(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        self.inspection(Passive(), FormVlm(), (5.0, 5.0)).run()
        ladder = self.objects.get("LAD-001")
        self.assertEqual(ladder.status, "INSPECTED")
        self.assertEqual(len(ladder.result["missing_views"]), 4)
        self.assertFalse([e for e in self.store.events() if e["kind"] == "view_requested"])

    def test_attempts_per_object_are_bounded_when_no_goal_is_reachable(self):
        self.see("ladder", (5.0, 5.0), 0, 3.0)
        navigator = Teleport(refuse={(view, side) for view in ("overview", "close") for side in range(8)})
        self.inspection(navigator, FormVlm(), (5.0, 5.0)).run()
        self.assertEqual(len(navigator.goals), 8)
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
             "remark": "Plakette 03/2020", "image": 2, "box": [400, 400, 600, 600], "confidence": 0.8}],
            "summary": "Prüffrist abgelaufen."}
        navigator, vlm = Teleport(), FormVlm(answer)
        # The test camera is coarser than the robot's: let its 0.4 m view count as readable.
        with unittest.mock.patch("ultron.pipeline.checklists.READING", 1000):
            self.inspection(navigator, vlm, (5.0, 5.0), "fire_extinguisher").run()
        self.assertEqual(navigator.goals, [("close", 2), ("detail", 2)])
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

    def test_a_date_read_from_a_photo_too_coarse_for_a_sticker_is_not_accepted(self):
        present = self.row(id="BS-1", fulfilled=True)
        def status(details):
            return checklists.assess("fire_extinguisher", {"rows": [present, self.row()], "summary": ""}, 1,
                                     date(2026, 11, 28), details=details)["rows"]["BS-2"]["answer"]
        self.assertEqual((status([400.0]), status([1600.0]), status(None)), ("offen", "ja", "ja"))

    def test_a_verified_object_is_present_whatever_the_model_says_about_the_row(self):
        absent = self.row(id="EH-2", fulfilled=False)
        def answer(verified):
            return checklists.assess("fire_alarm", {"rows": [absent], "summary": ""}, 1, date(2026, 11, 28),
                                     verified=verified)["rows"]["EH-2"]["answer"]
        self.assertEqual((answer(True), answer(False)), ("ja", "nein"))

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
        self.assertFalse(self.navigator([{"seq": 4, "status": "REJECTED"}]).go_to(self.REQUEST))

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


class RelayTest(unittest.TestCase):
    """The robot-side relay's stream, sent over a real local socket, becomes posed frames with depth."""

    def test_messages_survive_the_wire(self):
        from ultron.protocol.relay_wire import cloud_message, frame_message, unpack
        import io
        color = np.random.default_rng(0).integers(0, 255, (48, 64, 3), dtype=np.uint8)
        depth = np.full((24, 32), 2000, dtype=np.uint16)
        stream = io.BytesIO(frame_message(12.5, color, depth, (50, 50, 32, 24), (25, 25, 16, 12), np.eye(3), [0, 0, 0],
                                          pose=(1.0, 2.0, 0.5)) + cloud_message(12.6, [[1, 2, 3], [4, 5, 6]]))
        header, blobs = unpack(stream.read)
        self.assertEqual((header["kind"], header["stamp"], header["pose"]), ("frame", 12.5, [1.0, 2.0, 0.5]))
        self.assertEqual(np.frombuffer(blobs["color"], np.uint8).reshape(header["color_shape"]).tolist(), color.tolist())
        header, blobs = unpack(stream.read)
        self.assertEqual((header["kind"], header["count"]), ("cloud", 2))
        with self.assertRaises(EOFError):
            unpack(stream.read)
        with self.assertRaises(ValueError):
            unpack(io.BytesIO(b"GET / HTTP/1.1\r\n").read)  # something else knocking on the port

    def test_relay_source_yields_posed_frames_with_registered_depth_and_camera(self):
        import socket
        from ultron.protocol.relay_wire import frame_message
        from ultron.protocol.sources import relay
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        frames = relay(port, host="127.0.0.1", log=lambda line: None)
        color = np.zeros((48, 64, 3), dtype=np.uint8)
        depth = np.full((48, 64), 2500, dtype=np.uint16)
        with socket.create_connection(("127.0.0.1", port), timeout=5) as robot:
            for stamp, pose in ((1.0, (0.0, 0.0, 0.0)), (2.0, (0.5, 0.0, 0.1))):
                robot.sendall(frame_message(stamp, color, depth, (50, 50, 32, 24), (50, 50, 32, 24), np.eye(3),
                                            [0, 0, 0], pose=pose))
        received = list(frames)  # ends when the relay disconnects
        self.assertEqual(received[-1].stamp, 2.0)
        last = received[-1]
        self.assertEqual((last.pose, last.image.size, last.depth.shape), ((0.5, 0.0, 0.1), (64, 48), (24, 32)))
        self.assertAlmostEqual(float(np.median(last.depth[last.depth > 0])), 2.5, places=2)
        self.assertEqual((last.camera.fx, last.camera.width), (50.0, 64))
        sighting = lift(last, Detection("ladder", CENTRE))  # no calibration given: the frame's camera is not used here
        self.assertIsNone(sighting.position)

    def test_survey_takes_the_camera_from_live_frames(self):
        with tempfile.TemporaryDirectory() as folder:
            survey = Survey(SeesEverything(), ObjectMap(folder), KeyframeStore(folder), Store(folder), interval=0)
            live = frame("ladder", (0.0, 0.0, 0.0), depth=3.0)
            live.camera = CALIBRATION
            self.assertEqual([o.id for o in survey.feed(live)], ["LAD-001"])
            self.assertIsNotNone(survey.objects.get("LAD-001").position)


class ContractNavigatorTest(unittest.TestCase):
    """Against the real navigation contract from ultron.autonomy, with a scripted backend instead of Nav2."""
    PARAMS = {"default_timeout_s": 30.0, "goal_xy_tolerance_m": 0.2, "goal_yaw_tolerance_rad": 0.2,
              "settle_speed_m_s": 0.05, "settle_yaw_rate_rad_s": 0.05, "settle_time_s": 1.0}
    REQUEST = ViewRequest("LAD-001", "detail", 0, 3.0, 1.0, 1.57, "no detail view yet")

    def setUp(self):
        try:
            from ultron.autonomy.navigation_contract import GoalTracker
        except ImportError:
            self.skipTest("ultron.autonomy not available")
        self.GoalTracker = GoalTracker

    def backend(self, nav2_codes, reject=None, settle=True):
        from ultron.pipeline.navigate import ContractNavigator
        tracker, codes, clock = self.GoalTracker(self.PARAMS), list(nav2_codes), [100.0]

        class Scripted:
            def __init__(self):
                self.tracker, self.goal = tracker, None

            def submit(self, goal):
                self.goal = goal
                return tracker.submit(goal, reject is None, reject or "OK", clock[0])

            def tick(self):
                clock[0] += 1.0
                if codes:
                    tracker.on_backend_status(self.goal.goal_id, codes.pop(0), clock[0])
                elif settle:
                    tracker.settle(clock[0], 0.0, 0.0, True, (3.0, 1.0, 1.57))

            def done(self):
                return tracker.status in ("REJECTED", "CANCELLED", "TIMEOUT", "FAILED") or (
                    tracker.status == "REACHED" and tracker.result.capture_ready) or clock[0] > 140

        scripted = Scripted()
        return ContractNavigator(scripted, self.PARAMS, "PATROL-1", "hall", 1, clock=lambda: clock[0]), scripted

    def test_reached_and_settled_counts_as_arrived(self):
        navigator, backend = self.backend([1, 2, 4])  # accepted, executing, succeeded
        self.assertTrue(navigator.go_to(self.REQUEST))
        goal = backend.goal
        self.assertEqual((goal.schema, goal.asset_id, goal.x, goal.y), ("ultron.navigation_goal/v1", "LAD-001", 3.0, 1.0))
        self.assertIn("detail view of LAD-001", goal.reason)
        self.assertTrue(navigator.results[0].capture_ready)

    def test_reached_but_never_settled_is_not_arrived(self):
        navigator, _ = self.backend([1, 4], settle=False)
        self.assertFalse(navigator.go_to(self.REQUEST))

    def test_rejected_aborted_and_cancelled_goals_fail(self):
        self.assertFalse(self.backend([], reject="TARGET_FOOTPRINT_OCCUPIED")[0].go_to(self.REQUEST))
        self.assertFalse(self.backend([1, 6])[0].go_to(self.REQUEST))
        self.assertFalse(self.backend([1, 5])[0].go_to(self.REQUEST))

    def test_free_space_comes_from_the_saved_map(self):
        try:
            import yaml  # noqa: F401
        except ImportError:
            self.skipTest("PyYAML not installed")
        from ultron.autonomy.navigation_contract import MapFootprint
        from ultron.pipeline.navigate import free_space
        with tempfile.TemporaryDirectory() as folder:
            grid = np.full((100, 100), 254, dtype=np.uint8)   # free
            grid[:, 60:] = 0                                  # a wall from x = 3 m
            Image.fromarray(grid).save(Path(folder, "map.pgm"))
            Path(folder, "map.yaml").write_text(
                "image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
            is_free = free_space(MapFootprint(Path(folder, "map.yaml")), 0.4)
            self.assertEqual((is_free(1.5, 2.5), is_free(2.8, 2.5), is_free(9.0, 2.5)), (True, False, False))


if __name__ == "__main__":
    unittest.main()
