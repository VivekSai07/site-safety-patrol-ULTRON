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
from ultron.pipeline.detect import ReplayDetector, VlmDetector, YoloDetector
from ultron.pipeline.inspection import Inspection
from ultron.pipeline.keyframes import KeyframeStore, view_name
from ultron.pipeline.lift import ASSUMED_RANGE, CELL, lift
from ultron.pipeline.navigate import DashboardNavigator, Passive
from ultron.pipeline.objects import ObjectMap
from ultron.pipeline.survey import Survey
from ultron.pipeline.types import Detection, Sighting, ViewRequest
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


def rectangle(cx, cy, length, width):
    """Outline of a floor rectangle as segments: `length` along x, `width` along y."""
    corners = [(cx - length / 2, cy - width / 2), (cx + length / 2, cy - width / 2),
               (cx + length / 2, cy + width / 2), (cx - length / 2, cy + width / 2)]
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


def render(pose, shapes, background=8.0):
    """Depth image (CALIBRATION camera at the robot origin) of upright `shapes` (lists of floor
    segments) standing in front of a wall, and each shape's outline in thousandths (None if unseen)."""
    x, y, yaw = pose
    depth = np.full((480, 640), background, dtype=np.float32)
    columns = [[] for _ in shapes]
    for u in range(640):
        angle = yaw - math.atan2(u + 0.5 - CALIBRATION.cx, CALIBRATION.fx)
        dx, dy = math.cos(angle), math.sin(angle)
        nearest = None
        for n, segments in enumerate(shapes):
            for (ax, ay), (bx, by) in segments:
                ex, ey = bx - ax, by - ay
                cross = dx * ey - dy * ex
                if abs(cross) < 1e-9:
                    continue
                t = ((ax - x) * ey - (ay - y) * ex) / cross
                s = ((ax - x) * dy - (ay - y) * dx) / cross
                if t > 0 and 0 <= s <= 1 and (nearest is None or t < nearest[0]):
                    nearest = (t, n)
        if nearest:
            depth[100:400, u] = nearest[0] * math.cos(angle - yaw)
            columns[nearest[1]].append(u)
    outlines = []
    for c in columns:  # a segmentation mask covers the widest stretch of the shape not hidden by another
        runs = np.split(np.asarray(c), np.nonzero(np.diff(c) > 1)[0] + 1) if c else []
        c = max(runs, key=len).tolist() if runs else []
        outlines.append([[c[0] * 1000 // 640, 208], [(c[-1] + 1) * 1000 // 640, 208], [(c[-1] + 1) * 1000 // 640, 833],
                         [c[0] * 1000 // 640, 833]] if c else None)
    return depth, outlines


class Rendered:
    """Detector for rendered frames: one segmented box per visible shape, of the shape's kind in
    `kinds` (ladder by default) and as tall as such an object looks, tracked by `tracks`."""

    def __init__(self, kind="ladder", tracks=None, kinds=None, missed=()):
        self.kind, self.tracks, self.kinds, self.missed = kind, tracks, kinds, missed

    def detect(self, frame):
        found = []
        for n, o in enumerate(frame.outlines):
            if not o or n in self.missed:
                continue
            kind = self.kinds[n] if self.kinds else self.kind
            if kind != "ladder":  # rows of a box as tall as this kind looks from where it stands
                column = int((o[0][0] + o[1][0]) / 2000 * 640)
                _, top, _, bottom = sized_box(kind, float(frame.depth[240, column]))
                o = [[o[0][0], top], [o[1][0], top], [o[1][0], bottom], [o[0][0], bottom]]
            found.append(Detection(kind, [o[0][0], o[0][1], o[2][0], o[2][1]], 0.9,
                                   None if self.tracks is None else self.tracks[n], o))
        return found


def rendered(name, pose, shapes, stamp=0.0):
    depth, outlines = render(pose, shapes)
    seen = Frame(Image.new("RGB", (640, 480)), name=name, stamp=stamp, pose=pose, depth=depth)
    seen.outlines = outlines
    return seen


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


class FootprintTest(unittest.TestCase):
    def test_the_footprint_lies_on_the_visible_surface(self):
        depth, (outline,) = render((0.0, 0.0, 0.0), [rectangle(3.25, 0.0, 0.5, 1.0)])
        seen = Frame(Image.new("RGB", (640, 480)), name="a", pose=(0.0, 0.0, 0.0), depth=depth)
        sighting = lift(seen, Detection("ladder", [0, 208, 1000, 833], outline=outline), CALIBRATION)
        xs = [i * CELL for i, _ in sighting.cells]
        ys = [j * CELL for _, j in sighting.cells]
        self.assertTrue(all(2.9 <= x <= 3.05 for x in xs))     # the near face, not the wall behind
        self.assertAlmostEqual(min(ys), -0.5, delta=0.11)
        self.assertAlmostEqual(max(ys), 0.4, delta=0.11)
        self.assertAlmostEqual(sighting.position[0], 3.0, delta=0.06)
        self.assertAlmostEqual(sighting.position[1], 0.0, delta=0.06)

    def test_background_seen_between_rungs_is_not_part_of_the_footprint(self):
        depth = np.full((480, 640), 2.8, dtype=np.float32)  # wall 0.8 m behind, within a ladder's extent
        depth[100:400:20, 280:360] = 2.0                    # rungs: every column also sees the wall between them
        seen = Frame(Image.new("RGB", (640, 480)), name="a", pose=(0.0, 0.0, 0.0), depth=depth)
        outline = [[440, 208], [560, 208], [560, 833], [440, 833]]
        sighting = lift(seen, Detection("ladder", [440, 208, 560, 833], outline=outline), CALIBRATION)
        self.assertTrue(all(1.9 <= i * CELL <= 2.05 for i, _ in sighting.cells), sighting.cells)

    def test_a_nearer_object_inside_the_outline_does_not_stretch_the_footprint(self):
        depth, (far, _) = render((0.0, 0.0, 0.0), [rectangle(5.0, 0.0, 0.2, 1.2), rectangle(2.5, 0.3, 0.2, 0.3)])
        seen = Frame(Image.new("RGB", (640, 480)), name="a", pose=(0.0, 0.0, 0.0), depth=depth)
        wide = [[far[0][0] - 40, 208], [far[1][0], 208], [far[1][0], 833], [far[0][0] - 40, 833]]  # sloppy mask
        sighting = lift(seen, Detection("ladder", [wide[0][0], 208, wide[1][0], 833], outline=wide), CALIBRATION)
        self.assertGreater(min(i for i, _ in sighting.cells) * CELL, 4.0)

    def test_depth_at_half_the_colour_resolution_lands_in_the_same_place(self):
        depth, (outline,) = render((1.0, 2.0, 0.5), [rectangle(3.5, 3.0, 0.4, 0.4)])
        full = lift(Frame(Image.new("RGB", (640, 480)), name="a", pose=(1.0, 2.0, 0.5), depth=depth),
                    Detection("fire_extinguisher", [0, 208, 1000, 833], outline=outline), CALIBRATION)
        half = lift(Frame(Image.new("RGB", (640, 480)), name="a", pose=(1.0, 2.0, 0.5), depth=depth[::2, ::2]),
                    Detection("fire_extinguisher", [0, 208, 1000, 833], outline=outline), CALIBRATION)
        self.assertLess(math.dist(full.position, half.position), 0.05)
        self.assertLess(math.dist(full.position, (3.5, 3.0)), 0.3)


def floor_scene(boards, height=0.8, wall=25.0):
    """Depth image (CALIBRATION camera `height` above a floor, looking along +x from the origin) of
    upright boards (distance ahead, left from, left to, bottom, top above the floor) and a wall."""
    v, u = np.mgrid[0:480, 0:640] + 0.5
    right, down = (u - CALIBRATION.cx) / CALIBRATION.fx, (v - CALIBRATION.cy) / CALIBRATION.fy
    depth = np.where(down * wall <= height, wall, np.inf)
    with np.errstate(divide="ignore"):
        depth = np.minimum(depth, np.where(down > 0, height / np.where(down > 0, down, 1), np.inf))
    for ahead, left_from, left_to, bottom, top in boards:
        hit = ((-right * ahead >= left_from) & (-right * ahead <= left_to)
               & (down * ahead >= height - top) & (down * ahead <= height - bottom))
        depth = np.where(hit, np.minimum(depth, ahead), depth)
    return np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)


def ladder_boards(ahead, half_width=0.25, top=2.5, rails=0.05, rungs=0.03):
    """A ladder seen face-on: two rails and a rung every 0.3 m, mostly gaps."""
    return ([(ahead, half_width - rails, half_width, 0.0, top), (ahead, -half_width, -half_width + rails, 0.0, top)]
            + [(ahead, -half_width, half_width, h, h + rungs) for h in np.arange(0.3, top, 0.3)])


def box_outline(ahead, left_from, left_to, bottom, top, height=0.8, below=0):
    """Outline (thousandths) of what lies between those bounds at `ahead`, reaching `below` rows lower."""
    columns = [CALIBRATION.cx - CALIBRATION.fx * left / ahead for left in (left_to, left_from)]
    rows = [CALIBRATION.cy + CALIBRATION.fy * (height - h) / ahead for h in (top, bottom)]
    x0, x1 = columns[0] * 1000 / 640, columns[1] * 1000 / 640
    y0, y1 = rows[0] * 1000 / 480, min(rows[1] + below, 479) * 1000 / 480
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


class FloorTest(unittest.TestCase):
    """Range, footprint and empty places with the floor and the far background in the picture."""

    def sighting(self, depth, outline):
        frame = Frame(Image.new("RGB", (640, 480)), name="a", pose=(0.0, 0.0, 0.0), depth=depth)
        box = [min(p[0] for p in outline), min(p[1] for p in outline), max(p[0] for p in outline), max(p[1] for p in outline)]
        return lift(frame, Detection("ladder", box, outline=outline), CALIBRATION)

    def test_a_far_ladder_is_placed_at_its_rails_not_at_the_background_seen_through_it(self):
        depth = floor_scene(ladder_boards(10.0))
        sighting = self.sighting(depth, box_outline(10.0, -0.3, 0.3, 0.0, 2.6, below=3))
        self.assertAlmostEqual(sighting.distance, 10.0, delta=0.3)
        self.assertLess(math.dist(sighting.position, (10.0, 0.0)), 0.3)

    def test_floor_inside_the_outline_does_not_pull_the_ladder_nearer(self):
        """A mask that took in the ladder's shadow down to the image's lower edge."""
        depth = floor_scene(ladder_boards(5.0))
        sighting = self.sighting(depth, box_outline(5.0, -0.3, 0.3, 0.0, 2.6, below=480))
        self.assertAlmostEqual(sighting.distance, 5.0, delta=0.2)
        self.assertLess(math.dist(sighting.position, (5.0, 0.0)), 0.3)

    def test_the_floor_where_an_object_stood_is_not_taken_for_the_object(self):
        """Its stored height band reaches below the floor (the box held its shadow): the rows below
        it see floor at its distance. The place is empty, not occupied."""
        from ultron.pipeline.moved import look_through
        from ultron.pipeline.types import SiteObject
        cells = {f"{i},{j}": 5 for i in range(33, 37) for j in range(-2, 2)}
        stored = SiteObject("LAD-001", "ladder", [3.5, 0.0], sightings=5, cells=cells, zband=[-1.1, 1.0])
        empty = Frame(Image.new("RGB", (640, 480)), name="a", pose=(0.0, 0.0, 0.0), depth=floor_scene([]))
        self.assertEqual(look_through(empty, CALIBRATION, stored), "empty")
        there = Frame(Image.new("RGB", (640, 480)), name="b", pose=(0.0, 0.0, 0.0), depth=floor_scene(ladder_boards(3.5)))
        self.assertEqual(look_through(there, CALIBRATION, stored), "occupied")


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
        # Only an object seen often enough is announced: one box may be a false hit or a part.
        self.assertEqual([e["object"] for e in self.store.events() if e["kind"] == "object_found"], ["LAD-001"])

    def test_two_boxes_in_one_frame_are_two_objects_even_when_close(self):
        class TwoLadders:
            def detect(self, frame):
                return [Detection("ladder", [300, 200, 480, 800], 0.9), Detection("ladder", [520, 200, 700, 800], 0.8)]
        survey = Survey(TwoLadders(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        for stamp in range(3):
            survey.feed(frame("pair", (0.0, 0.0, 0.0), float(stamp), depth=3.0))
        self.assertEqual([(o.id, o.sightings) for o in self.objects.all()], [("LAD-001", 3), ("LAD-002", 3)])
        self.assertLess(math.dist(self.objects.get("LAD-001").position, self.objects.get("LAD-002").position), 1.5)

    def test_an_object_seen_only_from_far_away_is_not_placed(self):
        """Beyond MAX_RANGE the depth camera's error is larger than the margin footprints are matched
        with: such sightings would split objects. A ladder seen from 14 m makes nothing; from 6 m it does."""
        class OneLadder:
            def detect(self, frame):
                return [Detection("ladder", [450, 400, 550, 623], 0.9)]  # 3 m tall at 14 m, 1.3 m at 6 m
        survey = Survey(OneLadder(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        for stamp in range(4):
            survey.feed(frame("far", (0.0, 0.0, 0.0), float(stamp), depth=14.0))
        self.assertEqual(self.objects.all(), [])
        survey.feed(frame("near", (0.0, 0.0, 0.0), 10.0, depth=6.0))
        self.assertEqual([o.id for o in self.objects.all()], ["LAD-001"])

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

    def test_a_wide_object_seen_from_its_side_and_its_front_is_one_object(self):
        """An A-frame 1.5 m long: its visible faces from two directions lie 0.8 m apart, beyond any
        radius that keeps nearby ladders apart. Their footprints touch."""
        ladder = [rectangle(5.0, 5.0, 1.5, 0.5)]
        survey = Survey(Rendered(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        for stamp, side in enumerate((0, 0, 0, 2, 2, 2)):
            survey.feed(rendered("a", facing((5.0, 5.0), side, 3.0), ladder, float(stamp)))
        self.assertEqual([(o.id, o.sightings) for o in self.objects.all()], [("LAD-001", 6)])
        position = self.objects.get("LAD-001").position
        self.assertTrue(4.25 <= position[0] <= 5.75 and 4.75 <= position[1] <= 5.3, position)

    def test_ladders_a_metre_apart_seen_one_at_a_time_stay_two(self):
        left, right = rectangle(5.0, 5.0, 0.3, 1.0), rectangle(5.0, 7.0, 0.3, 1.0)
        survey = Survey(Rendered(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        for stamp in range(3):
            survey.feed(rendered("left", facing((5.0, 5.0), 0, 3.0), [left], float(stamp)))
        for stamp in range(3, 6):
            survey.feed(rendered("right", facing((5.0, 7.0), 0, 3.0), [right], float(stamp)))
        self.assertEqual([(o.id, o.sightings) for o in self.objects.all()], [("LAD-001", 3), ("LAD-002", 3)])
        self.assertEqual(self.survey.tidy(), {})

    def test_touching_footprints_in_one_frame_are_two_objects_and_tracks_keep_them_apart(self):
        pair = [rectangle(5.0, 4.8, 0.3, 0.3), rectangle(5.0, 5.2, 0.3, 0.3)]
        survey = Survey(Rendered(tracks=[1, 2]), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        for stamp in range(3):
            survey.feed(rendered("pair", facing((5.0, 5.0), 0, 3.0), pair, float(stamp)))
        survey.detector = Rendered(tracks=[2])  # now only the second, the tracker still knows it
        for stamp in range(3, 6):
            survey.feed(rendered("one", facing((5.0, 5.0), 0, 3.0), pair[1:], float(stamp)))
        self.assertEqual([(o.id, o.sightings) for o in self.objects.all()], [("LAD-001", 3), ("LAD-002", 6)])
        self.assertEqual(survey.tidy(), {})

    def test_stray_cells_do_not_grow_a_footprint_over_many_sightings(self):
        from ultron.pipeline.objects import footprint
        body = [[i, j] for i in range(10, 13) for j in range(0, 5)]
        for stamp in range(200):  # every tenth view adds a stray cell twice, wandering off
            stray = [[13 + stamp // 20, 0]] if stamp % 10 < 2 else []
            self.objects.observe(Sighting(Detection("ladder", CENTRE), (1.15, 0.2), 3.0, cells=body + stray), float(stamp))
        cells = footprint(self.objects.get("LAD-001"))
        self.assertEqual(sorted(map(tuple, cells.tolist())), sorted(map(tuple, body)))

    def test_a_merged_object_brings_its_footprint_and_frees_no_id_for_reuse(self):
        def sighting(x, cells):
            return Sighting(Detection("ladder", CENTRE), (x, 0.0), 3.0, cells=cells)
        first = [[i, j] for i in range(10, 13) for j in range(0, 5)]
        for stamp in range(6):
            self.objects.observe(sighting(1.15, first), float(stamp))
        fragment = self.objects.observe(sighting(1.5, [[15, 2], [16, 2]]), 7.0)  # beside it, seen in a frame of its own
        far = self.objects.observe(sighting(9.0, [[90, 0], [90, 1]]), 8.0)
        self.assertEqual((fragment.id, far.id), ("LAD-001", "LAD-002"))  # within the margin: the same ladder
        self.objects.update("LAD-001", cells={})  # forget the footprint: the next one starts its own
        self.objects._objects["LAD-003"] = self.objects._objects.pop("LAD-002")
        self.objects._objects["LAD-003"].id = "LAD-003"
        new = self.objects.observe(sighting(20.0, [[200, 0]]), 9.0)
        self.assertNotIn(new.id, ("LAD-001", "LAD-003"))

    def posted_events(self):
        """Event types and asset ids the dashboard would receive from this workspace's store."""
        from ultron.protocol.publish import Publisher
        posted = []

        def post(path, body=None, raw=None):
            if path == "/api/events":
                posted.append((body["type"], body["asset_id"], body["data"].get("note")))
            return 201, ""
        self.store.subscribe(Publisher(self.store, post, "PATROL-1", "site-a", log=lambda *_: None).on_event)
        return posted

    def test_a_part_boxed_beside_the_whole_never_reaches_the_dashboard(self):
        posted = self.posted_events()

        class Halves:
            def detect(self, frame):
                whole = [Detection("ladder", [300, 200, 600, 800], 0.9, track=1)]
                # The half box is tracked for four frames: enough sightings, but a fragment of the whole.
                return whole + ([Detection("ladder", [560, 200, 640, 800], 0.6, track=2)] if 10 <= frame.stamp < 14 else [])
        survey = Survey(Halves(), self.objects, self.keyframes, self.store, CALIBRATION, interval=0)
        for stamp in range(20):
            survey.feed(frame("x", (0.0, 0.0, 0.0), float(stamp), depth=3.0))
        views = len(self.keyframes.for_object("LAD-001")) + len(self.keyframes.for_object("LAD-002"))
        self.assertEqual(survey.tidy(), {"LAD-002": "LAD-001"})
        self.assertEqual([(kind, asset) for kind, asset, _ in posted if kind.startswith("OBJECT_")], [("OBJECT_FOUND", "LAD-001")])
        self.assertFalse(any("LAD-002" in (note or "") for _, _, note in posted))
        self.assertEqual(len(self.keyframes.for_object("LAD-001")), views)  # the part's views joined the whole

    def test_an_announced_object_keeps_its_id_when_a_better_seen_one_is_folded_into_it(self):
        posted = self.posted_events()
        for stamp in range(3):
            self.see("ladder", (5.0, 5.0), 0, 3.0, float(stamp))       # announced as LAD-001
        for stamp in range(3, 23):
            self.see("ladder", (5.9, 5.0), 0, 3.0, float(stamp))       # 0.9 m on, seen far more often
        self.assertEqual(self.survey.tidy(), {"LAD-002": "LAD-001"})
        self.assertEqual([(o.id, o.sightings) for o in self.objects.all()], [("LAD-001", 23)])
        self.assertEqual([(kind, asset) for kind, asset, _ in posted if kind.startswith("OBJECT_")], [("OBJECT_FOUND", "LAD-001")])

    def test_a_merge_of_an_announced_object_is_forwarded(self):
        posted = self.posted_events()
        for stamp in range(20):
            self.see("ladder", (5.0, 5.0), 0, 3.0, float(stamp))
        for stamp in range(20, 23):
            self.see("ladder", (5.0, 6.2), 0, 3.0, float(stamp))       # a fragment seen three times
        self.objects.update("LAD-002", sightings=4)                    # ... before the strong one had a footprint to tell
        self.survey.announced.add("LAD-002")
        self.store.emit("object_found", object="LAD-002", object_kind="ladder")
        self.assertEqual(self.survey.tidy(), {"LAD-002": "LAD-001"})
        self.assertIn(("OBJECT_MERGED", "LAD-002"), [(kind, asset) for kind, asset, _ in posted])

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


class TwinsTest(unittest.TestCase):
    """Two identical ladders 0.3 m apart, once seen together (so known to be two)."""

    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.objects = ObjectMap(folder.name)

    @staticmethod
    def sighting(first_i, shift=0):
        """A ladder covering 0.5 m x 1 m of floor from cell (first_i, 100), moved `shift` cells along x."""
        cells = [[i + shift, j] for i in range(first_i, first_i + 5) for j in range(100, 110)]
        return Sighting(Detection("ladder", [400, 200, 600, 800], 0.9), (((first_i + 2.5 + shift) * CELL), 10.5), 3.0,
                        cells=cells)

    def test_a_twin_seen_alone_is_matched_to_the_one_it_lies_on(self):
        for stamp in range(10):  # both in view together
            left = self.objects.observe(self.sighting(100), stamp)
            right = self.objects.observe(self.sighting(108), stamp, exclude={left.id})
            self.objects.seen_together([left.id, right.id])
        self.assertEqual((left.id, right.id), ("LAD-001", "LAD-002"))
        # Seen together from further away, each picked up a few cells of the other (as on the
        # simulated yard): now their footprints overlap, though thinly where they overlap.
        for one, other in (("LAD-001", 108), ("LAD-002", 100)):
            cells = dict(self.objects.get(one).cells)
            cells.update({f"{i},{j}": 2 for i in range(other, other + 5) for j in range(100, 110)})
            self.objects.update(one, cells=cells)
        # Alone, and 0.1 m off as depth noise puts it: within reach of both, but on the right one.
        self.assertEqual(self.objects.observe(self.sighting(108, shift=-1), 20).id, "LAD-002")
        self.assertEqual(self.objects.observe(self.sighting(100, shift=1), 21).id, "LAD-001")
        # Halfway between them nothing says which: neither, and no third ladder.
        self.assertIsNone(self.objects.observe(self.sighting(104), 22))
        self.assertEqual(len(self.objects.all()), 2)


class SiteMapTest(unittest.TestCase):
    """One object map per site and map: a second patrol re-observes what the first one found."""

    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name)
        self.ladder, self.extinguisher = [rectangle(5.0, 5.0, 1.5, 0.5)], [rectangle(8.0, 2.0, 0.3, 0.3)]

    def patrol(self, run, stamps, sides, map_id="hall-v1", pose_frame="map"):
        """One run: its own store (events, mission), the shared object map, a fresh tracker."""
        from ultron.pipeline.__main__ import object_folder
        import argparse
        args = argparse.Namespace(map_id=map_id, map=None, sites=str(self.root / "sites"), site="hall",
                                  data=str(self.root / run))
        folder = object_folder(args)
        store = Store(self.root / run)
        survey = Survey(Rendered(tracks=[1]), ObjectMap(folder), KeyframeStore(folder), store,
                        CALIBRATION, interval=0, map_id=map_id)
        for stamp, side in zip(stamps, sides):
            seen = rendered("a", facing((5.0, 5.0), side, 3.0), self.ladder, float(stamp))
            seen.pose_frame = pose_frame
            survey.feed(seen)
        survey.tidy()
        return survey, store

    def test_a_second_patrol_on_the_same_map_finds_the_same_object_under_the_same_id(self):
        first, _ = self.patrol("run-1", range(4), [0, 0, 1, 1])
        second, store = self.patrol("run-2", range(100, 104), [2, 2, 3, 3])  # other directions, other start
        self.assertEqual([(o.id, o.sightings) for o in second.objects.all()], [("LAD-001", 8)])
        # Each patrol is its own mission on the dashboard, so each announces what it saw. One stored
        # object cannot line a patrol up, so it is reported when the survey settles.
        found = [e for e in store.events(limit=None) if e["kind"] == "object_found"]
        self.assertEqual([(e["object"], e["sightings"]) for e in found], [("LAD-001", 8)])
        self.assertFalse(any(e["kind"].startswith("localisation_") for e in store.events(limit=None)))
        self.assertTrue((self.root / "sites" / "hall" / "hall-v1" / "objects.json").is_file())

    def test_each_patrol_inspects_what_it_saw_again(self):
        def inspect(survey, store, mission):
            Inspection(survey.objects, survey.keyframes, FormVlm(), store, survey, Passive(), mission=mission,
                       by_section=False).run()
            return [e["object"] for e in store.events(limit=None) if e["kind"] == "inspecting"]
        first, store = self.patrol("run-1", range(4), [0, 0, 1, 1])
        self.assertEqual(inspect(first, store, "run-1"), ["LAD-001"])
        second, store = self.patrol("run-2", range(100, 104), [2, 2, 3, 3])
        self.assertEqual(inspect(second, store, "run-2"), ["LAD-001"])
        third, store = self.patrol("run-3", [], [])  # saw nothing: nothing to inspect from old frames
        self.assertEqual(inspect(third, store, "run-3"), [])
        self.assertEqual(third.objects.get("LAD-001").result["mission"], "run-2")

    def test_without_a_map_id_each_run_keeps_its_own_objects(self):
        self.patrol("run-1", range(4), [0, 0, 1, 1], map_id=None)
        second, _ = self.patrol("run-2", range(100, 104), [2, 2, 3, 3], map_id=None)
        self.assertEqual([(o.id, o.sightings) for o in second.objects.all()], [("LAD-001", 4)])

    def test_a_known_object_this_patrol_does_not_see_is_not_reported(self):
        self.patrol("run-1", range(4), [0, 0, 1, 1])
        second, store = self.patrol("run-2", [], [])
        self.assertEqual([e for e in store.events(limit=None) if e["kind"] == "object_found"], [])

    def test_odometry_poses_on_a_shared_map_are_reported(self):
        _, store = self.patrol("run-1", range(4), [0, 0, 1, 1], pose_frame="odometry")
        self.assertEqual([e["kind"] for e in store.events(limit=None)].count("localisation_unconfirmed"), 1)

    def test_the_map_id_comes_from_the_saved_map(self):
        from ultron.pipeline.__main__ import object_folder
        import argparse, json
        saved = self.root / "map"
        saved.mkdir()
        (saved / "map_meta.json").write_text(json.dumps({"map_id": "ctrl-hall", "map_version": 3}))
        args = argparse.Namespace(map_id=None, map=str(saved), sites="sites", site="hall", data="run")
        self.assertEqual(object_folder(args), Path("sites/hall/ctrl-hall-v3"))
        self.assertEqual(object_folder(argparse.Namespace(map_id=None, map=None, sites="s", site="h", data="run")), Path("run"))


class AnchoringTest(unittest.TestCase):
    """A later patrol on a shared map is lined up with the objects stored by an earlier one."""
    LADDER, LEFT, RIGHT, NEW = (5.0, 5.0), (8.0, 2.0), (2.0, 8.0), (8.0, 8.0)

    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name)
        self.folder = self.root / "sites" / "hall" / "hall-v1"

    def shapes(self, moved=None):
        places = [self.LADDER, self.LEFT, self.RIGHT, self.NEW]
        if moved:
            places[1] = moved
        # Index 4: a wall 0.35 m behind RIGHT as the robot looks at it; nothing the detector finds.
        return ([rectangle(*self.LADDER, 1.5, 0.5)] + [rectangle(x, y, 0.3, 0.3) for x, y in places[1:]]
                + [rectangle(self.RIGHT[0] + 0.35, self.RIGHT[1], 0.1, 4.0)],
                ["ladder", "fire_extinguisher", "fire_extinguisher", "fire_extinguisher", None])

    def patrol(self, run, targets, error=(0.0, 0.0, 0.0), moved=None, new=True, gone=(), missed=(), wall=False):
        """Look at each target from three nearby directions; the robot believes it stands where
        `error` (angle, dx, dy about the origin) puts it. The new object exists from run 2 on."""
        from ultron.pipeline.anchor import apply
        shapes, kinds = self.shapes(moved)
        present = [n for n in (range(4) if new and run != "run-1" else range(3)) if n not in gone] + [4] * wall
        store = Store(self.root / run)
        survey = Survey(Rendered(kinds=kinds, missed=(*missed, 4)), ObjectMap(self.folder), KeyframeStore(self.folder),
                        store, CALIBRATION, interval=0, map_id="hall-v1")
        stamp = 0.0
        for target in targets:
            for side in (3, 4, 5) * 2:
                true = facing(target, side, 2.5)
                believed = (*apply(error, true[:2]), true[2] + error[0])
                seen = rendered("a", believed, [], stamp)
                seen.depth, seen.outlines = render(true, [shapes[n] if n in present else [] for n in range(5)])
                survey.feed(seen)
                stamp += 1.0
        merged = survey.tidy()
        return survey, store, merged

    def events(self, store, kind):
        return [e for e in store.events(limit=None) if e["kind"] == kind]

    def first_patrol(self):
        self.patrol("run-1", [self.LADDER, self.LEFT, self.RIGHT])
        objects = ObjectMap(self.folder).all()
        self.assertEqual(sorted(o.id for o in objects), ["FEX-001", "FEX-002", "LAD-001"])
        return {o.id: o.position for o in objects}

    def test_a_patrol_found_off_in_the_middle_of_a_frame_drops_what_it_made(self):
        """In one frame two new objects are seen from a pose that is off, then a stored one seen in
        the same frame shows the robot is off: the new objects go, and the frame ends cleanly."""
        from ultron.pipeline.anchor import apply
        self.first_patrol()
        new, error = (8.0, 4.6), (0.0, 0.0, 1.5)  # across the ladder; 2.6 m from LEFT: nothing stored it could be
        shapes = [rectangle(*self.LADDER, 1.5, 0.5), rectangle(*new, 0.3, 0.3), rectangle(new[0] + 1.2, new[1], 0.3, 0.3),
                  rectangle(*self.LEFT, 0.3, 0.3)]
        store = Store(self.root / "run-2")
        survey = Survey(Rendered(kinds=["ladder"] + ["fire_extinguisher"] * 3), ObjectMap(self.folder),
                        KeyframeStore(self.folder), store, CALIBRATION, interval=0, map_id="hall-v1")
        stamp = 0.0
        for target, sides in ((self.LADDER, (3, 4, 5) * 2), ((8.5, 3.3), (4,) * 4)):  # then all three in view
            for side in sides:
                true = facing(target, side, 2.5)
                seen = rendered("a", (*apply(error, true[:2]), true[2]), [], stamp)
                seen.depth, seen.outlines = render(true, shapes)
                survey.feed(seen)
                stamp += 1.0
        survey.tidy()
        self.assertEqual(len(self.events(store, "localisation_suspect")), 1)
        self.assertEqual(sorted(o.id for o in survey.objects.all()), ["FEX-001", "FEX-002", "LAD-001"])
        self.assertEqual(self.events(store, "object_found"), [])  # not even the stored ones it re-saw

    def test_a_drifted_patrol_is_lined_up_and_places_a_new_object_where_it_is(self):
        before = self.first_patrol()
        error = (math.radians(3.0), 0.3, -0.2)
        survey, store, _ = self.patrol("run-2", [self.LEFT, self.RIGHT, self.LADDER, self.NEW], error)
        self.assertEqual(len(self.events(store, "localisation_anchored")), 1)
        self.assertEqual(self.events(store, "localisation_suspect"), [])
        objects = {o.id: o for o in survey.objects.all()}
        self.assertEqual(sorted(objects), ["FEX-001", "FEX-002", "FEX-003", "LAD-001"])
        new = objects["FEX-003"].position
        self.assertLess(math.dist(new, self.NEW), 0.25, new)          # uncorrected it would be about 0.7 m off
        for object_id, position in before.items():
            self.assertLess(math.dist(objects[object_id].position, position), 0.25, object_id)

    def test_a_patrol_far_off_its_map_creates_and_merges_nothing(self):
        before = self.first_patrol()
        survey, store, merged = self.patrol("run-2", [self.LEFT, self.RIGHT, self.LADDER, self.NEW], (0.0, 1.5, 0.0))
        suspect = self.events(store, "localisation_suspect")
        self.assertEqual(len(suspect), 1)
        self.assertGreater(suspect[0]["shift"], 1.0)
        self.assertEqual(merged, {})
        self.assertEqual({o.id: o.position for o in survey.objects.all()}, before)

    def test_two_stored_objects_that_disagree_make_no_decision(self):
        """One of them moved, or the robot is off: two cannot tell which. Nothing is changed."""
        before = self.first_patrol()
        survey, store, merged = self.patrol("run-2", [self.LEFT, self.LADDER], moved=(8.0, 3.5), new=False)
        self.assertEqual([e for e in store.events(limit=None) if e["kind"].startswith(("localisation", "object_moved"))], [])
        self.assertEqual({o.id: o.position for o in survey.objects.all()}, before)

    def test_an_object_moved_and_seen_first_is_told_from_a_robot_that_is_off(self):
        """The extinguisher stands 1.5 m from its stored place and is the first thing seen; the two
        other objects agree with each other: it moved, the robot is fine."""
        self.first_patrol()
        survey, store, _ = self.patrol("run-2", [self.LEFT, self.LADDER, self.RIGHT, self.LEFT], moved=(8.0, 3.5), new=False)
        self.assertEqual(self.events(store, "localisation_suspect"), [])
        self.assertEqual(len(self.events(store, "localisation_anchored")), 1)
        moved = self.events(store, "object_moved")
        self.assertEqual([e["object"] for e in moved], ["FEX-001"])
        self.assertLess(math.dist(moved[0]["seen_at"], (8.0, 3.5)), 0.3)
        # After the move is known, the extinguisher at its new place is an object of its own.
        objects = {o.id: o for o in survey.objects.all()}
        self.assertEqual(objects["FEX-001"].status, "MOVED")
        self.assertTrue(any(o.status == "FOUND" and o.kind == "fire_extinguisher"
                            and math.dist(o.position, (8.0, 3.5)) < 0.3 for o in objects.values()))

    def test_an_object_taken_away_is_marked_moved_not_left_silently(self):
        self.first_patrol()
        survey, store, _ = self.patrol("run-2", [self.LEFT, self.LADDER, self.RIGHT], new=False, gone=[2])
        self.assertEqual([e["object"] for e in self.events(store, "object_moved")], ["FEX-002"])
        self.assertEqual({o.id: o.status for o in survey.objects.all()},
                         {"LAD-001": "FOUND", "FEX-001": "FOUND", "FEX-002": "MOVED"})
        # ... and when it is back in its place, it is found again under its id.
        survey, store, _ = self.patrol("run-3", [self.LEFT, self.LADDER, self.RIGHT], new=False)
        self.assertEqual([e["object"] for e in self.events(store, "object_back")], ["FEX-002"])
        self.assertEqual(len(survey.objects.all()), 3)
        self.assertEqual(survey.objects.get("FEX-002").status, "FOUND")

    def test_an_object_the_detector_misses_is_not_marked_moved_without_a_record_of_finding_it(self):
        """Present (its place looks occupied) and missed, with too few earlier views to know the
        detector finds it from these sides. With such a record, missing it would mean gone."""
        self.first_patrol()
        survey, store, _ = self.patrol("run-2", [self.LEFT, self.LADDER, self.RIGHT], new=False, missed=[2])
        self.assertEqual(self.events(store, "object_moved"), [])
        self.assertEqual(survey.objects.get("FEX-002").status, "FOUND")

    def test_an_object_taken_from_in_front_of_a_wall_is_marked_moved(self):
        """The wall stands where the extinguisher stood: its place never looks empty. Found in
        every clear view from these sides on two patrols, then in none from three places: gone."""
        self.first_patrol()
        self.patrol("run-1b", [self.LEFT, self.LADDER, self.RIGHT, self.RIGHT], new=False)
        sides = ObjectMap(self.folder).get("FEX-002").sides
        self.assertTrue(sides and all(views == detected >= 5 for views, detected in sides.values()), sides)
        survey, store, _ = self.patrol("run-2", [self.LEFT, self.LADDER, self.RIGHT, self.RIGHT], new=False,
                                       gone=[2], wall=True)
        moved = self.events(store, "object_moved")
        self.assertEqual([(e["object"], e["reason"]) for e in moved], [("FEX-002", "not detected in clear view")])
        self.assertEqual(survey.objects.get("FEX-001").status, "FOUND")

    def test_a_wall_backed_object_seen_once_is_not_marked_moved(self):
        """Too few earlier views to know the detector finds it: not finding it says nothing."""
        self.first_patrol()
        survey, store, _ = self.patrol("run-2", [self.LEFT, self.LADDER, self.RIGHT, self.RIGHT], new=False,
                                       gone=[2], wall=True)
        self.assertEqual(self.events(store, "object_moved"), [])

    def test_nothing_is_marked_moved_before_the_patrol_is_lined_up(self):
        self.first_patrol()
        survey, store, _ = self.patrol("run-2", [self.RIGHT, self.LEFT], new=False, gone=[2])
        self.assertEqual(self.events(store, "object_moved"), [])  # one landmark seen before the empty place

    def test_the_same_patrol_again_lines_up_without_correction(self):
        self.first_patrol()
        survey, store, _ = self.patrol("run-2", [self.LADDER, self.LEFT, self.RIGHT], new=False)
        anchored = self.events(store, "localisation_anchored")
        self.assertEqual(len(anchored), 1)
        # A 0.3 m object seen from one side lies up to half its size from its stored middle.
        self.assertLess(anchored[0]["shift"], 0.15)
        self.assertEqual(len(survey.objects.all()), 3)


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

    def exported(self, names):
        """A YoloDetector around an exported model whose classes are `names` (no ultralytics needed)."""
        detector = YoloDetector.__new__(YoloDetector)
        detector.exported, detector.model = True, unittest.mock.Mock(names=dict(enumerate(names)))
        return detector

    def test_an_exported_detector_maps_its_baked_prompts_back_to_kinds(self):
        prompts = {"ladder": {"prompt": "ladder", "min_score": 0.3},
                   "fire_extinguisher": {"prompt": "red fire extinguisher", "min_score": 0.5}}
        detector = self.exported(["red fire extinguisher", "ladder"])
        detector.use(prompts)
        self.assertEqual((detector.kinds, detector.lowest), (["fire_extinguisher", "ladder"], 0.3))

    def test_an_exported_detector_refuses_prompts_it_was_not_made_for(self):
        detector = self.exported(["ladder", "fire extinguisher"])
        with self.assertRaisesRegex(ValueError, "export it again"):
            detector.use({"ladder": {"prompt": "ladder", "min_score": 0.3},
                          "fire_extinguisher": {"prompt": "red fire extinguisher", "min_score": 0.5}})

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
