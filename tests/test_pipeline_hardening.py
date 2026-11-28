"""Hardening for a tight indoor demo room: ladders lying on the floor, windows, lidar candidates.

Scenes are rendered: a camera 0.8 m above a floor, boxes standing or lying on it, and depth noise
as the D457's (6 mm at 2 m, 97 mm at 8 m: sigma = 0.0015 z^2). No real recording of a lying ladder
exists yet.
"""
import math
import tempfile
import unittest
from pathlib import Path

try:
    import numpy as np
    from PIL import Image
except ImportError:
    raise unittest.SkipTest("Pillow and numpy are required for the pipeline package")

from ultron.pipeline.catalog import KINDS
from ultron.pipeline.keyframes import KeyframeStore
from ultron.pipeline.lift import CELL, doubt, lift, lying
from ultron.pipeline.objects import ObjectMap, footprint
from ultron.pipeline.survey import Survey
from ultron.pipeline.types import Detection
from ultron.protocol.frames import Frame
from ultron.protocol.store import Store
from tests.test_pipeline import CALIBRATION

HEIGHT = 0.8  # camera above the floor


def ladder_parts(length=3.0, width=0.45, thick=0.07, standing=False):
    """A ladder as boxes in its own frame (x along it, y across, z up): two rails and a rung
    every 0.28 m. Lying it is `thick` high; standing it is a face-on ladder `length` tall."""
    if standing:
        rails = [(0.0, s * (width / 2 - 0.03), 0.06, 0.06, 0.0, length) for s in (-1, 1)]
        return rails + [(0.0, 0.0, 0.03, width, h, h + 0.03) for h in np.arange(0.28, length, 0.28)]
    rails = [(0.0, s * (width / 2 - 0.03), length, 0.06, 0.0, thick) for s in (-1, 1)]
    return rails + [(a, 0.0, 0.03, width, 0.01, thick - 0.01) for a in np.arange(-length / 2 + 0.14, length / 2, 0.28)]


def place(parts, x, y, yaw):
    """Boxes (centre x, centre y, length, width, yaw, bottom, top) in the robot's frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * a - s * b, y + s * a + c * b, length, width, yaw, bottom, top)
            for a, b, length, width, bottom, top in parts]


SIM_NOISE = 0.0015   # sigma = SIM_NOISE * z^2: the D457 as fitted by the simulator
FLOOR_NOISE = 0.0065  # as wide as the hall recording's floor about its plane (sd 0.9 cm at 1.5 m, 2.6 cm at 5 m)


def scene(boxes, wall=12.0, noise=SIM_NOISE, seed=0):
    """Depth image of the CALIBRATION camera HEIGHT above the floor, looking along +x from the
    origin, at oriented boxes and a wall `wall` metres ahead."""
    v, u = np.mgrid[0:480, 0:640] + 0.5
    right, down = (u - CALIBRATION.cx) / CALIBRATION.fx, (v - CALIBRATION.cy) / CALIBRATION.fy
    with np.errstate(divide="ignore"):
        depth = np.where(down > 0, HEIGHT / np.where(down > 0, down, 1), np.inf)
    depth = np.minimum(depth, np.where(HEIGHT - down * wall >= 0, wall, np.inf))
    # Ray: origin (0, 0, HEIGHT), direction (1, -right, -down) per metre of depth.
    for cx, cy, length, width, yaw, bottom, top in boxes:
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy = c * -cx + s * -cy, -s * -cx + c * -cy  # origin in the box frame
        dx, dy = c + s * -right, -s + c * -right
        near, far = np.full(depth.shape, -np.inf), np.full(depth.shape, np.inf)
        for o, d, low, high in ((ox, dx, -length / 2, length / 2), (oy, dy, -width / 2, width / 2),
                                (HEIGHT, -down, bottom, top)):
            with np.errstate(divide="ignore", invalid="ignore"):
                t1, t2 = (low - o) / d, (high - o) / d
            inside = (o >= low) & (o <= high)
            parallel = np.abs(d) < 1e-12
            t1 = np.where(parallel, np.where(inside, -np.inf, np.inf), t1)
            t2 = np.where(parallel, np.inf, t2)  # parallel: all of the ray inside the slab, or none
            near, far = np.maximum(near, np.minimum(t1, t2)), np.minimum(far, np.maximum(t1, t2))
        hit = (far >= near) & (near > 0)
        depth = np.where(hit, np.minimum(depth, near), depth)
    if noise:
        finite = np.isfinite(depth)
        depth[finite] += np.random.default_rng(seed).normal(0, 1, finite.sum()) * noise * depth[finite] ** 2
    return np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)


def outline(boxes):
    """Segmentation outline (thousandths) a detector would draw: the convex hull of the corners."""
    points = []
    for cx, cy, length, width, yaw, bottom, top in boxes:
        c, s = math.cos(yaw), math.sin(yaw)
        for a in (-length / 2, length / 2):
            for b in (-width / 2, width / 2):
                x, y = cx + c * a - s * b, cy + s * a + c * b
                for z in (bottom, top):
                    if x > 0.1:
                        u = CALIBRATION.cx - CALIBRATION.fx * y / x
                        v = CALIBRATION.cy + CALIBRATION.fy * (HEIGHT - z) / x
                        points.append((min(max(u, 0), 639) * 1000 / 640, min(max(v, 0), 479) * 1000 / 480))
    points = sorted(set(points))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    hull = []
    for sequence in (points, points[::-1]):
        part = []
        for p in sequence:
            while len(part) >= 2 and cross(part[-2], part[-1], p) <= 0:
                part.pop()
            part.append(p)
        hull += part[:-1]
    return [[round(x), round(y)] for x, y in hull]


def ladder_cells(x, y, yaw, length=3.0, width=0.45):
    """Floor cells under a lying ladder."""
    c, s = math.cos(yaw), math.sin(yaw)
    a, b = np.meshgrid(np.arange(-length / 2, length / 2, 0.02), np.arange(-width / 2, width / 2, 0.02))
    xs, ys = x + c * a - s * b, y + s * a + c * b
    return {(int(i), int(j)) for i, j in zip(np.floor(xs.ravel() / CELL), np.floor(ys.ravel() / CELL))}


def in_view(x, y):
    """Whether the floor at (x, y) in the robot's frame is in the image."""
    return x > 0.1 and 0 <= CALIBRATION.cx - CALIBRATION.fx * y / x < 640 and CALIBRATION.cy + CALIBRATION.fy * HEIGHT / x < 480


def seen(name, boxes, pose=(0.0, 0.0, 0.0), seed=0, stamp=0.0, noise=SIM_NOISE):
    frame = Frame(Image.new("RGB", (640, 480)), name=name, stamp=stamp, pose=pose,
                  depth=scene(boxes, noise=noise, seed=seed))
    frame.outlines = [outline(boxes)]
    return frame


def detection(boxes, kind="ladder"):
    o = outline(boxes)
    box = [min(p[0] for p in o), min(p[1] for p in o), max(p[0] for p in o), max(p[1] for p in o)]
    return Detection(kind, box, 0.9, None, o)


class LyingLadderTest(unittest.TestCase):
    def check(self, x, y, yaw, length=3.0, seed=0, noise=FLOOR_NOISE):
        boxes = place(ladder_parts(length), x, y, yaw)
        frame = seen("a", boxes, seed=seed, noise=noise)
        sighting = lift(frame, detection(boxes), CALIBRATION)
        self.assertTrue(sighting.lying, (x, y, yaw))
        self.assertIsNone(sighting.height)  # the size check must not judge its foreshortened box
        truth = ladder_cells(x, y, yaw, length)
        found = {tuple(c) for c in sighting.cells}
        near = {(i + di, j + dj) for i, j in truth for di in (-1, 0, 1) for dj in (-1, 0, 1)}
        # The part of the ladder in view is covered, and the cells found lie on it (within a cell)
        # or past it by no more than the floor a mask takes in beyond its top edge (at most the
        # ladder's 7 cm thickness times range over camera height: 0.5 m at 6 m), plus two cells.
        visible = {(i, j) for i, j in truth if in_view((i + 0.5) * CELL, (j + 0.5) * CELL)}
        self.assertGreater(len(visible & {(i + di, j + dj) for i, j in found for di in (-1, 0, 1) for dj in (-1, 0, 1)})
                           / len(visible), 0.9, (x, y, yaw))
        stray = [c for c in found if c not in near]
        far_stray = [c for c in stray if min(math.dist(c, t) for t in truth)
                     > 2 + 0.07 * math.hypot(*c) * CELL / HEIGHT / CELL]
        self.assertFalse(far_stray, (x, y, yaw))
        # Beyond about 5 m the floor's noise hides which pixels are raised, and that overshoot is a
        # third of a narrow ladder's cells.
        self.assertLess(len(stray) / len(found), 0.25 if x <= 5 else 0.4, (x, y, yaw))
        return sighting

    def test_a_ladder_lying_across_the_view_is_found_lying_with_its_whole_footprint(self):
        for ahead in (1.5, 2.5, 4.0, 6.0):
            self.check(ahead, 0.0, math.pi / 2, seed=int(ahead * 10))

    def test_a_ladder_lying_end_on_keeps_its_far_end(self):
        """Pointing away: per image column its far end is 3 m behind its near end."""
        for ahead in (2.5, 4.0, 6.0):
            sighting = self.check(ahead, 0.0, 0.0, seed=int(ahead * 10))
            reach = [(i + 0.5) * CELL for i, _ in sighting.cells]
            visible = [x for x in np.arange(ahead - 1.5, ahead + 1.5, 0.05) if in_view(x, 0.0)]
            self.assertGreater(max(reach) - min(reach), max(visible) - min(visible) - 0.2, ahead)

    def test_a_ladder_lying_at_an_angle_and_off_centre(self):
        self.check(3.0, 1.0, math.pi / 4)
        self.check(3.5, -0.8, -math.pi / 3, length=4.5)

    def test_with_the_simulators_depth_noise_and_none(self):
        for noise in (SIM_NOISE, 0.0):
            self.check(4.0, 0.0, math.pi / 2, noise=noise)
            self.check(4.0, 0.0, 0.0, noise=noise)

    def test_standing_ladders_are_not_taken_for_lying_ones(self):
        for ahead in (2.0, 4.0, 8.0):
            boxes = place(ladder_parts(1.8, standing=True), ahead, 0.0, 0.0)  # face-on
            sighting = lift(seen("a", boxes, seed=3), detection(boxes), CALIBRATION)
            self.assertFalse(sighting.lying, ahead)
            self.assertAlmostEqual(sighting.distance, ahead, delta=0.1 * ahead)
            self.assertGreater(sighting.height, 1.5)

    def test_a_low_crate_is_not_lying_and_kinds_that_never_lie_are_untouched(self):
        crate = [(3.0, 0.0, 0.6, 0.6, 0.0, 0.0, 0.6)]
        self.assertFalse(lift(seen("a", crate), detection(crate), CALIBRATION).lying)
        flat = place(ladder_parts(), 3.0, 0.0, math.pi / 2)
        self.assertFalse(lift(seen("a", flat), detection(flat, "fire_extinguisher"), CALIBRATION).lying)

    def test_without_a_floor_in_view_nothing_is_called_lying(self):
        boxes = place(ladder_parts(), 3.0, 0.0, math.pi / 2)
        frame = seen("a", boxes)
        frame.depth[240:] = 0.0  # no depth in the lower half: no floor plane
        self.assertFalse(lift(frame, detection(boxes), CALIBRATION).lying)


def window_wall(boxes, wall=3.0, sill=0.9, top=2.4, beyond=12.0, columns=(120, 520), door=None, seed=0):
    """`scene` with a wall `wall` metres ahead holding a window from `sill` to `top` metres high
    over image `columns` (glass: the depth camera reads the room `beyond` it), or with a door
    opening (floor to `top`) over `door` columns. `boxes` behind the wall show only through it."""
    behind = [b for b in boxes if min(b[0] - b[2], b[0] + b[2]) > wall]
    front = [b for b in boxes if b not in behind]
    depth = scene(front, wall=wall, seed=seed)
    far = scene(behind, wall=beyond, seed=seed + 1)
    v, u = np.mgrid[0:480, 0:640] + 0.5
    up = HEIGHT - (v - CALIBRATION.cy) / CALIBRATION.fy * wall  # height where the ray meets the wall
    hole = np.zeros(depth.shape, dtype=bool)
    if columns:
        hole |= (u >= columns[0]) & (u < columns[1]) & (up >= sill) & (up <= top)
    if door:
        hole |= (u >= door[0]) & (u < door[1]) & (up <= top)
    through = hole & (np.abs(depth - wall) < 0.3 * wall)  # where the wall would be seen, not a front object
    depth[through] = far[through]
    return depth


def framed(depth):
    return Frame(Image.new("RGB", (640, 480)), name="w", pose=(0.0, 0.0, 0.0), depth=depth)


def visible_outline(boxes, depth, near, far):
    """Outline of what of `boxes` is visible in `depth` (pixels in [near, far]) inside their hull."""
    o = outline(boxes)
    from ultron.pipeline.lift import _mask
    mask = _mask(depth.shape, o) & (depth >= near) & (depth <= far)
    rows, columns = np.nonzero(mask)
    x0, x1 = columns.min() * 1000 / 640, (columns.max() + 1) * 1000 / 640
    y0, y1 = rows.min() * 1000 / 480, (rows.max() + 1) * 1000 / 480
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def outlined(points, kind="ladder"):
    box = [min(p[0] for p in points), min(p[1] for p in points), max(p[0] for p in points), max(p[1] for p in points)]
    return Detection(kind, box, 0.9, None, points)


class GlassTest(unittest.TestCase):
    """Second line behind the fence: detections seen on or through glass (lift.doubt)."""

    def test_a_ladder_seen_through_a_window_over_its_sill_is_doubted(self):
        """A reflection or a ladder in the next room: only its top shows, the sill right below it."""
        ladder = place(ladder_parts(1.8, standing=True), 6.0, 0.0, 0.0)
        depth = window_wall(ladder)
        detection = outlined(visible_outline(ladder, depth, 5.5, 6.5))
        self.assertEqual(doubt(framed(depth), detection, CALIBRATION), "over a sill")
        sighting = lift(framed(depth), detection, CALIBRATION)
        self.assertEqual(sighting.doubt, "over a sill")

    def test_a_ladder_standing_in_front_of_the_window_is_not_doubted(self):
        for ahead in (1.8, 2.5):
            ladder = place(ladder_parts(1.8, standing=True), ahead, 0.0, 0.0)
            depth = window_wall(ladder)
            self.assertIsNone(doubt(framed(depth), outlined(outline(ladder)), CALIBRATION), ahead)

    def test_a_ladder_seen_through_an_open_door_is_not_doubted(self):
        """The fence decides whether the next room counts; the floor runs on below it."""
        ladder = place(ladder_parts(1.8, standing=True), 6.0, 0.0, 0.0)
        depth = window_wall(ladder, columns=None, door=(250, 390))
        self.assertIsNone(doubt(framed(depth), outlined(visible_outline(ladder, depth, 5.5, 6.5)), CALIBRATION))

    def test_wall_mounted_equipment_and_a_lying_ladder_are_not_doubted(self):
        box = [(2.98, 0.0, 0.04, 0.25, 0.0, 1.2, 1.55)]  # an alarm box on a plain wall 3 m ahead
        depth = scene(box, wall=3.0)
        self.assertIsNone(doubt(framed(depth), outlined(outline(box), "fire_alarm"), CALIBRATION))
        flat = place(ladder_parts(), 3.0, 0.0, math.pi / 2)
        self.assertIsNone(doubt(framed(window_wall(flat, wall=5.0)), outlined(outline(flat)), CALIBRATION))

    def test_an_outline_the_depth_camera_cannot_see_is_doubted_unless_the_whole_camera_is_blind(self):
        """Clear glass often gives no depth at all; the outline would then be placed 2.5 m away."""
        ladder = place(ladder_parts(1.8, standing=True), 6.0, 0.0, 0.0)
        depth = window_wall(ladder)
        detection = outlined(visible_outline(ladder, depth, 5.5, 6.5))
        from ultron.pipeline.lift import _mask
        holes = depth.copy()
        holes[_mask(depth.shape, detection.outline)] = 0.0
        self.assertEqual(doubt(framed(holes), detection, CALIBRATION), "no depth")
        blind = np.zeros_like(depth)
        blind[:, :100] = 3.0
        self.assertIsNone(doubt(framed(blind), detection, CALIBRATION))


def lidar_sweep(boxes, height=0.74, seed=0, noise=0.02):
    """One sweep of a 32-ring lidar `height` above the floor at the origin (rings -53 to +15
    degrees, as the B2's on the hall recording; 0.5 degree azimuth steps) against the floor and
    oriented boxes (centre x, centre y, length, width, yaw, bottom, top). Nx3, z up, floor at 0."""
    elevation = np.radians(np.linspace(-53, 15, 32))[:, None]
    azimuth = np.radians(np.arange(0, 360, 0.5))[None, :]
    dx, dy = (np.cos(elevation) * np.cos(azimuth)).ravel(), (np.cos(elevation) * np.sin(azimuth)).ravel()
    dz = np.broadcast_to(np.sin(elevation), (32, 720)).ravel()
    with np.errstate(divide="ignore"):
        t = np.where(dz < 0, -height / np.where(dz < 0, dz, -1), np.inf)
    for cx, cy, length, width, yaw, bottom, top in boxes:
        c, s = math.cos(yaw), math.sin(yaw)
        o = (c * -cx + s * -cy, -s * -cx + c * -cy, height)
        d = (c * dx + s * dy, -s * dx + c * dy, dz)
        near, far = np.full(dx.shape, -np.inf), np.full(dx.shape, np.inf)
        for oo, dd, low, high in ((o[0], d[0], -length / 2, length / 2), (o[1], d[1], -width / 2, width / 2),
                                  (o[2], d[2], bottom, top)):
            with np.errstate(divide="ignore", invalid="ignore"):
                t1, t2 = (low - oo) / dd, (high - oo) / dd
            parallel = np.abs(dd) < 1e-12
            inside = (oo >= low) & (oo <= high)
            t1 = np.where(parallel, np.where(inside, -np.inf, np.inf), t1)
            t2 = np.where(parallel, np.inf, t2)  # parallel: all of the ray inside the slab, or none
            near, far = np.maximum(near, np.minimum(t1, t2)), np.minimum(far, np.maximum(t1, t2))
        t = np.where((far >= near) & (near > 0), np.minimum(t, near), t)
    keep = np.isfinite(t) & (t < 30)
    t = t[keep] + np.random.default_rng(seed).normal(0, noise, keep.sum())
    return np.stack([dx[keep] * t, dy[keep] * t, height + dz[keep] * t], axis=1).astype(np.float32)


ROOM = [(5.0, 4.0, 14.0, 0.2, 0.0, 0.0, 2.8), (5.0, -4.0, 14.0, 0.2, 0.0, 0.0, 2.8),   # a 10 x 8 m room
        (10.0, 0.0, 0.2, 8.0, 0.0, 0.0, 2.8), (-2.0, 0.0, 0.2, 8.0, 0.0, 0.0, 2.8)]


class LidarLyingTest(unittest.TestCase):
    def find(self, boxes, seed=0, walk=(0.0, 0.3, 0.6)):
        """Candidates after sweeps from a robot walking along +x by `walk` metres (yaw 0)."""
        from ultron.pipeline.discover import LidarCandidates
        finder, found = LidarCandidates(), []
        for n, step in enumerate(walk):
            moved = [(b[0] - step,) + tuple(b[1:]) for b in ROOM + boxes]
            cloud = lidar_sweep(moved, seed=seed * 10 + n) + np.float32([step, 0.0, 0.0])
            found = finder.find(cloud, (step, 0.0, 0.0))
        return found

    def test_a_ladder_lying_on_the_floor_is_a_lying_candidate(self):
        for x, y, yaw in ((3.0, 0.0, math.pi / 2), (3.5, 0.0, 0.0), (2.5, 1.5, math.pi / 4), (4.0, -2.0, 0.3)):
            found = self.find(place(ladder_parts(3.0), x, y, yaw))
            lying = [c for c in found if c.lying]
            self.assertEqual(len(lying), 1, (x, y, yaw, found))
            self.assertLess(math.dist(lying[0], (x, y)), 0.6, (x, y, yaw))
            self.assertFalse([c for c in found if not c.lying and math.dist(c, (x, y)) < 1.5])

    def test_an_empty_room_a_low_crate_and_a_standing_ladder_give_no_lying_candidate(self):
        for seed in range(3):
            self.assertEqual([c for c in self.find([], seed) if c.lying], [])
        crate = [(3.0, 1.0, 0.5, 0.4, 0.0, 0.0, 0.2)]
        self.assertEqual([c for c in self.find(crate) if c.lying], [])
        standing = place(ladder_parts(1.8, standing=True), 3.0, 0.0, 0.0)
        found = self.find(standing)
        self.assertEqual([c for c in found if c.lying], [])
        self.assertTrue([c for c in found if math.dist(c, (3.0, 0.0)) < 0.5])


class Outlined:
    """Detector for `seen` frames: one ladder per frame, its hull as the mask."""

    def detect(self, frame):
        o = frame.outlines[0]
        box = [min(p[0] for p in o), min(p[1] for p in o), max(p[0] for p in o), max(p[1] for p in o)]
        return [Detection("ladder", box, 0.9, 7, o)]


class LyingSurveyTest(unittest.TestCase):
    def test_a_lying_ladder_becomes_a_ladder_object_marked_lying_with_its_footprint(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        root = Path(folder.name)
        objects = ObjectMap(root)
        survey = Survey(Outlined(), objects, KeyframeStore(root), Store(root), CALIBRATION, interval=0)
        x, y, yaw = 5.0, 2.0, 0.3  # a 3 m ladder on the floor
        for n, (px, py) in enumerate([(1.0, 2.0), (1.5, 1.0), (2.0, 3.2), (1.2, 2.5), (2.5, 1.5)]):
            heading = math.atan2(y - py, x - px)
            c, s = math.cos(-heading), math.sin(-heading)
            local = (c * (x - px) - s * (y - py), s * (x - px) + c * (y - py), yaw - heading)
            boxes = place(ladder_parts(), *local)
            frame = seen(f"f{n}", boxes, seed=n, stamp=float(n))
            frame.pose = (px, py, heading)
            survey.feed(frame)
        found = [o for o in objects.all() if o.kind == "ladder"]
        self.assertEqual(len(found), 1)
        self.assertTrue(lying(found[0]))
        cells = {tuple(c) for c in footprint(found[0]).tolist()}
        truth = ladder_cells(x, y, yaw)
        covered = {t for t in truth if any(abs(t[0] - c[0]) <= 1 and abs(t[1] - c[1]) <= 1 for c in cells)}
        self.assertGreater(len(covered) / len(truth), 0.9)
        self.assertLess(math.dist(found[0].position, (x, y)), 0.3)

    def test_a_standing_ladder_is_not_marked_lying(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        root = Path(folder.name)
        objects = ObjectMap(root)
        survey = Survey(Outlined(), objects, KeyframeStore(root), Store(root), CALIBRATION, interval=0)
        for n in range(4):
            boxes = place(ladder_parts(1.8, standing=True), 3.0 + 0.2 * n, 0.0, 0.0)
            survey.feed(seen(f"f{n}", boxes, seed=n, stamp=float(n)))
        found = objects.all()
        self.assertEqual(len(found), 1)
        self.assertFalse(lying(found[0]))


if __name__ == "__main__":
    unittest.main()
