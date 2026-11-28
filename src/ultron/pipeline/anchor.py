"""Line a patrol up with the objects an earlier patrol on the same map left behind.

Objects stored in the site's object map are landmarks. Each time one is seen again, its
position measured in this run's frame is added up. With two or more of them the 2D shift and
rotation that carries this run's frame onto the stored one is estimated (weighted least squares)
and applied to every later pose, so drift between runs does not create duplicates.

A large shift, or landmarks that no single shift and rotation can line up, means the robot is
not where it thinks it is (localisation lost, a wrong map, a different start under odometry):
the patrol then creates and merges nothing until it ends.
"""
import math

from .catalog import KINDS

GROSS = 1.0       # metres: moving a landmark further than this is a localisation failure, not drift
# On the hall recording, one wall box measured on the robot's two passes of a single run lies up to
# 0.6 m apart (pose and extrinsics error): landmarks disagreeing by less than this are normal.
DISAGREE = 0.75   # metres: landmarks that cannot be lined up within this contradict each other
REACH = 2.0       # metres: an unmatched sighting this near a single stored object of its kind may be it, displaced
MIN_SIGHTINGS = 3  # of one landmark in this run before it counts
# Landmarks a few metres apart, each known to a few decimetres, say little about rotation: it is
# estimated only from three or more, and kept only where it explains the landmarks clearly better.
ROTATION_GAIN = 0.7   # the rotated fit's residual must be below this share of the shift-only one


def fit(pairs, rotate=True):
    """Weighted rigid 2D fit: (angle, tx, ty) taking each measured point onto its stored one.

    `pairs` is [(measured (x, y), stored (x, y), weight)]; one pair, or not `rotate`, gives a shift only."""
    total = sum(w for _, _, w in pairs)
    mx = [sum(w * p[i] for p, _, w in pairs) / total for i in range(2)]
    sx = [sum(w * q[i] for _, q, w in pairs) / total for i in range(2)]
    angle = 0.0
    if rotate and len(pairs) > 1:
        a = sum(w * ((p[0] - mx[0]) * (q[0] - sx[0]) + (p[1] - mx[1]) * (q[1] - sx[1])) for p, q, w in pairs)
        b = sum(w * ((p[0] - mx[0]) * (q[1] - sx[1]) - (p[1] - mx[1]) * (q[0] - sx[0])) for p, q, w in pairs)
        angle = math.atan2(b, a)
    c, s = math.cos(angle), math.sin(angle)
    return angle, sx[0] - (c * mx[0] - s * mx[1]), sx[1] - (s * mx[0] + c * mx[1])


def apply(transform, point):
    angle, tx, ty = transform
    c, s = math.cos(angle), math.sin(angle)
    return c * point[0] - s * point[1] + tx, s * point[0] + c * point[1] + ty


def invert(transform):
    angle, tx, ty = transform
    c, s = math.cos(-angle), math.sin(-angle)
    return -angle, -(c * tx - s * ty), -(s * tx + c * ty)


class Anchoring:
    """Landmarks of one patrol on a shared map: which stored objects it re-saw, and where."""

    UNANCHORED, ANCHORED, SUSPECT = "unanchored", "anchored", "suspect"

    def __init__(self, stored):
        # Wide objects are poor landmarks: the middle of what is seen of a ladder depends on the view.
        self.stored = {o.id: (o.kind, tuple(o.position), 1.0 / max(KINDS[o.kind].extent, 0.25))
                       for o in stored if o.position and o.status not in ("REJECTED", "MOVED")}
        self.measured = {}  # stored id -> [sum x, sum y, sightings], in this run's own frame
        self.transform = None
        self.state, self.details = self.UNANCHORED, {}

    def near(self, kind, position):
        """The single stored object of `kind` within REACH of `position`, or None (none or several)."""
        close = [i for i, (k, p, _) in self.stored.items() if k == kind and math.dist(p, position) <= REACH]
        return close[0] if len(close) == 1 else None

    def to_map(self, pose):
        """A pose of this run in the stored map's frame."""
        if self.transform is None or pose is None:
            return pose
        x, y = apply(self.transform, pose[:2])
        return (x, y, pose[2] + self.transform[0])

    def to_run(self, position):
        return position if self.transform is None else apply(invert(self.transform), position)

    def add(self, object_id, position):
        """`position`: where this run, uncorrected, measured stored object `object_id`. Returns
        the new state if it changed, else None. A suspect patrol stays suspect."""
        if object_id not in self.stored or self.state == self.SUSPECT:
            return None
        sums = self.measured.setdefault(object_id, [0.0, 0.0, 0])
        sums[0], sums[1], sums[2] = sums[0] + position[0], sums[1] + position[1], sums[2] + 1
        return self._estimate()

    @staticmethod
    def _rms(transform, pairs):
        total = sum(w for _, _, w in pairs)
        return math.sqrt(sum(w * math.dist(apply(transform, p), q) ** 2 for p, q, w in pairs) / total)

    def _estimate(self):
        pairs = [((sx / n, sy / n), self.stored[i][1], self.stored[i][2])
                 for i, (sx, sy, n) in self.measured.items() if n >= MIN_SIGHTINGS]
        if not pairs:
            return None
        transform = fit(pairs, rotate=False)
        if len(pairs) >= 3:
            rotated = fit(pairs, rotate=True)
            if self._rms(rotated, pairs) < ROTATION_GAIN * self._rms(transform, pairs):
                transform = rotated
        shift = max(math.dist(apply(transform, p), p) for p, _, _ in pairs)
        residual = max(math.dist(apply(transform, p), q) for p, q, _ in pairs)
        details = {"landmarks": len(pairs), "shift": round(shift, 2), "residual": round(residual, 2),
                   "rotation_deg": round(math.degrees(transform[0]), 1)}
        state = self.state
        if residual > DISAGREE:
            state, details["reason"] = self.SUSPECT, "stored objects disagree about where the robot is"
        elif shift > GROSS:
            state, details["reason"] = self.SUSPECT, f"objects seen {shift:.1f} m from where they were stored"
        elif len(pairs) > 1:
            state, self.transform = self.ANCHORED, transform
        self.details = details
        if state != self.state:
            self.state = state
            return state
        return None
