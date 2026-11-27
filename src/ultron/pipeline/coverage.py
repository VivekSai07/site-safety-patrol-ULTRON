"""Which views of an object are still missing, and where the robot should stand to get them."""
import math

from .catalog import KINDS
from .keyframes import view_name
from .types import SIDE_STEP, ViewRequest


def _request(site_object, view, side, reason):
    distance = (view.min_range + view.max_range) / 2
    x = site_object.position[0] + distance * math.cos(side * SIDE_STEP)
    y = site_object.position[1] + distance * math.sin(side * SIDE_STEP)
    yaw = math.atan2(site_object.position[1] - y, site_object.position[0] - x)
    return ViewRequest(site_object.id, view.name, side, round(x, 2), round(y, 2), round(yaw, 3), reason)


def missing_views(site_object, keyframes, is_free=None, exclude=()):
    """View requests for this object, most useful first. Empty when nothing is missing.

    `is_free(x, y)` filters out goals the map says the robot cannot stand on, and `exclude`
    holds (view, side) pairs already tried in vain, so the next best direction is offered.
    Without a map position nothing can be requested.
    """
    def usable(view, side):
        if (view.name, side) in exclude:
            return False
        goal = _request(site_object, view, side, "")
        return is_free is None or is_free(goal.x, goal.y)

    if site_object.position is None:
        return []
    kind = KINDS[site_object.kind]
    seen = {(view_name(kind.name, k.distance), k.side) for k in keyframes if k.side is not None}
    seen_sides = sorted({side for _, side in seen})
    requests = []
    # Every view at least once, from a direction the robot has already managed to reach.
    for view in kind.views:
        sides = [side for side in seen_sides if usable(view, side)]
        if not any(name == view.name for name, _ in seen) and sides:
            requests.append(_request(site_object, view, sides[0], f"no {view.name} view yet"))
    # Then more directions, working outwards from the ones already seen.
    wanted = kind.sides - len(seen_sides)
    if wanted > 0 and seen_sides:
        by_closeness = sorted((s for s in range(8) if s not in seen_sides and usable(kind.views[0], s)),
                              key=lambda s: min(min((s - t) % 8, (t - s) % 8) for t in seen_sides))
        for side in by_closeness[:wanted]:
            requests.append(_request(site_object, kind.views[0], side, f"seen from {len(seen_sides)} of {kind.sides} sides"))
    return requests
