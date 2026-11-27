"""Deterministic 2D visibility simulator; observations are scripted, not inferred."""
import math


def visible(pose, target, max_range, fov_degrees):
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    distance = math.hypot(dx, dy)
    angle = (math.atan2(dy, dx) - math.radians(pose[2]) + math.pi) % (2 * math.pi) - math.pi
    return distance <= max_range and abs(angle) <= math.radians(fov_degrees / 2)


def observe(scenario):
    """Each waypoint can rediscover an object. Stable IDs simulate association.

    Ground truth IDs and scripted findings must never be mistaken for detector output.
    No collision checking, occlusion, physical dynamics or motion control is simulated.
    """
    sensor = scenario["sensor"]
    for step, pose in enumerate(scenario["route"]):
        yield {"type": "pose", "step": step, "pose": pose}
        for ladder in scenario["ladders"]:
            if not ladder.get("detectable", True):
                continue
            if visible(pose, ladder["position"], sensor["range_m"], sensor["fov_degrees"]):
                yield {"type": "observation", "step": step,
                       "track_id": ladder["id"], "position": ladder["position"],
                       "checks": ladder["checks"], "label": ladder["label"],
                       "confidence": ladder["confidence"]}
