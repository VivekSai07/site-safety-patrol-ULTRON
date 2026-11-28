"""Getting the robot to a requested view. The pipeline never sends velocity commands itself.

    Passive             never moves: missing views stay missing (survey-only patrols)
    Operator            asks a person to drive there and confirm
    DashboardNavigator  posts the goal to the dashboard; its ROS bridge hands it to Nav2
    ContractNavigator   submits the goal through `ultron.autonomy`'s navigation contract
                        (pre-checks, motion gate, timeout, settle before capture)

All return True once the robot stands at the request and may capture, False otherwise.
"""
import json
import time
import urllib.error
import urllib.request


class Passive:
    moves = False  # the inspection does not even ask

    def go_to(self, request):
        return False


class Operator:
    moves = True

    def __init__(self, ask=input):
        self.ask = ask

    def go_to(self, request):
        reply = self.ask(f"{request.object_id}: {request.reason}. Drive to x={request.x} y={request.y} "
                         f"yaw={request.yaw} and press Enter (or type s to skip): ")
        return reply.strip().lower() not in ("s", "skip", "n")


def http_json(url):
    """call(method, path, body) -> (status, parsed JSON or None) against the dashboard at `url`."""
    def call(method, path, body=None):
        request = urllib.request.Request(url.rstrip("/") + path, json.dumps(body).encode() if body else None,
                                         {"Content-Type": "application/json"}, method=method)
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, json.loads(response.read() or b"null")
        except urllib.error.HTTPError as error:
            return error.code, None
        except (urllib.error.URLError, TimeoutError, OSError):
            return 0, None
    return call


class DashboardNavigator:
    """Goals go through the dashboard's `/api/goal` (start it with --goals and run dashboard_bridge.py)."""
    moves = True
    FINAL = {"SUCCEEDED": True, "REJECTED": False, "FAILED": False, "CANCELED": False}

    def __init__(self, call, timeout=90.0, poll=1.0, sleep=time.sleep):
        self.call, self.timeout, self.poll, self.sleep = call, timeout, poll, sleep

    def go_to(self, request):
        status, goal = self.call("POST", "/api/goal", {"x": request.x, "y": request.y, "yaw": request.yaw})
        if status != 201:
            return False
        waited = 0.0
        while waited < self.timeout:
            status, current = self.call("GET", "/api/goal")
            if status == 200 and current and (current.get("seq"), current.get("request_id")) != (
                    goal["seq"], goal.get("request_id")):
                return False  # someone set a newer goal on the map
            if status == 200 and current and current.get("status") in self.FINAL:
                return self.FINAL[current["status"]]
            self.sleep(self.poll)
            waited += self.poll
        return False


class ContractNavigator:
    """View requests as `ultron.navigation_goal/v1` goals through an `ultron.autonomy` backend.

    `backend` is a `Nav2Adapter` or anything with its `submit(goal)`, `tick()`, `done()` and
    `tracker`; `spin()` lets the backend process messages between ticks (rclpy.spin_once on
    the robot). The request counts as reached only once the backend reports the robot settled
    and ready to capture.
    """
    moves = True

    def __init__(self, backend, params, mission_id, map_id, map_version, spin=lambda: None, clock=time.time):
        self.backend, self.params, self.spin, self.clock = backend, params, spin, clock
        self.mission_id, self.map_id, self.map_version = mission_id, map_id, map_version
        self.results = []  # every NavigationResult, for the mission record

    def go_to(self, request):
        from ..autonomy.navigation_contract import make_goal
        goal = make_goal(request.x, request.y, request.yaw, mission_id=self.mission_id, map_id=self.map_id,
                         map_version=self.map_version, params=self.params, now=self.clock(),
                         reason=f"INSPECT: {request.view} view of {request.object_id}, {request.reason}",
                         asset_id=request.object_id)
        rejected = self.backend.submit(goal)
        while rejected is None and not self.backend.done():
            self.spin()
            self.backend.tick()
        result = rejected or self.backend.tracker.result
        self.results.append(result)
        return bool(result and result.status == "REACHED" and result.capture_ready)


def free_space(footprint, clearance_m):
    """`is_free(x, y)` for the inspection stage from an `ultron.autonomy` MapFootprint:
    the robot's footprint at the goal must be free and mapped."""
    return lambda x, y: footprint.check(x, y, clearance_m)[0]
