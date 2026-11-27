"""Getting the robot to a requested view. The pipeline never sends velocity commands itself.

    Passive             never moves: missing views stay missing (survey-only patrols)
    Operator            asks a person to drive there and confirm
    DashboardNavigator  posts the goal to the dashboard; its ROS bridge hands it to Nav2

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
    FINAL = {"SUCCEEDED": True, "FAILED": False, "CANCELED": False}

    def __init__(self, call, timeout=90.0, poll=1.0, sleep=time.sleep):
        self.call, self.timeout, self.poll, self.sleep = call, timeout, poll, sleep

    def go_to(self, request):
        status, goal = self.call("POST", "/api/goal", {"x": request.x, "y": request.y, "yaw": request.yaw})
        if status != 201:
            return False
        waited = 0.0
        while waited < self.timeout:
            status, current = self.call("GET", "/api/goal")
            if status == 200 and current and current.get("seq") != goal["seq"]:
                return False  # someone set a newer goal on the map
            if status == 200 and current and current.get("status") in self.FINAL:
                return self.FINAL[current["status"]]
            self.sleep(self.poll)
            waited += self.poll
        return False
