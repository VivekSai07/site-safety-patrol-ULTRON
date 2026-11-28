"""Where the pipeline may send the robot (gate, fence) and when (operator first, pause the explorer)."""
import math
import unittest

import numpy as np

from tests.test_pipeline import FormVlm, Workspace, facing, frame
from ultron.pipeline.gate import Fence, Gate, GridMap
from ultron.pipeline.inspection import Inspection
from ultron.pipeline.navigate import DashboardNavigator, Exploration
from ultron.pipeline.online import OnlinePatrol

A = (5.0, 5.0)
ROOM = Fence([[0, 0], [10, 0], [10, 10], [0, 10]])


def room_map(wall_x=None):
    """10 x 10 m free room at 0.1 m, walls around, unknown outside; optional wall across at x."""
    pixels = np.full((120, 120), 205, dtype=np.uint8)          # 12 m square from (-1, -1), unknown
    pixels[10:110, 10:110] = 254                               # the room is free ...
    pixels[10:110, [10, 109]] = 0                              # ... between walls
    pixels[[10, 109], 10:110] = 0
    if wall_x is not None:
        pixels[:, int((wall_x + 1) / 0.1)] = 0
    return GridMap(pixels[::-1], 0.1, (-1.0, -1.0), seq=1)     # PNG rows: top = highest y


class FakeDashboard:
    """The contract's goal and exploration rules (docs/EXPLORATION_CONTRACT.md), confirmed at once."""

    def __init__(self, exploring=False, arrive=True):
        self.exploration = {"seq": 0, "state": "EXPLORING" if exploring else "IDLE", "by": ""}
        self.goal, self.posted, self.actions, self.arrive = None, [], [], arrive
        self.obstacles = None

    def call(self, method, path, body=None):
        if path == "/api/goal" and method == "POST":
            state, by = self.exploration["state"], body.get("by", "operator")
            if state in ("STARTING", "EXPLORING", "RESUMING"):
                return 409, {"detail": "exploring"}
            if by == "pipeline" and self.goal and self.goal["status"] in ("SENT", "ACCEPTED") \
                    and self.goal.get("by") == "operator":
                return 409, {"detail": "operator goal underway"}
            self.goal = {"seq": (self.goal or {}).get("seq", 0) + 1, **body, "status": "SENT"}
            self.posted.append(body)
            return 201, dict(self.goal)
        if path == "/api/goal":
            if self.goal and self.goal.get("by") == "pipeline" and self.goal["status"] == "SENT":
                self.goal["status"] = "SUCCEEDED" if self.arrive else "FAILED"
            return 200, dict(self.goal) if self.goal else None
        if path == "/api/exploration" and method == "GET":
            return 200, dict(self.exploration)
        if path == "/api/exploration":
            self.actions.append(body["action"])
            following = {"pause": "PAUSED", "resume": "EXPLORING", "renew": self.exploration["state"]}
            self.exploration["state"] = following[body["action"]]
            return 201, dict(self.exploration)
        if path == "/api/obstacles":
            self.obstacles = body
            return 200, {}
        return 404, None


class GateTest(Workspace):
    def test_outside_the_fence_or_near_its_edge_is_refused(self):
        gate = Gate(ROOM, clearance=0.5)
        self.assertEqual(gate.check(5, 5), (True, ""))
        self.assertEqual(gate.check(12, 5)[1], "outside the fence")
        self.assertEqual(gate.check(9.8, 5)[1], "outside the fence")  # inside, but closer than the clearance

    def test_unknown_occupied_and_cut_off_space_is_refused(self):
        gate = Gate(grid=lambda: room_map(wall_x=6.0), clearance=0.4)
        gate.robot = (2.0, 5.0, 0.0)
        self.assertTrue(gate(4.0, 5.0))
        self.assertEqual(gate.check(0.2, 5.0)[1], "not known free in the map")       # against the wall
        self.assertEqual(gate.check(10.5, 5.0)[1], "not known free in the map")      # unknown, outside
        self.assertEqual(gate.check(8.0, 5.0)[1], "not reachable from the robot")    # behind the wall

    def test_no_goal_next_to_a_known_object_even_one_lying_on_the_floor(self):
        for stamp in range(3):
            self.see("ladder", A, 0, 3.0, float(stamp))
        gate = Gate(objects=self.objects, clearance=0.45)
        ladder = self.objects.get("LAD-001").position
        self.assertTrue(gate.check(ladder[0], ladder[1])[1].startswith("too close to LAD-001"))
        self.assertTrue(gate(ladder[0] + 3.0, ladder[1]))

    def test_in_a_tight_room_a_view_is_placed_nearer_when_the_middle_of_its_range_is_in_a_wall(self):
        from ultron.pipeline.coverage import missing_views
        for stamp in range(3):
            self.see("ladder", A, 0, 3.0, float(stamp))
        ladder = self.objects.get("LAD-001")
        wall = ladder.position[1] + 2.0     # a wall 2 m north: the north-east view (side 1) moves nearer
        requests = missing_views(ladder, self.keyframes.for_object("LAD-001"), lambda x, y: y < wall)
        north_east = [r for r in requests if r.side == 1]
        self.assertTrue(north_east)
        distance = math.dist((north_east[0].x, north_east[0].y), ladder.position)
        self.assertAlmostEqual(distance, 2.2, places=1)  # not the middle of the 2-4.5 m range

    def test_a_sighting_outside_the_fence_never_becomes_an_object(self):
        self.survey.inside = Fence([[0, 0], [4, 0], [4, 10], [0, 10]]).contains  # A (5, 5) lies beyond it
        for stamp in range(3):
            self.see("ladder", A, 4, 3.0, float(stamp))
        self.assertEqual((self.objects.all(), self.survey.outside), ([], 3))


class DriveTest(Workspace):
    def setUp(self):
        super().setUp()
        self.now = [0.0]

    def online(self, dashboard, exploration=False, gate=None, **options):
        navigator = DashboardNavigator(dashboard.call, timeout=5, poll=0, sleep=lambda s: None)
        inspection = Inspection(self.objects, self.keyframes, FormVlm(), self.store, self.survey,
                                min_sightings=1, mission="PATROL-1", by_section=False)
        options = {"leave_seconds": 100.0, "stall_seconds": 100.0, "budget_seconds": 1000.0, **options}
        return OnlinePatrol(self.survey, inspection, self.store, navigator=navigator, is_free=gate,
                            exploration=Exploration(dashboard.call) if exploration else None,
                            log=lambda *_: None, clock=lambda: self.now[0], **options)

    def step(self, online, side=0, distance=3.0, stamp=0.0, target=A):
        online.step(frame(f"ladder-{side}-{stamp}", facing(target, side, distance), stamp, depth=distance))
        if online.goal:
            online.goal[0].result()

    def kinds(self):
        return [e["kind"] for e in self.store.events(limit=None)]

    def test_a_refused_goal_while_exploring_burns_no_side(self):
        dashboard = FakeDashboard(exploring=True)
        online = self.online(dashboard)  # the pipeline was not told about the explorer: it learns from the 409
        for stamp in range(6):
            self.step(online, stamp=float(stamp))
        self.assertEqual(online.tried, {})
        self.assertIn("navigation_held", self.kinds())
        online.finish()

    def test_an_operator_goal_underway_is_never_replaced(self):
        dashboard = FakeDashboard()
        dashboard.goal = {"seq": 1, "x": 1, "y": 1, "yaw": 0, "by": "operator", "status": "ACCEPTED"}
        online = self.online(dashboard)
        for stamp in range(5):
            self.step(online, stamp=float(stamp))
        self.assertEqual(dashboard.posted, [])
        held = [e for e in self.store.events(limit=None) if e["kind"] == "navigation_held"]
        self.assertEqual(held[0]["reason"], "preempted")
        dashboard.goal["status"] = "SUCCEEDED"     # the operator's goal ends: after the grace period, ours go
        self.now[0] += 11.0
        self.step(online, stamp=5.0)
        self.step(online, stamp=6.0)
        self.assertTrue(dashboard.posted and dashboard.posted[0]["by"] == "pipeline")
        online.finish()

    def test_the_explorer_is_paused_for_an_objects_views_and_resumed_after(self):
        dashboard = FakeDashboard(exploring=True)
        online = self.online(dashboard, exploration=True)
        for stamp in range(4):
            self.step(online, stamp=float(stamp))
            self.now[0] += 1.0
        self.assertEqual(dashboard.actions[:1], ["pause"])
        self.assertTrue(dashboard.posted, "view goals go out once the explorer is paused")
        for stamp in range(4, 30):                # the views come in; the object is read; exploring again
            self.step(online, stamp=float(stamp))
            self.now[0] += 1.0
            if "resume" in dashboard.actions:
                break
        self.assertIn("resume", dashboard.actions)
        self.assertEqual(dashboard.exploration["state"], "EXPLORING")
        self.assertEqual(dashboard.actions.count("pause"), 1)  # each object pauses once
        online.finish()

    def test_a_crash_mid_pause_still_resumes_the_explorer(self):
        dashboard = FakeDashboard(exploring=True, arrive=False)
        online = self.online(dashboard, exploration=True)

        def frames():
            for stamp in range(3):
                self.now[0] += 1.0
                yield frame(f"ladder-0-{stamp}", facing(A, 0, 3.0), float(stamp), depth=3.0)
            raise RuntimeError("camera gone")

        with self.assertRaises(RuntimeError):
            online.run(frames())
        self.assertEqual(dashboard.actions, ["pause", "resume"])

    def test_the_pause_has_a_cap(self):
        dashboard = FakeDashboard(exploring=True, arrive=False)  # views never come in
        online = self.online(dashboard, exploration=True, pause_cap=10.0)
        for stamp in range(20):
            self.step(online, stamp=float(stamp))
            self.now[0] += 1.0
        self.assertEqual(dashboard.actions.count("pause"), 1)
        resumed = [e for e in self.store.events(limit=None) if e["kind"] == "exploration_resumed"]
        self.assertTrue(resumed and resumed[0]["reason"] in ("time cap", "views collected"))
        online.finish()

    def test_an_object_whose_views_stop_growing_is_read_in_a_tight_room(self):
        dashboard = FakeDashboard()
        online = self.online(dashboard, stall_seconds=5.0, gate=Gate(ROOM, grid=lambda: room_map(), clearance=0.4))
        online.navigator = None                    # a person drives; the robot stays near the ladder
        for stamp in range(12):
            self.step(online, stamp=float(stamp))
        ready = [e for e in self.store.events(limit=None) if e["kind"] == "object_ready"]
        self.assertEqual(ready[0]["reason"], "no new views")
        online.finish()

    def test_without_a_map_yet_nothing_is_read_for_want_of_a_reachable_view(self):
        dashboard = FakeDashboard()
        online = self.online(dashboard, gate=Gate(ROOM, grid=lambda: None, clearance=0.4, require_map=True))
        for stamp in range(5):
            self.step(online, stamp=float(stamp))
        self.assertNotIn("object_ready", self.kinds())
        self.assertEqual(dashboard.posted, [])
        online.finish()

    def test_offline_inspection_waits_while_the_explorer_drives_and_burns_no_view(self):
        self.see("ladder", A, 0, 3.0)
        dashboard = FakeDashboard(exploring=True)
        navigator = DashboardNavigator(dashboard.call, timeout=5, poll=0, sleep=lambda s: None)
        inspection = Inspection(self.objects, self.keyframes, FormVlm(), self.store, self.survey, navigator,
                                camera=lambda: frame("x", facing(A, 0, 3.0), 9.0, depth=3.0),
                                min_sightings=1, mission="PATROL-1", by_section=False)
        inspection.sleep = lambda seconds: None
        inspection.run()
        self.assertNotIn("view_failed", self.kinds())
        self.assertIn("navigation_held", self.kinds())
        self.assertEqual(self.objects.get("LAD-001").status, "INSPECTED")

    def test_known_footprints_go_to_the_navigation_stack(self):
        dashboard = FakeDashboard()
        online = self.online(dashboard, obstacles=lambda cells: dashboard.call("PUT", "/api/obstacles", {"cells": cells}))
        for stamp in range(3):
            self.step(online, stamp=float(stamp))
            self.now[0] += 3.0
        self.assertTrue(dashboard.obstacles and dashboard.obstacles["cells"])
        online.finish()


if __name__ == "__main__":
    unittest.main()
