"""Online patrol: objects are read on a worker while the survey goes on to the next one."""
import threading
import unittest

from tests.test_pipeline import FormVlm, Opinion, Teleport, Workspace, facing, frame
from ultron.pipeline.inspection import Inspection
from ultron.pipeline.online import OnlinePatrol
from ultron.pipeline.verify import Verifier

A, B = (5.0, 5.0), (30.0, 5.0)  # two ladders far apart: neither is a fragment of the other


class GatedVlm(FormVlm):
    """Form reader that answers only once `gate` is set, so a test can watch the survey go on meanwhile."""

    def __init__(self):
        super().__init__()
        self.gate, self.started = threading.Event(), threading.Event()

    def inspect(self, frames):
        self.started.set()
        assert self.gate.wait(10), "the test never released the reader"
        return super().inspect(frames)


class OnlineTest(Workspace):
    def online(self, vlm, **options):
        inspection = Inspection(self.objects, self.keyframes, vlm, self.store, self.survey,
                                min_sightings=1, mission="PATROL-1", by_section=False)
        options = {"leave_seconds": 2.0, "leave_distance": 3.0, **options}
        return OnlinePatrol(self.survey, inspection, self.store, log=lambda *_: None, **options)

    def step(self, online, target, side, distance, stamp, kind="ladder"):
        online.step(frame(f"{kind}-{side}-{distance}-{stamp}", facing(target, side, distance), stamp, depth=distance))

    def kinds(self):
        return [e["kind"] for e in self.store.events(limit=None)]

    def test_a_ladder_left_behind_is_read_while_the_survey_goes_on_to_the_next(self):
        vlm = GatedVlm()
        online = self.online(vlm)
        for stamp in range(3):
            self.step(online, A, stamp % 2, 3.0, float(stamp))
        self.assertEqual(online.tracks["LAD-001"].state, "collecting")
        stamp = 3.0
        while not vlm.started.is_set() and stamp < 12:   # walk on to B: A falls behind
            self.step(online, B, 0, 3.0, stamp)
            stamp += 1
        self.assertTrue(vlm.started.wait(5))
        self.assertEqual(online.tracks["LAD-001"].state, "reading")
        ready = next(e for e in self.store.events(limit=None) if e["kind"] == "object_ready")
        self.assertEqual((ready["object"], ready["reason"]), ("LAD-001", "left behind"))
        # The reader is still busy with A, and the survey keeps finding and collecting B.
        for _ in range(3):
            self.step(online, B, 1, 3.0, stamp)
            stamp += 1
        self.assertEqual(online.tracks["LAD-002"].state, "collecting")
        self.assertEqual(online.focus, "LAD-002")
        self.assertNotIn("protocol", self.kinds())
        vlm.gate.set()
        for _ in range(20):                                # the sheet arrives during the walk, not after it
            self.step(online, B, 0, 3.0, stamp)
            stamp += 1
            if self.objects.get("LAD-001").status == "INSPECTED":
                break
        self.assertEqual(self.objects.get("LAD-001").status, "INSPECTED")
        self.assertIn("protocol", self.kinds())
        self.assertEqual(self.objects.get("LAD-002").status, "FOUND")
        online.finish()                                    # B is read when the stream ends
        self.assertEqual(self.objects.get("LAD-002").status, "INSPECTED")
        self.assertEqual(len(self.store.ladder("LAD-001")["inspections"]), 1)  # not inspected twice

    def test_the_vision_model_checks_an_object_before_its_form_is_read(self):
        self.survey.verifier = Verifier(Opinion({}, default="none"))
        vlm = FormVlm()
        online = self.online(vlm)
        for stamp in range(3):
            self.step(online, A, stamp % 2, 3.0, float(stamp))
        for stamp in range(3, 12):
            self.step(online, B, 0, 6.0, float(stamp))
        online.finish()
        self.assertEqual(self.objects.get("LAD-001").status, "REJECTED")
        self.assertEqual(vlm.calls, [])
        self.assertNotIn("object_ready", self.kinds())

    def test_coming_back_with_views_it_lacked_reads_the_object_again(self):
        vlm = FormVlm()
        online = self.online(vlm)
        for stamp in range(3):
            self.step(online, A, 0, 3.0, float(stamp))
        for stamp in range(3, 8):
            self.step(online, B, 0, 3.0, float(stamp))
        online.worker.submit(lambda: None).result()       # the read of A is done
        self.step(online, B, 0, 3.0, 8.0)
        self.assertEqual(online.tracks["LAD-001"].reads, 1)
        for stamp in range(9, 12):                         # back at A, now from close by and another side
            self.step(online, A, 2, 1.5, float(stamp))
        self.assertIn("object_reopened", self.kinds())
        for stamp in range(12, 18):
            self.step(online, B, 0, 3.0, float(stamp))
        online.finish()
        self.assertEqual(len(self.store.ladder("LAD-001")["inspections"]), 2)

    def test_view_goals_are_driven_off_thread_and_the_focus_gets_them(self):
        navigator = Teleport()
        online = self.online(FormVlm(), navigator=navigator, leave_seconds=100.0)
        for stamp in range(3):
            self.step(online, A, 0, 3.0, float(stamp))
        for stamp in range(3, 6):
            self.step(online, A, 0, 3.0, float(stamp))
            online.goal and online.goal[0].result()
        self.assertTrue(navigator.goals)
        self.assertEqual(navigator.goals[0], ("close", 0))
        requested = [e for e in self.store.events(limit=None) if e["kind"] == "view_requested"]
        self.assertEqual(requested[0]["object"], "LAD-001")
        online.finish()

    def test_a_candidate_nothing_explains_is_looked_at_then_explained_by_the_object_found_there(self):
        online = self.online(FormVlm(), candidates=lambda frame: [B])
        self.step(online, (0.0, 0.0), 0, 3.0, 0.0, kind="nothing")
        focus = [e for e in self.store.events(limit=None) if e["kind"] == "focus"][-1]
        self.assertIsNone(focus["object"])
        self.assertEqual(focus["view"], "look")
        for stamp in range(1, 4):
            self.step(online, B, 0, 3.0, float(stamp))
        self.assertIn("candidate_explained", self.kinds())
        self.assertEqual(online.candidates[0].explained_by, "LAD-001")
        online.finish()


if __name__ == "__main__":
    unittest.main()
