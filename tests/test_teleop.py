import json
import os
from pathlib import Path
import queue
import shutil
import threading
from types import SimpleNamespace
from http.server import ThreadingHTTPServer
import urllib.error
import urllib.request
import time
import unittest
from unittest.mock import patch
import uuid

try:
    from PIL import Image
except ImportError:
    Image = None

from ultron.kit_sim import KitSimulation, build_scene_xml
from ultron.teleop import TeleopController, make_handler, serve
from ultron.adaptive_draft import build_adaptive_draft_from_file


class FakeSimulation:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.blocked = False
        self.frames = 0
        self.view = "overview"
        self._rgb = bytes((255, 255, 255)) * 64 * 48

    def step(self, action):
        if self.blocked:
            return False
        if action == "forward":
            self.x += 0.1
        elif action == "back":
            self.x -= 0.1
        elif action == "turn_left":
            self.yaw += 0.1
        elif action == "turn_right":
            self.yaw -= 0.1
        else:
            raise ValueError("invalid action")
        return True

    def pose(self):
        return {"x_m": self.x, "y_m": self.y, "yaw_rad": self.yaw}

    def camera_metadata(self):
        return {"type": self.view, "onboard_sensor": self.view == "inspection"}

    def set_view(self, view):
        if view not in {"overview", "inspection"}:
            raise ValueError("invalid view")
        self.view = view

    def render_rgb(self):
        self.frames += 1
        return self._rgb

    def frame_size(self):
        return 64, 48


class TeleopControllerTests(unittest.TestCase):
    def setUp(self):
        self.work = Path(__file__).resolve().parent / f".teleop-test-{uuid.uuid4().hex}"
        self.simulation = FakeSimulation()
        self.controller = TeleopController(self.simulation, self.work)

    def tearDown(self):
        shutil.rmtree(self.work, ignore_errors=True)

    def _session_and_arm(self):
        status = self.controller.start_session(ladder_id="LAD-001", session_id="session-a")
        self.assertEqual(status["mode"], "SIMULATED KINEMATIC POSITIONING — NOT WALKING")
        self.assertFalse(status["armed"])
        return self.controller.arm()

    def _move(self, status, *, sequence=0, action="forward", sent_at_ms=None):
        return self.controller.move(
            {
                "action": action,
                "sequence": sequence,
                "sent_at_ms": sent_at_ms or int(time.time() * 1000),
                "generation": status["generation"],
            }
        )

    def test_motion_requires_session_and_explicit_arm_then_stops_on_focus_loss(self):
        with self.assertRaisesRegex(ValueError, "session"):
            self.controller.arm()
        armed = self._session_and_arm()
        moved = self._move(armed)
        self.assertTrue(moved["moved"])
        self.assertAlmostEqual(moved["pose"]["x_m"], 0.1)

        stopped = self.controller.stop("FOCUS_LOST")
        self.assertFalse(stopped["armed"])
        with self.assertRaisesRegex(ValueError, "stopped"):
            self._move(armed, sequence=1)

    def test_enable_request_delayed_past_stop_cannot_rearm(self):
        status = self.controller.start_session(ladder_id="LAD-001", session_id="session-a")
        self.controller.stop("FOCUS_LOST")
        with self.assertRaisesRegex(ValueError, "stale enable"):
            self.controller.arm(expected_generation=status["generation"])
        self.assertFalse(self.controller.status()["armed"])

    def test_selecting_inspection_camera_is_recorded_with_capture(self):
        self._session_and_arm()
        self.assertEqual(self.controller.set_view("inspection")["camera"]["type"], "inspection")
        result = self.controller.capture(kind="OVERVIEW")
        self.assertEqual(result["evidence"]["camera"]["type"], "inspection")
        with self.assertRaises(ValueError):
            self.controller.set_view("invalid")

    def test_rejects_stale_out_of_order_and_unbounded_action_inputs(self):
        armed = self._session_and_arm()
        with self.assertRaisesRegex(ValueError, "stale"):
            self._move(armed, sent_at_ms=1)
        self._move(armed, sequence=2)
        with self.assertRaisesRegex(ValueError, "out-of-order"):
            self._move(armed, sequence=1)
        with self.assertRaisesRegex(ValueError, "not supported"):
            self._move(armed, sequence=3, action="move_to_xyz")

    def test_motion_rate_is_bounded_even_for_direct_api_clients(self):
        armed = self._session_and_arm()
        with patch("ultron.teleop.time.monotonic", side_effect=(4.0, 4.05)):
            self._move(armed, sequence=0)
            with self.assertRaisesRegex(ValueError, "rate limit"):
                self._move(armed, sequence=1)

    def test_stop_does_not_wait_for_pending_render(self):
        armed = self._session_and_arm()
        entered = threading.Event()
        release = threading.Event()
        original_render = self.simulation.render_rgb

        def delayed_render():
            entered.set()
            if not release.wait(5):
                raise RuntimeError("test render was never released")
            return original_render()

        self.simulation.render_rgb = delayed_render
        renderer = threading.Thread(target=self.controller.render_frame)
        renderer.start()
        self.assertTrue(entered.wait(2))
        try:
            with self.assertRaisesRegex(ValueError, "busy rendering"):
                self._move(armed)
            stopped = self.controller.stop("PROBE_STOP")
            self.assertFalse(stopped["armed"])
        finally:
            release.set()
            renderer.join(timeout=3)
        self.assertFalse(renderer.is_alive())

    def test_renderer_context_is_created_used_and_closed_on_one_thread(self):
        threads = []

        class FakeRenderer:
            def __init__(self, _model, *, height, width):
                threads.append(("create", threading.get_ident()))

            def update_scene(self, _data, *, camera):
                threads.append(("update", threading.get_ident()))

            def render(self):
                threads.append(("render", threading.get_ident()))
                return SimpleNamespace(tobytes=lambda **kwargs: b"frame")

            def close(self):
                threads.append(("close", threading.get_ident()))

        sim = KitSimulation.__new__(KitSimulation)
        sim._mujoco = SimpleNamespace(Renderer=FakeRenderer)
        sim.model, sim.data = object(), object()
        sim._height_pixels, sim._width = 120, 160
        sim._view, sim._camera = "overview", object()
        sim._render_queue = queue.Queue()
        sim._render_thread = threading.Thread(target=sim._render_loop)
        sim._render_thread.start()
        responses = []
        callers = [threading.Thread(target=lambda: responses.append(sim.render_rgb())) for _ in range(3)]
        for caller in callers:
            caller.start()
        for caller in callers:
            caller.join(timeout=3)
        sim.close()
        self.assertEqual(responses, [b"frame"] * 3)
        self.assertEqual([entry[0] for entry in threads].count("render"), 3)
        self.assertEqual(len({entry[1] for entry in threads}), 1)
        self.assertEqual(threads[-1][0], "close")

    def test_blocked_kinematic_increment_does_not_change_pose(self):
        armed = self._session_and_arm()
        self.simulation.blocked = True
        result = self._move(armed)
        self.assertTrue(result["blocked"])
        self.assertEqual(result["pose"]["x_m"], 0.0)

    def test_label_roi_keeps_original_and_links_derived_crop(self):
        self._session_and_arm()
        self.controller.capture(kind="OVERVIEW")
        result = self.controller.capture(
            kind="LABEL", roi={"x1": 8, "y1": 8, "x2": 56, "y2": 40}
        )
        self.assertEqual(result["evidence"]["kind"], "LABEL")
        self.assertEqual(result["crop"]["kind"], "DERIVED_CROP")
        self.assertEqual(
            result["crop"]["parent_evidence_id"], result["evidence"]["evidence_id"]
        )
        self.assertEqual(result["crop"]["provenance"]["source_metadata"]["reader"], "NONE")
        self.assertEqual(len(self.controller.session.list_evidence()), 3)
        manifest_path = self.controller.session.path / "adaptive_manifest.template.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.assertEqual(manifest["schema_version"], "1.0.0")
        self.assertEqual([view["kind"] for view in manifest["views"]], ["OVERVIEW", "LABEL"])
        self.assertEqual(manifest["views"][1]["visual_fields"], [])
        self.assertEqual(
            manifest["views"][1]["provenance"]["suggested_visual_field_targets_unreviewed"],
            [],
        )
        self.assertEqual(manifest["views"][1]["crops"][0]["box"], [8, 8, 56, 40])

    @unittest.skipIf(Image is None, "Pillow is required by the adaptive draft pipeline")
    def test_teleop_manifest_invokes_adaptive_pipeline_locally(self):
        self._session_and_arm()
        self.controller.capture(kind="OVERVIEW")
        self.controller.capture(
            kind="LABEL", roi={"x1": 8, "y1": 8, "x2": 56, "y2": 40}
        )
        manifest_path = self.controller.session.path / "adaptive_manifest.template.json"
        draft = build_adaptive_draft_from_file(
            manifest_path,
            sessions_root=self.work / "draft-sessions",
            ladder_id="LAD-001",
            session_id="draft-copy",
        )
        self.assertTrue(Path(draft["draft_json"]).is_file())
        self.assertTrue(Path(draft["review_html"]).is_file())

    def test_label_capture_requires_a_valid_roi_and_session_ids_are_safe(self):
        self._session_and_arm()
        with self.assertRaisesRegex(ValueError, "requires"):
            self.controller.capture(kind="LABEL")
        with self.assertRaisesRegex(ValueError, "inside"):
            self.controller.capture(
                kind="LABEL", roi={"x1": 4, "y1": 4, "x2": 80, "y2": 20}
            )
        with self.assertRaisesRegex(ValueError, "safe path-component"):
            self.controller.start_session(ladder_id="LAD-001", session_id="../escape")

    def test_scene_uses_only_the_organizer_b2_model(self):
        b2_dir = self.work / "unitree_robots" / "b2"
        b2_dir.mkdir(parents=True)
        model = b2_dir / "b2.xml"
        model.write_text("<mujoco/>", encoding="utf-8")
        scene = build_scene_xml(model)
        self.assertIn(str(b2_dir / "assets"), scene)
        self.assertIn('name="fixture_ladder_alpha"', scene)
        self.assertNotIn("src/ultron/assets/b2", scene)
        with self.assertRaises(ValueError):
            build_scene_xml(self.work / "upstream.xml")

    def test_non_loopback_listener_requires_isolation_marker(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(ValueError, "private container network"):
                serve(self.controller, host="0.0.0.0", port=8765)

    def test_local_ui_session_motion_preview_and_capture_routes(self):
        server = ThreadingHTTPServer(
            ("127.0.0.1", 0), make_handler(self.controller, 8765)
        )
        port = server.server_address[1]
        server.RequestHandlerClass = make_handler(self.controller, port)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{port}"

        def request(path, payload=None, *, origin=None):
            headers = {}
            body = None
            if payload is not None:
                body = json.dumps(payload).encode("utf-8")
                headers["Content-Type"] = "application/json"
            if origin is not None:
                headers["Origin"] = origin
            req = urllib.request.Request(base + path, data=body, headers=headers)
            return urllib.request.urlopen(req, timeout=2)

        try:
            with request("/") as response:
                self.assertIn(b"NOT WALKING", response.read())
            with self.assertRaises(urllib.error.HTTPError) as denied:
                request("/api/status", origin="http://example.invalid")
            self.assertEqual(denied.exception.code, 403)
            with request(
                "/api/session",
                {"ladder_id": "LAD-001", "session_id": "http-session"},
            ) as response:
                session_status = json.load(response)
            with self.assertRaises(urllib.error.HTTPError) as invalid_arm:
                request("/api/arm", {})
            self.assertEqual(invalid_arm.exception.code, 400)
            with request("/api/arm", {"generation": session_status["generation"]}) as response:
                armed_status = json.load(response)
            with request(
                "/api/move",
                {
                    "action": "forward",
                    "sequence": 0,
                    "sent_at_ms": int(time.time() * 1000),
                    "generation": armed_status["generation"],
                },
            ) as response:
                self.assertTrue(json.load(response)["moved"])
            with request("/api/frame") as response:
                self.assertEqual(response.headers.get_content_type(), "image/png")
                self.assertTrue(response.read())
            with request("/api/capture", {"kind": "OVERVIEW"}) as response:
                evidence = json.load(response)["evidence"]
            self.assertEqual(evidence["kind"], "OVERVIEW")
            self.assertEqual(session_status["session_id"], "http-session")
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
