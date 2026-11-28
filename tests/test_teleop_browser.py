import os
from pathlib import Path
import shutil
import threading
import unittest
import uuid
from http.server import ThreadingHTTPServer

try:
    from playwright.sync_api import expect, sync_playwright
except ImportError:
    sync_playwright = None

from test_teleop import FakeSimulation
from ultron.teleop import TeleopController, make_handler


@unittest.skipUnless(
    sync_playwright and os.environ.get("ULTRON_TEST_BROWSER"),
    "optional browser regression requires Playwright and ULTRON_TEST_BROWSER",
)
class TeleopBrowserTests(unittest.TestCase):
    def setUp(self):
        self.work = Path(__file__).resolve().parent / f".teleop-browser-{uuid.uuid4().hex}"
        self.work.mkdir()
        self.addCleanup(shutil.rmtree, self.work, ignore_errors=True)
        self.controller = TeleopController(FakeSimulation(), self.work / "sessions")
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(self.controller, 0))
        self.port = self.server.server_address[1]
        self.server.RequestHandlerClass = make_handler(self.controller, self.port)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.thread.join, timeout=2)
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.playwright = sync_playwright().start()
        self.addCleanup(self.playwright.stop)
        self.context = self.playwright.chromium.launch_persistent_context(
            str(self.work / "profile"),
            executable_path=os.environ["ULTRON_TEST_BROWSER"],
            headless=True,
            args=[
                "--disable-background-networking",
                "--disable-dev-shm-usage",
                "--host-resolver-rules=MAP * ~NOTFOUND, EXCLUDE localhost, EXCLUDE 127.0.0.1",
            ],
        )
        self.addCleanup(self.context.close)
        self.context.route(
            "**/*",
            lambda route: route.continue_()
            if route.request.url.startswith(f"http://127.0.0.1:{self.port}/")
            else route.abort(),
        )
        self.page = self.context.new_page()
        self.requests = []
        self.page.on(
            "request",
            lambda request: self.requests.append(request.url)
            if "/api/move" in request.url
            else None,
        )
        self.page.goto(f"http://127.0.0.1:{self.port}/")
        self.page.locator("#session").fill("browser-test")
        self.page.locator("#start").click()
        expect(self.page.locator("#sessioninfo")).to_contain_text("browser-test")

    def test_stop_prevents_new_keyboard_motion_requests(self):
        self.page.locator("#arm").click()
        expect(self.page.locator("#status")).to_contain_text("Motion: ENABLED")
        self.page.keyboard.down("w")
        self.page.wait_for_timeout(350)
        self.page.locator("#stop").click()
        expect(self.page.locator("#status")).to_contain_text("Motion: STOPPED")
        self.page.keyboard.up("w")
        self.page.wait_for_timeout(250)
        before = len(self.requests)
        self.page.keyboard.press("w")
        self.page.wait_for_timeout(250)
        self.assertGreater(before, 0)
        self.assertEqual(len(self.requests), before)
        self.assertFalse(self.controller.status()["armed"])

    def test_held_key_retries_after_render_contention(self):
        self.page.locator("#arm").click()
        expect(self.page.locator("#status")).to_contain_text("Motion: ENABLED")
        self.controller._simulation_lock.acquire()
        try:
            self.page.keyboard.down("w")
            self.page.wait_for_timeout(350)
            self.assertEqual(self.controller.simulation.x, 0)
        finally:
            self.controller._simulation_lock.release()
        self.page.wait_for_timeout(400)
        self.page.keyboard.up("w")
        self.assertGreater(self.controller.simulation.x, 0)

    def test_pointer_drag_selects_and_captures_label_roi(self):
        preview = self.page.locator("#preview")
        for _ in range(50):
            if preview.evaluate("el => el.complete && el.naturalWidth > 0"):
                break
            self.page.wait_for_timeout(100)
        else:
            self.fail("preview failed to load")
        preview.scroll_into_view_if_needed()
        box = preview.bounding_box()
        self.assertIsNotNone(box)
        self.page.mouse.move(box["x"] + box["width"] * .2, box["y"] + box["height"] * .2)
        self.page.mouse.down()
        self.page.mouse.move(
            box["x"] + box["width"] * .8, box["y"] + box["height"] * .8, steps=6
        )
        self.page.mouse.up()
        expect(self.page.locator("#notice")).to_contain_text("Selected ROI")
        self.page.locator("#captureLabel").click()
        expect(self.page.locator("#sessioninfo")).to_contain_text("2 evidence items")
        evidence = self.controller.session.list_evidence()
        crop = next(item for item in evidence if item["kind"] == "DERIVED_CROP")
        label = next(item for item in evidence if item["kind"] == "LABEL")
        self.assertEqual(crop["parent_evidence_id"], label["evidence_id"])

    def test_synthetic_focus_and_visibility_events_disarm(self):
        for expression, reason in (
            ("window.dispatchEvent(new Event('blur'))", "FOCUS_LOST"),
            (
                "Object.defineProperty(document, 'hidden', {value: true, configurable: true});"
                "document.dispatchEvent(new Event('visibilitychange'));"
                "delete document.hidden",
                "PAGE_HIDDEN",
            ),
        ):
            self.page.locator("#arm").click()
            expect(self.page.locator("#status")).to_contain_text("Motion: ENABLED")
            self.page.evaluate("() => {" + expression + "}")
            expect(self.page.locator("#status")).to_contain_text(f"Motion: STOPPED ({reason})")
            before = len(self.requests)
            self.page.keyboard.press("w")
            self.page.wait_for_timeout(250)
            self.assertEqual(len(self.requests), before)


if __name__ == "__main__":
    unittest.main()
