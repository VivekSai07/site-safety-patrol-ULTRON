"""The ROS-free half of the bridge's exploration, fence and obstacle side (docs/EXPLORATION_CONTRACT.md)."""
import struct
import sys
import time
import unittest

try:
    from ultron.dashboard.ros_link import (Explorer, explore_on_connect, explore_request, fence_mask, fenced_grid,
                                           keepout_grid, obstacle_cloud)
    HAVE_LINK = True
except ImportError:  # Pillow missing
    HAVE_LINK = False

SLEEPER = f'{sys.executable} -c "import signal, sys, time; signal.signal(signal.SIGINT, lambda *a: sys.exit(0)); time.sleep(60)"'


def inside(x, y, polygon):
    """Brute force even-odd point-in-polygon."""
    hit = False
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y0 <= y) != (y1 <= y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            hit = not hit
    return hit


@unittest.skipUnless(HAVE_LINK, 'Pillow not installed')
class FenceTest(unittest.TestCase):
    def check(self, width, height, res, ox, oy, polygon):
        mask = fence_mask(width, height, res, ox, oy, polygon)
        expected = bytearray(int(inside(ox + (i + 0.5) * res, oy + (j + 0.5) * res, polygon))
                             for j in range(height) for i in range(width))
        self.assertEqual(mask, expected)
        return mask

    def test_scanline_matches_brute_force(self):
        self.check(40, 30, 0.1, -1.03, -0.51, [[0, 0], [2.02, 0.03], [1.51, 1.77], [0.13, 1.21]])
        # L-shaped room (concave), partly off the map on the left and top
        self.check(50, 20, 0.05, 0.01, 0.02, [[-0.5, 0.11], [1.83, 0.11], [1.83, 0.52], [0.73, 0.52], [0.73, 1.6], [-0.5, 1.6]])
        self.check(25, 25, 0.2, -2.49, -2.51, [[0, 2.03], [-1.97, -1.5], [2.01, -1.52]])   # triangle, clockwise
        self.check(10, 10, 0.1, 0.0, 0.0, [[5, 5], [6, 5], [6, 6]])                       # entirely off the map

    def test_square_room(self):
        # 4 x 3 cells of 1 m, fence around the middle two columns: centres at x 1.5 and 2.5
        mask = self.check(4, 3, 1.0, 0.0, 0.0, [[1.1, -1], [2.9, -1], [2.9, 9], [1.1, 9]])
        self.assertEqual(list(mask), [0, 1, 1, 0] * 3)
        self.assertEqual(fenced_grid([-1, 0, 50, 0] * 3, mask), [100, 0, 50, 100] * 3)
        self.assertEqual(keepout_grid(mask), [100, 0, 0, 100] * 3)

    def test_large_map_is_quick(self):
        start = time.monotonic()
        mask = fence_mask(1000, 1000, 0.05, -25, -25, [[-10, -8], [12, -8], [12, 9], [3, 9], [3, 4], [-10, 4]])
        fenced_grid(bytes(1000 * 1000), mask)
        self.assertLess(time.monotonic() - start, 3.0)

    def test_obstacle_cloud(self):
        count, data = obstacle_cloud([[1.05, 2.05], [-0.25, 3.0]])
        self.assertEqual((count, len(data)), (4, 48))
        points = [struct.unpack_from('<3f', data, 12 * i) for i in range(count)]
        self.assertEqual([tuple(round(v, 3) for v in p) for p in points],
                         [(1.05, 2.05, 0.05), (1.05, 2.05, 0.3), (-0.25, 3.0, 0.05), (-0.25, 3.0, 0.3)])
        self.assertEqual(obstacle_cloud([]), (0, b''))


@unittest.skipUnless(HAVE_LINK, 'Pillow not installed')
class ExploreDecisionTest(unittest.TestCase):
    def test_each_request_once(self):
        self.assertEqual(explore_request({'seq': 1, 'state': 'STARTING'}, 0), 'start')
        self.assertIsNone(explore_request({'seq': 1, 'state': 'EXPLORING'}, 1))     # our own confirmation
        self.assertEqual(explore_request({'seq': 2, 'state': 'PAUSING'}, 1), 'pause')
        self.assertIsNone(explore_request({'seq': 2, 'state': 'PAUSING'}, 2))       # lease renewed: same seq
        self.assertEqual(explore_request({'seq': 3, 'state': 'RESUMING'}, 2), 'resume')
        self.assertEqual(explore_request({'seq': 4, 'state': 'FINISHING'}, 3), 'finish')
        self.assertEqual(explore_request({'seq': 1, 'state': 'STARTING'}, 7), 'start')   # dashboard restarted

    def test_nothing_is_replayed_after_a_bridge_restart(self):
        self.assertEqual(explore_on_connect({'seq': 1, 'state': 'STARTING'}, None, False), 'fail')
        self.assertEqual(explore_on_connect({'seq': 3, 'state': 'PAUSED'}, None, False), 'fail')
        self.assertEqual(explore_on_connect({'seq': 4, 'state': 'FINISHING'}, None, False), 'finish')  # nothing runs: MAPPED
        self.assertIsNone(explore_on_connect({'seq': 2, 'state': 'MAPPED'}, None, False))

    def test_reconnect(self):
        self.assertIsNone(explore_on_connect({'seq': 2, 'state': 'EXPLORING'}, 2, True))          # stream blip
        self.assertEqual(explore_on_connect({'seq': 3, 'state': 'PAUSING'}, 2, True), 'pause')    # asked meanwhile
        self.assertEqual(explore_on_connect({'seq': 0, 'state': 'IDLE'}, 5, True), 'stop')        # dashboard restarted
        self.assertEqual(explore_on_connect({'seq': 2, 'state': 'EXPLORING'}, 2, False), 'fail')  # explorer gone


@unittest.skipUnless(HAVE_LINK, 'Pillow not installed')
class ExplorerProcessTest(unittest.TestCase):
    """The explorer as a real child process: a sleeping Python stands in for `ros2 launch explore_lite ...`."""

    def make(self, command=SLEEPER, listeners=1, status=None, **kw):
        self.reports, self.published, self.listening = [], [], listeners
        explorer = Explorer(command, self.published.append, lambda: self.listening,
                            lambda *r: self.reports.append(r), status, ready_wait_s=kw.pop('ready_wait_s', 2.0),
                            confirm_wait_s=0.3, stop_wait_s=2.0, poll_s=0.02, settle_s=0.2, **kw)
        self.addCleanup(explorer.stop)
        return explorer

    def test_start_pause_resume_finish(self):
        e = self.make()
        e.handle('start', 1)
        self.assertEqual(self.reports[-1], (1, 'EXPLORING', '', ''))
        proc = e.proc
        e.handle('pause', 2, 'pipeline')
        self.assertEqual((self.published, self.reports[-1]), ([False], (2, 'PAUSED', '', '')))
        e.handle('resume', 3)
        self.assertEqual((self.published, self.reports[-1]), ([False, True], (3, 'EXPLORING', '', '')))
        e.handle('finish', 4, 'operator')
        self.assertEqual((self.published, self.reports[-1]), ([False, True, False], (4, 'MAPPED', '', 'operator')))
        self.assertIsNotNone(proc.poll())        # the whole process group is gone
        self.assertFalse(e.running())

    def test_finish_while_paused_sends_no_second_stop(self):
        # explore_lite cancels every Nav2 goal on resume=false; paused, it has nothing of its own left
        e = self.make()
        e.handle('start', 1)
        e.handle('pause', 2)
        e.handle('finish', 3)
        self.assertEqual(self.published, [False])
        self.assertEqual(self.reports[-1][1], 'MAPPED')

    def test_explorer_that_cannot_start(self):
        e = self.make('/nonexistent/ros2 launch explore_lite explore.launch.py')
        e.handle('start', 1)
        self.assertEqual(self.reports[-1][1], 'FAILED')
        self.assertIn('could not start the explorer', self.reports[-1][2])
        e = self.make(f'{sys.executable} -c "raise SystemExit(3)"')
        e.handle('start', 1)
        self.assertEqual(self.reports[-1], (1, 'FAILED', 'explorer exited with code 3 during start-up', ''))
        e = self.make('"unbalanced')
        e.handle('start', 1)
        self.assertEqual(self.reports[-1][1], 'FAILED')

    def test_explorer_that_never_listens_is_stopped(self):
        e = self.make(listeners=0, ready_wait_s=0.3)
        e.handle('start', 1)
        self.assertEqual(self.reports[-1][1], 'FAILED')
        self.assertIn('no sign of life', self.reports[-1][2])
        self.assertFalse(e.running())

    def test_explorer_waiting_for_nav2_is_not_exploring(self):
        # explore_lite listens on explore/resume before it waits (for ever) for the Nav2 action server
        e = self.make(ready_wait_s=0.4, nav_ready=lambda: False)
        e.handle('start', 1)
        self.assertEqual(self.reports[-1][1], 'FAILED')
        self.assertIn('no Nav2', self.reports[-1][2])
        self.assertFalse(e.running())

    def test_pause_nobody_hears_stops_the_explorer(self):
        e = self.make()
        e.handle('start', 1)
        self.listening = 0
        e.handle('pause', 2)
        self.assertEqual(self.reports[-1], (2, 'FAILED', 'explorer not listening on explore/resume; stopped it', ''))
        self.assertFalse(e.running())
        self.assertEqual(self.published, [])

    def test_requests_without_an_explorer(self):
        e = self.make()
        e.handle('pause', 1)
        self.assertEqual(self.reports[-1], (1, 'FAILED', 'explorer not running', ''))
        e.handle('finish', 2)
        self.assertEqual(self.reports[-1], (2, 'MAPPED', '', 'operator'))
        e.handle('fail', 3)
        self.assertEqual(self.reports[-1][1:3], ('FAILED', 'explorer not running (bridge restarted); press Start again'))

    def test_watch_reports_a_dead_explorer(self):
        e = self.make()
        e.handle('start', 1)
        e.proc.kill()
        e.proc.wait()
        e.watch(1, 'EXPLORING')
        self.assertEqual(self.reports[-1], (1, 'FAILED', 'explorer exited with code -9', ''))
        self.assertIsNone(e.proc)

    def test_status_drives_confirmations(self):
        log = []
        say = log.append
        e = self.make(status=lambda: log, listeners=0, ready_wait_s=1.0)
        import threading
        threading.Timer(0.2, say, ['exploration_started']).start()
        e.handle('start', 1)
        self.assertEqual(self.reports[-1][:2], (1, 'EXPLORING'))
        self.listening = 1
        threading.Timer(0.05, say, ['exploration_paused']).start()
        e.handle('pause', 2)
        self.assertEqual(self.reports[-1][:2], (2, 'PAUSED'))
        e.handle('resume', 3)
        self.assertEqual(self.reports[-1][:2], (3, 'EXPLORING'))   # no status came: still confirmed, it was heard
        # explore_lite says complete and, with return_to_init, returning in the same breath
        say('exploration_complete')
        say('returning_to_origin')
        e.watch(3, 'EXPLORING')
        self.assertEqual(self.reports[-1], (3, 'MAPPED', 'no frontiers left', 'explorer'))
        say('returned_to_origin')
        e.watch(3, 'MAPPED')
        self.assertEqual(len([r for r in self.reports if r[1] == 'MAPPED']), 1)
        self.assertTrue(e.running())   # left idle (it may drive back to its start); the next Start replaces it
        old = e.proc
        threading.Timer(0.2, say, ['exploration_started']).start()
        e.handle('start', 4)
        self.assertIsNotNone(old.poll())
        self.assertEqual(self.reports[-1][:2], (4, 'EXPLORING'))

    def test_start_on_a_mapped_room_completes_at_once(self):
        # started, then at once complete (nothing left to explore): up, then MAPPED, not a 30 s FAILED
        log = []
        e = self.make(status=lambda: log, listeners=0, ready_wait_s=1.0)
        import threading
        threading.Timer(0.1, log.extend, [['exploration_started', 'exploration_complete']]).start()
        e.handle('start', 1)
        self.assertEqual(self.reports[-1][:2], (1, 'EXPLORING'))
        e.watch(1, 'EXPLORING')
        self.assertEqual(self.reports[-1], (1, 'MAPPED', 'no frontiers left', 'explorer'))

    def test_status_topic_we_cannot_hear(self):
        # msgs package present but no status arrives (QoS or namespace mismatch): listening + Nav2 still counts
        e = self.make(status=lambda: [], listeners=1)
        e.handle('start', 1)
        self.assertEqual(self.reports[-1][:2], (1, 'EXPLORING'))

    def test_a_newer_request_cuts_a_slow_start_short(self):
        e = self.make(listeners=0, ready_wait_s=30.0, interrupted=lambda: True)
        start = time.monotonic()
        e.handle('start', 1)
        self.assertLess(time.monotonic() - start, 2.0)
        self.assertFalse(e.running())


if __name__ == '__main__':
    unittest.main()
