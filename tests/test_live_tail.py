import json
from pathlib import Path
import tempfile
import unittest


class FakeApi:
    def __init__(self):
        self.posts = []

    def post(self, path, body=None, raw=None, content_type='application/json'):
        self.posts.append((path, body if body is not None else ('raw', len(raw or b''))))
        return 201, {}


class LiveTailTests(unittest.TestCase):
    def test_streams_rows_incrementally_and_starts_mission_from_event(self):
        from ultron.dashboard.live_tail import LiveTail
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run / 'frames').mkdir()
            (run / 'frames/00000.jpg').write_bytes(b'\xff\xd8jpg')
            api = FakeApi()
            tail = LiveTail(run, api, site='test-site')
            self.assertEqual(tail.step(), 0)  # nothing yet
            (run / 'events.jsonl').write_text(json.dumps(
                {'time': 0.0, 'mission_id': 'M1', 'type': 'MISSION_START', 'synthetic': True,
                 'zone_bounds': [0, 0, 5, 5], 'keepouts': [], 'base': [1, 1]}) + '\n')
            (run / 'telemetry.jsonl').write_text(json.dumps(
                {'index': 0, 'time': 0.1, 'mission_id': 'M1', 'state': 'PATROL',
                 'pose': [1, 1, 0], 'rgb': 'frames/00000.jpg'}) + '\n')
            posted = tail.step()
            self.assertGreaterEqual(posted, 2)
            paths = [p for p, _ in api.posts]
            self.assertIn('/api/missions', paths)
            self.assertIn('/api/missions/M1/pose', paths)
            self.assertIn('/api/missions/M1/frames/front_rgb', paths)
            self.assertTrue(any(p == '/api/events' for p in paths))
            # appending another event streams only the new row
            with (run / 'events.jsonl').open('a') as f:
                f.write(json.dumps({'time': 0.2, 'mission_id': 'M1', 'type': 'PATROL_GOAL_SELECTED',
                                    'goal': {'schema': 'ultron.navigation_goal/v1'}}) + '\n')
            before = len(api.posts)
            tail.step()
            self.assertGreater(len(api.posts), before)


if __name__ == '__main__':
    unittest.main()
