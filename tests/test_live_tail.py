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

class LiveTailFailureTests(unittest.TestCase):
    def test_failed_event_is_retained_for_retry(self):
        from ultron.dashboard.live_tail import LiveTail
        class Flaky(FakeApi):
            def __init__(self):super().__init__();self.fail=True
            def post(self,path,*args,**kwargs):
                if path=='/api/events' and self.fail:
                    self.fail=False;return 500,{}
                return super().post(path,*args,**kwargs)
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp);api=Flaky();tail=LiveTail(run,api)
            (run/'events.jsonl').write_text(json.dumps({'mission_id':'M1','time':0.,'type':'MISSION_START'})+'\n')
            with self.assertRaises(RuntimeError):tail.step()
            self.assertEqual(len(tail.backlog['events.jsonl']),1)
            self.assertEqual(tail.step(),1)
            self.assertFalse(tail.backlog['events.jsonl'])

    def test_mission_end_failure_cannot_report_finished(self):
        from ultron.dashboard.live_tail import LiveTail
        class Flaky(FakeApi):
            def __init__(self):super().__init__();self.fail=True
            def post(self,path,*args,**kwargs):
                if path.endswith('/end') and self.fail:
                    self.fail=False;return 500,{}
                return super().post(path,*args,**kwargs)
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp);api=Flaky();tail=LiveTail(run,api)
            (run/'telemetry.jsonl').write_text('')
            (run/'protocol.json').write_text(json.dumps({'missions':[{'mission_id':'M1','records':[],'status':'COVERAGE_INCOMPLETE'}]}))
            with self.assertRaises(RuntimeError):tail.finish()
            self.assertFalse(tail.closed_missions)
            self.assertTrue(tail.finish())
            self.assertIn('M1',tail.closed_missions)

    def test_cloud_finish_without_job_event_never_falls_back_to_oracle_pass(self):
        from ultron.dashboard.live_tail import LiveTail
        from ultron.patrol_assessment import LiveAssessmentWorker
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);Image.new('RGB',(16,16),'red').save(root/'one.jpg')
            record={'mission_id':'M1','asset_id':'L1','ladder_type':'LEANING_SINGLE','position':[1.,2.,0.],
                    'status':'AUTOMATED_PASS','checks':{'rungs':'PASS'},'synthetic':True,
                    'oracle_key':'ladder_a','views':[{'rgb':'one.jpg'}]}
            (root/'telemetry.jsonl').write_text('')
            (root/'protocol.json').write_text(json.dumps({'missions':[{
                'mission_id':'M1','records':[record],'status':'MISSION_COMPLETE'}]}))
            calls=[];worker=LiveAssessmentWorker(root,model='test',api_call=lambda b:calls.append(b))
            try:
                api=FakeApi();tail=LiveTail(root,api,assessment_worker=worker)
                self.assertTrue(tail.finish())
                payload=next(b for p,b in api.posts if p=='/api/inspections')
                self.assertEqual(payload['status'],'MANUAL_REVIEW')
                self.assertEqual(len(payload['criterion_details']),23)
                self.assertFalse(calls)
            finally:worker.close()
