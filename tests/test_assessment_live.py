import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from ultron.assessment_live import load_settings, run_milestone, TransportError
from ultron.assessment import review_proposal


class LiveAssessmentTests(unittest.TestCase):
    def test_dotenv_alias_and_env_precedence_without_execution(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'.env';p.write_text('OpenAI_API="test-secret"\nULTRON_OPENAI_MODEL=test-model\n')
            self.assertEqual(load_settings(p,{})['key'],'test-secret')
            self.assertEqual(load_settings(p,{'OPENAI_API_KEY':'environment-secret'})['key'],'environment-secret')
            p.write_text('OpenAI_API="test-secret"\nULTRON_OPENAI_MODEL = explicit-model\n')
            self.assertTrue(load_settings(p,{})['model_explicit'])
            self.assertEqual(load_settings(p,{})['model'],'explicit-model')

    def fixture(self,root):
        from PIL import Image
        root.mkdir();(root/'frames').mkdir();views=[]
        for index,(angle,band) in enumerate((a,b) for a in range(0,360,45) for b in ['lower','middle','upper']):
            p=root/'frames'/f'{index}.jpg';Image.new('RGB',(20,20),(index*8,60,80)).save(p)
            views.append({'image_id':f'v-{index}','path':str(p.relative_to(root)),
                'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'azimuth_deg':angle,'band':band,
                'pose_valid':True,'same_asset_confirmed':True,'quality_accepted':True,'region_visible':True})
        m={'mission_id':'SOURCE-01','asset_id':'LAD-01','map_version':'test','inspection_date':'2026-11-28',
           'position':[1.,2.,0.],'ladder_type':'A_FRAME_STEPLADDER','verified_not_applicable':[],
           'synthetic':True,'views':views}
        (root/'keyframes.json').write_text(json.dumps(m));return root/'keyframes.json'

    def dashboard(self,external=False):
        class Dashboard:
            calls=[]
            def get(self,path):
                if path=='/api/live':return 200,{'mission':None}
                return 200,{'channels':['dashboard','email'] if external else ['dashboard']}
            def post(self,path,body=None,raw=None):
                self.calls.append((path,body,raw))
                return (200 if path.endswith('/end') else 201),{}
        return Dashboard()

    def test_complete_fake_transport_posts_photos_before_inspection_and_saves_usage(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);manifest=self.fixture(root/'source');output=root/'job';dash=self.dashboard()
            def api(body):
                m=json.loads((output/'keyframes.json').read_text());p=review_proposal(m,'Injected test fixture')
                return {'status':'completed','model':'test-model','usage':{'input_tokens':10,'output_tokens':20},
                    'output':[{'content':[{'type':'output_text','text':json.dumps(p)}]}]}
            report=run_milestone(manifest,output,model='test-model',api_call=api,dashboard=dash)
            self.assertTrue(report['api_validated']);self.assertTrue(report['dashboard_published'])
            self.assertEqual(report['transport_mode'],'INJECTED_TEST')
            paths=[c[0] for c in dash.calls]
            self.assertEqual(len([p for p in paths if '/photos/' in p]),24)
            self.assertLess(max(i for i,p in enumerate(paths) if '/photos/' in p),paths.index('/api/inspections'))
            self.assertEqual(json.loads((output/'usage.json').read_text())['output_tokens'],20)
            self.assertEqual(json.loads((output/'inspection.json').read_text())['status'],'MANUAL_REVIEW')

    def test_api_failure_is_published_as_review_and_not_retried(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);manifest=self.fixture(root/'source');count=[]
            def fail(body):count.append(1);raise TransportError('OpenAI transport unavailable')
            report=run_milestone(manifest,root/'job',model='test-model',api_call=fail,dashboard=self.dashboard())
            self.assertEqual(len(count),1);self.assertFalse(report['api_validated'])
            self.assertTrue(report['dashboard_published'])
            self.assertEqual(json.loads((root/'job'/'inspection.json').read_text())['status'],'MANUAL_REVIEW')

    def test_external_notification_channels_stop_before_api_call(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);manifest=self.fixture(root/'source');called=[]
            report=run_milestone(manifest,root/'job',model='test-model',api_call=lambda _:called.append(1),dashboard=self.dashboard(external=True))
            self.assertFalse(called);self.assertFalse(report['dashboard_published'])
            self.assertEqual(report['stage'],'DASHBOARD_PREFLIGHT')

    def test_upload_or_publication_failure_closes_created_mission(self):
        for failing_endpoint in ['photos','inspections']:
            with self.subTest(endpoint=failing_endpoint),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);manifest=self.fixture(root/'source');output=root/'job';delegate=self.dashboard()
                class FailingDashboard:
                    def get(self,path):return delegate.get(path)
                    def post(self,path,body=None,raw=None):
                        if ('/photos/' in path and failing_endpoint=='photos') or (path=='/api/inspections' and failing_endpoint=='inspections'):
                            return 500,{}
                        return delegate.post(path,body,raw)
                def api(body):
                    p=review_proposal(json.loads((output/'keyframes.json').read_text()),'Injected fixture')
                    return {'status':'completed','output':[{'content':[{'type':'output_text','text':json.dumps(p)}]}]}
                report=run_milestone(manifest,output,model='test-model',api_call=api,dashboard=FailingDashboard())
                self.assertFalse(report['dashboard_published'])
                self.assertTrue(report['failure_cleanup']['mission_ended'])
                end=[c for c in delegate.calls if c[0].endswith('/end')]
                self.assertEqual(end[-1][1]['status'],'ASSESSMENT_ABORTED')
