import json
from pathlib import Path
import tempfile
import unittest
from PIL import Image
from ultron.patrol_assessment import record_manifest, assess_job
from ultron.assessment import review_proposal


class PatrolAssessmentTests(unittest.TestCase):
    def fixture(self, root):
        Image.new('RGB', (32, 32), 'red').save(root/'frame.jpg')
        record = {'mission_id':'M1', 'asset_id':'L1', 'ladder_type':'A_FRAME_STEPLADDER',
                  'position':[2.,3.,0.], 'oracle_key':'ladder_1', 'synthetic':True,
                  'status':'MANUAL_REVIEW', 'checks':{}, 'views':[{'rgb':'frame.jpg'}]}
        return record_manifest(record, root)

    def test_manifest_no_invented_360_and_checksum_binding(self):
        with tempfile.TemporaryDirectory() as tmp:
            m = self.fixture(Path(tmp))
            self.assertFalse(m['coverage']['complete'])
            self.assertTrue(m['views'][0]['same_asset_confirmed'])
            self.assertEqual(len(m['views'][0]['sha256']), 64)
            self.assertNotIn('due_date', m)

    def test_transport_failure_produces_all_23_review_findings(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); m=self.fixture(root)
            def fail(body): raise OSError('secret transport exception must not be persisted')
            result=assess_job(m, root, root/'job', model='test', api_call=fail)
            self.assertEqual(result['status'], 'MANUAL_REVIEW')
            self.assertFalse(result['safe_to_use'])
            self.assertEqual(sum('.' in k for k in result['checklist']),23)
            self.assertNotIn('secret', (root/'job/result.json').read_text())

    def test_prior_hard_fail_cannot_be_cleared_and_job_is_immutable(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); m=self.fixture(root)
            m['prior_status']='FLAGGED'; m['prior_checks']={'rungs':'FAIL'}
            def reply(body):
                p=review_proposal(m,'Unknown')
                return {'status':'completed','output':[{'content':[{'type':'output_text','text':json.dumps(p)}]}]}
            result=assess_job(m,root,root/'job',model='test',api_call=reply)
            self.assertEqual(result['status'],'FLAGGED')
            self.assertEqual(result['checks'],{'rungs':'FAIL'})
            with self.assertRaises(FileExistsError):
                assess_job(m,root,root/'job',model='test',api_call=reply)

    def test_live_tailer_publishes_before_protocol_and_without_duplicate_api_call(self):
        from ultron.patrol_assessment import LiveAssessmentWorker
        from ultron.dashboard.live_tail import LiveTail
        class Api:
            def __init__(self):self.posts=[]
            def post(self,path,body=None,**kwargs):
                self.posts.append((path,body));return 201,{}
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=self.fixture(root);calls=[]
            (root/'job.json').write_text(json.dumps(m))
            def reply(body):
                calls.append(body)
                # The tagged mission is part of the immutable API evidence bundle.
                return {'status':'completed','output':[{'content':[{'type':'output_text',
                    'text':json.dumps(review_proposal({**m,'mission_id':'M1-live'},'Human review'))}]}]}
            worker=LiveAssessmentWorker(root,model='test',api_call=reply)
            try:
                rows=[{'time':0.,'mission_id':'M1','type':'MISSION_START'},
                      {'time':1.,'mission_id':'M1','type':'ASSESSMENT_READY','manifest':'job.json'}]
                (root/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
                api=Api();tail=LiveTail(root,api,tag='-live',assessment_worker=worker)
                tail.step();worker.publish_ready(api,wait=True)
                inspections=[b for p,b in api.posts if p=='/api/inspections']
                self.assertEqual(len(inspections),1)
                self.assertEqual(inspections[0]['mission_id'],'M1-live')
                self.assertEqual(len(calls),1)
                self.assertFalse((root/'protocol.json').exists())
                worker.submit('job.json','M1-live');tail.step()
                self.assertEqual(len(calls),1)
                self.assertTrue(any(p.endswith('/photos/'+inspections[0]['photos'][0]['id']) for p,b in api.posts))
            finally:worker.close()

    def test_model_defects_persist_across_jobs_and_ledger_reopen(self):
        from ultron.patrol_assessment import AssessmentLedger
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=self.fixture(root);ledger_path=root/'ledger.json'
            def defect(body):
                p=review_proposal(m,'Bent stile')
                p['criteria']['holme.verformung'].update(state='DEFECT',confidence=.8,evidence_ids=['frame-000'])
                return {'status':'completed','output':[{'content':[{'type':'output_text','text':json.dumps(p)}]}]}
            first=assess_job(m,root,root/'first',model='test',api_call=defect,ledger=AssessmentLedger(ledger_path))
            self.assertEqual(first['status'],'FLAGGED')
            later={**m,'mission_id':'M2'}
            Image.new('RGB',(32,32),'blue').save(root/'later.jpg')
            later['views']=[{**m['views'][0],'path':'later.jpg','sha256':__import__('hashlib').sha256((root/'later.jpg').read_bytes()).hexdigest()}]
            def clean(body):
                p=review_proposal(later,'Current image appears straight')
                p['criteria']['holme.verformung'].update(state='OK',confidence=.95,evidence_ids=['frame-000'])
                return {'status':'completed','output':[{'content':[{'type':'output_text','text':json.dumps(p)}]}]}
            second=assess_job(later,root,root/'second',model='test',api_call=clean,
                              ledger=AssessmentLedger(ledger_path))
            self.assertEqual(second['status'],'FLAGGED')
            self.assertEqual(second['checklist']['holme.verformung'],'DEFECT')
            self.assertIn('M1',second['reason'])
            self.assertNotIn('holme.verformung',second['item_photos'])
            retained=second['retained_findings']['holme.verformung']
            self.assertEqual(retained['photos'][0]['sha256'],m['views'][0]['sha256'])
            self.assertNotEqual(retained['photos'][0]['sha256'],later['views'][0]['sha256'])
            self.assertEqual(retained['reason'],'Bent stile')
            self.assertEqual(second['criterion_details']['holme.verformung']['confidence'],.8)

    def test_paid_budget_exhaustion_becomes_review_not_oracle_pass(self):
        from ultron.patrol_assessment import LiveAssessmentWorker
        class Api:
            def __init__(self):self.inspections=[]
            def post(self,path,body=None,**kwargs):
                if path=='/api/inspections':self.inspections.append(body)
                return 201,{}
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=self.fixture(root);calls=[]
            (root/'job.json').write_text(json.dumps(m))
            worker=LiveAssessmentWorker(root,model='test',api_call=lambda b:calls.append(b) or {},max_jobs=1)
            try:
                worker.submit('job.json','M1');worker.submit('job.json','M2')
                api=Api();worker.publish_ready(api,wait=True)
                self.assertEqual(len(calls),1)
                self.assertEqual(len(api.inspections),2)
                self.assertTrue(all(p['status']=='MANUAL_REVIEW' for p in api.inspections))
                self.assertIn('BUDGET_EXHAUSTED',api.inspections[1]['reason'])
            finally:worker.close()

    def test_publication_retry_does_not_make_another_paid_call(self):
        from ultron.patrol_assessment import LiveAssessmentWorker
        class Api:
            def __init__(self):self.failed=False;self.stored=None
            def post(self,path,body=None,**kwargs):
                if '/photos/' in path and not self.failed:
                    self.failed=True;return 500,{}
                if path=='/api/inspections':
                    if self.stored:return 409,{}
                    self.stored=body
                    raise OSError('Response lost after commit')
                return 201,{}
            def get(self,path):
                from ultron.dashboard.models import Inspection
                return 200,{'inspections':[Inspection(**self.stored).model_dump(mode='json')]}
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=self.fixture(root);(root/'job.json').write_text(json.dumps(m));calls=[]
            worker=LiveAssessmentWorker(root,model='test',api_call=lambda b:calls.append(b) or {})
            try:
                worker.submit('job.json','M1');api=Api()
                with self.assertRaises(RuntimeError):worker.publish_ready(api,wait=True)
                with self.assertRaises(OSError):worker.publish_ready(api,wait=True)
                self.assertEqual(worker.publish_ready(api,wait=True),1)
                self.assertEqual(len(calls),1)
                self.assertIn(('M1','L1'),worker.completed)
            finally:worker.close()

    def test_shared_ledger_concurrent_flags_are_not_lost(self):
        from concurrent.futures import ThreadPoolExecutor
        from ultron.patrol_assessment import AssessmentLedger
        from ultron.assessment import dashboard_payload
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=self.fixture(root);path=root/'shared.json'
            def flag(n):
                manifest={**m,'asset_id':f'L{n}'}
                proposal=review_proposal(manifest,'Visible bend')
                proposal['criteria']['holme.verformung'].update(state='DEFECT',confidence=.8,evidence_ids=['frame-000'])
                payload=dashboard_payload(proposal,manifest)
                payload['criterion_details']={k:{'reason':v['reason'],'confidence':v['confidence']} for k,v in proposal['criteria'].items()}
                AssessmentLedger(path).retain(manifest,payload)
            with ThreadPoolExecutor(max_workers=4) as pool:list(pool.map(flag,range(32)))
            self.assertEqual(len(json.loads(path.read_text())),32)
