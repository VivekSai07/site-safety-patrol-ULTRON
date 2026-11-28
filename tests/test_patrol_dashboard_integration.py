"""Real dashboard ASGI routes and store; injected model response, no TCP or paid calls."""
import asyncio
import json
from pathlib import Path
import tempfile
import unittest
from PIL import Image
from ultron.assessment import review_proposal
from ultron.patrol_assessment import record_manifest, LiveAssessmentWorker


class PatrolDashboardIntegration(unittest.IsolatedAsyncioTestCase):
    async def test_evidence_and_23_explained_findings_survive_dashboard_routes(self):
        import httpx
        from ultron.dashboard.app import create_app
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);Image.new('RGB',(32,32),'red').save(root/'one.jpg')
            m=record_manifest({'mission_id':'M1','asset_id':'L1','ladder_type':'LEANING_SINGLE',
                              'position':[2.,3.,0.],'oracle_key':'ladder_a','synthetic':True,
                              'views':[{'rgb':'one.jpg'}]},root)
            (root/'job.json').write_text(json.dumps(m))
            def model(body):
                p=review_proposal(m,'Unresolved visual evidence')
                p['criteria']['holme.verformung'].update(state='DEFECT',confidence=.81,
                    reason='Visible bend in stile',evidence_ids=['frame-000'])
                return {'status':'completed','output':[{'content':[{'type':'output_text','text':json.dumps(p)}]}]}
            app=create_app(db_path=str(root/'dashboard.sqlite'),media_dir=root/'media',channels=[])
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app),base_url='http://test') as client:
                self.assertEqual((await client.post('/api/missions',json={
                    'mission_id':'M1','site_id':'integration','synthetic':True,'map_version':'sim'})).status_code,201)
                from unittest.mock import patch
                from ultron.patrol_assessment import assess_job
                from ultron.assessment import photo_upload_plan
                payload=assess_job(m,root,root/'assessment',model='injected-test',api_call=model)
                for photo in photo_upload_plan(m):
                    r=await client.post('/api/missions/M1/photos/'+photo['photo_id'],
                        content=(root/photo['path']).read_bytes(),headers={'Content-Type':'image/jpeg'})
                self.assertEqual(r.status_code,201)
                # This sandbox cannot wake asyncio from executor threads. Execute the
                # same notification function inline; TCP/thread dispatch stays unverified.
                async def inline(func,*args,**kwargs):return func(*args,**kwargs)
                with patch('ultron.dashboard.app.asyncio.to_thread',inline):
                    r=await client.post('/api/inspections',json=payload)
                self.assertEqual(r.status_code,201,r.text)
                response=await client.get('/api/missions/M1')
                record=response.json()['inspections'][0]
                self.assertEqual(record['status'],'FLAGGED')
                self.assertEqual(len(record['criterion_details']),23)
                self.assertEqual(record['criterion_details']['holme.verformung']['confidence'],.81)
                self.assertEqual(record['checklist']['holme.verformung'],'DEFECT')
                self.assertFalse(record['safe_to_use'])
                image=root/'media'/'M1'/(record['photos'][0]['id']+'.jpg')
                self.assertEqual(image.read_bytes(),(root/'one.jpg').read_bytes())

            app.state.store.db.close()
