import json
from pathlib import Path
import tempfile
import unittest

from ultron.assessment import rubric, assess_coverage, validate_proposal, dashboard_payload, parse_api_response, build_request, evidence_bundle_id


class AssessmentTests(unittest.TestCase):
    def proposal(self,manifest=None):
        manifest=manifest or self.manifest()
        return {'bundle_id':evidence_bundle_id(manifest),'asset_id':manifest['asset_id'],'criteria':{c['key']:{'state':'UNKNOWN','reason':'Not visible',
            'confidence':0.0,'evidence_ids':[]} for s in rubric()['sections'] for c in s['criteria']},
            'label':{'raw_text':None,'due_date_iso':None,'due_date_explicit':False,'evidence_ids':[]},
            'summary':'Unresolved evidence'}

    def manifest(self):
        return {'mission_id':'PATROL-01','asset_id':'LAD-001','map_version':'test',
            'inspection_date':'2026-11-27','ladder_type':'A_FRAME_STEPLADDER',
            'position':[1.,2.,0.],'views':[],'verified_not_applicable':[]}

    def test_missing_vertical_and_rear_views_cannot_claim_360(self):
        result=assess_coverage(self.manifest())
        self.assertFalse(result['complete'])
        self.assertEqual(len(result['missing_cells']),24)

    def test_all_24_distinct_valid_sampling_cells_and_reused_image_failure(self):
        import hashlib
        m=self.manifest()
        for angle in range(0,360,45):
            for band in ['lower','middle','upper']:
                m['views'].append({'image_id':f'a{angle}-{band}','azimuth_deg':angle,'band':band,
                    'pose_valid':True,'same_asset_confirmed':True,'quality_accepted':True,'region_visible':True,
                    'sha256':hashlib.sha256(f'{angle}-{band}'.encode()).hexdigest()})
        self.assertTrue(assess_coverage(m)['complete'])
        m['views'][-1]['sha256']=m['views'][0]['sha256']
        self.assertFalse(assess_coverage(m)['complete'])

    def test_unknown_and_model_clean_results_never_publish_pass(self):
        p=self.proposal();m=self.manifest()
        result=dashboard_payload(p,m)
        self.assertEqual(result['status'],'MANUAL_REVIEW')
        self.assertFalse(result['safe_to_use'])
        self.assertTrue(result['human_signoff_required'])
        self.assertEqual(len([k for k in result['checklist'] if '.' in k]),23)
        m['views']=[{'image_id':'one','sha256':'a'*64}]
        p=self.proposal(m)
        for section in rubric()['sections']:
            for c in section['criteria']:
                p['criteria'][c['key']].update(state='NOT_CHECKED' if c['mode']=='person' else 'OK',
                    reason='No visible issue in this fixture',confidence=.99,evidence_ids=['one'])
        p['label']={'raw_text':'Next inspection 2030-12','due_date_iso':'2030-12','due_date_explicit':True,'evidence_ids':['one']}
        self.assertEqual(dashboard_payload(p,m)['status'],'MANUAL_REVIEW')

    def test_unknown_evidence_duplicate_scope_and_person_pass_are_rejected(self):
        m=self.manifest();p=self.proposal()
        p['criteria']['holme.verformung'].update(state='DEFECT',evidence_ids=['alien'])
        with self.assertRaises(ValueError):validate_proposal(p,m)
        p=self.proposal();p['criteria']['spreiz.funktion']['state']='OK'
        with self.assertRaises(ValueError):validate_proposal(p,m)
        p=self.proposal();p['criteria'].pop('holme.verformung')
        with self.assertRaises(ValueError):validate_proposal(p,m)

    def test_defect_dominates_missing_360_and_maps_exact_dashboard_keys(self):
        m=self.manifest();m['views']=[{'image_id':'view-1','sha256':'a'*64}]
        p=self.proposal(m);p['criteria']['holme.verformung'].update(state='DEFECT',reason='Visible bend',confidence=.95,evidence_ids=['view-1'])
        result=dashboard_payload(p,m)
        self.assertEqual(result['status'],'FLAGGED')
        self.assertEqual(result['checklist']['holme'],'DEFECT')
        self.assertEqual(result['item_photos']['holme.verformung'],[result['photos'][0]['id']])

    def test_two_ladders_with_same_local_image_id_never_overwrite_evidence(self):
        m=self.manifest();m['views']=[{'image_id':'view-1','sha256':'a'*64}]
        first=dashboard_payload(self.proposal(m),m)['photos'][0]['id']
        m['asset_id']='LAD-002';p=self.proposal(m)
        second=dashboard_payload(p,m)['photos'][0]['id']
        self.assertNotEqual(first,second)
        m['views'][0]['sha256']='b'*64
        third=dashboard_payload(self.proposal(m),m)['photos'][0]['id']
        self.assertNotEqual(second,third)

    def test_unknown_position_is_not_fabricated_as_origin(self):
        m=self.manifest();m['position']=None
        with self.assertRaises(ValueError):dashboard_payload(self.proposal(m),m)

    def test_stale_response_rejected_for_new_mission_or_changed_pixels(self):
        m=self.manifest();m['views']=[{'image_id':'view-1','sha256':'a'*64}]
        p=self.proposal(m)
        self.assertIs(validate_proposal(p,m),p)
        m['views'][0]['sha256']='b'*64
        with self.assertRaises(ValueError):validate_proposal(p,m)
        m['views'][0]['sha256']='a'*64;m['mission_id']='PATROL-02'
        with self.assertRaises(ValueError):validate_proposal(p,m)

    def test_refusal_incomplete_and_malformed_response_produce_review(self):
        m=self.manifest()
        cases=[{'status':'incomplete'}, {'status':'completed','output':[{'content':[{'type':'refusal'}]}]},
               {'status':'completed','output':[{'content':[{'type':'output_text','text':'not json'}]}]}]
        for response in cases:
            p=parse_api_response(response,m)
            self.assertEqual(dashboard_payload(p,m)['status'],'MANUAL_REVIEW')

    def test_request_hash_verification_and_root_confinement(self):
        import hashlib
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);path=root/'one.jpg';Image.new('RGB',(20,20),'red').save(path)
            m=self.manifest();m['views']=[{'image_id':'one','path':'one.jpg','same_asset_confirmed':True,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}]
            request=build_request(m,root,'test-model')
            self.assertEqual(sum(i['type']=='input_image' for i in request['input'][0]['content']),1)
            self.assertTrue(request['text']['format']['strict'])
            self.assertFalse(request['store'])
            m['views'][0]['sha256']='0'*64
            with self.assertRaises(ValueError):build_request(m,root,'test-model')
            m['views'][0]['path']='../outside.jpg'
            with self.assertRaises(ValueError):build_request(m,root,'test-model')
