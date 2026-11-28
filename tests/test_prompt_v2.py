import json
from pathlib import Path
import tempfile
import unittest
from PIL import Image
from ultron.assessment import build_request,rubric,review_proposal,dashboard_payload,evidence_bundle_id
from ultron.patrol_assessment import record_manifest


class PromptV2Tests(unittest.TestCase):
    def fixture(self,root,pack):
        Image.new('RGB',(32,32),'red').save(root/'one.jpg')
        return record_manifest({'mission_id':'M1','asset_id':'L1','ladder_type':'LEANING_SINGLE',
            'position':[1,2,0],'synthetic':True,'oracle_key':'fixture','views':[{'rgb':'one.jpg'}]},root,pack=pack)

    def test_candidate_preserves_seven_sections_23_keys_and_legacy_default(self):
        self.assertEqual(len(rubric('inspection_v2')['sections']),7)
        old={c['key'] for s in rubric()['sections'] for c in s['criteria']}
        new={c['key'] for s in rubric('inspection_v2')['sections'] for c in s['criteria']}
        self.assertEqual(len(new),23);self.assertEqual(old,new)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=self.fixture(root,'inspection_v2')
            body=build_request(m,root,'test')
            self.assertEqual(body['text']['format']['name'],'ultron_seven_sections_v2')
            self.assertIn('Lead the summary with confirmed defects',body['instructions'])
            self.assertIn('spacing alone',body['instructions'])
            self.assertIn('must not erase a confirmed visible DEFECT',body['instructions'])
            oldmanifest={k:v for k,v in m.items() if k!='pack'}
            self.assertNotEqual(evidence_bundle_id(m),evidence_bundle_id(oldmanifest))
            baseline=record_manifest({'mission_id':'M2','asset_id':'L2','ladder_type':'LEANING_SINGLE',
                'position':[1,2,0],'synthetic':True,'views':[]},root)
            self.assertEqual(baseline.get('pack','inspection_v1'),'inspection_v1')

    def test_candidate_preserves_hard_failure_over_missing_coverage_and_no_safe_use(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=self.fixture(root,'inspection_v2')
            p=review_proposal(m,'Visible damage with missing views')
            p['criteria']['sprossen.beschaedigung'].update(state='DEFECT',evidence_ids=['frame-000'],confidence=.8)
            result=dashboard_payload(p,m)
            self.assertEqual(result['status'],'FLAGGED')
            self.assertFalse(result['safe_to_use'])
            self.assertTrue(result['human_signoff_required'])
            self.assertTrue(result['tactile_checks_pending'])

    def test_v2_references_remain_separate_and_missing_reference_cannot_bypass_validation(self):
        import hashlib
        from ultron.assessment import validate_proposal
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=self.fixture(root,'inspection_v2')
            m['design_id']='fixture-v1'
            m['reference_views']=[{'image_id':'reference-only','path':'one.jpg',
                'sha256':hashlib.sha256((root/'one.jpg').read_bytes()).hexdigest(),
                'design_id':'fixture-v1','design_match_verified':True,'intact_verified':True,
                'verification_source':'Synthetic geometry inspection'}]
            self.assertIn('REFERENCE_ONLY',str(build_request(m,root,'test')))
            p=review_proposal(m,'Unknown')
            p['criteria']['sprossen.beschaedigung'].update(state='DEFECT',evidence_ids=['reference-only'])
            with self.assertRaises(ValueError):validate_proposal(p,m)
            m['reference_views'][0]['intact_verified']=False
            with self.assertRaises(ValueError):build_request(m,root,'test')
