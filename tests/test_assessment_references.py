import hashlib
from pathlib import Path
import tempfile
import unittest
from PIL import Image
from ultron.assessment import build_request,evidence_bundle_id,review_proposal,validate_proposal


class ReferenceAssessmentTests(unittest.TestCase):
    def manifest(self,root):
        for name,color in [('current','red'),('reference','green')]:
            Image.new('RGB',(32,32),color).save(root/(name+'.jpg'))
        def view(name):return {'image_id':name,'path':name+'.jpg',
            'sha256':hashlib.sha256((root/(name+'.jpg')).read_bytes()).hexdigest()}
        return {'mission_id':'M1','asset_id':'L1','position':[1,2,0],
            'inspection_date':'2026-11-28','map_version':'test','synthetic':True,
            'design_id':'fixture-v1','views':[{**view('current'),'same_asset_confirmed':True}],
            'reference_views':[{**view('reference'),'design_id':'fixture-v1',
                'design_match_verified':True,'intact_verified':True,
                'verification_source':'Synthetic fixture CAD geometry'}]}

    def test_reference_is_separate_from_current_evidence_and_changes_bundle_binding(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=self.manifest(root)
            b=build_request(m,root,'test')
            content=b['input'][0]['content']
            self.assertEqual(sum(x['type']=='input_image' for x in content),2)
            self.assertIn('REFERENCE_ONLY',str(content))
            self.assertIn('not current inspection evidence',b['instructions'])
            proposal=review_proposal(m,'Review')
            proposal['criteria']['sprossen.beschaedigung'].update(state='DEFECT',evidence_ids=['reference'])
            with self.assertRaises(ValueError):validate_proposal(proposal,m)
            changed={**m,'reference_views':[{**m['reference_views'][0],'sha256':'a'*64}]}
            self.assertNotEqual(evidence_bundle_id(m),evidence_bundle_id(changed))

    def test_unverified_or_wrong_design_reference_fails_before_network(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=self.manifest(root)
            for change in [{'intact_verified':False},{'design_match_verified':False},
                           {'design_id':'other'},{'verification_source':''},{'image_id':'current'}]:
                bad={**m,'reference_views':[{**m['reference_views'][0],**change}]}
                with self.assertRaises(ValueError):build_request(bad,root,'test')

    def test_reference_path_and_pixels_are_verified_and_empty_reference_is_legacy(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);m=self.manifest(root)
            for change in [{'path':'../outside.jpg'},{'sha256':'a'*64}]:
                bad={**m,'reference_views':[{**m['reference_views'][0],**change}]}
                with self.assertRaises(ValueError):build_request(bad,root,'test')
            legacy={k:v for k,v in m.items() if k not in ('reference_views','design_id')}
            self.assertEqual(evidence_bundle_id(legacy),evidence_bundle_id({**legacy,'reference_views':[]}))
