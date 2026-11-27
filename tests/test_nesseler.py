import unittest
from ultron.nesseler import checklist_record


class NesselerTests(unittest.TestCase):
    def test_every_policy_component_survives_export(self):
        from ultron.inspection import REQUIRED
        components=set(c for fields in REQUIRED.values() for c in fields)
        record=checklist_record({'checks':dict.fromkeys(components,'FAIL')})
        self.assertEqual(set(record['raw_checks']),components)
        mapped={k for s in record['sections'] for k in s['observations']}
        self.assertEqual(mapped,components-{'usage_state','type_scope'})

    def test_partial_visual_pass_never_completes_official_checklist(self):
        record=checklist_record({'asset_id':'LAD-001','status':'AUTOMATED_PASS',
            'checks':{'spreader':'PASS'},'views':[{'rgb':'frames/one.jpg'}]})
        self.assertEqual(len(record['sections']),7)
        self.assertEqual(record['result']['human_decision'],'PENDING')
        self.assertFalse(record['result']['safe_to_use'])
        self.assertTrue(all(c['status']=='PLEASE_CHECK' for s in record['sections'] for c in s['criteria']))
        self.assertEqual(record['sections'][2]['observations']['spreader'],'PASS')

    def test_flag_and_model_evidence_are_retained_without_certification(self):
        record=checklist_record({'asset_id':'LAD-002','status':'FLAGGED',
            'checks':{'rungs':'FAIL'},'model_evidence':[{'raw':'untrusted proposal'}],
            'views':[{'rgb':'frames/x.jpg','model_response':'responses/x.txt'}]})
        self.assertEqual(record['result']['screening_status'],'FLAGGED')
        self.assertEqual(record['sections'][1]['observations']['rungs'],'FAIL')
        self.assertEqual(record['sections'][1]['evidence'][0]['model_response'],'responses/x.txt')
        self.assertFalse(record['result']['safe_to_use'])
