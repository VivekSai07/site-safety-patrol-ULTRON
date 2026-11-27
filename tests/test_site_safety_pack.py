import json
from pathlib import Path
import unittest

ASSETS = Path(__file__).resolve().parents[1] / 'src/ultron/assets/site_safety_v1'
KEYS = ['ext.vorhanden', 'ext.zugaenglich', 'ext.pruefplakette',
        'firstaid.aushang', 'firstaid.kasten',
        'flucht.kennzeichnung', 'flucht.frei',
        'psa.helm', 'psa.warnweste']


class SiteSafetyPackTests(unittest.TestCase):
    def test_pack_files_rubric_keys_and_schema_agree(self):
        rubric = json.loads((ASSETS / 'rubric.json').read_text())
        self.assertEqual(rubric['version'], 'ultron.site_safety.v1')
        keys = [c['key'] for s in rubric['sections'] for c in s['criteria']]
        self.assertEqual(keys, KEYS)
        for s in rubric['sections']:
            for c in s['criteria']:
                self.assertEqual(c['mode'], 'visual')
                self.assertTrue(c['guidance'].strip())
        schema = json.loads((ASSETS / 'response_schema.json').read_text())
        crit = schema['properties']['criteria']
        self.assertEqual(sorted(crit['properties']), sorted(KEYS))
        self.assertEqual(sorted(crit['required']), sorted(KEYS))
        self.assertTrue((ASSETS / 'system.txt').read_text().strip())
        self.assertEqual(len(sorted(ASSETS.glob('section_*.txt'))), 4)


from ultron.assessment import (rubric, evidence_bundle_id, review_proposal,
                               validate_proposal, dashboard_payload)


class PackAwareAssessmentTests(unittest.TestCase):
    def manifest(self, pack='site_safety_v1'):
        m = {'mission_id': 'PATROL-01', 'asset_id': 'SITE-SWEEP-PATROL-01',
             'map_version': 'test', 'inspection_date': '2026-11-28',
             'position': [1., 2., 0.], 'views': [], 'verified_not_applicable': []}
        if pack:
            m['pack'] = pack
        return m

    def test_site_pack_round_trip_and_distinct_bundles(self):
        m = self.manifest()
        proposal = review_proposal(m, 'MANUAL REVIEW: offline')
        self.assertEqual(sorted(proposal['criteria']), sorted(KEYS))
        for item in proposal['criteria'].values():
            self.assertEqual(item['state'], 'UNKNOWN')  # all criteria are mode=visual
        validate_proposal(proposal, m)
        ladder_implicit = self.manifest(pack=None)
        ladder_explicit = self.manifest(pack='inspection_v1')
        self.assertEqual(evidence_bundle_id(ladder_implicit), evidence_bundle_id(ladder_explicit))
        self.assertNotEqual(evidence_bundle_id(m), evidence_bundle_id(ladder_implicit))

    def test_unknown_pack_rejected_and_dashboard_stays_ladder_only(self):
        with self.assertRaises(ValueError):
            evidence_bundle_id(self.manifest(pack='nonsense_v9'))
        m = self.manifest()
        proposal = review_proposal(m, 'MANUAL REVIEW: offline')
        with self.assertRaises(ValueError):
            dashboard_payload(proposal, m)

    def test_ladder_rubric_still_default(self):
        keys = [c['key'] for s in rubric()['sections'] for c in s['criteria']]
        self.assertEqual(len(keys), 23)


if __name__ == '__main__':
    unittest.main()
