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


if __name__ == '__main__':
    unittest.main()
