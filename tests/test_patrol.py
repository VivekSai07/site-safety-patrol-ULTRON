import contextlib
import copy
from datetime import date
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from site_safety_patrol.cli import run
from site_safety_patrol.policy import assess
from site_safety_patrol.simulation import visible

ROOT = Path(__file__).resolve().parents[1]
TODAY = date(2026, 11, 27)


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.observation = {
            'checks': dict.fromkeys(('rails', 'rungs', 'feet', 'spreaders'), 'pass'),
            'confidence': .95, 'evidence': ['synthetic.svg'],
            'label': {'readable': True, 'asset_id': 'DEMO-01', 'valid_until': '2027-01-01'}}

    def test_complete_observation_still_requires_human(self):
        result = assess(self.observation, TODAY)
        self.assertEqual(result['status'], 'candidate_for_review')
        self.assertFalse(result['safe'])
        self.assertTrue(result['human_signoff_required'])

    def test_defect_wins_over_missing_label_and_low_confidence(self):
        self.observation.update(label={}, confidence=.1)
        self.observation['checks']['rungs'] = 'fail'
        self.assertEqual(assess(self.observation, TODAY)['status'], 'unsafe')

    def test_each_missing_field_blocks_candidate(self):
        for field in self.observation:
            with self.subTest(field=field):
                observation = copy.deepcopy(self.observation)
                del observation[field]
                self.assertEqual(assess(observation, TODAY)['status'], 'needs_review')

    def test_unknown_or_missing_check_requires_review(self):
        for value in ('unknown', 'unsupported', None):
            self.observation['checks']['feet'] = value
            self.assertEqual(assess(self.observation, TODAY)['status'], 'needs_review')

    def test_confidence_invalid_or_weak(self):
        for value in (0, .84, float('nan'), float('inf'), 1.1, True, 'high', None):
            with self.subTest(confidence=value):
                self.observation['confidence'] = value
                self.assertEqual(assess(self.observation, TODAY)['status'], 'needs_review')

    def test_label_expiry_boundary_and_invalid_date(self):
        for value, expected in [('2026-11-26', 'needs_review'), ('2026-11-27', 'candidate_for_review'),
                                ('not-a-date', 'needs_review'), (None, 'needs_review')]:
            self.observation['label']['valid_until'] = value
            self.assertEqual(assess(self.observation, TODAY)['status'], expected)


class SimulationTests(unittest.TestCase):
    def test_range_and_field_of_view(self):
        self.assertTrue(visible([0, 0, 0], [2, 0], 3, 90))
        self.assertFalse(visible([0, 0, 0], [-2, 0], 3, 90))
        self.assertFalse(visible([0, 0, 0], [4, 0], 3, 90))
        self.assertTrue(visible([0, 0, 359], [2, 0], 3, 90))

    def test_conflicting_views_never_clear_defect_and_retain_reasons(self):
        base = {'type': 'observation', 'track_id': 'L01', 'position': [2, 1],
                'checks': dict.fromkeys(('rails', 'rungs', 'feet', 'spreaders'), 'pass'),
                'label': {'readable': True, 'asset_id': 'DEMO', 'valid_until': '2027-01-01'},
                'confidence': .95}
        events = []
        for step in range(3):
            event = copy.deepcopy(base)
            event['step'] = step
            if step == 0:
                event['confidence'] = .1
            if step == 1:
                event['checks']['rungs'] = 'fail'
            events.append(event)
        with tempfile.TemporaryDirectory() as tmp, patch('site_safety_patrol.cli.observe', return_value=iter(events)):
            with contextlib.redirect_stdout(io.StringIO()):
                result = run(ROOT / 'scenarios/mock-site.json', Path(tmp) / 'run', TODAY)
        assessment = result['records'][0]['assessment']
        self.assertEqual(assessment['status'], 'unsafe')
        self.assertIn('Missing, invalid or low confidence', assessment['reasons'])
        self.assertIn('Observed defect: rungs', assessment['reasons'])

    def test_normal_and_missing_course_artifacts(self):
        for fixture, count in [('mock-site.json', 5), ('missed-ladder.json', 4)]:
            with self.subTest(fixture=fixture), tempfile.TemporaryDirectory() as tmp:
                output = Path(tmp) / 'run'
                with contextlib.redirect_stdout(io.StringIO()):
                    protocol = run(ROOT / 'scenarios' / fixture, output, TODAY)
                self.assertEqual(protocol['simulation_metrics']['found_ladders'], count)
                self.assertEqual(protocol['simulation_metrics']['recall'], count / 5)
                self.assertEqual(len(protocol['records']), count)
                self.assertEqual(protocol['simulation_metrics']['missed_ground_truth_ids'],
                                 ['L05'] if count == 4 else [])
                self.assertEqual(json.loads((output / 'protocol.json').read_text()), protocol)
                self.assertIn('SYNTHETIC DEMO', (output / 'report.html').read_text())
                events = [json.loads(line) for line in (output / 'events.jsonl').read_text().splitlines()]
                observations = [e for e in events if e['type'] == 'observation']
                self.assertGreater(len(observations), count)
                for record in protocol['records']:
                    self.assertFalse(record['assessment']['safe'])
                    for path in record['evidence']:
                        self.assertIn('SYNTHETIC', (output / path).read_text())
                statuses = {r['track_id']: r['assessment']['status'] for r in protocol['records']}
                self.assertEqual(statuses['L02'], 'unsafe')
                self.assertEqual(statuses['L03'], 'needs_review')
                self.assertEqual(statuses['L04'], 'needs_review')
                with self.assertRaises(FileExistsError):
                    run(ROOT / 'scenarios' / fixture, output, TODAY)


if __name__ == '__main__':
    unittest.main()
