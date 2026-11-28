"""Where site checks come from: a pipeline workspace (importer) and a simulated patrol's safety props."""
import io
import json
import tempfile
import unittest
from pathlib import Path

try:
    from fastapi.testclient import TestClient
    from PIL import Image
    from ultron.dashboard.app import create_app
    from ultron.dashboard.import_site_checks import site_checks
    from ultron.dashboard.sim_site_checks import SimSiteChecks
    HAVE_DASHBOARD = True
except ImportError:
    HAVE_DASHBOARD = False


def jpeg():
    out = io.BytesIO()
    Image.new('RGB', (8, 8), (200, 30, 30)).save(out, 'JPEG')
    return out.getvalue()


@unittest.skipUnless(HAVE_DASHBOARD, 'dashboard extras not installed')
class Sources(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.client = TestClient(create_app(':memory:', media_dir=self.dir / 'media'))
        self.client.post('/api/missions', json={'mission_id': 'P1', 'site_id': 'hall'})

    def tearDown(self):
        self.tmp.cleanup()

    def post(self, path, body=None, raw=None, content_type='application/json'):
        if raw is not None:
            r = self.client.post(path, content=raw, headers={'content-type': content_type})
        else:
            r = self.client.post(path, json=body)
        return r.status_code, r.json()

    def summary_rows(self):
        summary = self.client.get('/api/missions/P1/site-summary').json()
        return {r['id']: r for c in summary['checklists'] for r in c['rows']}

    def test_pipeline_workspace(self):
        ws = self.dir / 'ws'
        (ws / 'objects' / 'FEX-1').mkdir(parents=True)
        (ws / 'objects.json').write_text(json.dumps([
            {'id': 'FEX-1', 'kind': 'fire_extinguisher', 'position': [1.0, 2.0], 'verified': True, 'status': 'INSPECTED'},
            {'id': 'AID-1', 'kind': 'first_aid_kit', 'position': [4.0, 0.5], 'verified': False, 'status': 'FOUND'},
            {'id': 'ALM-1', 'kind': 'fire_alarm', 'position': [0, 0], 'status': 'REJECTED'},
            {'id': 'LAD-1', 'kind': 'ladder', 'position': [3, 3], 'status': 'INSPECTED'}]))
        for name in ('photo-0.jpg', 'beleg-BS-2.jpg'):
            (ws / 'objects' / 'FEX-1' / name).write_bytes(jpeg())
        (ws / 'objects' / 'FEX-1' / 'checklist.json').write_text(json.dumps({'checklist': 'fire_extinguisher', 'rows': {
            'BS-1': {'answer': 'ja', 'remark': 'Löscher da', 'confidence': 0.9, 'evidence': 'photo-0.jpg'},
            'BS-2': {'answer': 'nein', 'remark': 'Prüffrist 2024 abgelaufen', 'confidence': 0.8, 'evidence': 'beleg-BS-2.jpg'}},
            'remarks': 'Plakette rot'}))
        checks = site_checks(ws, log=lambda *_: None)
        self.assertEqual([b['object_id'] for b, _ in checks], ['FEX-1', 'AID-1'])  # no ladder, no rejected
        for body, files in checks:
            for pid, path in files:
                self.assertEqual(self.post(f'/api/missions/P1/photos/{pid}', raw=path.read_bytes(), content_type='image/jpeg')[0], 201)
            self.assertEqual(self.post('/api/missions/P1/site-checks', body)[0], 201)
        rows = self.summary_rows()
        self.assertEqual((rows['BS-1']['answer'], rows['BS-1']['photo']), ('ja', 'FEX-1-photo-0'))
        self.assertEqual((rows['BS-2']['answer'], rows['BS-2']['photo']), ('nein', 'FEX-1-beleg-BS-2'))
        self.assertEqual(rows['EH-1']['answer'], 'offen')        # kit seen but not confirmed
        self.assertEqual(rows['EH-2']['answer'], 'nicht_gefunden')  # the rejected alarm box does not count

    def test_simulated_safety_props(self):
        run = self.dir / 'run'
        (run / 'frames').mkdir(parents=True)
        (run / 'frames' / '00282.jpg').write_bytes(jpeg())
        (run / 'frames' / '00414.jpg').write_bytes(jpeg())
        feed = SimSiteChecks(run, self.post, log=lambda *_: None)
        for row in [
            {'type': 'SAFETY_ASSET_CONFIRMED', 'asset_id': 'ext_01', 'asset_kind': 'extinguisher',
             'observation': {'position': [10.8, 8.4, 0.0], 'source': 'MUJOCO_SEGMENTATION_ORACLE'}},
            {'type': 'SAFETY_EVIDENCE_CAPTURED', 'asset_id': 'ext_01', 'asset_kind': 'extinguisher', 'rgb': 'frames/00282.jpg'},
            {'type': 'SAFETY_ASSET_CONFIRMED', 'asset_id': 'person_nohelm', 'asset_kind': 'person', 'observation': {'position': [3, 4, 0]}},
            {'type': 'SAFETY_EVIDENCE_CAPTURED', 'asset_id': 'person_nohelm', 'asset_kind': 'person', 'rgb': 'frames/00414.jpg'},
            {'type': 'SAFETY_VIOLATION', 'asset_id': 'person_nohelm', 'violation': 'PPE_HELMET_MISSING'},
            {'type': 'LADDER_CONFIRMED', 'asset_id': 'LAD-001'}]:
            feed.on_event('P1', row)
        checks = {c['object_id']: c for c in self.client.get('/api/missions/P1/site-checks').json()}
        self.assertEqual(set(checks), {'ext_01', 'person_nohelm'})
        self.assertEqual((checks['ext_01']['position'], checks['ext_01']['photos']), ([10.8, 8.4], ['site-ext_01']))
        self.assertIn('Helm fehlt', checks['person_nohelm']['remarks'])
        rows = self.summary_rows()
        self.assertEqual((rows['BS-1']['answer'], rows['BS-2']['answer']), ('ja', 'offen'))  # sticker not modelled
        self.assertTrue((self.dir / 'media' / 'P1' / 'site-ext_01.jpg').exists())


if __name__ == '__main__':
    unittest.main()
