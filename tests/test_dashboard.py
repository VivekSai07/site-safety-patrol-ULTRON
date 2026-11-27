import asyncio
import json
import sqlite3
import unittest
from pathlib import Path

try:
    from fastapi.testclient import TestClient
    from ultron.dashboard.app import create_app
    from ultron.dashboard.bus import Bus, sse_stream
    from ultron.dashboard.models import Inspection
    from ultron.dashboard.checklist import from_sim_checks
    from ultron.dashboard.notify import Notifier, load_config
    from ultron.dashboard.store import Store
    from ultron.dashboard import protocol_pdf
    from ultron.dashboard.replay import convert_record
    HAVE_DASHBOARD = True
except ImportError:
    HAVE_DASHBOARD = False

ROOT = Path(__file__).resolve().parents[1]
HISTORY_RUN = ROOT / 'runs' / 'history-demo'
FORM = ROOT / 'data' / 'nesseler' / 'kontrollblatt_checkliste_leitern_tritte.pdf'


def tiny_jpeg():
    import io as _io
    from PIL import Image as _Image
    out = _io.BytesIO()
    _Image.new('RGB', (16, 9), (90, 120, 150)).save(out, 'JPEG')
    return out.getvalue()


JPEG = tiny_jpeg() if HAVE_DASHBOARD else b''


def record(**overrides):
    base = {
        'mission_id': 'PATROL-01', 'asset_id': 'LAD-002', 'ladder_type': 'LEANING_SINGLE',
        'status': 'FLAGGED', 'reason': 'Visible defect: rungs', 'missing': ['rungs', 'bottom_feet'],
        'checks': {'stiles': 'PASS', 'rungs': 'FAIL', 'bottom_feet': 'UNKNOWN'},
        'label_status': 'OVERDUE', 'position': [5.7, 6.0, 1.3], 'safe_to_use': False,
        'tactile_checks_pending': True, 'human_signoff_required': True,
        'views': [{'robot_geoms': ['ignored sim internals']}],
    }
    base.update(overrides)
    return base


PASS_RECORD = dict(asset_id='LAD-001', status='AUTOMATED_PASS', reason='No defect detected',
                   checks={'stiles': 'PASS', 'rungs': 'PASS', 'bottom_feet': 'PASS'}, label_status='VALID')


@unittest.skipUnless(HAVE_DASHBOARD, 'dashboard extras not installed (pip install -e .[dashboard])')
class DashboardApiTest(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.app = create_app(':memory:', media_dir=self.tmp.name)
        self.client = TestClient(self.app)
        self.published = []
        self.app.state.bus.publish = lambda kind, data: self.published.append((kind, data))
        response = self.client.post('/api/missions', json={'mission_id': 'PATROL-01', 'site_id': 'site-a'})
        self.assertEqual(response.status_code, 201)

    def test_pages_are_served(self):
        for path in ('/', '/ladder', '/history', '/static/i18n.js', '/static/ladder.js', '/static/history.js'):
            with self.subTest(path):
                self.assertEqual(self.client.get(path).status_code, 200)

    def test_inspection_round_trip_drops_sim_internals(self):
        self.assertEqual(self.client.post('/api/inspections', json=record()).status_code, 201)
        mission = self.client.get('/api/missions/PATROL-01').json()
        self.assertEqual(mission['site_id'], 'site-a')
        self.assertEqual([i['asset_id'] for i in mission['inspections']], ['LAD-002'])
        self.assertNotIn('views', mission['inspections'][0])
        self.assertEqual([k for k, _ in self.published[1:]], ['inspection', 'alert'])  # FLAGGED also alerts

    def test_events_are_stored_in_order_and_published(self):
        for t, kind in [(12.4, 'LADDER_CONFIRMED'), (50.5, 'INSPECTION_COMMITTED')]:
            self.assertEqual(self.client.post('/api/events', json={
                'mission_id': 'PATROL-01', 'type': kind, 'time': t, 'asset_id': 'LAD-001'}).status_code, 201)
        events = self.client.get('/api/missions/PATROL-01/events').json()
        self.assertEqual([e['type'] for e in events], ['LADDER_CONFIRMED', 'INSPECTION_COMMITTED'])
        after = self.client.get('/api/missions/PATROL-01/events', params={'after_seq': events[0]['seq']}).json()
        self.assertEqual(len(after), 1)
        self.assertEqual([k for k, _ in self.published], ['mission', 'event', 'event'])

    def test_rejects_records_that_would_mark_unsafe_ladder_safe(self):
        bad = {
            'certified safe': record(safe_to_use=True),
            'fail but pass': record(status='AUTOMATED_PASS'),
            'overdue label but pass': record(**{**PASS_RECORD, 'label_status': 'OVERDUE'}),
            'unknown but pass': record(**{**PASS_RECORD, 'checks': {'rungs': 'UNKNOWN'}}),
            'fail but review': record(status='MANUAL_REVIEW'),
            'sign-off skipped': record(human_signoff_required=False),
        }
        for name, body in bad.items():
            with self.subTest(name):
                self.assertEqual(self.client.post('/api/inspections', json=body).status_code, 422)
        self.assertEqual(self.client.get('/api/missions/PATROL-01').json()['inspections'], [])

    def test_second_verdict_for_same_ladder_is_refused(self):
        self.client.post('/api/inspections', json=record())
        response = self.client.post('/api/inspections', json=record(**{**PASS_RECORD, 'asset_id': 'LAD-002'}))
        self.assertEqual(response.status_code, 409)
        self.assertEqual(self.client.get('/api/missions/PATROL-01').json()['inspections'][0]['status'], 'FLAGGED')

    def test_unknown_or_duplicate_mission(self):
        self.assertEqual(self.client.post('/api/inspections', json=record(mission_id='NOPE')).status_code, 404)
        self.assertEqual(self.client.post('/api/missions', json={
            'mission_id': 'PATROL-01', 'site_id': 'site-a'}).status_code, 409)

    def test_site_history_and_mission_list(self):
        self.client.post('/api/inspections', json=record(**PASS_RECORD))
        self.client.post('/api/missions/PATROL-01/end', json={'status': 'MISSION_COMPLETE'})
        self.client.post('/api/missions', json={'mission_id': 'PATROL-02', 'site_id': 'site-a'})
        self.client.post('/api/missions', json={'mission_id': 'OTHER-01', 'site_id': 'site-b'})
        self.client.post('/api/inspections', json=record(mission_id='PATROL-02', asset_id='LAD-001',
                                                         reason='Visible defect: bottom_feet',
                                                         checks={'bottom_feet': 'FAIL'}, label_status='VALID'))
        history = self.client.get('/api/sites/site-a/ladders').json()
        self.assertEqual([h['status'] for h in history['LAD-001']], ['AUTOMATED_PASS', 'FLAGGED'])
        ids = {m['id'] for m in self.client.get('/api/missions', params={'site_id': 'site-a'}).json()}
        self.assertEqual(ids, {'PATROL-01', 'PATROL-02'})
        self.assertEqual(self.client.get('/api/missions/PATROL-01').json()['status'], 'MISSION_COMPLETE')


@unittest.skipUnless(HAVE_DASHBOARD, 'dashboard extras not installed')
class LiveStreamTest(unittest.TestCase):
    def test_stream_delivers_published_messages(self):
        async def scenario():
            bus = Bus()

            async def connected():
                return False

            stream = sse_stream(bus, connected, keepalive_s=1)
            self.assertEqual(await stream.__anext__(), ': connected\n\n')
            pending = asyncio.ensure_future(stream.__anext__())
            await asyncio.sleep(0)
            bus.publish('inspection', {'asset_id': 'LAD-002', 'status': 'FLAGGED'})
            chunk = await asyncio.wait_for(pending, 2)
            await stream.aclose()
            return chunk, len(bus.subscribers)

        chunk, remaining = asyncio.run(scenario())
        self.assertTrue(chunk.startswith('event: inspection\n'))
        self.assertEqual(json.loads(chunk.split('data: ', 1)[1])['status'], 'FLAGGED')
        self.assertEqual(remaining, 0)




@unittest.skipUnless(HAVE_DASHBOARD, 'dashboard extras not installed')
class ChecklistMappingTest(unittest.TestCase):
    def test_failed_component_marks_group_defect_without_guessing_the_kind(self):
        items = from_sim_checks({'stiles': 'PASS', 'rungs': 'FAIL', 'bottom_feet': 'UNKNOWN'},
                                ['stiles', 'rungs', 'bottom_feet'], 'Anlegeleiter')
        self.assertEqual(items['sprossen'], 'DEFECT')
        self.assertEqual(items['sprossen.beschaedigung'], 'UNKNOWN')
        self.assertEqual(items['holme.verformung'], 'OK')
        self.assertEqual(items['holme.kanten'], 'NOT_CHECKED')  # tactile: always a person
        self.assertEqual(items['fuesse'], 'UNKNOWN')
        self.assertEqual(items['spreiz'], 'N_A')  # a leaning ladder has no spreader
        self.assertEqual(items['kennzeichnung'], 'NOT_CHECKED')

    def test_stepladder_spreader_is_in_scope(self):
        items = from_sim_checks({'stiles': 'PASS', 'rungs': 'PASS', 'front_feet': 'PASS', 'rear_feet': 'PASS',
                                 'spreader': 'PASS', 'hinge_visual': 'PASS'},
                                ['stiles', 'rungs', 'front_feet', 'rear_feet', 'spreader', 'hinge_visual'], 'Stehleiter')
        self.assertEqual((items['spreiz'], items['spreiz.funktion'], items['beschlag.schmierung']), ('OK', 'NOT_CHECKED', 'NOT_CHECKED'))


@unittest.skipUnless(HAVE_DASHBOARD, 'dashboard extras not installed')
class NotifierTest(unittest.TestCase):
    class FakeChannel:
        name = 'fake'

        def __init__(self):
            self.sent = []

        def send(self, alert, recipient):
            self.sent.append((alert['title'], recipient['role']))
            return 'sent', ''

    def setUp(self):
        from datetime import datetime, timedelta, timezone
        self.now = datetime.now(timezone.utc)
        self.later = lambda s: self.now + timedelta(seconds=s)
        self.clock_value = self.now
        self.store = Store(':memory:')
        self.channel = self.FakeChannel()
        self.published = []
        bus = type('B', (), {'publish': lambda _s, k, d: self.published.append(k)})()
        self.notifier = Notifier(self.store, bus, load_config(), [self.channel], clock=lambda: self.clock_value)

    def test_flagged_goes_to_on_site_and_off_site_officers(self):
        alert = self.notifier.on_inspection(record())
        self.assertEqual(alert['severity'], 'critical')
        self.assertEqual([r['role'] for r in alert['recipients']], ['site_safety_officer', 'safety_engineer'])
        self.assertEqual(sorted(role for _, role in self.channel.sent), ['safety_engineer', 'site_safety_officer'])
        self.assertEqual([d['state'] for d in alert['recipients'][0]['deliveries']], ['shown', 'sent'])
        self.assertIsNone(self.notifier.on_inspection(record(**PASS_RECORD)))

    def test_unacknowledged_critical_alert_escalates_once(self):
        alert = self.notifier.on_inspection(record())
        self.assertEqual(self.notifier.check_escalations(), [])  # too early
        self.clock_value = self.later(301)
        self.assertEqual(self.notifier.check_escalations(), [alert['id']])
        self.assertIn(('ESCALATED: ' + alert['title'], 'site_manager'), self.channel.sent)
        self.assertEqual(self.notifier.check_escalations(), [])  # only once

    def test_acknowledged_alert_does_not_escalate(self):
        alert = self.notifier.on_inspection(record())
        acked = self.notifier.ack(alert['id'], 'site_safety_officer')
        self.assertEqual(acked['acked_by'], 'site_safety_officer')
        self.clock_value = self.later(1000)
        self.assertEqual(self.notifier.check_escalations(), [])

    def test_robot_stop_goes_to_operator(self):
        alert = self.notifier.on_event({'mission_id': 'M', 'type': 'ESTOP', 'time': 12.0})
        self.assertEqual([r['role'] for r in alert['recipients']], ['robot_operator'])
        self.assertIsNone(self.notifier.on_event({'mission_id': 'M', 'type': 'PATROL_WAYPOINT', 'time': 1.0}))


@unittest.skipUnless(HAVE_DASHBOARD, 'dashboard extras not installed')
class EvidenceAndExportTest(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.client = TestClient(create_app(':memory:', media_dir=self.tmp.name))
        self.client.post('/api/missions', json={'mission_id': 'PATROL-01', 'site_id': 'site-a',
                                                'map': {'bounds': [0, 0, 12, 9], 'keepouts': [[1, 1, 2, 2]]}})

    def post_photo(self, photo_id='LAD-002-70'):
        return self.client.post(f'/api/missions/PATROL-01/photos/{photo_id}', content=JPEG, headers={'content-type': 'image/jpeg'})

    def flagged_with_photo(self):
        items = from_sim_checks({'stiles': 'PASS', 'rungs': 'FAIL', 'bottom_feet': 'UNKNOWN'}, ['stiles', 'rungs', 'bottom_feet'], 'Anlegeleiter')
        return record(checklist=items, photos=[{'id': 'LAD-002-70', 'caption': 'front camera'}],
                      item_photos={'sprossen': ['LAD-002-70']},
                      ladder={'ascent_type': 'Anlegeleiter', 'sticker_due': '2026-10-01', 'sticker_status': 'OVERDUE'})

    def test_inspection_needs_its_photos_uploaded_first(self):
        self.assertEqual(self.client.post('/api/inspections', json=self.flagged_with_photo()).status_code, 422)
        self.assertEqual(self.post_photo().status_code, 201)
        self.assertEqual(self.client.post('/api/inspections', json=self.flagged_with_photo()).status_code, 201)
        self.assertEqual(self.client.get('/media/PATROL-01/LAD-002-70.jpg').content, JPEG)

    def test_rejects_bad_images_and_ids(self):
        bad_type = self.client.post('/api/missions/PATROL-01/photos/p1', content=JPEG, headers={'content-type': 'image/png'})
        not_jpeg = self.client.post('/api/missions/PATROL-01/photos/p1', content=b'GIF89a', headers={'content-type': 'image/jpeg'})
        fake = self.client.post('/api/missions/PATROL-01/photos/p1', content=b'\xff\xd8' + b'0' * 64, headers={'content-type': 'image/jpeg'})
        self.assertEqual(fake.status_code, 400)
        bad_id = self.client.post('/api/missions/PATROL-01/photos/..%2Fx', content=JPEG, headers={'content-type': 'image/jpeg'})
        self.assertEqual((bad_type.status_code, not_jpeg.status_code), (415, 400))
        self.assertIn(bad_id.status_code, (400, 404))

    def test_live_state_has_pose_trail_and_camera(self):
        self.client.post('/api/missions/PATROL-01/pose', json={'time': 3.6, 'x': 1.5, 'y': 2.0, 'yaw': 1.57, 'state': 'PATROL'})
        self.client.post('/api/missions/PATROL-01/frames/front_rgb', content=JPEG, headers={'content-type': 'image/jpeg'})
        live = self.client.get('/api/live').json()
        self.assertEqual((live['mission']['id'], live['pose']['x'], live['trail']), ('PATROL-01', 1.5, [[1.5, 2.0]]))
        self.assertEqual(live['cameras'], {'front_rgb': {'seq': 1}})
        self.assertEqual(self.client.get('/api/cameras/front_rgb/latest.jpg').content, JPEG)

    def test_csv_has_one_row_per_ladder_with_nesseler_items(self):
        self.post_photo()
        self.client.post('/api/inspections', json=self.flagged_with_photo())
        self.client.post('/api/inspections', json=record(**PASS_RECORD))
        rows = self.client.get('/api/missions/PATROL-01/export.csv').text.strip().splitlines()
        self.assertEqual(len(rows), 3)
        self.assertIn('sprossen.beschaedigung', rows[0])
        self.assertIn('DEFECT', rows[1] + rows[2])

    @unittest.skipUnless(FORM.exists(), 'nesseler form missing: python -m ultron.dashboard fetch-form')
    def test_pdf_fills_nesseler_form_and_never_releases_a_ladder(self):
        import io
        from pypdf import PdfReader
        self.post_photo()
        self.client.post('/api/inspections', json=self.flagged_with_photo())
        self.client.post('/api/inspections', json=record(**PASS_RECORD, ladder={'ascent_type': 'Anlegeleiter'}))
        self.client.post('/api/missions/PATROL-01/end', json={'status': 'MISSION_COMPLETE'})
        report = '/api/sites/site-a/ladders/{}/report.pdf'
        flagged, passed = self.client.get(report.format('LAD-002')), self.client.get(report.format('LAD-001'))
        self.assertEqual({flagged.status_code, passed.status_code}, {200})
        self.assertEqual(self.client.get('/api/missions/PATROL-01/protocol.pdf').status_code, 404)  # no patrol report any more
        f_pages, p_pages = PdfReader(io.BytesIO(flagged.content)).pages, PdfReader(io.BytesIO(passed.content)).pages
        self.assertEqual(len(f_pages), 4)  # 3 form pages + evidence page
        self.assertIn('Blatt 1', f_pages[1].extract_text())
        self.assertIn('Roboter-ID LAD-002', f_pages[0].extract_text())
        self.assertIn('ABGELAUFEN', f_pages[0].extract_text())
        self.assertIn('Vorschlag', f_pages[2].extract_text())       # take-out-of-use proposal
        self.assertNotIn('Vorschlag', p_pages[2].extract_text())    # a pass proposes nothing
        self.assertTrue(all('/Annots' not in p for p in f_pages))   # empty widgets removed

    @unittest.skipUnless(FORM.exists(), 'nesseler form missing: python -m ultron.dashboard fetch-form')
    def test_one_report_per_ladder_with_a_new_sheet_every_five_patrols(self):
        import io
        from pypdf import PdfReader
        for n in range(1, 7):  # LAD-001 inspected on six patrols
            mission = f'P-{n:02d}'
            if n > 1:
                self.client.post('/api/missions', json={'mission_id': mission, 'site_id': 'site-a'})
            else:
                mission = 'PATROL-01'
            self.assertEqual(self.client.post('/api/inspections', json=record(**{**PASS_RECORD, 'mission_id': mission})).status_code, 201)
        detail = self.client.get('/api/sites/site-a/ladders/LAD-001').json()
        self.assertEqual([(s['sheet'], s['first'], s['last']) for s in detail['sheets']], [(1, 1, 5), (2, 6, 6)])
        text = lambda r: PdfReader(io.BytesIO(r.content)).pages[1].extract_text()
        newest = self.client.get('/api/sites/site-a/ladders/LAD-001/report.pdf')
        as_of_5 = self.client.get('/api/sites/site-a/ladders/LAD-001/report.pdf', params={'mission': 'P-05'})
        first = self.client.get('/api/sites/site-a/ladders/LAD-001/report.pdf', params={'sheet': 1})
        self.assertIn('Blatt 2', text(newest))
        self.assertIn('Prüfung 6–10', text(newest))
        self.assertIn('Blatt 1', text(as_of_5))
        self.assertIn('Blatt 1', text(first))
        for page in PdfReader(io.BytesIO(newest.content)).pages[:3]:  # every label must sit on the visible page
            spots = []
            page.extract_text(visitor_text=lambda t, cm, tm, fd, fs: spots.append((tm[4], tm[5])) if 'Blatt' in t else None)
            self.assertTrue(spots and all(0 <= x <= float(page.mediabox.width) and 0 <= y <= float(page.mediabox.height) for x, y in spots))
        self.assertEqual(self.client.get('/api/sites/site-a/ladders/LAD-001/report.pdf', params={'sheet': 3}).status_code, 404)
        self.assertEqual(self.client.get('/api/sites/site-a/ladders/LAD-999/report.pdf').status_code, 404)

    def test_sheet_slice_rule(self):
        history = [{'mission_id': f'M{i}'} for i in range(1, 12)]
        self.assertEqual(protocol_pdf.sheet_slice(history[:5])[0::2], (1, 1))
        sheet, cols, first = protocol_pdf.sheet_slice(history)
        self.assertEqual((sheet, [c['mission_id'] for c in cols], first), (3, ['M11'], 11))
        self.assertEqual(protocol_pdf.sheet_slice(history, 2)[1][0]['mission_id'], 'M6')

    def test_pdf_reports_missing_form_clearly(self):
        import os
        os.environ['ULTRON_NESSELER_FORM'] = str(Path(self.tmp.name) / 'missing.pdf')
        self.addCleanup(os.environ.pop, 'ULTRON_NESSELER_FORM')
        self.client.post('/api/inspections', json=record(**PASS_RECORD))
        response = self.client.get('/api/sites/site-a/ladders/LAD-001/report.pdf')
        self.assertEqual(response.status_code, 503)
        self.assertIn('fetch-form', response.text)


@unittest.skipUnless(HAVE_DASHBOARD and HISTORY_RUN.exists(), 'needs runs/history-demo from `ultron demo --out runs/history-demo`')
class SimulationRecordsTest(unittest.TestCase):
    def test_every_sim_record_converts_to_a_valid_inspection(self):
        protocol = json.loads((HISTORY_RUN / 'protocol.json').read_text())
        telemetry = {t['index']: t for t in map(json.loads, (HISTORY_RUN / 'telemetry.jsonl').read_text().splitlines())}
        seen = 0
        for mission in protocol['missions']:
            for rec in mission['records']:
                with self.subTest(f"{mission['mission_id']}/{rec['asset_id']}"):
                    payload, files = convert_record(rec, telemetry, HISTORY_RUN, 0.0, 1.0)
                    inspection = Inspection.model_validate(payload)
                    self.assertEqual(inspection.status.value, rec['status'])
                    self.assertTrue(files and all(path.exists() for _, path in files))
                    seen += 1
        self.assertEqual(seen, 6)




@unittest.skipUnless(HAVE_DASHBOARD, 'dashboard extras not installed')
class NtfyChannelTest(unittest.TestCase):
    def test_push_carries_title_priority_and_acknowledge_button(self):
        import threading
        from http.server import BaseHTTPRequestHandler, HTTPServer
        from ultron.dashboard.notify import NtfyChannel
        received = {}

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                received.update(path=self.path, headers=dict(self.headers),
                                body=self.rfile.read(int(self.headers['Content-Length'])).decode())
                self.send_response(200)
                self.end_headers()

            def log_message(self, *args):
                pass

        server = HTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=server.handle_request, daemon=True).start()
        channel = NtfyChannel(f'http://127.0.0.1:{server.server_port}', 'http://192.168.1.20:8000')
        alert = {'id': 7, 'title': 'FLAGGED: LAD-002 take out of use', 'severity': 'critical',
                 'message': 'Visible defect: rungs', 'action': 'Take LAD-002 out of use now.'}
        state = channel.send(alert, {'role': 'site_safety_officer', 'ntfy_topic': 'ultron-sso-test'})
        server.server_close()
        self.assertEqual(state, ('sent', ''))
        self.assertEqual(received['path'], '/ultron-sso-test')
        self.assertEqual(received['headers']['Priority'], 'urgent')
        self.assertIn('http://192.168.1.20:8000/api/alerts/7/ack?by=site_safety_officer', received['headers']['Actions'])
        self.assertIn('Visible defect: rungs', received['body'])
        self.assertEqual(channel.send(alert, {'role': 'x', 'ntfy_topic': None})[0], 'skipped')


if __name__ == '__main__':
    unittest.main()
