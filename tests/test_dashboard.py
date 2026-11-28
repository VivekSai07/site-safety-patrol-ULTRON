import asyncio
import io
import json
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


@unittest.skipUnless(HAVE_DASHBOARD, 'dashboard extras not installed')
class NavigationTest(unittest.TestCase):
    """Occupancy map in from the robot side, 2D goal pose out from the map."""

    def make(self, allow_goals=True):
        app = create_app(':memory:', allow_goals=allow_goals)
        published = []
        app.state.bus.publish = lambda kind, data: published.append((kind, data))
        return TestClient(app), published

    def png(self, size=(40, 20)):
        import io as _io
        from PIL import Image as _Image
        out = _io.BytesIO()
        _Image.new('L', size, 205).save(out, 'PNG')
        return out.getvalue()

    def test_map_upload_is_served_and_announced(self):
        client, published = self.make()
        r = client.post('/api/map?resolution=0.05&origin_x=-1&origin_y=-0.5', content=self.png(),
                        headers={'content-type': 'image/png'})
        self.assertEqual(r.status_code, 204)
        meta = client.get('/api/map').json()
        self.assertEqual((meta['width'], meta['height'], meta['resolution'], meta['origin'], meta['seq']), (40, 20, 0.05, [-1, -0.5], 1))
        self.assertEqual(client.get('/api/map.png').content, self.png())
        self.assertEqual(published[-1], ('map', meta))
        self.assertEqual(client.get('/api/live').json()['grid'], meta)

    def test_map_rejects_bad_input(self):
        client, _ = self.make()
        q = '/api/map?resolution=0.05&origin_x=0&origin_y=0'
        self.assertEqual(client.post(q, content=JPEG, headers={'content-type': 'image/jpeg'}).status_code, 415)
        self.assertEqual(client.post(q, content=b'\x89PNG broken', headers={'content-type': 'image/png'}).status_code, 400)
        self.assertEqual(client.post('/api/map?resolution=0&origin_x=0&origin_y=0', content=self.png(),
                                     headers={'content-type': 'image/png'}).status_code, 400)
        self.assertEqual(client.get('/api/map.png').status_code, 404)

    def test_pages_and_scripts_are_revalidated(self):
        # a cached old live.js against a new index.html hid the Start exploration button
        client, _ = self.make()
        for path in ('/', '/history', '/ladder', '/patrol', '/static/live.js', '/static/i18n.js', '/static/patrol.js'):
            response = client.get(path)
            self.assertEqual((response.status_code, response.headers.get('cache-control')), (200, 'no-cache'), path)

    def test_goals_are_off_unless_enabled(self):
        client, published = self.make(allow_goals=False)
        self.assertEqual(client.post('/api/goal', json={'x': 1, 'y': 2, 'yaw': 0}).status_code, 403)
        self.assertIsNone(client.get('/api/goal').json())
        self.assertFalse(client.get('/api/live').json()['goals_enabled'])
        self.assertEqual(published, [])

    def test_live_map_does_not_reuse_saved_navigation_identity(self):
        client, _ = self.make()
        url = '/api/map?resolution=0.05&origin_x=0&origin_y=0&map_id=hall&map_version=2'
        headers = {'content-type': 'image/png'}
        self.assertEqual(client.post(url, content=self.png(), headers=headers).status_code, 204)
        self.assertEqual(client.get('/api/map').json()['map_id'], 'hall')
        image = self.png(size=(100, 50))
        self.assertEqual(client.post(url + '&source=live_slam', content=image, headers=headers).status_code, 204)
        grid = client.get('/api/map').json()
        self.assertEqual(grid['source'], 'live_slam')
        self.assertNotIn('map_id', grid)
        self.assertNotIn('map_version', grid)
        self.assertEqual(client.get('/api/map.png?v=1').content, self.png())
        self.assertEqual(client.get('/api/map.png?v=2').content, image)
        self.assertEqual(client.get('/api/map.png?v=3').status_code, 404)
        rejected = client.post('/api/goal', json={'x': 1, 'y': 2, 'map_id': 'hall', 'map_version': 2})
        self.assertEqual(rejected.status_code, 409)
        self.assertIn('LIVE_MAP_PREVIEW', rejected.json()['detail'])
        self.assertIsNone(client.get('/api/goal').json())

    def test_goal_round_trip(self):
        client, published = self.make()
        goal = client.post('/api/goal', json={'x': 3.5, 'y': -1.0, 'yaw': 1.57}).json()
        self.assertTrue(goal['request_id'])
        self.assertEqual(goal, {'seq': 1, 'request_id': goal['request_id'], 'x': 3.5, 'y': -1.0, 'yaw': 1.57,
                                'frame_id': 'map', 'status': 'SENT', 'detail': ''})
        self.assertEqual(published[-1], ('goal', goal))
        # the bridge reports progress on the current goal; a newer goal replaces it
        self.assertEqual(client.post('/api/goal/status', json={'seq': 1, 'status': 'ACCEPTED'}).json()['status'], 'ACCEPTED')
        client.post('/api/goal', json={'x': 0, 'y': 0})
        self.assertEqual(client.post('/api/goal/status', json={'seq': 1, 'status': 'SUCCEEDED'}).status_code, 409)
        self.assertEqual(client.post('/api/goal/status', json={'seq': 2, 'status': 'BOGUS'}).status_code, 422)
        self.assertEqual(client.get('/api/goal').json()['seq'], 2)

    def test_nav_goal_keeps_map_identity_and_reports_rejection(self):
        client, _ = self.make()
        q = '/api/map?resolution=0.05&origin_x=0&origin_y=0&map_id=hall&map_version=2'
        self.assertEqual(client.post(q, content=self.png(), headers={'content-type': 'image/png'}).status_code, 204)
        self.assertEqual(client.get('/api/map').json()['map_version'], 2)
        implied = client.post('/api/goal', json={'x': 1, 'y': 2}).json()
        self.assertEqual((implied['map_id'], implied['map_version']), ('hall', 2))
        explicit = client.post('/api/goal', json={'x': 1, 'y': 2, 'map_id': 'hall', 'map_version': 1}).json()
        self.assertEqual(explicit['map_version'], 1)
        status = {'seq': explicit['seq'], 'status': 'REJECTED', 'detail': 'MAP_MISMATCH'}
        self.assertEqual(client.post('/api/goal/status', json=status | {'request_id': implied['request_id']}).status_code, 409)
        rejected = client.post('/api/goal/status', json=status | {'request_id': explicit['request_id']}).json()
        self.assertEqual((rejected['status'], rejected['detail']), ('REJECTED', 'MAP_MISMATCH'))
        self.assertEqual(client.post('/api/goal', json={'x': 'NaN', 'y': 0}).status_code, 422)


@unittest.skipUnless(HAVE_DASHBOARD, 'dashboard extras not installed')
class ExplorationTest(unittest.TestCase):
    """Start exploration / Finish mapping from the dashboard; only the bridge confirms what the robot does."""

    make = NavigationTest.make

    def ask(self, client, action):
        return client.post('/api/exploration', json={'action': action})

    def report(self, client, seq, state, **extra):
        return client.post('/api/exploration/status', json={'seq': seq, 'state': state, **extra})

    def state(self, client):
        return client.get('/api/exploration').json()['state']

    def test_off_in_demo_mode(self):
        client, published = self.make(allow_goals=False)
        self.assertEqual(self.ask(client, 'start').status_code, 403)
        self.assertEqual(client.get('/api/live').json()['exploration']['state'], 'IDLE')
        self.assertEqual(published, [])

    def test_operator_starts_and_finishes(self):
        client, published = self.make()
        started = self.ask(client, 'start').json()
        self.assertEqual((started['seq'], started['state'], started['by']), (1, 'STARTING', 'operator'))
        self.assertEqual(published[-1], ('exploration', started))
        self.assertEqual(self.report(client, 1, 'EXPLORING').json()['state'], 'EXPLORING')
        self.assertEqual(self.ask(client, 'finish').json()['state'], 'FINISHING')
        done = self.report(client, 2, 'MAPPED').json()
        self.assertEqual((done['state'], done['by']), ('MAPPED', 'operator'))
        self.assertEqual(published[-1], ('exploration', done))
        # mapping again later is the operator's call
        self.assertEqual(self.ask(client, 'start').json()['state'], 'STARTING')

    def test_explorer_ends_it_when_no_frontiers_are_left(self):
        client, _ = self.make()
        self.ask(client, 'start')
        self.report(client, 1, 'EXPLORING')
        done = self.report(client, 1, 'MAPPED', by='explorer', detail='no frontiers left').json()
        self.assertEqual((done['state'], done['by'], done['detail']), ('MAPPED', 'explorer', 'no frontiers left'))
        self.assertEqual(self.ask(client, 'finish').status_code, 409)

    def test_requests_that_make_no_sense_are_refused(self):
        client, _ = self.make()
        self.assertEqual(self.ask(client, 'finish').status_code, 409)       # nothing running
        self.ask(client, 'start')
        self.assertEqual(self.ask(client, 'start').status_code, 409)        # already starting
        self.assertEqual(self.ask(client, 'pause').status_code, 422)        # unknown action
        self.assertEqual(self.report(client, 1, 'STARTING').status_code, 422)  # the operator's state, not the bridge's
        self.ask(client, 'finish')
        self.assertEqual(self.report(client, 1, 'EXPLORING').status_code, 409)  # stale: seq 2 is current
        self.assertEqual(self.state(client), 'FINISHING')

    def test_bridge_failure_is_shown_and_can_be_retried(self):
        client, _ = self.make()
        self.ask(client, 'start')
        failed = self.report(client, 1, 'FAILED', detail='explore node did not come up').json()
        self.assertEqual((failed['state'], failed['detail']), ('FAILED', 'explore node did not come up'))
        self.assertEqual(self.ask(client, 'start').json()['state'], 'STARTING')

    def test_goals_and_exploration_exclude_each_other(self):
        # both drive Nav2's navigate_to_pose, and pausing the explorer cancels every goal on it
        client, _ = self.make()
        self.ask(client, 'start')
        self.assertEqual(client.post('/api/goal', json={'x': 1, 'y': 1}).status_code, 409)
        self.report(client, 1, 'EXPLORING')
        self.assertEqual(client.post('/api/goal', json={'x': 1, 'y': 1}).status_code, 409)
        self.ask(client, 'finish')
        self.report(client, 2, 'MAPPED')
        self.assertEqual(client.post('/api/goal', json={'x': 1, 'y': 1}).status_code, 201)
        self.assertEqual(self.ask(client, 'start').status_code, 409)        # goal SENT, not finished
        client.post('/api/goal/status', json={'seq': 1, 'status': 'SUCCEEDED'})
        self.assertEqual(self.ask(client, 'start').status_code, 201)


@unittest.skipUnless(HAVE_DASHBOARD, 'dashboard extras not installed')
class SiteCheckTest(unittest.TestCase):
    """Non-ladder objects and their checklist rows, per patrol, for the history pages."""

    EXT = {'object_id': 'FEX-001', 'kind': 'fire_extinguisher', 'position': [2.0, 3.5], 'verified': True,
           'rows': {'BS-1': {'answer': 'ja', 'photo': 'fex-1'}, 'BS-2': {'answer': 'offen', 'remark': 'zu weit'}}}

    def setUp(self):
        self.client = TestClient(create_app(':memory:'))
        self.client.post('/api/missions', json={'mission_id': 'PATROL-01', 'site_id': 'site-a'})

    def post(self, body, mission='PATROL-01'):
        return self.client.post(f'/api/missions/{mission}/site-checks', json=body)

    def test_round_trip_and_summary(self):
        self.assertEqual(self.post(self.EXT).status_code, 201)
        checks = self.client.get('/api/missions/PATROL-01/site-checks').json()
        self.assertEqual([(c['object_id'], c['kind'], c['position']) for c in checks], [('FEX-001', 'fire_extinguisher', [2.0, 3.5])])
        summary = self.client.get('/api/missions/PATROL-01/site-summary').json()
        rows = {r['id']: r for c in summary['checklists'] for r in c['rows']}
        self.assertEqual((rows['BS-1']['answer'], rows['BS-2']['answer'], rows['EH-1']['answer']), ('ja', 'offen', 'nicht_gefunden'))
        self.assertEqual(len(summary['checklists']), 5)

    def test_a_later_post_replaces_the_object(self):
        self.post(self.EXT)
        self.post({**self.EXT, 'rows': {'BS-2': {'answer': 'nein', 'remark': 'abgelaufen'}}})
        checks = self.client.get('/api/missions/PATROL-01/site-checks').json()
        self.assertEqual((len(checks), checks[0]['rows']['BS-2']['answer']), (1, 'nein'))

    def test_rejects_what_the_checklists_do_not_cover(self):
        self.assertEqual(self.post(self.EXT, mission='NOPE').status_code, 404)
        self.assertEqual(self.post({**self.EXT, 'kind': 'coffee_machine'}).status_code, 422)
        self.assertEqual(self.post({**self.EXT, 'rows': {'XX-1': {'answer': 'ja'}}}).status_code, 422)
        self.assertEqual(self.post({**self.EXT, 'rows': {'EH-1': {'answer': 'ja'}}}).status_code, 422)  # a kit row from an extinguisher
        self.assertEqual(self.post({**self.EXT, 'rows': {'BS-1': {'answer': 'maybe'}}}).status_code, 422)
        self.assertEqual(self.post({**self.EXT, 'object_id': '../x'}).status_code, 422)
        self.assertEqual(self.client.get('/api/missions/PATROL-01/site-checks').json(), [])

    def test_catalogue_is_served(self):
        catalogue = self.client.get('/api/site-checklists').json()
        self.assertEqual([c['key'] for c in catalogue][0], 'erste_hilfe_brandschutz')

    def csv(self, path):
        import csv as _csv
        response = self.client.get(path)
        self.assertEqual((response.status_code, response.headers['content-type'].split(';')[0]), (200, 'text/csv'))
        return list(_csv.DictReader(io.StringIO(response.text)))

    def test_downloads(self):
        self.post(self.EXT)
        rows = {r['row']: r for r in self.csv('/api/missions/PATROL-01/site-checks.csv')}
        self.assertEqual((rows['BS-1']['answer'], rows['BS-1']['objects'], rows['BS-1']['photo']),
                         ('ja', 'FEX-001', '/media/PATROL-01/fex-1.jpg'))
        self.assertEqual(rows['EH-4']['answer'], 'person')
        self.assertEqual(len(rows), 92)
        objects = self.csv('/api/missions/PATROL-01/objects.csv')
        by_kind = {o['kind']: o for o in objects}
        self.assertEqual((by_kind['fire_extinguisher']['found'], by_kind['fire_extinguisher']['object'],
                          by_kind['fire_extinguisher']['x_m'], by_kind['fire_extinguisher']['confirmed']), ('ja', 'FEX-001', '2.0', 'ja'))
        self.assertEqual(by_kind['first_aid_kit']['found'], 'nein')          # looked for, not found
        self.assertEqual(by_kind['scaffold']['found'], 'kein Detektor')      # nothing looks for it yet
        export = self.client.get('/api/missions/PATROL-01/export.json').json()
        self.assertEqual(export['site_checks'][0]['object_id'], 'FEX-001')
        self.assertIn('presence', export['site_summary'])

    def test_site_view_across_patrols(self):
        self.post(self.EXT)
        self.client.post('/api/missions', json={'mission_id': 'PATROL-02', 'site_id': 'site-a'})
        site = self.client.get('/api/sites/site-a/site-checks').json()
        by_id = {p['mission_id']: p for p in site['patrols']}
        self.assertEqual((by_id['PATROL-01']['objects'], by_id['PATROL-02']['objects']), (1, 0))
        self.assertEqual(by_id['PATROL-01']['headlines']['erste_hilfe_brandschutz'], 'offen')
        self.assertIn({'kind': 'scaffold', 'label': 'Gerüst', 'detector': False}, site['kinds'])
        matrix = {r['kind']: r for r in self.csv('/api/sites/site-a/objects.csv')}
        self.assertEqual((matrix['fire_extinguisher']['PATROL-01'], matrix['fire_extinguisher']['PATROL-02']), ('1', '0'))


@unittest.skipUnless(HAVE_DASHBOARD, 'dashboard extras not installed (pip install -e .[dashboard])')
class RouteHistoryTest(unittest.TestCase):
    """Each patrol keeps its route and the occupancy map it drove on, for the history page."""

    png = NavigationTest.png

    def setUp(self):
        self.client = TestClient(create_app(':memory:'))

    def start(self, mission_id='PATROL-01'):
        return self.client.post('/api/missions', json={'mission_id': mission_id, 'site_id': 'site-a'})

    def pose(self, t, x, y, yaw=0.0, mission_id='PATROL-01'):
        self.client.post(f'/api/missions/{mission_id}/pose', json={'time': t, 'x': x, 'y': y, 'yaw': yaw})

    def send_map(self, origin_x=-1):
        return self.client.post(f'/api/map?resolution=0.05&origin_x={origin_x}&origin_y=0', content=self.png(),
                                headers={'content-type': 'image/png'})

    def test_route_keeps_moves_and_turns_and_drops_standing_still(self):
        self.start()
        self.pose(0.0, 1.0, 1.0)
        self.pose(0.5, 1.01, 1.0)       # 1 cm: dropped
        self.pose(1.0, 1.1, 1.0)        # 10 cm: kept
        self.pose(1.5, 1.1, 1.0, 0.05)  # small turn: dropped
        self.pose(2.0, 1.1, 1.0, 0.3)   # 0.3 rad turn on the spot: kept
        self.pose(2.5, 1.1, 1.0, 0.3)   # standing still: dropped
        route = self.client.get('/api/missions/PATROL-01/route').json()
        self.assertEqual(route['poses'], [[0.0, 1.0, 1.0, 0.0], [1.0, 1.1, 1.0, 0.0], [2.0, 1.1, 1.0, 0.3]])
        self.assertIsNone(route['grid'])
        # the live trail still gets every pose
        self.assertEqual(len(self.client.get('/api/live').json()['trail']), 6)

    def test_turn_across_plus_minus_pi_is_small(self):
        self.start()
        self.pose(0.0, 0, 0, 3.13)
        self.pose(1.0, 0, 0, -3.13)  # 0.02 rad apart, not 6.26
        self.assertEqual(len(self.client.get('/api/missions/PATROL-01/route').json()['poses']), 1)

    def test_map_sent_before_or_during_the_mission_is_kept_with_it(self):
        self.send_map(origin_x=-1)       # bridge started before the patrol
        self.start('PATROL-01')
        self.assertEqual(self.client.get('/api/missions/PATROL-01/route').json()['grid']['origin'], [-1, 0])
        self.send_map(origin_x=-2)       # SLAM grew the map during the patrol: newest one wins
        grid = self.client.get('/api/missions/PATROL-01/route').json()['grid']
        self.assertEqual((grid['origin'], grid['width'], grid['height'], grid['resolution']), ([-2, 0], 40, 20, 0.05))
        self.assertEqual(self.client.get('/api/missions/PATROL-01/map.png').content, self.png())
        # a finished patrol keeps its map when the next patrol's map arrives
        self.client.post('/api/missions/PATROL-01/end', json={'status': 'MISSION_COMPLETE'})
        self.start('PATROL-02')
        self.send_map(origin_x=-3)
        self.assertEqual(self.client.get('/api/missions/PATROL-01/route').json()['grid']['origin'], [-2, 0])
        self.assertEqual(self.client.get('/api/missions/PATROL-02/route').json()['grid']['origin'], [-3, 0])

    def test_routes_are_kept_per_mission(self):
        self.start('PATROL-01')
        self.start('PATROL-02')
        self.pose(0, 1, 1, mission_id='PATROL-01')
        self.pose(0, 5, 5, mission_id='PATROL-02')
        self.assertEqual(self.client.get('/api/missions/PATROL-01/route').json()['poses'], [[0, 1, 1, 0]])
        self.assertEqual(self.client.get('/api/missions/PATROL-02/route').json()['poses'], [[0, 5, 5, 0]])

    def test_patrol_without_route_or_map(self):
        self.start()
        self.assertEqual(self.client.get('/api/missions/PATROL-01/route').json(), {'poses': [], 'grid': None})
        self.assertEqual(self.client.get('/api/missions/PATROL-01/map.png').status_code, 404)
        self.assertEqual(self.client.get('/api/missions/NOPE/route').status_code, 404)

    def test_older_database_gains_route_tables_and_keeps_its_rows(self):
        import sqlite3
        import tempfile
        from ultron.dashboard import store as store_module
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'old.sqlite'
            old_schema = store_module.SCHEMA.split('CREATE TABLE IF NOT EXISTS poses')[0]
            db = sqlite3.connect(path)
            db.executescript(old_schema)
            db.execute("INSERT INTO missions VALUES ('OLD-1','site-a','',0,'2026-11-01T10:00:00+00:00',NULL,'MISSION_COMPLETE',NULL,NULL)")
            db.commit()
            db.close()
            store = Store(path)
            self.assertEqual([m['id'] for m in store.missions()], ['OLD-1'])
            self.assertEqual((store.route('OLD-1'), store.mission_map('OLD-1')), ([], None))
            store.db.close()


if __name__ == '__main__':
    unittest.main()
