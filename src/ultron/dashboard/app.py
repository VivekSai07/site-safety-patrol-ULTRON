"""FastAPI app: the robot side posts records here, browsers read and subscribe."""
import asyncio
import contextlib
import csv
import io
from collections import deque
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image

from . import checklist, protocol_pdf
from .bus import Bus, sse_stream
from .models import Event, Inspection, MissionEnd, MissionStart, Pose, safe_id
from .notify import Notifier, load_config
from .store import Duplicate, Store

STATIC = Path(__file__).parent / 'static'
MAX_IMAGE_BYTES = 5_000_000
TRAIL_POINTS = 2000


def create_app(db_path=':memory:', media_dir='runs/dashboard-media', notify_config=None, channels=None,
               escalation_period_s=5.0):
    store, bus = Store(db_path), Bus()
    media = Path(media_dir)
    notifier = Notifier(store, bus, notify_config or load_config(), channels)
    live = {'pose': {}, 'trail': {}, 'frames': {}}

    @contextlib.asynccontextmanager
    async def lifespan(_app):
        bus.attach(asyncio.get_running_loop())

        async def escalations():
            while True:
                await asyncio.sleep(escalation_period_s)
                await asyncio.to_thread(notifier.check_escalations)

        task = asyncio.create_task(escalations())
        yield
        task.cancel()

    app = FastAPI(title='ULTRON ladder patrol dashboard', lifespan=lifespan)
    app.state.store, app.state.bus, app.state.notifier, app.state.live = store, bus, notifier, live
    app.mount('/static', StaticFiles(directory=STATIC), name='static')

    def require_mission(mission_id):
        mission = store.mission(mission_id)
        if mission is None:
            raise HTTPException(404, f'unknown mission {mission_id}')
        return mission

    def checked_id(value):
        try:
            return safe_id(value)
        except ValueError as error:
            raise HTTPException(400, str(error))

    async def jpeg_body(request):
        if request.headers.get('content-type') != 'image/jpeg':
            raise HTTPException(415, 'send the image as image/jpeg')
        data = await request.body()
        if not data or len(data) > MAX_IMAGE_BYTES or not data.startswith(b'\xff\xd8'):
            raise HTTPException(400, 'expected a JPEG under 5 MB')
        try:  # Pillow ships with reportlab; a corrupt image must not reach the evidence record
            with Image.open(io.BytesIO(data)) as image:
                image.verify()
        except Exception:
            raise HTTPException(400, 'the JPEG could not be decoded')
        return data

    # Pages
    @app.get('/', include_in_schema=False)
    async def live_page():
        return FileResponse(STATIC / 'index.html')

    @app.get('/history', include_in_schema=False)
    async def history_page():
        return FileResponse(STATIC / 'history.html')

    @app.get('/ladder', include_in_schema=False)
    async def ladder_page():
        return FileResponse(STATIC / 'ladder.html')

    # Missions
    @app.post('/api/missions', status_code=201)
    async def start_mission(body: MissionStart):
        try:
            mission = store.start_mission(body)
        except Duplicate:
            raise HTTPException(409, f'mission {body.mission_id} already exists')
        bus.publish('mission', mission)
        return mission

    @app.post('/api/missions/{mission_id}/end')
    async def end_mission(mission_id: str, body: MissionEnd):
        require_mission(mission_id)
        mission = store.end_mission(mission_id, body)
        bus.publish('mission', mission)
        await asyncio.to_thread(notifier.on_mission_end, mission, store.inspections(mission_id))
        return mission

    @app.get('/api/missions')
    async def list_missions(site_id: str | None = None):
        return store.missions(site_id)

    @app.get('/api/missions/{mission_id}')
    async def get_mission(mission_id: str):
        return {**require_mission(mission_id), 'inspections': store.inspections(mission_id)}

    @app.get('/api/live')
    async def live_state():
        missions = store.missions()
        running = [m for m in missions if m['status'] == 'RUNNING']
        mission = (running or missions or [None])[0]
        if mission is None:
            return {'mission': None}
        mid = mission['id']
        return {'mission': mission, 'inspections': store.inspections(mid), 'alerts': store.alerts(mid),
                'events': store.events(mid)[-60:], 'pose': live['pose'].get(mid),
                'trail': list(live['trail'].get(mid, [])),
                'cameras': {name: {'seq': f['seq']} for name, f in live['frames'].items() if f['mission_id'] == mid}}

    # Robot stream: poses, camera frames, events
    @app.post('/api/missions/{mission_id}/pose', status_code=204)
    async def add_pose(mission_id: str, body: Pose):
        require_mission(mission_id)
        pose = body.model_dump()
        live['pose'][mission_id] = pose
        live['trail'].setdefault(mission_id, deque(maxlen=TRAIL_POINTS)).append([round(body.x, 2), round(body.y, 2)])
        bus.publish('pose', {'mission_id': mission_id, **pose})

    @app.post('/api/missions/{mission_id}/frames/{camera}', status_code=204)
    async def add_frame(mission_id: str, camera: str, request: Request):
        require_mission(mission_id)
        camera = checked_id(camera)
        data = await jpeg_body(request)
        seq = live['frames'].get(camera, {}).get('seq', 0) + 1
        live['frames'][camera] = {'mission_id': mission_id, 'seq': seq, 'jpeg': data}
        bus.publish('frame', {'mission_id': mission_id, 'camera': camera, 'seq': seq})

    @app.get('/api/cameras/{camera}/latest.jpg')
    async def latest_frame(camera: str):
        frame = live['frames'].get(camera)
        if not frame:
            raise HTTPException(404, 'no frame yet')
        return Response(frame['jpeg'], media_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    @app.post('/api/events', status_code=201)
    async def add_event(body: Event):
        require_mission(body.mission_id)
        seq = store.add_event(body)
        record = {'seq': seq, **body.model_dump()}
        bus.publish('event', record)
        await asyncio.to_thread(notifier.on_event, record)
        return {'seq': seq}

    @app.get('/api/missions/{mission_id}/events')
    async def get_events(mission_id: str, after_seq: int = 0):
        require_mission(mission_id)
        return store.events(mission_id, after_seq)

    # Evidence photos and inspections
    @app.post('/api/missions/{mission_id}/photos/{photo_id}', status_code=201)
    async def add_photo(mission_id: str, photo_id: str, request: Request):
        require_mission(mission_id)
        photo_id = checked_id(photo_id)
        data = await jpeg_body(request)
        folder = media / checked_id(mission_id)
        folder.mkdir(parents=True, exist_ok=True)
        (folder / f'{photo_id}.jpg').write_bytes(data)
        return {'url': f'/media/{mission_id}/{photo_id}.jpg'}

    @app.get('/media/{mission_id}/{photo_id}.jpg')
    async def get_photo(mission_id: str, photo_id: str):
        path = media / checked_id(mission_id) / f'{checked_id(photo_id)}.jpg'
        if not path.exists():
            raise HTTPException(404, 'no such photo')
        return FileResponse(path, media_type='image/jpeg')

    @app.post('/api/inspections', status_code=201)
    async def add_inspection(body: Inspection):
        require_mission(body.mission_id)
        missing = [p.id for p in body.photos if not (media / body.mission_id / f'{p.id}.jpg').exists()]
        if missing:
            raise HTTPException(422, f'upload these evidence photos first: {missing}')
        try:
            store.add_inspection(body)
        except Duplicate:
            raise HTTPException(409, f'{body.asset_id} already has a verdict in {body.mission_id}')
        record = body.model_dump(mode='json')
        bus.publish('inspection', record)
        await asyncio.to_thread(notifier.on_inspection, record)
        return {'ok': True}

    @app.get('/api/checklist')
    async def checklist_structure():
        return checklist.describe()

    # Alerts
    @app.get('/api/alerts')
    async def list_alerts(mission_id: str | None = None):
        return store.alerts(mission_id)

    @app.post('/api/alerts/{alert_id}/ack')
    async def ack_alert(alert_id: int, by: str = 'dashboard'):
        alert = await asyncio.to_thread(notifier.ack, alert_id, by[:64])
        if alert is None:
            raise HTTPException(404, 'unknown alert')
        return alert

    # Sites and history
    @app.get('/api/sites')
    async def sites():
        return store.sites()

    @app.get('/api/sites/{site_id}/ladders')
    async def ladder_history(site_id: str):
        return store.ladder_history(site_id)

    # One report per ladder: its nesseler sheet, one column per patrol, a new sheet every 5 patrols
    def ladder_history_or_404(site_id, asset_id, mission_id=None):
        history = store.ladder_inspections(site_id, asset_id, mission_id)
        if mission_id and history and history[-1]['mission_id'] != mission_id:
            raise HTTPException(404, f'{asset_id} has no verdict in {mission_id}')
        if not history:
            raise HTTPException(404, f'no inspections of {asset_id} on {site_id}')
        return history

    @app.get('/api/sites/{site_id}/ladders/{asset_id}')
    async def ladder_detail(site_id: str, asset_id: str):
        history = ladder_history_or_404(site_id, asset_id)
        sheets = (len(history) - 1) // protocol_pdf.PER_SHEET + 1
        return {'site_id': site_id, 'asset_id': asset_id, 'inspections': history, 'sheets': [
            {'sheet': n, 'first': (n - 1) * protocol_pdf.PER_SHEET + 1,
             'last': min(n * protocol_pdf.PER_SHEET, len(history)),
             'url': f'/api/sites/{site_id}/ladders/{asset_id}/report.pdf?sheet={n}'} for n in range(1, sheets + 1)]}

    @app.get('/api/sites/{site_id}/ladders/{asset_id}/report.pdf')
    async def ladder_report(site_id: str, asset_id: str, mission: str | None = None, sheet: int | None = None):
        """Default: the sheet holding the newest inspection. ?mission= the report as it stood after
        that patrol; ?sheet= a full earlier sheet."""
        history = ladder_history_or_404(site_id, asset_id, mission)
        missions = {m['id']: m for m in store.missions(site_id)}
        try:
            data = await asyncio.to_thread(protocol_pdf.ladder_report, missions, history, media, None, sheet)
        except protocol_pdf.FormMissing as error:
            raise HTTPException(503, str(error))
        except ValueError as error:
            raise HTTPException(404, str(error))
        sheet_no = sheet or (len(history) - 1) // protocol_pdf.PER_SHEET + 1
        return Response(data, media_type='application/pdf', headers={
            'Content-Disposition': f'inline; filename="Kontrollblatt_{asset_id}_Blatt{sheet_no}.pdf"'})

    @app.get('/api/missions/{mission_id}/export.json')
    async def export_json(mission_id: str):
        mission = require_mission(mission_id)
        return {'mission': mission, 'inspections': store.inspections(mission_id), 'alerts': store.alerts(mission_id),
                'checklist': checklist.describe(),
                'disclaimer': 'Automated visual pre-screening only. Never a safe-to-use certificate.'}

    @app.get('/api/missions/{mission_id}/export.csv')
    async def export_csv(mission_id: str):
        mission = require_mission(mission_id)
        out = io.StringIO()
        writer = csv.writer(out)
        writer.writerow(['site', 'mission', 'ladder', 'ascent_type', 'status', 'reason', 'sticker_due', 'sticker_status',
                         'x_m', 'y_m', *checklist.ALL_KEYS, 'safe_to_use', 'human_signoff_required'])
        for record in store.inspections(mission_id):
            ladder, items = record.get('ladder') or {}, record.get('checklist') or {}
            writer.writerow([mission['site_id'], mission_id, record['asset_id'], ladder.get('ascent_type') or record['ladder_type'],
                             record['status'], record['reason'], ladder.get('sticker_due') or '', ladder.get('sticker_status') or '',
                             f"{record['position'][0]:.2f}", f"{record['position'][1]:.2f}",
                             *[items.get(k, '') for k in checklist.ALL_KEYS], record['safe_to_use'], record['human_signoff_required']])
        return Response(out.getvalue(), media_type='text/csv',
                        headers={'Content-Disposition': f'attachment; filename="ladders_{mission_id}.csv"'})

    @app.get('/api/stream')
    async def stream(request: Request):
        return StreamingResponse(sse_stream(bus, request.is_disconnected), media_type='text/event-stream',
                                 headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'})

    return app
