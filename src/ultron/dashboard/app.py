"""FastAPI app: the robot side posts records here, browsers read and subscribe."""
import asyncio
import contextlib
import csv
import io
import math
from collections import deque
from pathlib import Path
import uuid

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image

from . import checklist, protocol_pdf, site_checklists
from .bus import Bus, sse_stream
from .models import (EXPLORATION_ACTIVE, Event, ExplorationAction, ExplorationRequest, ExplorationState,
                     ExplorationUpdate, Goal, GoalUpdate, Inspection, MissionEnd, MissionStart, Pose, SiteCheck, safe_id)
from .notify import Notifier, load_config
from .store import Duplicate, Store, now

STATIC = Path(__file__).parent / 'static'
# Pages and their scripts change between versions of the dashboard: browsers must revalidate them
# (a cheap If-None-Match round trip) instead of running a cached old script against a new page.
NO_CACHE = {'Cache-Control': 'no-cache'}


class RevalidatedStatic(StaticFiles):
    def file_response(self, *args, **kwargs):
        response = super().file_response(*args, **kwargs)
        response.headers.update(NO_CACHE)
        return response
MAX_IMAGE_BYTES = 5_000_000
TRAIL_POINTS = 2000
# A pose is saved to the mission's route only once the robot has moved or turned this much since the
# last saved one, so a robot standing still for minutes adds nothing. Display choices, not hardware.
ROUTE_MIN_STEP_M = 0.05
ROUTE_MIN_TURN_RAD = 0.1


def route_point_due(last, pose):
    """True when `pose` differs enough from the last saved route point (None = nothing saved yet)."""
    if last is None:
        return True
    turn = abs(math.remainder(pose['yaw'] - last['yaw'], math.tau))
    return math.hypot(pose['x'] - last['x'], pose['y'] - last['y']) >= ROUTE_MIN_STEP_M or turn >= ROUTE_MIN_TURN_RAD


def create_app(db_path=':memory:', media_dir='runs/dashboard-media', notify_config=None, channels=None,
               escalation_period_s=5.0, allow_goals=False):
    """allow_goals: accept navigation goals from the map. Off by default: a goal is motion authority."""
    store, bus = Store(db_path), Bus()
    media = Path(media_dir)
    notifier = Notifier(store, bus, notify_config or load_config(), channels)
    live = {'pose': {}, 'trail': {}, 'frames': {}, 'map': None, 'map_images': deque(maxlen=3), 'goal': None, 'route_last': {},
            'exploration': {'seq': 0, 'state': ExplorationState.IDLE.value, 'detail': '', 'by': '', 'since': now()}}

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
    app.mount('/static', RevalidatedStatic(directory=STATIC), name='static')

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
        return FileResponse(STATIC / 'index.html', headers=NO_CACHE)

    @app.get('/history', include_in_schema=False)
    async def history_page():
        return FileResponse(STATIC / 'history.html', headers=NO_CACHE)

    @app.get('/patrol', include_in_schema=False)
    async def patrol_page():
        return FileResponse(STATIC / 'patrol.html', headers=NO_CACHE)

    @app.get('/ladder', include_in_schema=False)
    async def ladder_page():
        return FileResponse(STATIC / 'ladder.html', headers=NO_CACHE)

    # Missions
    @app.post('/api/missions', status_code=201)
    async def start_mission(body: MissionStart):
        try:
            mission = store.start_mission(body)
        except Duplicate:
            raise HTTPException(409, f'mission {body.mission_id} already exists')
        if live['map'] is not None:  # the bridge may have sent the map before the mission started
            store.save_mission_map(mission['id'], live['map'])
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

    def map_meta():
        m = live['map']
        return None if m is None else {k: v for k, v in m.items() if k != 'png'}

    @app.get('/api/live')
    async def live_state():
        missions = store.missions()
        running = [m for m in missions if m['status'] == 'RUNNING']
        mission = (running or missions or [None])[0]
        nav = {'grid': map_meta(), 'goal': live['goal'], 'goals_enabled': allow_goals,
               'exploration': live['exploration']}
        if mission is None:
            return {'mission': None, **nav}
        mid = mission['id']
        return {**nav, 'mission': mission, 'inspections': store.inspections(mid), 'alerts': store.alerts(mid),
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
        if route_point_due(live['route_last'].get(mission_id), pose):
            store.add_pose(mission_id, body.time, body.x, body.y, body.yaw)
            live['route_last'][mission_id] = pose
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

    # Occupancy map (robot side -> browsers): a PNG of the ROS OccupancyGrid, top row = largest y
    @app.post('/api/map', status_code=204)
    async def set_map(request: Request, resolution: float, origin_x: float, origin_y: float,
                      map_id: str | None = None, map_version: int | None = None, source: str | None = None,
                      grid_stamp_ns: int | None = None):
        if request.headers.get('content-type') != 'image/png':
            raise HTTPException(415, 'send the map as image/png')
        if not 0 < resolution <= 10:
            raise HTTPException(400, 'resolution must be metres per pixel, 0 < r <= 10')
        if source == 'live_navigation' and (not map_id or not map_version or not grid_stamp_ns):
            raise HTTPException(400, 'live navigation grids require map identity and a grid timestamp')
        data = await request.body()
        if not data or len(data) > MAX_IMAGE_BYTES or not data.startswith(b'\x89PNG'):
            raise HTTPException(400, 'expected a PNG under 5 MB')
        try:
            with Image.open(io.BytesIO(data)) as image:
                width, height = image.size
                image.verify()
        except Exception:
            raise HTTPException(400, 'the PNG could not be decoded')
        seq = (live['map'] or {}).get('seq', 0) + 1
        live['map'] = {'seq': seq, 'resolution': resolution, 'origin': [origin_x, origin_y],
                       'width': width, 'height': height, 'png': data}
        if source == 'live_slam':
            live['map']['source'] = source
        elif map_id is not None and map_version is not None:
            live['map'].update(map_id=map_id, map_version=map_version)
        if source == 'live_navigation':
            live['map'].update(source=source, grid_stamp_ns=grid_stamp_ns)
        live['map_images'].append((seq, data))
        for mission in store.missions():  # every running patrol keeps the newest map it drove on
            if mission['status'] == 'RUNNING':
                store.save_mission_map(mission['id'], live['map'])
        bus.publish('map', map_meta())

    @app.get('/api/map')
    async def get_map():
        return map_meta()

    @app.get('/api/map.png')
    async def get_map_png(v: int | None = None):
        for seq, png in reversed(live['map_images']):
            if v is None or v == seq:
                return Response(png, media_type='image/png', headers={'Cache-Control': 'no-store'})
        raise HTTPException(404, 'map revision unavailable')

    # Route history: where a finished (or running) patrol went, on the map it used
    @app.get('/api/missions/{mission_id}/route')
    async def get_route(mission_id: str):
        require_mission(mission_id)
        grid = store.mission_map(mission_id)
        return {'poses': store.route(mission_id),
                'grid': None if grid is None else {k: v for k, v in grid.items() if k != 'png'}}

    @app.get('/api/missions/{mission_id}/map.png')
    async def get_mission_map_png(mission_id: str):
        require_mission(mission_id)
        grid = store.mission_map(mission_id)
        if grid is None:
            raise HTTPException(404, 'no occupancy map was recorded for this mission')
        return Response(grid['png'], media_type='image/png')

    # Navigation goal (browser -> robot side). One current goal; a new one replaces it, like RViz.
    @app.post('/api/goal', status_code=201)
    async def set_goal(body: Goal):
        if not allow_goals:
            raise HTTPException(403, 'goals are disabled; start the dashboard with --goals')
        if exploring():  # the explorer drives the same Nav2 action, and pausing it cancels every goal
            raise HTTPException(409, 'the robot is exploring; finish mapping before setting a goal')
        if (live['map'] or {}).get('source') == 'live_slam':
            raise HTTPException(409, 'LIVE_MAP_PREVIEW: navigation requires a checked map')
        seq = (live['goal'] or {}).get('seq', 0) + 1
        goal = body.model_dump(exclude_none=True)
        grid = live['map'] or {}
        if body.map_id is None and body.map_version is None and 'map_id' in grid:
            goal.update(map_id=grid['map_id'], map_version=grid['map_version'])
        if body.grid_stamp_ns is None and grid.get('source') == 'live_navigation':
            goal['grid_stamp_ns'] = grid['grid_stamp_ns']
        live['goal'] = {'seq': seq, 'request_id': uuid.uuid4().hex, **goal, 'status': 'SENT', 'detail': ''}
        bus.publish('goal', dict(live['goal']))
        return live['goal']

    @app.get('/api/goal')
    async def get_goal():
        return live['goal']

    @app.post('/api/goal/status')
    async def goal_status(body: GoalUpdate):
        goal = live['goal']
        if goal is None or goal['seq'] != body.seq or (body.request_id is not None and goal['request_id'] != body.request_id):
            raise HTTPException(409, 'that goal is no longer the current one')
        goal.update(status=body.status.value, detail=body.detail[:200])
        bus.publish('goal', dict(goal))
        return goal

    # Exploration (browser -> bridge -> m-explore-ros2): the operator starts mapping whenever they want
    # and can finish it at any time; the explorer also ends it itself when no frontiers are left.
    def exploring():
        return ExplorationState(live['exploration']['state']) in EXPLORATION_ACTIVE

    def goal_underway():
        return live['goal'] is not None and live['goal']['status'] in ('SENT', 'ACCEPTED')

    def set_exploration(seq, state, detail='', by=''):
        live['exploration'] = {'seq': seq, 'state': state.value, 'detail': detail[:200], 'by': by, 'since': now()}
        bus.publish('exploration', live['exploration'])
        return live['exploration']

    @app.post('/api/exploration', status_code=201)
    async def request_exploration(body: ExplorationRequest):
        if not allow_goals:
            raise HTTPException(403, 'robot control is off; start the dashboard in live mode')
        current = ExplorationState(live['exploration']['state'])
        seq = live['exploration']['seq'] + 1
        if body.action == ExplorationAction.START:
            if current in EXPLORATION_ACTIVE:
                raise HTTPException(409, f'exploration is already {current.value}')
            if goal_underway():
                raise HTTPException(409, 'a navigation goal is underway; wait for it or set no new one')
            return set_exploration(seq, ExplorationState.STARTING, by='operator')
        if current not in (ExplorationState.STARTING, ExplorationState.EXPLORING):
            raise HTTPException(409, f'nothing to finish: exploration is {current.value}')
        return set_exploration(seq, ExplorationState.FINISHING, by='operator')

    @app.get('/api/exploration')
    async def get_exploration():
        return live['exploration']

    @app.post('/api/exploration/status')
    async def exploration_status(body: ExplorationUpdate):
        if body.seq != live['exploration']['seq']:
            raise HTTPException(409, 'that request is no longer the current one')
        return set_exploration(body.seq, body.state, body.detail, body.by or live['exploration']['by'])

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

    # Site checks: nesseler's other checklists. Taken in and kept for the history pages only;
    # the live page stays about ladders.
    @app.get('/api/site-checklists')
    async def site_checklist_catalogue():
        return site_checklists.describe()

    @app.post('/api/missions/{mission_id}/site-checks', status_code=201)
    async def add_site_check(mission_id: str, body: SiteCheck):
        require_mission(mission_id)
        store.put_site_check(mission_id, body)
        return {'mission_id': mission_id, 'object_id': body.object_id}

    @app.get('/api/missions/{mission_id}/site-checks')
    async def get_site_checks(mission_id: str):
        require_mission(mission_id)
        return store.site_checks(mission_id)

    @app.get('/api/missions/{mission_id}/site-summary')
    async def get_site_summary(mission_id: str):
        require_mission(mission_id)
        return {'mission_id': mission_id,
                **site_checklists.patrol_summary(store.site_checks(mission_id), store.inspections(mission_id))}

    def csv_response(rows, filename):
        out = io.StringIO()
        csv.writer(out).writerows(rows)
        return Response(out.getvalue(), media_type='text/csv',
                        headers={'Content-Disposition': f'attachment; filename="{filename}"'})

    def patrol_summary(mission_id):
        return site_checklists.patrol_summary(store.site_checks(mission_id), store.inspections(mission_id))

    @app.get('/api/missions/{mission_id}/site-checks.csv')
    async def site_checks_csv(mission_id: str):
        mission = require_mission(mission_id)
        rows = [['site', 'mission', 'checklist', 'row', 'section', 'text', 'answer', 'objects', 'remark', 'photo']]
        for c in patrol_summary(mission_id)['checklists']:
            for r in c['rows']:
                photo = f"/media/{mission_id}/{r['photo']}.jpg" if r['photo'] else ''
                rows.append([mission['site_id'], mission_id, c['title'], r['id'], r['section'], r['text'], r['answer'],
                             ' '.join(r['objects']), r['remark'], photo])
        return csv_response(rows, f'checklists_{mission_id}.csv')

    @app.get('/api/missions/{mission_id}/objects.csv')
    async def objects_csv(mission_id: str):
        """Every kind the checklists ask about, found or not: one line per object, one line for a kind not found."""
        mission = require_mission(mission_id)
        checks = {c['object_id']: c for c in store.site_checks(mission_id)}
        rows = [['site', 'mission', 'kind', 'label', 'detector', 'found', 'object', 'x_m', 'y_m', 'confirmed', 'remarks']]
        for p in patrol_summary(mission_id)['presence']:
            base = [mission['site_id'], mission_id, p['kind'], p['label'], 'ja' if p['detector'] else 'nein']
            if not p['found']:
                rows.append([*base, 'nein' if p['detector'] else 'kein Detektor', '', '', '', '', ''])
            for object_id in p['objects']:
                c = checks[object_id]
                x, y = c['position'] or ('', '')
                rows.append([*base, 'ja', object_id, x, y, 'ja' if c['verified'] else 'nein', c['remarks']])
        return csv_response(rows, f'objects_{mission_id}.csv')

    @app.get('/api/sites/{site_id}/site-checks')
    async def site_checks_by_patrol(site_id: str):
        """Per patrol of the site: each checklist's headline and how many objects of each kind it found."""
        patrols = []
        for m in store.missions(site_id):
            summary = patrol_summary(m['id'])
            patrols.append({'mission_id': m['id'], 'started': m['started'],
                            'headlines': {c['key']: c['headline'] for c in summary['checklists']},
                            'found': {p['kind']: p['found'] for p in summary['presence']},
                            'objects': sum(p['found'] for p in summary['presence'])})
        kinds = [{'kind': k, 'label': site_checklists.KIND_LABELS[k], 'detector': k not in site_checklists.NO_DETECTOR}
                 for k in site_checklists.kinds()]
        return {'patrols': patrols, 'kinds': kinds}

    @app.get('/api/sites/{site_id}/objects.csv')
    async def site_objects_csv(site_id: str):
        """Objects found per kind across the site's patrols, oldest patrol first."""
        missions = list(reversed(store.missions(site_id)))
        found = {m['id']: {p['kind']: p['found'] for p in patrol_summary(m['id'])['presence']} for m in missions}
        rows = [['kind', 'label', 'detector', *[m['id'] for m in missions]]]
        for k in site_checklists.kinds():
            rows.append([k, site_checklists.KIND_LABELS[k], 'ja' if k not in site_checklists.NO_DETECTOR else 'nein',
                         *[found[m['id']][k] for m in missions]])
        return csv_response(rows, f'objects_{site_id}.csv')

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
                'checklist': checklist.describe(), 'site_checks': store.site_checks(mission_id),
                'site_summary': patrol_summary(mission_id),
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
