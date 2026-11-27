"""Play a recorded simulation run into a running dashboard, as if it were live.

    python -m ultron.dashboard.replay runs/history-demo --speed 4

Posts, in mission-time order: mission start (with the site map), robot poses, front RGB and
depth frames, events, evidence photos + inspections, and mission end. Uses only the HTTP API,
the same one the ROS side will use on the real robot.
"""
import argparse
import io
import json
import time
import urllib.error
import urllib.request
from pathlib import Path

from .checklist import SIM_TYPE_TO_ASCENT, from_sim_checks

DEPTH_RANGE_M = (0.3, 8.0)


class Api:
    def __init__(self, url):
        self.url = url.rstrip('/')

    def post(self, path, body=None, raw=None, content_type='application/json'):
        data = raw if raw is not None else json.dumps(body or {}).encode()
        request = urllib.request.Request(self.url + path, data=data, method='POST',
                                         headers={'Content-Type': content_type})
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, json.loads(response.read() or b'null')
        except urllib.error.HTTPError as error:
            return error.code, error.read().decode(errors='replace')


def depth_jpeg(path):
    import numpy as np
    from PIL import Image
    depth = np.load(path)
    near, far = DEPTH_RANGE_M
    valid = (depth > near) & (depth < far)
    t = np.clip((depth - near) / (far - near), 0, 1)
    anchors = np.array([[253, 231, 37], [94, 201, 98], [33, 145, 140], [59, 82, 139], [68, 1, 84]], float)
    xs = np.linspace(0, 1, len(anchors))
    rgb = np.stack([np.interp(t, xs, anchors[:, c]) for c in range(3)], -1)
    rgb[~valid] = (20, 22, 26)
    out = io.BytesIO()
    Image.fromarray(rgb.astype('uint8')).save(out, 'JPEG', quality=80)
    return out.getvalue()


def convert_record(record, telemetry, run_dir, mission_t0, verdict_time):
    """Simulation inspection record -> dashboard inspection payload + evidence photo files."""
    ascent = SIM_TYPE_TO_ASCENT.get(record['ladder_type'], 'Sonstige')
    checklist = from_sim_checks(record.get('checks', {}), record.get('required_components', []), ascent)
    photos, files = [], []
    for n, view in enumerate(record.get('views', []), 1):
        frame = telemetry.get(view['index'])
        if not frame:
            continue
        photo_id = f"{record['asset_id']}-{view['index']}"
        files.append((photo_id, run_dir / frame['rgb']))
        photos.append({'id': photo_id, 'camera': 'front_rgb',
                       'caption': f"Frontkamera, Missionszeit {view['time'] - mission_t0:.0f} s ({frame.get('state', '')})"})
    ids = [p['id'] for p in photos]
    item_photos = {key: ids for key, result in checklist.items() if result in ('OK', 'DEFECT', 'UNKNOWN') and ids}
    label = record.get('label_reading') or {}
    payload = {
        **{k: record[k] for k in ('mission_id', 'asset_id', 'ladder_type', 'status', 'reason', 'checks', 'missing',
                                  'label_status', 'position', 'safe_to_use', 'tactile_checks_pending',
                                  'human_signoff_required') if k in record},
        'time': verdict_time,
        'checklist': checklist, 'photos': photos, 'item_photos': item_photos,
        'ladder': {'ascent_type': ascent, 'sticker_due': label.get('due_date'), 'sticker_status': label.get('status')},
    }
    return payload, files


def mission_timeline(mission, events, telemetry_rows):
    rows = [(t['time'], 0, 'pose', t) for t in telemetry_rows]
    rows += [(e['time'], 1, 'event', e) for e in events]
    return sorted(rows, key=lambda r: (r[0], r[1]))


def replay(run_dir, url, speed=4.0, site='demo-site', tag='', sleep=time.sleep, log=print):
    run_dir = Path(run_dir)
    api = Api(url)
    protocol = json.loads((run_dir / 'protocol.json').read_text())
    events = [json.loads(line) for line in (run_dir / 'events.jsonl').read_text().splitlines() if line]
    telemetry = [json.loads(line) for line in (run_dir / 'telemetry.jsonl').read_text().splitlines() if line]
    by_index = {t['index']: t for t in telemetry}
    for mission in protocol['missions']:
        source_id = mission['mission_id']
        mission_id = source_id + tag
        tel = [t for t in telemetry if t.get('mission_id') == source_id]
        evs = [e for e in events if e['mission_id'] == source_id]
        t0 = min([t['time'] for t in tel] + [e['time'] for e in evs])
        records = {r['asset_id']: r for r in mission['records']}
        site_map = {'bounds': mission['zone_bounds'], 'keepouts': mission.get('keepout_rectangles', []),
                    'base': tel[0]['pose'][:2] if tel else None}
        status, body = api.post('/api/missions', {'mission_id': mission_id, 'site_id': site, 'synthetic': True,
                                                  'map_version': 'sim', 'map': site_map})
        if status != 201:
            raise SystemExit(f'could not start {mission_id}: {status} {body}. Use --tag or a fresh --db.')
        log(f'{mission_id}: started')
        last = None
        for when, _, kind, row in mission_timeline(mission, evs, tel):
            if last is not None and speed > 0:
                sleep(max(0.0, (when - last) / speed))
            last = when
            t = round(when - t0, 2)
            if kind == 'pose':
                x, y, yaw = row['pose']
                api.post(f'/api/missions/{mission_id}/pose', {'time': t, 'x': x, 'y': y, 'yaw': yaw, 'state': row.get('state', '')})
                api.post(f'/api/missions/{mission_id}/frames/front_rgb', raw=(run_dir / row['rgb']).read_bytes(),
                         content_type='image/jpeg')
                if row.get('depth') and (run_dir / row['depth']).exists():
                    api.post(f'/api/missions/{mission_id}/frames/front_depth', raw=depth_jpeg(run_dir / row['depth']),
                             content_type='image/jpeg')
                continue
            data = {k: v for k, v in row.items() if k not in ('mission_id', 'type', 'time', 'asset_id', 'observation')}
            if isinstance(row.get('observation'), dict):  # keep the map pin, drop bulky sim internals
                data['position'] = row['observation'].get('position')
                data['ladder_type'] = row['observation'].get('type')
            api.post('/api/events', {'mission_id': mission_id, 'type': row['type'], 'time': t,
                                     'asset_id': row.get('asset_id'), 'data': json.loads(json.dumps(data, default=str))})
            if row['type'] == 'INSPECTION_COMMITTED' and row.get('asset_id') in records:
                record = {**records[row['asset_id']], 'mission_id': mission_id}
                payload, files = convert_record(record, by_index, run_dir, t0, t)
                for photo_id, path in files:
                    api.post(f'/api/missions/{mission_id}/photos/{photo_id}', raw=path.read_bytes(), content_type='image/jpeg')
                status, body = api.post('/api/inspections', payload)
                log(f"{mission_id}: {row['asset_id']} {payload['status']} -> HTTP {status}")
                if status != 201:
                    log(f'   rejected: {body}')
            if row['type'] == 'MISSION_FINISHED':
                summary = {k: mission.get(k) for k in ('found', 'missing_expected', 'waypoint_coverage_pct', 'coverage_note',
                                                        'operator_interventions', 'returned_to_base')}
                api.post(f'/api/missions/{mission_id}/end', {'status': row.get('status') or mission['status'], 'summary': summary})
                log(f'{mission_id}: finished')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Replay a simulation run into the dashboard')
    parser.add_argument('run_dir')
    parser.add_argument('--url', default='http://127.0.0.1:8000')
    parser.add_argument('--speed', type=float, default=4.0, help='times faster than real time; 0 = no waiting')
    parser.add_argument('--site', default='demo-site')
    parser.add_argument('--tag', default='', help='suffix for mission ids, to replay the same run twice')
    args = parser.parse_args(argv)
    replay(args.run_dir, args.url, args.speed, args.site, args.tag)


if __name__ == '__main__':
    main()
