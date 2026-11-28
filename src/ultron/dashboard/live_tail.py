"""Stream a RUNNING simulation's JSONL output to the dashboard as it is written.

Same posting rules as replay.py, but incremental: poll the run directory,
post new telemetry rows (pose + camera frames) and events as they appear.
Inspections, photos and mission end are posted once protocol.json exists.
The simulation never depends on this process; any HTTP failure only logs.

  python3 -m ultron.dashboard.live_tail runs/live-e2e --url http://127.0.0.1:8000
"""
import argparse
import json
import time
from pathlib import Path

from .replay import Api, convert_record, depth_jpeg


class LiveTail:
    def __init__(self, run_dir, api, site='demo-site', tag='', log=print, assessment_worker=None):
        self.assessment_worker = assessment_worker
        self.run = Path(run_dir)
        self.api = api
        self.site = site
        self.tag = tag
        self.log = log
        self.offsets = {'events.jsonl': 0, 'telemetry.jsonl': 0}
        self.started = set()
        self.t0 = None
        self.finished_missions = set()
        self.closed_missions = set()
        self.published_records = set()
        self.backlog = {'events.jsonl': [], 'telemetry.jsonl': []}

    def _new_rows(self, name):
        path = self.run / name
        if not path.exists():
            return []
        with path.open('rb') as handle:
            handle.seek(self.offsets[name])
            chunk = handle.read()
            # Only consume complete lines; keep a partial trailing line for the next poll.
            complete = chunk.rfind(b'\n') + 1
            rows=[json.loads(line) for line in chunk[:complete].splitlines() if line.strip()]
            self.offsets[name] += complete
            return rows

    def _mission_id(self, row):
        return row['mission_id'] + self.tag

    def _ensure_started(self, row):
        mission = self._mission_id(row)
        if mission in self.started:
            return
        site_map = None
        if row.get('zone_bounds'):
            site_map = {'bounds': row['zone_bounds'], 'keepouts': row.get('keepouts', []),
                        'base': row.get('base')}
        status, body = self.api.post('/api/missions', {
            'mission_id': mission, 'site_id': self.site, 'synthetic': True,
            'map_version': 'sim', 'map': site_map})
        if status not in (201, 409):
            raise RuntimeError(f'Mission publication failed: HTTP {status}')
        self.started.add(mission)

    def step(self):
        posted = 0
        self.backlog['events.jsonl'].extend(self._new_rows('events.jsonl'))
        while self.backlog['events.jsonl']:
            row=self.backlog['events.jsonl'][0]
            if self.t0 is None:
                self.t0 = row['time']
            mission = self._mission_id(row)
            if row['type'] == 'MISSION_START':
                self._ensure_started(row)
            if mission not in self.started:
                self._ensure_started(row)
            t = round(row['time'] - self.t0, 2)
            data = {k: v for k, v in row.items() if k not in ('mission_id', 'type', 'time', 'asset_id', 'observation')}
            if isinstance(row.get('observation'), dict):  # keep the map pin, drop bulky sim internals
                data['position'] = row['observation'].get('position')
                data['ladder_type'] = row['observation'].get('type')
            status, _ = self.api.post('/api/events', {'mission_id': mission, 'type': row['type'], 'time': max(t, 0.0),
                                          'asset_id': row.get('asset_id'),
                                          'data': json.loads(json.dumps(data, default=str))})
            if status!=201:raise RuntimeError('Event publication failed')
            if row['type'] == 'ASSESSMENT_READY' and self.assessment_worker:
                self.assessment_worker.submit(row['manifest'], mission)
            if row['type'] == 'MISSION_FINISHED':
                self.finished_missions.add(row['mission_id'])
            self.backlog['events.jsonl'].pop(0)
            posted += 1
        self.backlog['telemetry.jsonl'].extend(self._new_rows('telemetry.jsonl'))
        while self.backlog['telemetry.jsonl']:
            row=self.backlog['telemetry.jsonl'][0]
            if self.t0 is None:
                self.t0 = row['time']
            mission = self._mission_id(row)
            if mission not in self.started:
                break
            x, y, yaw = row['pose']
            t = round(row['time'] - self.t0, 2)
            status, _ = self.api.post(f'/api/missions/{mission}/pose',
                          {'time': max(t, 0.0), 'x': x, 'y': y, 'yaw': yaw, 'state': row.get('state', '')})
            if status not in (200,201,204):raise RuntimeError('Pose publication failed')
            rgb = self.run / row.get('rgb', '')
            if row.get('rgb') and rgb.exists():
                status, _ = self.api.post(f'/api/missions/{mission}/frames/front_rgb', raw=rgb.read_bytes(),
                              content_type='image/jpeg')
                if status not in (200,201,204):raise RuntimeError('Frame publication failed')
            if row.get('depth') and (self.run / row['depth']).exists():
                status, _ = self.api.post(f'/api/missions/{mission}/frames/front_depth',
                              raw=depth_jpeg(self.run / row['depth']), content_type='image/jpeg')
                if status not in (200,201,204):raise RuntimeError('Depth publication failed')
            self.backlog['telemetry.jsonl'].pop(0)
            posted += 1
        if self.assessment_worker:
            posted += self.assessment_worker.publish_ready(self.api)
        return posted

    def finish(self):
        """Post inspections, evidence photos and mission end once the run is complete."""
        protocol_path = self.run / 'protocol.json'
        if not protocol_path.exists():
            return False
        if self.assessment_worker:
            self.assessment_worker.publish_ready(self.api, wait=True)
        protocol = json.loads(protocol_path.read_text())
        telemetry = [json.loads(line) for line in (self.run / 'telemetry.jsonl').read_text().splitlines() if line]
        by_index = {t['index']: t for t in telemetry}
        for mission_data in protocol['missions']:
            source = mission_data['mission_id']
            mission = source + self.tag
            if mission in self.closed_missions:continue
            t0 = self.t0 if self.t0 is not None else 0.0
            for record in mission_data['records']:
                if (mission,record['asset_id']) in self.published_records:continue
                if self.assessment_worker and (mission, record['asset_id']) in self.assessment_worker.completed:
                    continue
                if self.assessment_worker:
                    from ultron.patrol_assessment import record_manifest
                    fallback=record_manifest({**record,'mission_id':mission},self.run,pack=record.get('assessment_pack','inspection_v1'))
                    relative=Path('assessment-fallbacks')/mission/(record['asset_id']+'.json')
                    (self.run/relative).parent.mkdir(parents=True,exist_ok=True)
                    (self.run/relative).write_text(json.dumps(fallback))
                    self.assessment_worker.submit(relative,mission,review_reason='REVIEW_NO_LIVE_JOB')
                    self.assessment_worker.publish_ready(self.api,wait=True)
                    continue
                payload, files = convert_record({**record, 'mission_id': mission}, by_index, self.run,
                                                t0, mission_data.get('duration_sim_s'))
                for photo_id, path in files:
                    code,_=self.api.post(f'/api/missions/{mission}/photos/{photo_id}',
                                  raw=path.read_bytes(), content_type='image/jpeg')
                    if code not in (200,201):raise RuntimeError('Evidence publication failed')
                status, body = self.api.post('/api/inspections', payload)
                if status!=201:raise RuntimeError('Inspection publication failed')
                self.published_records.add((mission,record['asset_id']))
                self.log(f"live_tail: {mission} {record['asset_id']} {payload['status']} -> HTTP {status}")
            summary = {k: mission_data.get(k) for k in ('found', 'missing_expected', 'waypoint_coverage_pct',
                                                        'observed_coverage_pct', 'coverage_note',
                                                        'operator_interventions', 'returned_to_base')}
            status,_=self.api.post(f'/api/missions/{mission}/end',
                          {'status': mission_data['status'], 'summary': summary})
            if status not in (200,201,204):raise RuntimeError('Mission closure failed')
            self.closed_missions.add(mission)
            self.log(f'live_tail: {mission} finished')
        return True


def main(argv=None):
    parser = argparse.ArgumentParser(description='Stream a running simulation to the dashboard')
    parser.add_argument('run', type=Path)
    parser.add_argument('--url', default='http://127.0.0.1:8000')
    parser.add_argument('--site', default='demo-site')
    parser.add_argument('--tag', default='')
    parser.add_argument('--poll', type=float, default=0.5)
    parser.add_argument('--idle-timeout', type=float, default=180.0)
    parser.add_argument('--assess-openai', action='store_true', help='Send per-ladder evidence to OpenAI; one paid request per job')
    parser.add_argument('--env-file', default='.env')
    parser.add_argument('--model', help='Available vision model supporting Responses structured outputs')
    parser.add_argument('--assessment-ledger', type=Path, help='Share only with runs using the same map/asset identity registry')
    parser.add_argument('--max-assessments', type=int, default=16)
    args = parser.parse_args(argv)
    worker = None
    if args.assess_openai:
        from ultron.assessment_live import load_settings, openai_call
        from ultron.patrol_assessment import LiveAssessmentWorker
        settings=load_settings(args.env_file)
        if not settings['key']: parser.error('No OpenAI credential configured')
        code,config=Api(args.url).get('/api/notify/config')
        if code!=200 or not isinstance(config,dict) or config.get('channels')!=['dashboard']:
            parser.error('Use a dashboard with external notifications disabled')
        worker=LiveAssessmentWorker(args.run, model=args.model or settings['model'],
            api_call=lambda body: openai_call(body, settings['key']), max_jobs=args.max_assessments, ledger_path=args.assessment_ledger)
    tail = LiveTail(args.run, Api(args.url), site=args.site, tag=args.tag, assessment_worker=worker)
    idle = 0.0
    print(f'live_tail: watching {args.run} -> {args.url}')
    failures=0
    try:
        while True:
            try:
                posted = tail.step()
                if (tail.run / 'manifest.json').exists() and not any(tail.backlog.values()):
                    if not tail.finish():raise RuntimeError('Completed run has no protocol')
                    print('live_tail: run complete, final records posted')
                    return 0
                failures=0
            except (OSError, ValueError, RuntimeError):
                # Details can include remote text: do not log arbitrary transport bodies.
                failures+=1
                print(f'live_tail: delivery failed ({failures}/10); retrying retained work')
                if failures>=10:
                    print('live_tail: delivery unavailable; local run/evidence preserved, dashboard may be incomplete')
                    return 1
                posted=0
            idle=0.0 if posted else idle+args.poll
            if idle>=args.idle_timeout:
                print('live_tail: idle timeout reached, exiting')
                return 1
            time.sleep(args.poll)
    finally:
        if worker:worker.close()


if __name__ == '__main__':
    raise SystemExit(main())
