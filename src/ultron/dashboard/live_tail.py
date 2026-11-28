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
    def __init__(self, run_dir, api, site='demo-site', tag='', log=print):
        self.run = Path(run_dir)
        self.api = api
        self.site = site
        self.tag = tag
        self.log = log
        self.offsets = {'events.jsonl': 0, 'telemetry.jsonl': 0}
        self.started = set()
        self.t0 = None
        self.finished_missions = set()

    def _new_rows(self, name):
        path = self.run / name
        if not path.exists():
            return []
        with path.open() as handle:
            handle.seek(self.offsets[name])
            chunk = handle.read()
            # Only consume complete lines; keep a partial trailing line for the next poll.
            complete = chunk.rfind('\n') + 1
            self.offsets[name] += complete
            return [json.loads(line) for line in chunk[:complete].splitlines() if line.strip()]

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
            self.log(f'live_tail: mission start {mission} -> HTTP {status} {body}')
        self.started.add(mission)

    def step(self):
        posted = 0
        for row in self._new_rows('events.jsonl'):
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
            self.api.post('/api/events', {'mission_id': mission, 'type': row['type'], 'time': max(t, 0.0),
                                          'asset_id': row.get('asset_id'),
                                          'data': json.loads(json.dumps(data, default=str))})
            posted += 1
            if row['type'] == 'MISSION_FINISHED':
                self.finished_missions.add(row['mission_id'])
        for row in self._new_rows('telemetry.jsonl'):
            if self.t0 is None:
                self.t0 = row['time']
            mission = self._mission_id(row)
            if mission not in self.started:
                continue
            x, y, yaw = row['pose']
            t = round(row['time'] - self.t0, 2)
            self.api.post(f'/api/missions/{mission}/pose',
                          {'time': max(t, 0.0), 'x': x, 'y': y, 'yaw': yaw, 'state': row.get('state', '')})
            rgb = self.run / row.get('rgb', '')
            if row.get('rgb') and rgb.exists():
                self.api.post(f'/api/missions/{mission}/frames/front_rgb', raw=rgb.read_bytes(),
                              content_type='image/jpeg')
            if row.get('depth') and (self.run / row['depth']).exists():
                self.api.post(f'/api/missions/{mission}/frames/front_depth',
                              raw=depth_jpeg(self.run / row['depth']), content_type='image/jpeg')
            posted += 1
        return posted

    def finish(self):
        """Post inspections, evidence photos and mission end once the run is complete."""
        protocol_path = self.run / 'protocol.json'
        if not protocol_path.exists():
            return False
        protocol = json.loads(protocol_path.read_text())
        telemetry = [json.loads(line) for line in (self.run / 'telemetry.jsonl').read_text().splitlines() if line]
        by_index = {t['index']: t for t in telemetry}
        for mission_data in protocol['missions']:
            source = mission_data['mission_id']
            mission = source + self.tag
            t0 = self.t0 if self.t0 is not None else 0.0
            for record in mission_data['records']:
                payload, files = convert_record({**record, 'mission_id': mission}, by_index, self.run,
                                                t0, mission_data.get('duration_sim_s'))
                for photo_id, path in files:
                    self.api.post(f'/api/missions/{mission}/photos/{photo_id}',
                                  raw=path.read_bytes(), content_type='image/jpeg')
                status, body = self.api.post('/api/inspections', payload)
                self.log(f"live_tail: {mission} {record['asset_id']} {payload['status']} -> HTTP {status}")
            summary = {k: mission_data.get(k) for k in ('found', 'missing_expected', 'waypoint_coverage_pct',
                                                        'observed_coverage_pct', 'coverage_note',
                                                        'operator_interventions', 'returned_to_base')}
            self.api.post(f'/api/missions/{mission}/end',
                          {'status': mission_data['status'], 'summary': summary})
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
    args = parser.parse_args(argv)
    tail = LiveTail(args.run, Api(args.url), site=args.site, tag=args.tag)
    idle = 0.0
    print(f'live_tail: watching {args.run} -> {args.url}')
    while True:
        try:
            posted = tail.step()
        except (OSError, ValueError) as exc:
            print(f'live_tail: {exc}; retrying')
            posted = 0
        if posted:
            idle = 0.0
        else:
            idle += args.poll
        if (tail.run / 'manifest.json').exists() and tail.step() == 0:
            tail.finish()
            print('live_tail: run complete, final records posted')
            return 0
        if idle >= args.idle_timeout:
            print('live_tail: idle timeout reached, exiting')
            return 1
        time.sleep(args.poll)


if __name__ == '__main__':
    raise SystemExit(main())
