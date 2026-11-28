"""Mission memory: SQLite registry via src/ultron/memory.py plus dashboard-compatible events.jsonl.

No schema change: missions/inspections go through ``Registry``; the mission event log is the
``events.jsonl`` format consumed by ``ultron.dashboard.live_tail`` (rows need mission_id, type,
time). Captures are committed as ``EVIDENCE_CAPTURED`` with the verdict pending; this module
never writes a safety verdict. AUTOMATED_PASS (if ever produced downstream) is pre-screening only.
"""
import json
import math
import pathlib
import time

from ultron_b2_perception.registry_store import open_registry


class MemoryBridge:
    def __init__(self, db_path='', run_dir='', clock=time.time):
        self.registry = open_registry(db_path) if db_path else None
        self.run_dir = pathlib.Path(run_dir) if run_dir else None
        if self.run_dir:
            self.run_dir.mkdir(parents=True, exist_ok=True)
        self.clock = clock
        self.mission_id = None
        self._local_seq = 0

    def start(self, map_key, requested_id='', zone=None):
        if self.registry is not None:
            self.mission_id = self.registry.start_mission(map_key)
        else:
            self._local_seq += 1
            self.mission_id = requested_id or f'PATROL-LOCAL-{self._local_seq:02}'
        row = {'map_version': map_key, 'requested_mission_id': requested_id}
        if zone is not None:
            xs = [p[0] for p in zone.boundary]
            ys = [p[1] for p in zone.boundary]
            row.update(zone_bounds=[min(xs), min(ys), max(xs), max(ys)],
                       keepouts=[[list(p) for p in k] for k in zone.keepout],
                       base=list(zone.base_pose[:2]) if zone.base_pose else None)
        self.event('MISSION_START', **row)
        return self.mission_id

    def event(self, event_type, asset_id=None, **data):
        if not self.run_dir or not self.mission_id:
            return
        row = {'mission_id': self.mission_id, 'type': event_type, 'time': self.clock(), 'asset_id': asset_id}
        row.update(data)
        with (self.run_dir / 'events.jsonl').open('a') as fh:
            fh.write(json.dumps(row, default=str, allow_nan=False) + '\n')

    def commit_capture(self, asset_id, *, ladder_type, position, capture_status, evidence_refs, reason,
                       from_oracle):
        """Append an inspection record. Status is evidence state, never a safety verdict."""
        payload = {
            'schema': 'ultron.mission_capture/v1',
            'status': 'EVIDENCE_CAPTURED' if capture_status in ('COMPLETE', 'PARTIAL') else 'CAPTURE_FAILED',
            'capture_status': capture_status,
            'reason': reason,
            'verdict': 'PENDING_ASSESSMENT',
            'human_review_required': True,
            'automated_pass_scope': 'PRE_SCREENING_ONLY_NOT_SAFE_TO_USE',
            'evidence_refs': list(evidence_refs),
            'ladder_type': ladder_type,
            'position': [float(v) for v in position] if all(math.isfinite(float(v)) for v in position) else None,
            'detection_source': 'ORACLE' if from_oracle else 'PERCEPTION',
        }
        if self.registry is not None and asset_id:
            known = self.registry.db.execute('SELECT 1 FROM assets WHERE id=?', (asset_id,)).fetchone()
            if known:
                try:
                    self.registry.append(self.mission_id, asset_id, dict(payload))
                except Exception as exc:  # noqa: BLE001  (UNIQUE(mission,asset) on re-capture)
                    payload['registry_error'] = str(exc)
        self.event('CAPTURE_COMMITTED', asset_id=asset_id, observation={'position': payload['position'],
                                                                        'type': ladder_type}, **{
            k: payload[k] for k in ('status', 'capture_status', 'verdict', 'human_review_required',
                                    'automated_pass_scope', 'evidence_refs', 'detection_source')})
        return payload

    def finish(self, summary):
        missing = []
        if self.registry is not None and self.mission_id:
            missing = self.registry.missing(self.mission_id)
            self.registry.finish(self.mission_id, dict(summary, not_inspected_this_mission=missing))
        self.event('MISSION_FINISHED', **summary)
        return missing

    def close(self):
        if self.registry is not None:
            self.registry.close()
            self.registry = None
