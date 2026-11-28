"""SQLite store for missions, events, inspections and alerts, grouped by site."""
import json
import sqlite3
import threading
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = '''
CREATE TABLE IF NOT EXISTS missions (
  id TEXT PRIMARY KEY, site_id TEXT NOT NULL, map_version TEXT, synthetic INTEGER,
  started TEXT NOT NULL, ended TEXT, status TEXT NOT NULL, summary TEXT, map TEXT);
CREATE TABLE IF NOT EXISTS events (
  seq INTEGER PRIMARY KEY AUTOINCREMENT, mission_id TEXT NOT NULL, time REAL,
  type TEXT NOT NULL, asset_id TEXT, payload TEXT);
CREATE TABLE IF NOT EXISTS inspections (
  seq INTEGER PRIMARY KEY AUTOINCREMENT, mission_id TEXT NOT NULL, asset_id TEXT NOT NULL,
  received TEXT NOT NULL, payload TEXT NOT NULL, UNIQUE(mission_id, asset_id));
CREATE TABLE IF NOT EXISTS alerts (
  id INTEGER PRIMARY KEY AUTOINCREMENT, mission_id TEXT NOT NULL, asset_id TEXT, kind TEXT NOT NULL,
  severity TEXT NOT NULL, created TEXT NOT NULL, payload TEXT NOT NULL,
  acked_by TEXT, acked_at TEXT, escalated_at TEXT);
CREATE TABLE IF NOT EXISTS poses (
  mission_id TEXT NOT NULL, time REAL NOT NULL, x REAL NOT NULL, y REAL NOT NULL, yaw REAL NOT NULL);
CREATE INDEX IF NOT EXISTS poses_by_mission ON poses (mission_id);
CREATE TABLE IF NOT EXISTS mission_maps (
  mission_id TEXT PRIMARY KEY, resolution REAL NOT NULL, origin_x REAL NOT NULL, origin_y REAL NOT NULL,
  width INTEGER NOT NULL, height INTEGER NOT NULL, png BLOB NOT NULL);
'''
# poses and mission_maps are additive tables: CREATE IF NOT EXISTS adds them to an older database
# without touching its existing rows, so no migration step is needed.


def now():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


class Duplicate(Exception):
    pass


class Store:
    def __init__(self, path):
        if str(path) != ':memory:':
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path), check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.lock = threading.Lock()
        with self.lock:
            self.db.executescript(SCHEMA)

    def _run(self, sql, args=()):
        with self.lock, self.db:
            return self.db.execute(sql, args)

    def _all(self, sql, args=()):
        with self.lock:
            return [dict(r) for r in self.db.execute(sql, args).fetchall()]

    # Missions
    def start_mission(self, m):
        try:
            self._run('INSERT INTO missions VALUES (?,?,?,?,?,NULL,?,NULL,?)',
                      (m.mission_id, m.site_id, m.map_version, int(m.synthetic), now(), 'RUNNING',
                       m.map.model_dump_json() if m.map else None))
        except sqlite3.IntegrityError as error:
            raise Duplicate(m.mission_id) from error
        return self.mission(m.mission_id)

    def end_mission(self, mission_id, end):
        self._run('UPDATE missions SET ended=?, status=?, summary=? WHERE id=?',
                  (now(), end.status, json.dumps(end.summary), mission_id))
        return self.mission(mission_id)

    def mission(self, mission_id):
        rows = self._all('SELECT * FROM missions WHERE id=?', (mission_id,))
        if not rows:
            return None
        row = rows[0]
        row['synthetic'] = bool(row['synthetic'])
        row['summary'] = json.loads(row['summary']) if row['summary'] else {}
        row['map'] = json.loads(row['map']) if row['map'] else None
        return row

    def missions(self, site_id=None):
        sql, args = 'SELECT id FROM missions', ()
        if site_id:
            sql, args = sql + ' WHERE site_id=?', (site_id,)
        return [self.mission(r['id']) for r in self._all(sql + ' ORDER BY started DESC, rowid DESC', args)]

    def sites(self):
        return [r['site_id'] for r in self._all('SELECT DISTINCT site_id FROM missions ORDER BY site_id')]

    # Events
    def add_event(self, event):
        cursor = self._run('INSERT INTO events (mission_id,time,type,asset_id,payload) VALUES (?,?,?,?,?)',
                           (event.mission_id, event.time, event.type, event.asset_id, json.dumps(event.data)))
        return cursor.lastrowid

    def events(self, mission_id, after_seq=0):
        rows = self._all('SELECT * FROM events WHERE mission_id=? AND seq>? ORDER BY seq', (mission_id, after_seq))
        for row in rows:
            row['data'] = json.loads(row.pop('payload') or '{}')
        return rows

    # Route: the robot's thinned-out poses and the occupancy map it drove on, kept per mission
    def add_pose(self, mission_id, time, x, y, yaw):
        self._run('INSERT INTO poses VALUES (?,?,?,?,?)', (mission_id, time, x, y, yaw))

    def route(self, mission_id):
        """[[time, x, y, yaw], ...] in the order received."""
        with self.lock:
            rows = self.db.execute('SELECT time, x, y, yaw FROM poses WHERE mission_id=? ORDER BY rowid',
                                   (mission_id,)).fetchall()
        return [list(r) for r in rows]

    def save_mission_map(self, mission_id, grid):
        """Keep the latest grid for this mission; a newer map from the same mission replaces it."""
        self._run('INSERT OR REPLACE INTO mission_maps VALUES (?,?,?,?,?,?,?)',
                  (mission_id, grid['resolution'], grid['origin'][0], grid['origin'][1],
                   grid['width'], grid['height'], grid['png']))

    def mission_map(self, mission_id):
        """{'resolution', 'origin', 'width', 'height', 'png'} or None."""
        rows = self._all('SELECT * FROM mission_maps WHERE mission_id=?', (mission_id,))
        if not rows:
            return None
        r = rows[0]
        return {'resolution': r['resolution'], 'origin': [r['origin_x'], r['origin_y']],
                'width': r['width'], 'height': r['height'], 'png': r['png']}

    # Inspections
    def add_inspection(self, inspection):
        try:
            self._run('INSERT INTO inspections (mission_id,asset_id,received,payload) VALUES (?,?,?,?)',
                      (inspection.mission_id, inspection.asset_id, now(), inspection.model_dump_json()))
        except sqlite3.IntegrityError as error:
            raise Duplicate(f'{inspection.mission_id}/{inspection.asset_id}') from error

    def inspections(self, mission_id):
        rows = self._all('SELECT payload, received FROM inspections WHERE mission_id=? ORDER BY seq', (mission_id,))
        return [{**json.loads(r['payload']), 'received': r['received']} for r in rows]

    def ladder_inspections(self, site_id, asset_id, up_to_mission=None):
        """This ladder's records on this site, oldest first, optionally ending at one mission."""
        rows = self._all('''SELECT i.payload, m.id AS mission_id, m.started FROM inspections i
                            JOIN missions m ON m.id=i.mission_id
                            WHERE m.site_id=? AND i.asset_id=? ORDER BY m.started, m.rowid''', (site_id, asset_id))
        out = []
        for row in rows:
            out.append({**json.loads(row['payload']), 'mission_started': row['started']})
            if row['mission_id'] == up_to_mission:
                break
        return out

    def ladder_history(self, site_id):
        """Each ladder's verdicts across this site's missions, oldest mission first."""
        rows = self._all('''SELECT i.payload FROM inspections i JOIN missions m ON m.id=i.mission_id
                            WHERE m.site_id=? ORDER BY m.started, m.rowid''', (site_id,))
        history = {}
        for row in rows:
            record = json.loads(row['payload'])
            history.setdefault(record['asset_id'], []).append(
                {'mission_id': record['mission_id'], 'status': record['status'], 'reason': record['reason'],
                 'ladder_type': record['ladder_type']})
        return history

    # Alerts
    def add_alert(self, alert):
        cursor = self._run('INSERT INTO alerts (mission_id,asset_id,kind,severity,created,payload) VALUES (?,?,?,?,?,?)',
                           (alert['mission_id'], alert.get('asset_id'), alert['kind'], alert['severity'], now(),
                            json.dumps(alert)))
        return self.alert(cursor.lastrowid)

    def update_alert_payload(self, alert_id, payload):
        self._run('UPDATE alerts SET payload=? WHERE id=?', (json.dumps(payload), alert_id))

    def ack_alert(self, alert_id, by):
        self._run('UPDATE alerts SET acked_by=?, acked_at=? WHERE id=? AND acked_at IS NULL', (by, now(), alert_id))
        return self.alert(alert_id)

    def mark_escalated(self, alert_id):
        self._run('UPDATE alerts SET escalated_at=? WHERE id=?', (now(), alert_id))

    def alert(self, alert_id):
        rows = self.alerts(where='id=?', args=(alert_id,))
        return rows[0] if rows else None

    def alerts(self, mission_id=None, where=None, args=()):
        sql = 'SELECT * FROM alerts'
        if mission_id:
            where, args = 'mission_id=?', (mission_id,)
        if where:
            sql += ' WHERE ' + where
        out = []
        for row in self._all(sql + ' ORDER BY id DESC', args):
            payload = json.loads(row.pop('payload'))
            out.append({**payload, **row})
        return out

    def unacked_critical(self):
        return self.alerts(where="severity='critical' AND acked_at IS NULL AND escalated_at IS NULL")
