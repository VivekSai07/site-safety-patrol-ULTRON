"""Persistent map-scoped spatial assets and append-only inspection history."""
from datetime import datetime, timezone
import json
import math
import sqlite3


def now():
    return datetime.now(timezone.utc).isoformat()


class Registry:
    def __init__(self, path, *, check_same_thread=True):
        self.db = sqlite3.connect(str(path), check_same_thread=check_same_thread)
        self.db.row_factory = sqlite3.Row
        self.db.executescript("""
        PRAGMA journal_mode=WAL;
        CREATE TABLE IF NOT EXISTS missions (
          id TEXT PRIMARY KEY, map_version TEXT, started TEXT, summary TEXT);
        CREATE TABLE IF NOT EXISTS assets (
          id TEXT PRIMARY KEY, type TEXT, map_version TEXT, x REAL,y REAL,z REAL,
          state TEXT, first_seen TEXT,last_seen TEXT, latest_status TEXT, status_changed INTEGER DEFAULT 0);
        CREATE TABLE IF NOT EXISTS world_objects (
          map_version TEXT, object_key TEXT, payload TEXT, last_mission TEXT,
          PRIMARY KEY(map_version,object_key));
        CREATE TABLE IF NOT EXISTS inspections (
          seq INTEGER PRIMARY KEY AUTOINCREMENT, mission_id TEXT, asset_id TEXT, timestamp TEXT,payload TEXT,
          UNIQUE(mission_id,asset_id));
        """)

    def remember_objects(self, mission_id,map_version,geometries):
        groups={}
        for geom in geometries:
            key=geom['entity']
            groups.setdefault(key,[]).append(geom)
        with self.db:
            for key,parts in groups.items():
                position=[sum(g['position'][axis] for g in parts)/len(parts) for axis in range(3)]
                payload={'object_key':key,'position':position,'parts':parts,
                         'source':'SIMULATION_WORLD_ORACLE','map_version':map_version}
                self.db.execute('INSERT OR REPLACE INTO world_objects VALUES (?,?,?,?)',
                                (map_version,key,json.dumps(payload),mission_id))

    def objects(self):
        return [json.loads(row[0]) for row in self.db.execute('SELECT payload FROM world_objects ORDER BY object_key')]

    def start_mission(self, map_version):
        idx = self.db.execute('SELECT count(*) FROM missions').fetchone()[0]+1
        mid = f'PATROL-{idx:02}'
        self.db.execute('INSERT INTO missions VALUES (?,?,?,NULL)',(mid,map_version,now()))
        self.db.commit()
        return mid

    def associate(self, position, ladder_type, map_version, radius=.65):
        x,y,z = map(float,position)
        if not all(math.isfinite(v) for v in (x,y,z)):
            raise ValueError('Nonfinite spatial observation')
        neighbors = [a for a in self.assets() if a['map_version']==map_version
                     and a['type']==ladder_type and math.hypot(a['x']-x,a['y']-y)<=radius]
        if len(neighbors)==1:
            asset=neighbors[0]['id']
            self.db.execute('UPDATE assets SET last_seen=?,state=? WHERE id=?',(now(),'ACTIVE',asset))
            self.db.commit()
            return asset,'REIDENTIFIED'
        state = 'REID_AMBIGUOUS' if neighbors else 'NEW'
        idx=self.db.execute('SELECT count(*) FROM assets').fetchone()[0]+1
        asset=f'LAD-{idx:03}'
        self.db.execute('INSERT INTO assets VALUES (?,?,?,?,?,?,?,?,?,?,?)',
            (asset,ladder_type,map_version,x,y,z,state,now(),now(),None,0))
        self.db.commit()
        return asset,state

    def append(self, mission_id, asset_id, payload):
        previous=self.db.execute('SELECT latest_status FROM assets WHERE id=?',(asset_id,)).fetchone()[0]
        payload['current_observation_status']=payload['status']
        if previous=='FLAGGED':
            prior=self.history(asset_id)[-1]['payload']
            from .inspection import fuse
            payload['checks']=fuse(prior.get('checks',{}),payload.get('checks',{}))
            if payload['status']!='FLAGGED':
                payload.update(status='FLAGGED',reason='Unresolved prior flag: '+prior['reason'])
                payload['prior_flag_requires_human_resolution']=True
        with self.db:
            self.db.execute('INSERT INTO inspections(mission_id,asset_id,timestamp,payload) VALUES (?,?,?,?)',
                            (mission_id,asset_id,now(),json.dumps(payload,allow_nan=False)))
            self.db.execute('UPDATE assets SET latest_status=?,status_changed=? WHERE id=?',
                            (payload['status'], int(previous is not None and previous!=payload['status']),asset_id))

    def assets(self):
        return [dict(row) for row in self.db.execute('SELECT * FROM assets ORDER BY id')]

    def history(self, asset_id):
        return [dict(row) | {'payload':json.loads(row['payload'])} for row in
                self.db.execute('SELECT * FROM inspections WHERE asset_id=? ORDER BY seq',(asset_id,))]

    def missing(self, mission_id):
        version=self.db.execute('SELECT map_version FROM missions WHERE id=?',(mission_id,)).fetchone()[0]
        ids=[row[0] for row in self.db.execute('''SELECT id FROM assets WHERE map_version=? AND id NOT IN
            (SELECT asset_id FROM inspections WHERE mission_id=?)''',(version,mission_id))]
        with self.db:
            self.db.executemany('UPDATE assets SET state=? WHERE id=?',[('MISSING_EXPECTED',i) for i in ids])
        return ids

    def finish(self, mission_id, summary):
        with self.db:
            self.db.execute('UPDATE missions SET summary=? WHERE id=?',(json.dumps(summary),mission_id))

    def query(self, text='', near=None, radius=2):
        matches=[]
        for a in self.assets():
            history=self.history(a['id'])
            if text.lower() not in json.dumps(a | {'history':history}).lower():
                continue
            if near is not None and math.hypot(a['x']-near[0],a['y']-near[1])>radius:
                continue
            matches.append(a | {'history':history})
        return matches

    def close(self):
        self.db.close()
