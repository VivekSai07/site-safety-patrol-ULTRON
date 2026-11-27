"""Optional ingestion into the real DimOS SqliteStore API; no B2 transport implied."""
import json
from pathlib import Path
from datetime import datetime


def ingest(run, destination):
    try:
        from dimos.memory.store.sqlite import SqliteStore
    except ImportError as exc:
        raise RuntimeError('Install DimOS in its own supported Python environment first; see docs/INTEGRATION.md') from exc
    data=json.loads((Path(run)/'protocol.json').read_text())
    store=SqliteStore(path=str(destination))
    try:
        objects=store.stream('ultron_world_objects')
        import time
        for obj in data.get('world_objects',[]):
            objects.append(obj,ts=time.time(),pose=tuple(obj['position']),
                           tags={'synthetic':True,'map_version':obj['map_version']})
        stream=store.stream('ultron_inspections')
        for asset in data['assets']:
            for event in asset['history']:
                stream.append(event['payload'],ts=datetime.fromisoformat(event['timestamp']).timestamp(),
                              pose=(asset['x'],asset['y'],asset['z']),
                              tags={'asset_id':asset['id'],'synthetic':True,'mission_id':event['mission_id']})
    finally:
        store.stop()
    print(f'DimOS memory written to {destination}')
