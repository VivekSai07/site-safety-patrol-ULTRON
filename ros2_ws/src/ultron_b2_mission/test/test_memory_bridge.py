import json
import sqlite3
from concurrent.futures import ThreadPoolExecutor

from ultron_b2_mission.memory_bridge import MemoryBridge


def test_registry_created_on_executor_thread_can_commit_and_close_on_another(tmp_path_local):
    db_path = tmp_path_local / 'registry.sqlite'

    def start():
        memory = MemoryBridge(db_path, tmp_path_local / 'run')
        asset, _ = memory.registry.associate((1, 2, 0), 'TRESTLE', 'hall@1')
        memory.start('hall@1')
        return memory, asset

    with ThreadPoolExecutor(max_workers=1) as executor:
        memory, asset = executor.submit(start).result()
        memory.commit_capture(asset, ladder_type='TRESTLE', position=(1, 2, 0),
                              capture_status='COMPLETE', evidence_refs=['sha256:test'],
                              reason='test', from_oracle=True)
        assert memory.finish({'status': 'DONE'}) == []
        memory.close()

    with sqlite3.connect(str(db_path)) as db:
        payload = json.loads(db.execute('SELECT payload FROM inspections').fetchone()[0])
    assert payload['detection_source'] == 'ORACLE'
    assert payload['verdict'] == 'PENDING_ASSESSMENT'
