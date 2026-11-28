"""Ladder identity persistence through ``src/ultron/memory.py`` (``Registry``), map-scoped.

The registry schema is unchanged. Asset rows use ``map_version = "<map_id>@<map_version>"``.
The tracker decides association; this module only records its decisions, so an AMBIGUOUS
candidate is stored as its own asset (state ``REID_AMBIGUOUS``) and never merged.
"""
import math
import os

from .tracker_core import AMBIGUOUS, MATCHED, MISSING, NEW
from .ultron_path import ensure_ultron_importable

STATE_TO_REGISTRY = {NEW: 'NEW', MATCHED: 'ACTIVE', AMBIGUOUS: 'REID_AMBIGUOUS', MISSING: 'MISSING_OBSERVED'}
REGISTRY_TO_STATE = {'NEW': MATCHED, 'ACTIVE': MATCHED, 'REIDENTIFIED': MATCHED, 'REID_AMBIGUOUS': AMBIGUOUS,
                     'MISSING_OBSERVED': MISSING, 'MISSING_EXPECTED': MATCHED}


def map_key(map_id, map_version):
    return f'{map_id}@{int(map_version)}'


def open_registry(path, *, check_same_thread=True):
    if not path:
        return None
    if not ensure_ultron_importable():
        raise RuntimeError('src/ultron not found (set ULTRON_SRC); cannot open the SQLite registry')
    from ultron.memory import Registry  # noqa: PLC0415
    path = os.path.expanduser(str(path))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    return Registry(path, check_same_thread=check_same_thread)


def load_priors(registry, key):
    priors = []
    for a in registry.assets():
        if a['map_version'] != key or not all(math.isfinite(v) for v in (a['x'], a['y'], a['z'])):
            continue
        priors.append({'candidate_id': a['id'], 'asset_id': a['id'], 'position': [a['x'], a['y'], a['z']],
                       'ladder_type': a['type'], 'state': REGISTRY_TO_STATE.get(a['state'], MATCHED)})
    return priors


def ensure_asset(registry, key, cand):
    """Create the registry asset for a tracker candidate if needed; returns the asset id."""
    if cand.get('asset_id'):
        return cand['asset_id']
    db = registry.db
    idx = db.execute('SELECT count(*) FROM assets').fetchone()[0] + 1
    asset = f'LAD-{idx:03}'
    while db.execute('SELECT 1 FROM assets WHERE id=?', (asset,)).fetchone():
        idx += 1
        asset = f'LAD-{idx:03}'
    x, y, z = (float(v) for v in cand['position'])
    from ultron.memory import now  # noqa: PLC0415
    # Same row layout as Registry.associate(); the tracker already decided NEW vs AMBIGUOUS.
    with db:
        db.execute('INSERT INTO assets VALUES (?,?,?,?,?,?,?,?,?,?,?)',
                   (asset, cand.get('ladder_type', 'UNKNOWN'), key, x, y, z,
                    STATE_TO_REGISTRY.get(cand.get('state'), 'NEW'), now(), now(), None, 0))
    return asset


def update_asset(registry, cand):
    from ultron.memory import now  # noqa: PLC0415
    with registry.db:
        registry.db.execute('UPDATE assets SET last_seen=?,state=? WHERE id=?',
                            (now(), STATE_TO_REGISTRY.get(cand['state'], 'ACTIVE'), cand['asset_id']))

