"""Versioned on-disk map store (no ROS dependency).

Layout: ``<root>/<map_id>/v<N>/`` holding ``map.yaml`` + ``map.pgm`` (2D occupancy grid from
slam_toolbox's map saver), ``map.posegraph`` + ``map.data`` (slam_toolbox serialized graph),
optional ``map3d.*`` (MOLA metric map) and ``map_meta.json`` (``ultron.map_meta/v1``) with
SHA-256 of every file. A saved version is never overwritten; saving again creates v<N+1>.

``map_saver`` writes ``free_thresh: 0.25``, which makes nav2_map_server read unknown cells
(pixel 205 -> occupancy 0.196) as FREE. ``fix_free_threshold`` rewrites it to 0.196 (finding
from docs/SAHIL_VALIDATION.md Phase 3) and the change is recorded in map_meta.json.
"""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import uuid

import numpy as np

META_SCHEMA = 'ultron.map_meta/v1'
META_FILE = 'map_meta.json'
UNKNOWN_PIXEL = 205
SAFE_FREE_THRESH = 0.196
_VERSION = re.compile(r'^v(\d+)$')
_MAP_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')


class MapStoreError(ValueError):
    pass


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def new_map_id(prefix='ultron-map'):
    return f'{prefix}-{uuid.uuid4().hex[:12]}'


def check_map_id(map_id):
    if not _MAP_ID.match(map_id or ''):
        raise MapStoreError(f'invalid map_id {map_id!r} (letters, digits, "_.-"; max 64)')
    return map_id


def versions(root, map_id):
    d = Path(root) / check_map_id(map_id)
    if not d.is_dir():
        return []
    return sorted(int(m.group(1)) for p in d.iterdir() if p.is_dir() and (m := _VERSION.match(p.name)))


def latest_version(root, map_id):
    v = [n for n in versions(root, map_id) if (version_dir(root, map_id, n) / META_FILE).is_file()]
    return v[-1] if v else None


def version_dir(root, map_id, version):
    return Path(root) / check_map_id(map_id) / f'v{int(version)}'


def allocate_version(root, map_id):
    """Create and return (version, directory) for the next unused version (never reuses one)."""
    existing = versions(root, map_id)
    version = (existing[-1] + 1) if existing else 1
    d = version_dir(root, map_id, version)
    d.mkdir(parents=True, exist_ok=False)
    return version, d


def fix_free_threshold(yaml_path):
    """Lower free_thresh so map_saver's unknown grey (205) stays unknown. Returns True if changed."""
    lines = Path(yaml_path).read_text().splitlines()
    changed = False
    for i, line in enumerate(lines):
        if line.startswith('free_thresh:') and float(line.split(':', 1)[1]) > SAFE_FREE_THRESH:
            lines[i] = f'free_thresh: {SAFE_FREE_THRESH}'
            changed = True
    Path(yaml_path).write_text('\n'.join(lines) + '\n')
    return changed


def read_pgm(path):
    """Binary (P5) 8-bit PGM -> uint8 array (rows top to bottom)."""
    data = Path(path).read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b'P5' or int(tokens[3]) > 255:
        raise MapStoreError(f'{path}: only 8-bit binary PGM (P5) is supported')
    width, height = int(tokens[1]), int(tokens[2])
    pixels = np.frombuffer(data[pos + 1:pos + 1 + width * height], dtype=np.uint8)
    if pixels.size != width * height:
        raise MapStoreError(f'{path}: truncated image')
    return pixels.reshape(height, width)


def load_map_yaml(yaml_path):
    """Trinary occupancy map as map_server would read it, plus occupied cell centres (map frame)."""
    import yaml
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    image = read_pgm(yaml_path.parent / meta['image'])
    if meta.get('negate', 0):
        image = 255 - image
    occupancy = (255 - image.astype(float)) / 255.0
    occupied = occupancy > float(meta['occupied_thresh'])
    free = occupancy < float(meta['free_thresh'])
    res = float(meta['resolution'])
    ox, oy, oyaw = (list(meta['origin']) + [0.0, 0.0, 0.0])[:3]
    if abs(oyaw) > 1e-9:
        raise MapStoreError('rotated map origins are not supported')
    height, width = image.shape
    rows, cols = np.nonzero(occupied)
    xy = np.column_stack((ox + (cols + 0.5) * res, oy + (height - 1 - rows + 0.5) * res))
    return {'occupied_xy': xy, 'resolution': res, 'width': width, 'height': height,
            'origin': [float(ox), float(oy), float(oyaw)], 'free_thresh': float(meta['free_thresh']),
            'occupied_thresh': float(meta['occupied_thresh']), 'image': meta['image'],
            'occupied_cells': int(occupied.sum()), 'free_cells': int(free.sum()),
            'unknown_cells': int((~occupied & ~free).sum()),
            'unknown_reads_as_free': bool((255 - UNKNOWN_PIXEL) / 255.0 < float(meta['free_thresh']))}


def write_meta(directory, map_id, version, *, source, provenance, extra=None):
    """Hash every file in ``directory`` and write map_meta.json. Returns the meta dict."""
    directory = Path(directory)
    files = {p.name: sha256(p) for p in sorted(directory.iterdir()) if p.is_file() and p.name != META_FILE}
    meta = {'schema': META_SCHEMA, 'map_id': check_map_id(map_id), 'map_version': int(version),
            'frame_id': 'map', 'units': 'metres, radians (REP-103)',
            'created_at': datetime.now(timezone.utc).isoformat(timespec='seconds'),
            'source': source, 'provenance': provenance, 'ground_truth_used_by_estimator': False,
            'files_sha256': files}
    meta.update(extra or {})
    (directory / META_FILE).write_text(json.dumps(meta, indent=2, sort_keys=True))
    return meta


def read_meta(directory):
    path = Path(directory) / META_FILE
    meta = json.loads(path.read_text())
    if meta.get('schema') != META_SCHEMA:
        raise MapStoreError(f'{path}: schema {meta.get("schema")!r} != {META_SCHEMA}')
    return meta


def verify(directory):
    """Re-hash files listed in map_meta.json. Returns list of problems (empty = intact)."""
    directory = Path(directory)
    meta = read_meta(directory)
    problems = []
    for name, digest in meta['files_sha256'].items():
        p = directory / name
        if not p.is_file():
            problems.append(f'missing {name}')
        elif sha256(p) != digest:
            problems.append(f'hash mismatch {name}')
    return problems


def resolve(root, map_id='', version=0, yaml_path=''):
    """Map directory for a load request: explicit yaml, map_id+version, map_id latest, or newest map."""
    if yaml_path:
        path = Path(yaml_path).resolve()
        return path if path.is_dir() else path.parent
    root = Path(root)
    if map_id:
        v = int(version) if version else latest_version(root, map_id)
        if v is None or not (version_dir(root, map_id, v) / META_FILE).is_file():
            raise MapStoreError(f'no saved map {map_id} v{version or "latest"} under {root}')
        return version_dir(root, map_id, v)
    candidates = sorted(root.glob(f'*/v*/{META_FILE}'), key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise MapStoreError(f'no saved maps under {root}')
    return candidates[-1].parent


def info_dict(directory):
    """Fields of ultron_b2_interfaces/MapInfo for a saved map directory."""
    directory = Path(directory)
    meta = read_meta(directory)
    files = meta['files_sha256']
    yaml_path = directory / 'map.yaml'
    grid = load_map_yaml(yaml_path) if yaml_path.is_file() else None
    map3d = next((n for n in sorted(files) if n.startswith('map3d')), '')
    return {
        'map_id': meta['map_id'], 'map_version': int(meta['map_version']),
        'yaml_path': str(yaml_path) if grid else '',
        'image_sha256': files.get(grid['image'], '') if grid else '',
        'posegraph_path': str(directory / 'map') if 'map.posegraph' in files else '',
        'map3d_path': str(directory / map3d) if map3d else '',
        'map3d_sha256': files.get(map3d, ''),
        'resolution': grid['resolution'] if grid else 0.0,
        'width': grid['width'] if grid else 0, 'height': grid['height'] if grid else 0,
        'origin': grid['origin'] if grid else [0.0, 0.0, 0.0],
        'free_thresh': grid['free_thresh'] if grid else 0.0,
        'occupied_thresh': grid['occupied_thresh'] if grid else 0.0,
        'created_at': meta.get('created_at', ''), 'source': meta.get('source', ''),
    }


def yaw_to_quaternion_z_w(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)
