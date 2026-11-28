"""Versioned map store: versions never overwritten, hashes detect tampering, unknown stays unknown."""
from types import SimpleNamespace

import numpy as np
import pytest

from ultron_b2_localization import map_store
from ultron_b2_localization.map_manager_node import MapManager, grid_to_pgm, parse_map_ref, write_grid


def _grid(width=6, height=4):
    data = np.full((height, width), -1, dtype=np.int16)
    data[0, :] = 0           # bottom row free
    data[1, 2] = 100         # one occupied cell
    data[2, 3] = 50          # in between -> unknown in trinary
    info = SimpleNamespace(width=width, height=height, resolution=0.05,
                           origin=SimpleNamespace(position=SimpleNamespace(x=-1.0, y=-2.0, z=0.0),
                                                  orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    return info, data.ravel().tolist()


def _save(root, map_id='hall-test'):
    version, directory = map_store.allocate_version(root, map_id)
    info, data = _grid()
    write_grid(directory, info, data)
    (directory / 'map.posegraph').write_bytes(b'graph')
    (directory / 'map.data').write_bytes(b'data')
    map_store.write_meta(directory, map_id, version, source='test', provenance='UNIT_TEST')
    return version, directory


def test_versions_increment_and_are_never_reused(tmp_path):
    v1, d1 = _save(tmp_path)
    v2, d2 = _save(tmp_path)
    assert (v1, v2) == (1, 2) and d1 != d2
    assert map_store.latest_version(tmp_path, 'hall-test') == 2
    assert map_store.resolve(tmp_path, 'hall-test') == d2
    assert map_store.resolve(tmp_path, 'hall-test', 1) == d1
    with pytest.raises(FileExistsError):
        (d1.parent / 'v1').mkdir()
    with pytest.raises(map_store.MapStoreError):
        map_store.resolve(tmp_path, 'hall-test', 7)
    with pytest.raises(map_store.MapStoreError):
        map_store.check_map_id('../escape')


def test_hashes_detect_tampering_and_missing_files(tmp_path):
    _, d = _save(tmp_path)
    assert map_store.verify(d) == []
    meta = map_store.read_meta(d)
    assert set(meta['files_sha256']) == {'map.pgm', 'map.yaml', 'map.posegraph', 'map.data'}
    assert meta['ground_truth_used_by_estimator'] is False
    (d / 'map.data').write_bytes(b'changed')
    (d / 'map.posegraph').unlink()
    assert sorted(map_store.verify(d)) == ['hash mismatch map.data', 'missing map.posegraph']


def test_trinary_grid_keeps_unknown_unknown(tmp_path):
    info, data = _grid()
    img = grid_to_pgm(info.width, info.height, data)
    assert img[-1, 0] == 254 and img[-2, 2] == 0 and img[-3, 3] == 205 and img[0, 0] == 205
    yaml_path = write_grid(tmp_path, info, data)
    grid = map_store.load_map_yaml(yaml_path)
    assert grid['free_thresh'] == map_store.SAFE_FREE_THRESH
    assert not grid['unknown_reads_as_free']
    assert grid['occupied_cells'] == 1 and grid['free_cells'] == 6
    assert grid['unknown_cells'] == 24 - 7
    np.testing.assert_allclose(grid['occupied_xy'], [[-1.0 + 2.5 * 0.05, -2.0 + 1.5 * 0.05]])


def test_fix_free_threshold_rewrites_map_saver_default(tmp_path):
    y = tmp_path / 'm.yaml'
    y.write_text('image: m.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    assert map_store.fix_free_threshold(y) is True
    assert 'free_thresh: 0.196' in y.read_text()
    assert map_store.fix_free_threshold(y) is False


def test_info_dict_and_map_refs(tmp_path):
    _, d = _save(tmp_path)
    info = map_store.info_dict(d)
    assert info['map_id'] == 'hall-test' and info['map_version'] == 1
    assert info['posegraph_path'] == str(d / 'map') and info['map3d_path'] == ''
    assert (info['width'], info['height'], info['resolution']) == (6, 4, 0.05)
    assert info['image_sha256'] == map_store.sha256(d / 'map.pgm')
    assert parse_map_ref('hall-test:v2') == ('hall-test', 2, '')
    assert parse_map_ref('hall-test') == ('hall-test', 0, '')
    assert parse_map_ref(str(d / 'map.yaml')) == ('', 0, str(d / 'map.yaml'))
    assert map_store.resolve(tmp_path, yaml_path=str(d / 'map.yaml')) == d
    assert map_store.resolve(tmp_path, yaml_path=str(d)) == d


def test_load_requires_posegraph_and_backend_before_announcement(tmp_path):
    _, directory = _save(tmp_path)
    manager = MapManager.__new__(MapManager)
    manager.root = tmp_path
    manager.initial_pose = [0.0, 0.0, 0.0]
    class Request:
        LOCALIZE_AT_POSE = 3

        def __init__(self):
            self.initial_pose = SimpleNamespace(x=0.0, y=0.0, theta=0.0)

    manager.Deserialize = SimpleNamespace(Request=Request)
    manager.deserialize = object()
    published = []
    manager.info_pub = SimpleNamespace(publish=published.append)
    manager._info_msg = lambda d: d
    manager._call = lambda client, req: (None, 'service unavailable')

    ok, message, _ = manager._announce('hall-test', 1, '', deserialize=True)
    assert not ok and 'service unavailable' in message and not published

    manager._call = lambda client, req: (object(), '')
    ok, _, _ = manager._announce('hall-test', 1, '', deserialize=True)
    assert ok and len(published) == 1

    (directory / 'map.posegraph').unlink()
    (directory / 'map.data').unlink()
    map_store.write_meta(directory, 'hall-test', 1, source='test', provenance='UNIT_TEST')
    ok, message, _ = manager._announce('hall-test', 1, '', deserialize=True)
    assert not ok and 'no slam_toolbox pose graph' in message and len(published) == 1


def test_load_rejects_unknown_schema():
    manager = MapManager.__new__(MapManager)
    manager.LoadMap = SimpleNamespace(Request=SimpleNamespace(SCHEMA_VERSION='ultron.load_map/v1'))
    manager.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda _: None, error=lambda _: None))
    manager._announce = lambda *_args, **_kwargs: pytest.fail('must not announce an incompatible request')
    response = manager._load(SimpleNamespace(schema_version='ultron.load_map/v2'), SimpleNamespace())
    assert not response.success and 'schema' in response.message
