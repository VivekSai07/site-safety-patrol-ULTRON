"""Filter maths on synthetic clouds: crop, self box, voxel, empty and NaN input, time parsing, deskew."""
import numpy as np
import pytest

from ultron_b2_preprocess import filter_core as fc

CFG = fc.FilterConfig(range_min=0.3, range_max=40.0, z_min=-1.5, z_max=3.0, self_box_enabled=True,
                      self_box_min=(-0.25, -0.35, -0.60), self_box_max=(0.20, 0.35, 0.05), voxel_size=0.0)


def run(xyz, cfg=CFG, t=None, **kw):
    xyz = np.asarray(xyz, float).reshape(-1, 3)
    n = len(xyz)
    t = np.zeros(n, np.float32) if t is None else np.asarray(t, np.float32)
    return fc.filter_cloud(xyz, np.arange(n, dtype=np.float32), np.arange(n, dtype=np.uint16) % 32, t, cfg, **kw)


def test_range_height_and_self_box():
    pts = [[5.0, 0, 0],        # keep
           [0.1, 0.1, -0.1],   # inside self box (and < range_min) -> range
           [0.0, 0.3, -0.4],   # r=0.5, inside self box -> self
           [0.0, 0.5, -0.4],   # just outside box in y -> keep
           [50.0, 0, 0],       # > range_max
           [3.0, 0, -2.0],     # below z_min
           [3.0, 0, 3.5]]      # above z_max
    out, s = run(pts)
    kept = np.stack([out['x'], out['y'], out['z']], 1)
    np.testing.assert_allclose(kept, [[5, 0, 0], [0, 0.5, -0.4]], atol=1e-6)
    assert (s.n_in, s.n_range, s.n_self, s.n_height, s.n_out) == (7, 2, 1, 2, 2)
    assert not fc.inside_box(kept, CFG.self_box_min, CFG.self_box_max).any()
    # attributes travel with their points
    assert list(out['intensity']) == [0.0, 3.0] and list(out['ring']) == [0, 3]


def test_self_box_disabled_keeps_body_points():
    cfg = fc.FilterConfig(**{**CFG.__dict__, 'self_box_enabled': False})
    out, s = run([[0.0, 0.3, -0.4]], cfg)
    assert s.n_out == 1 and s.n_self == 0


def test_voxel_keeps_first_point_per_cell_in_scan_order():
    cfg = fc.FilterConfig(**{**CFG.__dict__, 'voxel_size': 0.1})
    pts = [[2.01, 0.01, 0.01], [2.02, 0.02, 0.02], [2.15, 0.0, 0.0], [-2.05, 0.0, 0.0], [2.09, 0.09, 0.09]]
    out, s = run(pts, cfg)
    np.testing.assert_allclose(out['x'], [2.01, 2.15, -2.05], atol=1e-6)
    assert s.n_voxel_merged == 2 and s.n_out == 3


def test_voxel_reduces_dense_cloud_and_bounds_density():
    rng = np.random.default_rng(1)
    pts = rng.uniform([1, -2, -0.5], [3, 2, 0.5], size=(20000, 3))
    cfg = fc.FilterConfig(**{**CFG.__dict__, 'voxel_size': 0.2})
    out, s = run(pts, cfg)
    assert s.n_out <= 11 * 21 * 6 and s.n_out < len(pts) / 10
    keys = np.floor(np.stack([out['x'], out['y'], out['z']], 1).astype(float) / 0.2)
    assert len(np.unique(keys, axis=0)) == s.n_out


def test_empty_cloud():
    out, s = run(np.zeros((0, 3)))
    assert len(out) == 0 and s.n_in == 0 and s.n_out == 0
    xyz, inten, ring, t, has_time = fc.parse(b'', [], 0, 0, 0.0)
    assert xyz.shape == (0, 3) and not has_time


def test_nan_and_inf_points_dropped():
    pts = [[np.nan, 0, 0], [1.0, np.inf, 0], [2.0, 0, 0], [0, 0, -np.inf]]
    out, s = run(pts)
    assert s.n_nonfinite == 3 and s.n_out == 1 and np.isfinite(out['x']).all()


def _pack_rslidar(xyz, t_abs):
    dt = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'timestamp'],
                   'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'], 'offsets': [0, 4, 8, 12, 16, 18],
                   'itemsize': 26})
    a = np.zeros(len(xyz), dt)
    a['x'], a['y'], a['z'] = xyz.T
    a['intensity'], a['ring'], a['timestamp'] = 7.0, 3, t_abs
    fields = [('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1), ('intensity', 12, 7, 1), ('ring', 16, 4, 1),
              ('timestamp', 18, 8, 1)]
    return a.tobytes(), fields, 26


def test_parse_bag_layout_absolute_timestamp_becomes_relative():
    stamp = 1795823322.84
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    buf, fields, step = _pack_rslidar(xyz, [stamp, stamp + 0.0999])
    p, inten, ring, t, has_time = fc.parse(buf, fields, step, 2, stamp)
    assert has_time
    np.testing.assert_allclose(p, xyz)
    np.testing.assert_allclose(t, [0.0, 0.0999], atol=1e-5)
    assert list(ring) == [3, 3] and list(inten) == [7.0, 7.0]


def test_parse_relative_time_field_kept():
    dt = fc.OUT_DTYPE
    a = np.zeros(2, dt)
    a['x'], a['time'] = [1.0, 2.0], [0.0, 0.05]
    fields = [(n, o, d, 1) for n, o, d in fc.OUT_FIELDS]
    _, _, _, t, has_time = fc.parse(a.tobytes(), fields, dt.itemsize, 2, 1.0e9)
    assert has_time
    np.testing.assert_allclose(t, [0.0, 0.05])


def test_rotation_matrices_match_closed_form():
    R = fc.rotation_vectors_to_matrices(np.array([[0, 0, np.pi / 2], [0, 0, 0]]))
    np.testing.assert_allclose(R[0], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12)
    np.testing.assert_allclose(R[1], np.eye(3))


def test_deskew_recovers_static_scene_seen_by_rotating_sensor():
    # Sensor yaws at omega; a static wall point seen at time t_i in the then-current sensor frame.
    omega = np.array([0.0, 0.0, 1.2])
    rng = np.random.default_rng(0)
    world = rng.uniform([-8, -8, -1], [8, 8, 2], size=(500, 3))  # = sensor frame at t_ref
    t = np.sort(rng.uniform(0, 0.1, size=500))
    t_ref = 0.1
    R_ref_i = fc.rotation_vectors_to_matrices(np.outer(t - t_ref, omega))      # frame i -> frame ref
    seen = np.einsum('nji,nj->ni', R_ref_i, world)                               # p_i = R^T p_ref
    fixed = fc.deskew_rotation(seen, t, omega, t_ref)
    np.testing.assert_allclose(fixed, world, atol=1e-9)
    assert np.abs(seen - world).max() > 0.5  # distortion was material


def test_filter_with_deskew_zeroes_time():
    out, s = run([[5.0, 0, 0], [0, 5.0, 0]], t=[0.0, 0.1], omega=np.array([0, 0, 1.0]), t_ref=0.1)
    assert s.deskewed and np.all(out['time'] == 0.0)


def test_mean_angular_velocity_window():
    samples = [(0.0, 1, 0, 0), (0.05, 3, 0, 0), (1.0, 100, 0, 0)]
    np.testing.assert_allclose(fc.mean_angular_velocity(samples, 0.0, 0.1), [2, 0, 0])
    assert fc.mean_angular_velocity(samples, 5.0, 5.1) is None
    assert fc.mean_angular_velocity([], 0, 1) is None


@pytest.mark.parametrize('rpy', [(0.1, -0.2, 0.3), (0.0, 0.0, np.pi / 2)])
def test_rpy_matrix_is_rotation(rpy):
    R = fc.rpy_to_matrix(*rpy)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)
