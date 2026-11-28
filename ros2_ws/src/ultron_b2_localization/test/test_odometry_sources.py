"""B2 adapter (mocked SportModeState), KISS-ICP helpers and a synthetic KISS-ICP registration check."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from ultron_b2_localization.b2_odom_adapter import SportModeRejected, sportmode_to_odometry
from ultron_b2_localization.geometry import flatten_pose, matrix_to_quat, quat_to_matrix, twist_from_delta, yaw_of
from ultron_b2_localization.kiss_icp_node import cloud_to_arrays, normalize_times, range_filter
from ultron_b2_localization.odom_planarizer import planarize


def _sport(yaw=0.5, pitch=0.0, pos=(1.0, 2.0, 0.3), vel=(0.4, 0.0, 0.0), stamp=(0, 0), quat=None):
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    q = quat or [cy * cp, -sy * sp, cy * sp, sy * cp]        # w, x, y, z (yaw then pitch)
    return SimpleNamespace(position=list(pos), velocity=list(vel), yaw_speed=0.2,
                           imu_state=SimpleNamespace(quaternion=q), stamp=SimpleNamespace(sec=stamp[0], nanosec=stamp[1]))


def test_b2_adapter_planar_pose_and_body_twist():
    d = sportmode_to_odometry(_sport(yaw=0.5, pitch=0.2, vel=(math.cos(0.5), math.sin(0.5), 0.1)))
    assert d['position'] == [1.0, 2.0, 0.0]
    x, y, z, w = d['orientation_xyzw']
    assert abs(x) < 1e-12 and abs(y) < 1e-12 and yaw_of(x, y, z, w) == pytest.approx(0.5)
    np.testing.assert_allclose(d['linear_body'], [1.0, 0.0, 0.0], atol=1e-9)   # world velocity along heading
    assert d['angular_body'] == [0.0, 0.0, 0.2]
    assert d['stamp_s'] is None                                                # zero stamp -> receive time


def test_b2_adapter_3d_and_stamp():
    d = sportmode_to_odometry(_sport(yaw=0.0, pitch=0.2, stamp=(12, 500_000_000)), planar=False)
    assert d['position'][2] == 0.3 and d['stamp_s'] == pytest.approx(12.5)
    assert abs(d['orientation_xyzw'][1]) > 0.05


@pytest.mark.parametrize('bad', [dict(pos=(float('nan'), 0, 0)), dict(quat=[0.5, 0, 0, 0]),
                                 dict(vel=(0, float('inf'), 0))])
def test_b2_adapter_rejects_malformed(bad):
    with pytest.raises(SportModeRejected):
        sportmode_to_odometry(_sport(**bad))


def test_geometry_roundtrip_and_flatten():
    q = (0.1, -0.2, 0.3, 0.9)
    q = tuple(v / np.linalg.norm(q) for v in q)
    np.testing.assert_allclose(matrix_to_quat(quat_to_matrix(*q)), q, atol=1e-12)
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(*q)
    m[:3, 3] = (1, 2, 3)
    f = flatten_pose(m)
    assert f[2, 3] == 0 and f[2, 2] == 1 and math.atan2(f[1, 0], f[0, 0]) == pytest.approx(math.atan2(m[1, 0], m[0, 0]))
    a, b = np.eye(4), np.eye(4)
    b[0, 3] = 0.5
    lin, ang = twist_from_delta(a, b, 0.1)
    np.testing.assert_allclose(lin, [5.0, 0, 0]) and np.testing.assert_allclose(ang, 0)


def test_planarizer_drops_z_roll_pitch_keeps_heading():
    yaw, roll, pitch = 0.7, 0.04, -0.05
    cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2),
                              math.cos(yaw / 2), math.sin(yaw / 2))
    q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
    (x, y, z), (qx, qy, qz, qw) = planarize((1.0, -2.0, 0.11), q)
    assert (x, y, z, qx, qy) == (1.0, -2.0, 0.0, 0.0, 0.0)
    assert yaw_of(qx, qy, qz, qw) == pytest.approx(yaw, abs=1e-3)
    with pytest.raises(ValueError):
        planarize((float('nan'), 0, 0), (0, 0, 0, 1))


def _cloud(points, times, time_name='timestamp', time_type=8):
    from sensor_msgs.msg import PointCloud2, PointField
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('ring', '<u2'),
                      (time_name, '<f8' if time_type == 8 else '<f4')])
    arr = np.zeros(len(points), dtype=dtype)
    arr['x'], arr['y'], arr['z'] = points.T
    arr[time_name] = times
    fields = [PointField(name=n, offset=dtype.fields[n][1], datatype=t, count=1)
              for n, t in (('x', 7), ('y', 7), ('z', 7), ('intensity', 7), ('ring', 4), (time_name, time_type))]
    return PointCloud2(height=1, width=len(points), fields=fields, is_bigendian=False, point_step=dtype.itemsize,
                       row_step=dtype.itemsize * len(points), data=arr.tobytes(), is_dense=True)


def test_cloud_parsing_time_fields_and_filters():
    pytest.importorskip('sensor_msgs.msg')
    pts = np.array([[1.0, 0, 0], [0, 2.0, 0], [np.nan, 0, 0], [0.2, 0, 0]])
    xyz, t = cloud_to_arrays(_cloud(pts, [100.0, 100.05, 100.02, 100.1]))
    assert xyz.shape == (3, 3) and np.allclose(t, [100.0, 100.05, 100.1])
    np.testing.assert_allclose(normalize_times(t), [0, 0.5, 1.0])
    xyz2, t2 = range_filter(xyz, t, 0.5, 30.0)
    assert len(xyz2) == 2 and len(t2) == 2
    xyz3, t3 = cloud_to_arrays(_cloud(pts, [0.0, 0.05, 0.02, 0.1], time_name='time', time_type=7))
    assert len(t3) == 3
    assert normalize_times(np.array([5.0, 5.0])).size == 0


def _room(n=20000, seed=0):
    """Points on the walls/floor/pillars of a 12 x 9 x 3 m room (room frame)."""
    rng = np.random.default_rng(seed)
    faces = []
    for _ in range(n):
        k = rng.integers(0, 7)
        u, v = rng.uniform(-1, 1), rng.uniform(0, 1)
        faces.append([(6, 4.5 * u, 3 * v), (-6, 4.5 * u, 3 * v), (6 * u, 4.5, 3 * v), (6 * u, -4.5, 3 * v),
                      (6 * u, 4.5 * rng.uniform(-1, 1), 0.0), (2 + 0.2 * math.cos(7 * u), -1 + 0.2 * math.sin(7 * u), 3 * v),
                      (-3 + 0.3 * u, 2 + 0.3 * rng.choice([-1, 1]), 3 * v)][k])
    return np.array(faces)


def test_kiss_icp_tracks_synthetic_motion():
    pytest.importorskip('kiss_icp')
    from ultron_b2_localization.kiss_icp_node import make_kiss
    room = _room()
    kiss = make_kiss(0.3, 20, False, 2.0, 0.1)
    est = None
    for i in range(15):
        x, yaw = 0.1 * i, 0.03 * i
        c, s = math.cos(yaw), math.sin(yaw)
        sensor_z = 0.8
        # room point p in sensor frame at pose (x, 0, sensor_z, yaw): R^T (p - t)
        d = room - (x, 0.0, sensor_z)
        local = np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]))
        kiss.register_frame(local, np.empty(0))
        est = kiss.last_pose
    assert est[0, 3] == pytest.approx(1.4, abs=0.05)
    assert abs(est[1, 3]) < 0.05
    assert math.atan2(est[1, 0], est[0, 0]) == pytest.approx(0.42, abs=0.02)
