import math
import struct
from types import SimpleNamespace
import unittest

try:
    import numpy as np
except ImportError:
    np = None

if np is not None:
    from ultron.autonomy.real_bag import BINS, cloud_xyz, fit_floor, flatten


@unittest.skipIf(np is None, 'NumPy is required for LiDAR replay')
class RealBagProjectionTests(unittest.TestCase):
    def test_xyz_decodes_named_fields_and_skips_padded_bytes_and_nonfinite_points(self):
        fields = [SimpleNamespace(name=axis, offset=offset, datatype=7, count=1)
                  for axis, offset in zip('xyz', (8, 4, 0))]
        data = (struct.pack('>fff', 3, 2, 1) + b'x' * 4
                + struct.pack('>fff', 0, math.nan, 0) + b'x' * 4)
        msg = SimpleNamespace(fields=fields, data=data, width=1, height=2,
                              is_bigendian=True, row_step=16, point_step=12)
        np.testing.assert_array_equal(cloud_xyz(msg), [[1, 2, 3]])
        msg.row_step = 11
        with self.assertRaisesRegex(ValueError, 'strides'):
            cloud_xyz(msg)

    def test_floor_fit_levels_tilted_points(self):
        x, y = np.meshgrid(np.linspace(-1, 1, 70), np.linspace(-1, 1, 70))
        floor = np.column_stack((x.ravel(), y.ravel(), -0.7 + 0.04 * x.ravel()))
        normal, height, rotation, _ = fit_floor(floor)
        np.testing.assert_allclose(rotation @ normal, [0, 0, 1], atol=1e-10)
        np.testing.assert_allclose((floor @ rotation.T)[:, 2] + height, 0, atol=1e-10)

    def test_projection_wraps_seam_and_keeps_nearest_in_band(self):
        points = np.array([[-3, 1e-6, 0], [-2, -1e-6, 0], [-1, 0, -0.6], [4, 0, 0]])
        ranges = flatten(points, height=0.7)
        self.assertEqual(len(ranges), BINS)
        self.assertEqual(np.count_nonzero(np.isfinite(ranges)), 2)
        self.assertEqual(ranges[0], 2.0)
        self.assertEqual(ranges[BINS // 2], 4.0)
