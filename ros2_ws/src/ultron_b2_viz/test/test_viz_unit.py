import io
import os
from types import SimpleNamespace

import numpy as np
import yaml
from PIL import Image

from ultron_b2_viz import gen_configs
from ultron_b2_viz import whitelist as wl
from ultron_b2_viz.viz_throttle import cloud_to_array, jpeg_encode, voxel_downsample

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _field(name, offset, datatype=7):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=1)


def test_cloud_to_array_drops_nan_and_handles_padding():
    pts = np.array([[1, 2, 3, 9, 0], [np.nan, 0, 0, 1, 0], [4, 5, 6, 7, 0]], '<f4')
    msg = SimpleNamespace(fields=[_field('x', 0), _field('y', 4), _field('z', 8), _field('intensity', 12)],
                          width=3, height=1, point_step=20, is_bigendian=False, data=pts.tobytes())
    out = cloud_to_array(msg)
    assert out.shape == (2, 4)
    np.testing.assert_allclose(out[1], [4, 5, 6, 7])


def test_voxel_downsample_keeps_one_point_per_voxel_and_caps():
    pts = np.array([[0.01, 0, 0, 0], [0.02, 0, 0, 0], [0.5, 0, 0, 0], [-0.01, 0, 0, 0]], np.float32)
    out = voxel_downsample(pts, 0.1, 0)
    np.testing.assert_allclose(out[:, 0], [0.01, 0.5, -0.01])
    dense = np.random.default_rng(0).uniform(-5, 5, (5000, 4)).astype(np.float32)
    assert len(voxel_downsample(dense, 0.0, 100)) == 100


def test_jpeg_encode_bgr_downscales_and_keeps_colour():
    bgr = np.zeros((48, 64, 3), np.uint8)
    bgr[..., 0] = 255  # blue in bgr8
    jpg = jpeg_encode(bgr.tobytes(), 64, 48, 'bgr8', 80, 32)
    img = np.asarray(Image.open(io.BytesIO(jpg)).convert('RGB'))
    assert img.shape == (24, 32, 3)
    assert img[..., 2].mean() > 200 and img[..., 0].mean() < 50


def test_whitelists_exclude_raw_sensor_and_ground_truth():
    for mode, ns in wl.MODE_NAMESPACES.items():
        for rel in ('sensors/lidar/points', 'sensors/lidar/points_filtered', 'sensors/cam_head/color/image_raw',
                    'sensors/cam_head/depth/image_rect_raw', 'sim/ground_truth/base_pose',
                    'sim/oracle/cam_head/ladder_labels', 'cmd_vel'):
            assert not wl.foxglove_allows(f'{ns}/{rel}'), rel
        for rel in ('viz/points', 'viz/cam_head/color/image/compressed', 'state/odom', 'sensors/scan'):
            assert wl.foxglove_allows(f'{ns}/{rel}'), rel
        for src, _typ, dst in wl.bridge_topics(mode):
            assert not wl.is_denied(src)
            assert dst in ('/tf', '/tf_static') or dst.startswith(wl.VIZ_NAMESPACE + '/')
    assert not wl.foxglove_allows('/ultron/other/viz/points')
    assert wl.viz42_name('viz/points') == '/ultron/viz/points'


def test_domain_bridge_config_targets_domain_42_only():
    for mode, dom in wl.MODE_DOMAINS.items():
        cfg = yaml.safe_load(gen_configs.domain_bridge_yaml(mode))
        assert cfg['from_domain'] == dom and cfg['to_domain'] == wl.VIZ_DOMAIN
        assert len(cfg['topics']) == len(wl.bridge_topics(mode))


def test_generated_files_up_to_date():
    assert gen_configs.main(['--check', '--root', PKG]) == 0
