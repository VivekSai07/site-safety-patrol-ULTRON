"""Pure (no ROS graph) checks of the MuJoCo head camera, ladder hooks and contract topic mapping."""
import math
import threading

import numpy as np
import pytest

from ultron_b2_sim.paths import ensure_ultron_importable

ensure_ultron_importable()
mujoco = pytest.importorskip('mujoco')

from ultron.autonomy.hall_scene import HallSim  # noqa: E402
from ultron_b2_sim.camera import HeadCamera, StreamSpec  # noqa: E402
from ultron_b2_sim.contract_bridge import contract_topics  # noqa: E402
from ultron_b2_sim.scenario import LadderHooks  # noqa: E402

COLOR = StreamSpec(160, 90, 80.3626, 'cam_head_color_optical_frame')   # 0.125 x real intrinsics
DEPTH = StreamSpec(106, 60, 53.8819, 'cam_head_depth_optical_frame')
MOUNT, RPY = (0.45, 0.0, 0.13), (0.0, -0.2219, 0.0)


@pytest.fixture(scope='module')
def rig():
    sim = HallSim()
    box = {}

    def make():  # EGL contexts are per thread: build and render in one worker, as the bridge does
        try:
            box['cam'] = HeadCamera(sim, MOUNT, RPY, COLOR, DEPTH)
        except Exception as exc:  # noqa: BLE001
            box['err'] = exc
    t = threading.Thread(target=make)
    t.start()
    t.join()
    if 'err' in box:
        pytest.skip(f'MuJoCo offscreen rendering unavailable: {box["err"]}')
    yield sim, box['cam']


def _render(cam):
    cam.prepare()
    out = {}
    t = threading.Thread(target=lambda: out.setdefault('r', cam.render_prepared()))
    t.start()
    t.join()
    return out['r']


def test_intrinsics_follow_render_fov():
    k = COLOR.intrinsics()
    assert k['fx'] == k['fy'] == COLOR.fy
    assert math.isclose(math.tan(math.radians(COLOR.fovy_deg) / 2) * k['fy'], COLOR.height / 2)


def test_centre_depth_matches_ray_cast(rig):
    sim, cam = rig
    sim.place(1.6, 5.0, math.pi)
    rgb, depth, labels = _render(cam)
    assert rgb.shape == (90, 160, 3) and rgb.dtype == np.uint8 and rgb.std() > 5
    assert depth.shape == (60, 106) and depth.dtype == np.uint16
    pos, _, _ = cam.pose()
    geom = np.zeros(1, np.int32)
    hit = mujoco.mj_ray(sim.model, sim.data, pos, cam.forward(), None, 1, -1, geom)
    assert 0.3 < hit < 10.0
    centre = depth[29:31, 52:54].astype(float)
    # planar depth equals ray distance on the optical axis; 2 cm covers pixel-centre offset + rounding
    assert np.all(np.abs(centre / 1000.0 - hit) < 0.02), (centre, hit)


def test_ladder_labels_and_hide_show(rig):
    sim, cam = rig
    hooks = LadderHooks(sim)
    sim.place(1.6, 4.0, 0.0)
    _, _, labels = _render(cam)
    seen = set(np.unique(labels)) - {0}
    assert seen, 'no ladder visible from the test pose'
    lab = next(iter(sorted(seen)))
    name = next(n for n, v in cam.ladder_labels.items() if v == lab)
    v0 = hooks.version
    hooks.hide(name)
    _, _, hidden = _render(cam)
    assert not np.any(hidden == lab)
    truth = {t['name']: t for t in hooks.truth(cam.ladder_labels)}
    assert truth[name]['present'] is False and truth[name]['footprint'] == []
    hooks.show(name)
    _, _, shown = _render(cam)
    assert np.any(shown == lab)
    assert hooks.version == v0 + 2 and truth[name]['label'] == lab


def test_ladder_move_updates_footprint_and_truth():
    sim = HallSim()
    hooks = LadderHooks(sim)
    name = sorted(hooks.ladders)[0]
    before = hooks.truth()[0]
    hooks.move(name, before['x'] + 1.0, before['y'], before['yaw'])
    after = hooks.truth()[0]
    assert math.isclose(after['x'], before['x'] + 1.0, abs_tol=1e-6)
    dx = [a[0] - b[0] for a, b in zip(after['footprint'], before['footprint'])]
    assert all(math.isclose(d, 1.0, abs_tol=1e-3) for d in dx)
    with pytest.raises(ValueError):
        hooks.move('ladder_99', 0, 0, 0)
    with pytest.raises(ValueError):
        hooks.move(name, float('nan'), 0, 0)


def test_contract_topics():
    base = {'clock': '/clock', 'points': '/ultron/sim/lidar/points', 'localization_state': 'x'}
    sim = contract_topics(base, 'sim')
    assert sim['points'] == 'sensors/lidar/points' and sim['odom'] == 'state/odom'
    assert sim['cmd_vel'] == 'cmd_vel' and sim['scan'] == 'sensors/scan'
    assert sim['ground_truth'].startswith('sim/ground_truth/')
    mola = contract_topics(base, 'mola', scan_on_contract=False)
    assert mola['odom'] == 'sim/leg_odom' and mola['scan'] == 'sim/scan'
    assert all(not v.startswith('/ultron') for v in sim.values())
