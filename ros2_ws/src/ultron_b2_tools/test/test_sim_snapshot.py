import hashlib
import json
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest

from ultron_b2_tools.sim_snapshot import FRAME, main, rgb_array, save_snapshot


def image():
    return SimpleNamespace(
        encoding='rgb8', width=2, height=2, step=8,
        header=SimpleNamespace(frame_id=FRAME, stamp=SimpleNamespace(sec=10, nanosec=12)),
        data=bytes([1, 2, 3, 4, 5, 6, 0, 0, 7, 8, 9, 10, 11, 12, 0, 0]),
    )


def test_snapshot_saves_pixels_and_provenance_without_overwriting(tmp_path):
    dest = tmp_path / 'capture'
    metadata = save_snapshot(image(), dest)
    assert np.asarray(Image.open(dest / 'color.png')).tolist() == [
        [[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]]
    assert json.loads((dest / 'metadata.json').read_text()) == metadata
    assert metadata['sha256'] == hashlib.sha256((dest / 'color.png').read_bytes()).hexdigest()
    assert metadata['settled'] == 'NOT_ASSESSED'
    with pytest.raises(FileExistsError):
        save_snapshot(image(), dest)


@pytest.mark.parametrize('field,value', [
    ('encoding', 'bgr8'), ('step', 5), ('width', 0), ('data', b'bad'),
])
def test_snapshot_rejects_malformed_image(field, value):
    msg = image()
    setattr(msg, field, value)
    with pytest.raises(ValueError):
        rgb_array(msg)


def test_snapshot_rejects_wrong_frame():
    msg = image()
    msg.header.frame_id = 'other'
    with pytest.raises(ValueError):
        rgb_array(msg)


def test_snapshot_refuses_non_simulation_domain(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    assert main(['--output', str(tmp_path / 'capture')]) == 2
    assert not (tmp_path / 'capture').exists()
