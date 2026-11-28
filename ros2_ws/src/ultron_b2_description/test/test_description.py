"""xacro expansion, frame set and calibration provenance checks for the B2 description."""
from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
REQUIRED = {'base_link', 'base_footprint', 'lidar_link', 'imu_link', 'cam_head_link',
            'cam_head_color_optical_frame', 'cam_head_depth_optical_frame'}


def expand(profile, legs='false'):
    out = subprocess.run(['xacro', str(PKG / 'urdf' / 'b2.urdf.xacro'),
                          f'calibration:={PKG / "calibration" / (profile + ".yaml")}', f'legs:={legs}'],
                         check=True, capture_output=True, text=True).stdout
    return ET.fromstring(out)


def joints(root):
    return {j.find('child').get('link'): j for j in root.findall('joint')}


@pytest.mark.parametrize('profile', ['sim', 'real'])
def test_frames_present_and_tree_rooted_at_base_link(profile):
    root = expand(profile)
    links = {l.get('name') for l in root.findall('link')}
    assert REQUIRED <= links
    children = joints(root)
    assert 'base_link' not in children, 'base_link must be the root (odom->base_link is external)'
    for link in links - {'base_link'}:
        assert link in children, f'{link} has no parent joint'


@pytest.mark.parametrize('profile', ['sim', 'real'])
def test_mounts_match_calibration(profile):
    cal = yaml.safe_load((PKG / 'calibration' / f'{profile}.yaml').read_text())
    children = joints(expand(profile))
    for name, f in cal['frames'].items():
        j = children[name]
        assert j.get('type') == 'fixed'
        assert j.find('parent').get('link') == f['parent']
        xyz = [float(v) for v in j.find('origin').get('xyz').split()]
        rpy = [float(v) for v in j.find('origin').get('rpy').split()]
        assert xyz == pytest.approx(f['xyz']) and rpy == pytest.approx(f['rpy'])


def test_sim_lidar_mount_matches_repo_profile():
    profile = REPO / 'config' / 'autonomy' / 'lidar_helios32.yaml'
    if not profile.exists():
        pytest.skip('repo lidar profile not available')
    lidar = yaml.safe_load(profile.read_text())
    sim = yaml.safe_load((PKG / 'calibration' / 'sim.yaml').read_text())['frames']['lidar_link']
    assert sim['xyz'] == pytest.approx(lidar['mount_xyz_m']['value'])
    assert sim['rpy'] == pytest.approx(lidar['mount_rpy_rad']['value'])


def test_real_calibration_is_marked_to_verify():
    cal = yaml.safe_load((PKG / 'calibration' / 'real.yaml').read_text())
    assert cal['profile'] == 'real'
    assert {f['source'] for f in cal['frames'].values()} == {'TO_VERIFY'}
    fp = yaml.safe_load((PKG / 'config' / 'footprint.yaml').read_text())
    assert fp['sim'] == {**fp['sim'], 'type': 'circle', 'radius_m': 0.55}
    assert fp['real']['source'] == 'TO_VERIFY' and fp['real']['radius_m'] is None


def test_optical_frames_follow_rep103():
    children = joints(expand('sim'))
    for name in ('cam_head_color_optical_frame', 'cam_head_depth_optical_frame'):
        rpy = [float(v) for v in children[name].find('origin').get('rpy').split()]
        assert rpy == pytest.approx([-math.pi / 2, 0.0, -math.pi / 2])


def test_legs_option_adds_revolute_chain():
    root = expand('sim', legs='true')
    revolute = [j for j in root.findall('joint') if j.get('type') == 'revolute']
    assert len(revolute) == 12
    assert {l.get('name') for l in root.findall('link')} >= {'FL_foot', 'RR_foot'}


def test_licence_and_provenance_retained():
    assert 'BSD 3-Clause' in (PKG / 'license' / 'UNITREE_B2_LICENSE').read_text()
    assert '88e1a789b67e90adbb120237b03a11cb54b1b210' in (PKG / 'license' / 'PROVENANCE.md').read_text()
