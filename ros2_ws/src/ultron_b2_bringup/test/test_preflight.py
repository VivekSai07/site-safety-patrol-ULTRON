"""Preflight isolation rules (FLEET_COMMON mode table). No ROS graph needed."""
from datetime import datetime, timedelta, timezone

import pytest

from ultron_b2_bringup.modes import MODES, sub_launches_for
from ultron_b2_bringup.preflight import check, check_hw_token, main

SIM_ENV = {'ROS_DOMAIN_ID': '71', 'ROS_LOCALHOST_ONLY': '1'}
REPLAY_ENV = {'ROS_DOMAIN_ID': '72', 'ROS_LOCALHOST_ONLY': '1'}
LIVE_ENV = {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '0'}
NOW = datetime(2026, 11, 28, 12, 0, tzinfo=timezone.utc)


def test_correct_environments_pass():
    assert check('sim', SIM_ENV, '/ultron/sim', odom_source='sim', launch_control=True) == []
    assert check('replay', REPLAY_ENV, '/ultron/replay', odom_source='mola', launch_control=False) == []
    assert check('live', LIVE_ENV, '/ultron/live', odom_source='b2', launch_control=True) == []


@pytest.mark.parametrize('domain', [None, '', '0', '42', '72'])
def test_sim_wrong_domain_refused(domain):
    env = {'ROS_LOCALHOST_ONLY': '1'} | ({} if domain is None else {'ROS_DOMAIN_ID': domain})
    errors = check('sim', env, '/ultron/sim')
    assert any('ROS_DOMAIN_ID' in e for e in errors)


def test_replay_wrong_domain_and_no_localhost_refused():
    assert any('requires 72' in e for e in check('replay', SIM_ENV, '/ultron/replay'))
    assert any('ROS_LOCALHOST_ONLY' in e for e in check('replay', {'ROS_DOMAIN_ID': '72'}, '/ultron/replay'))


def test_live_with_localhost_only_refused():
    assert any('robot network' in e for e in check('live', {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'},
                                                   '/ultron/live'))


@pytest.mark.parametrize('mode,ns', [('sim', '/ultron/replay'), ('sim', '/ultron/live'), ('sim', ''),
                                     ('replay', '/ultron/sim'), ('live', '/ultron/sim')])
def test_wrong_namespace_refused(mode, ns):
    env = {'sim': SIM_ENV, 'replay': REPLAY_ENV, 'live': LIVE_ENV}[mode]
    assert any('namespace' in e for e in check(mode, env, ns))


def test_robot_dds_profile_refused_in_localhost_modes():
    env = SIM_ENV | {'CYCLONEDDS_URI': 'file:///x/cyclonedds_robot.xml'}
    assert any('CYCLONEDDS_URI' in e for e in check('sim', env, '/ultron/sim'))


def test_hardware_enable_refused_in_sim_and_replay():
    assert any('hardware_enable' in e for e in check('sim', SIM_ENV, '/ultron/sim', hardware_enable=True))
    assert any('hardware_enable' in e for e in check('replay', REPLAY_ENV, '/ultron/replay', hardware_enable=True))


def test_live_hardware_enable_without_token_refused(tmp_path):
    missing = tmp_path / 'absent'
    errors = check('live', LIVE_ENV, '/ultron/live', hardware_enable=True, hw_token=str(missing), now=NOW)
    assert any('does not exist' in e for e in errors)
    errors = check('live', LIVE_ENV | {'ULTRON_HW_AUTHORIZED_FILE': str(missing)}, '/ultron/live',
                   hardware_enable=True, now=NOW)
    assert errors


@pytest.mark.parametrize('content,why', [
    ('', 'missing'),
    ('operator: Om\nplan: HW-PLAN-1\n', 'missing'),
    ('operator: Om\nplan: HW-PLAN-1\nexpires: "2026-11-28T11:00:00+00:00"\n', 'expired'),
    ('operator: Om\nplan: HW-PLAN-1\nexpires: "2026-11-28T13:00:00"\n', 'timezone'),
    ('[not, a, mapping]', 'mapping'),
])
def test_invalid_tokens_refused(tmp_path, content, why):
    token = tmp_path / 'ULTRON_HW_AUTHORIZED'
    token.write_text(content)
    ok, reason = check_hw_token(token, now=NOW)
    assert not ok and why in reason


def test_valid_token_only_unlocks_live(tmp_path):
    # Test fixture in a pytest tmp dir only; agents never create a real token.
    token = tmp_path / 'ULTRON_HW_AUTHORIZED'
    token.write_text('operator: Test\nplan: TEST-ONLY\nexpires: "2026-11-28T13:00:00+00:00"\n')
    assert check('live', LIVE_ENV, '/ultron/live', hardware_enable=True, hw_token=str(token), now=NOW) == []
    assert check('sim', SIM_ENV, '/ultron/sim', hardware_enable=True, hw_token=str(token), now=NOW)


def test_odom_source_restricted_per_mode():
    assert check('replay', REPLAY_ENV, '/ultron/replay', odom_source='b2')    # no sportmodestate in bag
    assert check('replay', REPLAY_ENV, '/ultron/replay', odom_source='sim')
    assert check('live', LIVE_ENV, '/ultron/live', odom_source='sim')


def test_replay_never_launches_control():
    assert check('replay', REPLAY_ENV, '/ultron/replay', launch_control=True)
    included = sub_launches_for('replay')
    assert not {'control', 'nav', 'mission'} & set(included)
    assert {'control', 'nav', 'mission'} <= set(sub_launches_for('sim'))
    assert MODES['replay']['control'] is False


def test_never_domain_zero_outside_live():
    for mode, spec in MODES.items():
        assert (spec['domain_id'] == 0) == (mode == 'live')


def test_cli_exit_codes(monkeypatch, capsys):
    monkeypatch.setenv('ROS_DOMAIN_ID', '71')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.delenv('CYCLONEDDS_URI', raising=False)
    assert main(['--mode', 'sim', '--namespace', '/ultron/sim']) == 0
    assert main(['--mode', 'replay', '--namespace', '/ultron/replay']) == 2
    assert main(['--mode', 'sim', '--namespace', '/ultron/sim', '--ros-args', '-r', '__ns:=/x']) == 0
    assert main(['--mode', 'nope', '--namespace', '/x']) == 64
    assert 'REFUSED' in capsys.readouterr().err
