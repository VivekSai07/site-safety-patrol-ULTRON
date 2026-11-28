"""Replay graph audit catches command and API names at any namespace depth."""
import importlib.util
from pathlib import Path


spec = importlib.util.spec_from_file_location('replay_audit', Path(__file__).parents[1] / 'tools/replay_audit.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_forbidden_replay_endpoints():
    for name in ('/cmd_vel', '/ultron/replay/sim/cmd_vel', '/robot/cmd_vel_stamped',
                 '/api/request', '/ultron/replay/control/mode', '/api'):
        assert audit.forbidden(name)
    for name in ('/ultron/replay/sensors/scan', '/ultron/replay/state/odom',
                 '/diagnostics', '/rapid', '/robot/controller_status'):
        assert not audit.forbidden(name)


def test_diagnostic_level_supports_ros_byte_and_integer():
    assert audit.diagnostic_level(b'\x00') == 0
    assert audit.diagnostic_level(b'\x02') == 2
    assert audit.diagnostic_level(1) == 1
