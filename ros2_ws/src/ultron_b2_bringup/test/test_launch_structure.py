"""Static checks of the launch skeleton (no processes started)."""
from pathlib import Path
import importlib.util

import pytest

from ultron_b2_bringup.launch_common import SUB_ARGS, build_stack, declare_arguments
from ultron_b2_bringup.modes import CONTROL_SUB_LAUNCHES, SUB_LAUNCHES

LAUNCH = Path(__file__).resolve().parents[1] / 'launch'


def _load(path):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


@pytest.mark.parametrize('run_mode', ['sim', 'replay', 'live'])
def test_top_level_launch_files_load_and_declare_args(run_mode):
    ld = _load(LAUNCH / f'{run_mode}.launch.py')
    declared = {a.name for a in ld.get_launch_arguments()}
    assert {'mode', 'odom_source', 'viz', 'record', 'hardware_enable', 'bag', 'namespace'} <= declared
    hw = next(a for a in declare_arguments(run_mode) if a.name == 'hardware_enable')
    assert hw.default_value[0].text == 'false'


def test_choices_match_brief():
    args = {a.name: a for a in declare_arguments('sim')}
    assert args['mode'].choices == ['mapping', 'localization', 'nav', 'mission']
    assert args['odom_source'].choices == ['sim', 'mola', 'kiss', 'b2']
    assert args['viz'].choices == ['none', 'foxglove', 'rviz_bridge', 'both']
    assert args['record'].choices == ['none', 'minimal', 'debug', 'full']


def test_every_sub_launch_exists_and_loads():
    for name, _owner in SUB_LAUNCHES:
        ld = _load(LAUNCH / 'sub' / f'{name}.launch.py')
        declared = {a.name for a in ld.get_launch_arguments()}
        assert set(SUB_ARGS) <= declared, name


@pytest.mark.parametrize('run_mode', ['sim', 'replay', 'live'])
def test_slam_sub_launch_forwards_run_mode(run_mode):
    from launch import LaunchContext

    ld = _load(LAUNCH / 'sub' / 'slam.launch.py')
    include = ld.entities[-1]
    context = LaunchContext()
    context.launch_configurations['run_mode'] = run_mode
    assert dict(include.launch_arguments)['run_mode'].perform(context) == run_mode


def test_replay_stack_excludes_command_sub_launches():
    def includes(run_mode):
        return {getattr(a, 'ultron_sub_launch', None) for a in build_stack(run_mode).get_sub_entities()} - {None}
    assert includes('sim') == {name for name, _ in SUB_LAUNCHES}
    assert includes('replay') == {name for name, _ in SUB_LAUNCHES} - set(CONTROL_SUB_LAUNCHES)
    assert includes('live') == includes('sim')
