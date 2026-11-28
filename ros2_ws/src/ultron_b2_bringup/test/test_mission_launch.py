"""mission sub-launch: placeholder outside mode:=mission; perception + mission nodes in mode:=mission."""
from pathlib import Path
import importlib.util

import pytest
from launch import LaunchContext
from launch.actions import OpaqueFunction

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'sub' / 'mission.launch.py'


def _entities(mode, run_mode):
    spec = importlib.util.spec_from_file_location('mission_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()
    ctx = LaunchContext()
    for arg in ld.get_launch_arguments():
        ctx.launch_configurations[arg.name] = ''.join(s.perform(ctx) for s in arg.default_value)
    ctx.launch_configurations.update(mode=mode, run_mode=run_mode)
    opaque = [e for e in ld.entities if isinstance(e, OpaqueFunction)]
    assert len(opaque) == 1
    return opaque[0].execute(ctx)


def _flat(node):
    """Parameter files and dict values of a launch Node, performed to plain text."""
    ctx = LaunchContext()
    out = []
    for p in node._Node__parameters:
        if isinstance(p, dict):
            for k, v in p.items():
                out += [''.join(s.perform(ctx) for s in k), ''.join(s.perform(ctx) for s in v)]
        else:
            out.append(str(p.param_file[0].perform(ctx) if isinstance(p.param_file, list) else p.param_file))
    return ' '.join(out)




@pytest.mark.parametrize('mode', ['mapping', 'localization', 'nav'])
def test_non_mission_modes_keep_placeholder(mode):
    nodes = _entities(mode, 'sim')
    assert [n.node_executable for n in nodes] == ['placeholder']


def test_sim_mission_mode_starts_perception_and_mission_with_oracle():
    pytest.importorskip('ament_index_python')
    try:
        nodes = _entities('mission', 'sim')
    except Exception as exc:  # packages not installed in this environment
        pytest.skip(f'mission/perception packages not installed: {exc}')
    exe = sorted(n.node_executable for n in nodes)
    assert exe == ['capture_coordinator', 'ladder_detector', 'ladder_localizer', 'ladder_tracker', 'mission_manager']
    det = next(n for n in nodes if n.node_executable == 'ladder_detector')
    flat = _flat(det)
    assert 'oracle_sim' in flat and 'oracle_hall_ladders.yaml' in flat


def test_live_mission_mode_never_uses_oracle():
    try:
        nodes = _entities('mission', 'live')
    except Exception as exc:
        pytest.skip(f'mission/perception packages not installed: {exc}')
    det = next(n for n in nodes if n.node_executable == 'ladder_detector')
    flat = _flat(det)
    assert 'oracle_sim' not in flat and 'oracle_hall_ladders' not in flat and 'yolo' in flat


def test_sim_mission_mode_graph_starts_mission_nodes_without_errors():
    """Graph check: perception + mission nodes come up under /ultron/sim with no tracebacks.

    C/D/F are stubs here, so the mission just idles (no map/localization => no goals)."""
    import time

    from _graph import node_list, ros_env, start, stop
    expected = {'ladder_detector', 'ladder_localizer', 'ladder_tracker', 'mission_manager', 'capture_coordinator'}
    env = ros_env(71)
    env['HOME'] = str(Path(__file__).resolve().parent / '.artifacts' / 'home')  # registry/missions scratch
    proc = start(['ros2', 'launch', 'ultron_b2_bringup', 'sim.launch.py', 'mode:=mission'], env)
    time.sleep(6.0)
    ns = '/ultron/sim/'
    nodes = node_list(env, until=lambda n: proc.poll() is not None
                      or expected <= {x[len(ns):] for x in n if x.startswith(ns)}) if proc.poll() is None else []
    alive = proc.poll() is None
    out = stop(proc)
    started = {x[len(ns):] for x in nodes if x.startswith(ns)}
    assert alive, out
    assert expected <= started and 'mission_placeholder' not in started, (nodes, out)
    assert 'Traceback' not in out and '[ERROR]' not in out, out
    assert 'ORACLE' in out, out  # the detector announces its oracle labelling in sim
