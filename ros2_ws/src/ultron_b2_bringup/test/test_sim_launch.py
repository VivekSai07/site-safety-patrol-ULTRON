"""Graph-level launch checks: sim/replay stubs start under their namespace; refusals start nothing."""
import time

from _graph import node_list, ros_env, start, stop

EXPECTED_SIM = {'robot_state_publisher', 'tf_authority_check', 'drivers_placeholder', 'preprocess_placeholder',
                'state_estimation_placeholder', 'slam_placeholder', 'nav_placeholder', 'control_placeholder',
                'mission_placeholder', 'viz_placeholder', 'record_placeholder'}
CONTROL = {'nav_placeholder', 'control_placeholder', 'mission_placeholder'}


def _launch(run_mode, domain, *args, wait_s=6.0, expect=None):
    env = ros_env(domain)
    proc = start(['ros2', 'launch', 'ultron_b2_bringup', f'{run_mode}.launch.py', *args], env)
    time.sleep(wait_s)
    until = None
    if expect is not None:
        until = lambda nodes: proc.poll() is not None or expect <= _stack_nodes(nodes, f'/ultron/{run_mode}')
    nodes = node_list(env, until=until) if proc.poll() is None else []
    return proc, nodes, env


def _stack_nodes(nodes, namespace):
    return {n[len(namespace) + 1:] for n in nodes if n.startswith(namespace + '/')}


def test_sim_mapping_starts_all_stubs_under_namespace():
    proc, nodes, _ = _launch('sim', 71, 'mode:=mapping', expect=EXPECTED_SIM)
    alive = proc.poll() is None
    out = stop(proc)
    assert alive, out
    assert _stack_nodes(nodes, '/ultron/sim') == EXPECTED_SIM, (nodes, out)
    stray = [n for n in nodes if not n.startswith('/ultron/sim/') and not n.startswith('/launch_ros_')
             and not n.startswith('/_ros2cli')]
    assert not stray, stray
    assert '[preflight] PASS mode=sim' in out
    assert 'Traceback' not in out and 'TF_AUTHORITY_VIOLATION' not in out, out
    assert '[ERROR]' not in out, out


def test_replay_starts_no_control_nodes():
    proc, nodes, _ = _launch('replay', 72, 'mode:=localization', expect=EXPECTED_SIM - CONTROL)
    alive = proc.poll() is None
    out = stop(proc)
    assert alive, out
    started = _stack_nodes(nodes, '/ultron/replay')
    assert started == EXPECTED_SIM - CONTROL, (nodes, out)
    assert 'Traceback' not in out and '[ERROR]' not in out, out


def test_sim_on_wrong_domain_is_refused_and_starts_nothing():
    proc, nodes, _ = _launch('sim', 74, 'mode:=mapping', wait_s=6.0)
    out = stop(proc)
    assert 'REFUSED' in out and 'ROS_DOMAIN_ID=74' in out, out
    assert not [n for n in nodes if n.startswith('/ultron/')], nodes


def test_sim_wrong_namespace_or_hardware_enable_refused():
    for extra in (['namespace:=/ultron/live'], ['hardware_enable:=true'], ['odom_source:=b2']):
        proc, nodes, _ = _launch('sim', 71, *extra, wait_s=6.0)
        out = stop(proc)
        assert 'REFUSED' in out, (extra, out)
        assert not [n for n in nodes if n.startswith('/ultron/')], (extra, nodes)
