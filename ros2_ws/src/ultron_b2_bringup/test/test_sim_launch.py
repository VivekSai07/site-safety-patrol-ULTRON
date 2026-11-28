"""Graph-level launch checks: sim/replay stubs start under their namespace; refusals start nothing."""
import time

from _graph import node_list, ros_env, start, stop

# Localization (Agent C): sim odometry comes from the simulator, so state_estimation starts no node; mapping
# starts slam_toolbox. Replay localization without map:= starts MOLA-LO (+ planar relay) plus monitor/map_manager.
SLAM_COMMON = {'localization_monitor', 'map_manager'}
EXPECTED_SIM = {'robot_state_publisher', 'tf_authority_check', 'drivers_placeholder', 'preprocess_placeholder',
                'slam_toolbox', 'nav_placeholder', 'control_placeholder',
                'mission_placeholder', 'viz_placeholder', 'record_placeholder'} | SLAM_COMMON
CONTROL = {'nav_placeholder', 'control_placeholder', 'mission_placeholder'}
EXPECTED_REPLAY_LOCALIZATION = (EXPECTED_SIM - CONTROL - {'slam_toolbox'}) | {'mola_lidar_odometry', 'mola_odom_planarizer'}


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
    # tf2_ros (C++, e.g. slam_toolbox) spawns hidden transform_listener_impl_<hash> helper nodes.
    return {n[len(namespace) + 1:] for n in nodes
            if n.startswith(namespace + '/') and not n[len(namespace) + 1:].startswith('transform_listener_impl_')}


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
    proc, nodes, _ = _launch('replay', 72, 'mode:=localization', expect=EXPECTED_REPLAY_LOCALIZATION)
    alive = proc.poll() is None
    out = stop(proc)
    assert alive, out
    started = _stack_nodes(nodes, '/ultron/replay')
    assert started == EXPECTED_REPLAY_LOCALIZATION, (nodes, out)
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
