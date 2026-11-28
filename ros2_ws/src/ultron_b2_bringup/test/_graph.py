"""Helpers for graph-level tests: isolated localhost domains, subprocess launch control."""
import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def ros_env(domain):
    env = dict(os.environ)
    env.update({'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1', 'PYTHONUNBUFFERED': '1',
                'RCUTILS_LOGGING_BUFFERED_STREAM': '0'})
    # Cyclone's default unicast participant-index limit (9) is too small for the full stub stack
    # once multicast is off on loopback; use the stack's localhost profile.
    env['CYCLONEDDS_URI'] = 'file://' + os.path.join(
        get_package_share_directory('ultron_b2_bringup'), 'config', 'dds', 'cyclonedds_localhost.xml')
    assert env['ROS_DOMAIN_ID'] != '0'
    return env


def executable(name, package='ultron_b2_bringup'):
    return os.path.join(get_package_prefix(package), 'lib', package, name)


def start(cmd, env):
    return subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)


def stop(proc, timeout=20):
    # SIGINT only the launch process: it forwards SIGINT to its children itself. Signalling the whole
    # group would deliver a second SIGINT to every child mid-cleanup.
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    return proc.stdout.read()


def node_list(env, settle_s=0.0, until=None, timeout_s=30.0):
    """List nodes; with `until(nodes) -> bool`, poll until it holds or `timeout_s` elapses."""
    time.sleep(settle_s)
    deadline = time.monotonic() + timeout_s
    while True:
        out = subprocess.run(['ros2', 'node', 'list', '--no-daemon', '--spin-time', '3'], env=env,
                             capture_output=True, text=True, timeout=60)
        nodes = sorted(set(out.stdout.split()))
        if until is None or until(nodes) or time.monotonic() > deadline:
            return nodes
