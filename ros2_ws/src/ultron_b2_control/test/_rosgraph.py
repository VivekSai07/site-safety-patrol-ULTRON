"""Shared helpers for graph tests: isolated localhost domain (never 0), subprocess control, probe node."""
import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def use_domain(domain):
    assert int(domain) != 0
    os.environ.update({'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1', 'PYTHONUNBUFFERED': '1',
                       'RCUTILS_LOGGING_BUFFERED_STREAM': '0'})
    os.environ['CYCLONEDDS_URI'] = 'file://' + os.path.join(
        get_package_share_directory('ultron_b2_bringup'), 'config', 'dds', 'cyclonedds_localhost.xml')
    return dict(os.environ)


def executable(package, name):
    """Direct path (not `ros2 run`, which does not forward SIGINT to its child in a new session)."""
    return os.path.join(get_package_prefix(package), 'lib', package, name)


def start(cmd, env=None):
    return subprocess.Popen(cmd, env=env or dict(os.environ), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)


def stop(proc, timeout=20):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    return proc.stdout.read()


def spin_for(executor, seconds, step=None):
    """Spin; call step(now) every loop (publishers decide their own rate)."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.005)
        if step:
            step(time.monotonic())


def spin_until(executor, pred, timeout, step=None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.005)
        if step:
            step(time.monotonic())
        if pred():
            return True
    return False
