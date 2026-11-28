"""Locate the team repository's ``src/`` (the ``ultron`` package) and ``runs/`` folder."""
import os
from pathlib import Path
import sys


def _repo_from_share():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('ultron_b2_sim'))
    except Exception:  # noqa: BLE001 - not installed (unit tests from source)
        share = None
    candidates = [Path(__file__).resolve()]
    if share is not None:
        candidates.append((share / 'package.xml').resolve())  # symlink-install points into ros2_ws/src
    for path in candidates:
        for parent in path.parents:
            if (parent / 'src' / 'ultron' / 'autonomy' / 'sim_bridge.py').is_file():
                return parent
    return None


def repo_root():
    env = os.environ.get('ULTRON_REPO')
    if env:
        return Path(env)
    return _repo_from_share()


def ensure_ultron_importable():
    """Make ``import ultron`` work: $ULTRON_SRC, an installed package, or <repo>/src."""
    src = os.environ.get('ULTRON_SRC')
    if src and src not in sys.path:
        sys.path.insert(0, src)
    try:
        import ultron  # noqa: F401
        return
    except ImportError:
        pass
    root = repo_root()
    if root is None:
        raise SystemExit('ultron_b2_sim: cannot find the ULTRON repo src/ (set ULTRON_SRC=<repo>/src)')
    sys.path.insert(0, str(root / 'src'))
    import ultron  # noqa: F401


def runs_dir():
    """Default output folder for runs/bags: $ULTRON_RUNS_DIR, else <repo>/runs (git-ignored)."""
    env = os.environ.get('ULTRON_RUNS_DIR')
    if env:
        return Path(env)
    root = repo_root()
    return (root / 'runs') if root else Path.cwd() / 'runs'
