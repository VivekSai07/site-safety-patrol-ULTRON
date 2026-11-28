"""Locate the team repository's ``src/ultron`` package (memory.py, supervisor.py) for ROS nodes.

Order: ``$ULTRON_SRC`` -> walk up from this file's real path (symlink-install / source tree).
Returns None when not found (e.g. a deployment image without the repository); callers degrade
gracefully and say so.
"""
import os
import sys
from pathlib import Path


def find_ultron_src():
    env = os.environ.get('ULTRON_SRC')
    candidates = [Path(env)] if env else []
    here = Path(os.path.realpath(__file__))
    candidates += [parent / 'src' for parent in here.parents]
    for c in candidates:
        if (c / 'ultron' / 'memory.py').is_file():
            return c
    return None


def ensure_ultron_importable():
    src = find_ultron_src()
    if src is None:
        return False
    if str(src) not in sys.path:
        sys.path.append(str(src))
    return True
