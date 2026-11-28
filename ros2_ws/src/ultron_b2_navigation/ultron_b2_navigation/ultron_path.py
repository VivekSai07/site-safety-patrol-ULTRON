"""Make the repository's ``ultron`` package (``<repo>/src/ultron``) importable from ROS nodes.

Order: already importable -> ``$ULTRON_REPO_SRC`` -> walk up from this file (symlink-install
resolves to the source tree inside the repo). Raises ImportError if none works.
"""
import os
from pathlib import Path
import sys


def ensure_ultron():
    try:
        import ultron  # noqa: F401
        return
    except ImportError:
        pass
    candidates = []
    if os.environ.get('ULTRON_REPO_SRC'):
        candidates.append(Path(os.environ['ULTRON_REPO_SRC']))
    for parent in Path(__file__).resolve().parents:
        candidates.append(parent / 'src')
    for c in candidates:
        if (c / 'ultron' / '__init__.py').is_file():
            sys.path.insert(0, str(c))
            import ultron  # noqa: F401
            return
    raise ImportError('ultron package not found; set ULTRON_REPO_SRC=<repo>/src')
