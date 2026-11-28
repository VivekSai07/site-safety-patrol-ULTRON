import os
import shutil

import pytest


@pytest.fixture
def tmp_path_local(request):
    """Scratch directory next to the tests (not the system temp dir); removed afterwards."""
    import pathlib
    path = pathlib.Path(os.path.dirname(os.path.abspath(__file__))) / '.artifacts' / request.node.name
    shutil.rmtree(path, ignore_errors=True)
    path.mkdir(parents=True)
    yield path
    shutil.rmtree(path, ignore_errors=True)
    try:
        path.parent.rmdir()
    except OSError:
        pass
