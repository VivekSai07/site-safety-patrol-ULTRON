#!/usr/bin/env python3
"""Standalone entry point (no ROS needed) for ultron_b2_tools.bag_inspect; see that module for usage.

    python3 ros2_ws/tools/bag_inspect.py /path/to/bag_dir --out runs/bag_inspect/<name>
Inside the workspace the same tool is `ros2 run ultron_b2_tools bag_inspect`.
"""
import importlib.util
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1] / 'src' / 'ultron_b2_tools' / 'ultron_b2_tools' / 'bag_inspect.py'
_spec = importlib.util.spec_from_file_location('ultron_b2_tools_bag_inspect', _SRC)
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
globals().update({k: v for k, v in vars(_module).items() if not k.startswith('__')})

if __name__ == '__main__':
    sys.exit(_module.main())
