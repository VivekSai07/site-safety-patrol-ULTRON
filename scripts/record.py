#!/usr/bin/env python3
"""Record a bag on the robot: `python3 scripts/record.py --out ~/ultron-bags`. Needs only ROS sourced.

Same as `python3 -m ultron.recording`, without having to set PYTHONPATH (which, set to `src`
alone, hides ROS's own Python packages and stops `ros2` from starting).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ultron.recording import main

if __name__ == "__main__":
    raise SystemExit(main())
