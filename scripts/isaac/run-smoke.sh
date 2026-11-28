#!/usr/bin/env bash
# Run the Isaac Sim smoke test with Isaac's bundled ROS 2 Humble libraries (Python 3.12) on the
# isolated ULTRON simulation domain. Does not source system ROS (Python 3.10).
set -euo pipefail
isaac="${ISAAC_SIM:-$HOME/isaac-sim}"
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=71 ROS_LOCALHOST_ONLY=1
export LD_LIBRARY_PATH="$isaac/exts/isaacsim.ros2.core/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$isaac/python.sh" "$repo_root/scripts/isaac/rtx_lidar_smoke.py" "$@"
