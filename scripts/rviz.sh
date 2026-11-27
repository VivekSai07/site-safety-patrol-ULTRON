#!/usr/bin/env bash
set -eo pipefail
ultron_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -lt 1 ]]; then
  echo "Usage: scripts/rviz.sh RUN_DIRECTORY [--once]" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ULTRON_SIM_DOMAIN:-71}"
export ROS_LOCALHOST_ONLY=1
/usr/bin/python3 "$ultron_root/scripts/rviz_replay.py" "$@" &
ultron_replay_pid=$!
trap 'kill "$ultron_replay_pid" 2>/dev/null || true' EXIT
rviz2 -d "$ultron_root/config/ultron.rviz"
