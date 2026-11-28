#!/usr/bin/env bash
# Launch a bringup mode with the preflight environment check run BEFORE `ros2 launch`.
#   ros2_ws/scripts/ultron_launch.sh sim mode:=mapping
#
# Why: `ros2 launch` itself creates a DDS participant on $ROS_DOMAIN_ID before the in-launch
# preflight can refuse. This wrapper refuses a wrong domain/localhost setting without touching DDS.
# The in-launch preflight still runs (namespace, hardware flags, odom_source, token).
set -euo pipefail
run_mode="${1:?usage: ultron_launch.sh sim|replay|live [launch args...]}"; shift
case "$run_mode" in sim|replay|live) ;; *) echo "unknown run mode $run_mode" >&2; exit 64 ;; esac
"$(ros2 pkg prefix ultron_b2_bringup)/lib/ultron_b2_bringup/preflight" --mode "$run_mode" --namespace "/ultron/$run_mode"
exec ros2 launch ultron_b2_bringup "$run_mode.launch.py" "$@"
