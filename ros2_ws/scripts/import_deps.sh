#!/usr/bin/env bash
# Import pinned third-party sources (deps.repos) into ros2_ws/external/ (gitignored).
# external/build/*  -> built by colcon with the workspace
# external/ref/*    -> pinned references only; COLCON_IGNORE added (binary preferred or owned later)
set -euo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws"
mkdir -p external
vcs import --input deps.repos --skip-existing external
mkdir -p external/ref
touch external/ref/COLCON_IGNORE
# unitree_ros2: only the message packages are built (examples need unitree_sdk2 / hardware).
for d in external/build/unitree_ros2/example external/build/unitree_ros2/cyclonedds_ws/src/cyclonedds \
         external/build/unitree_ros2/cyclonedds_ws/src/rmw_cyclonedds; do
  [[ -d "$d" ]] && touch "$d/COLCON_IGNORE"
done
find external -name package.xml -not -path "external/ref/*" | sort
