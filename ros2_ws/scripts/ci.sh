#!/usr/bin/env bash
# ULTRON B2 ROS 2 CI: deps check, colcon build, colcon test, repo unittest suite.
# Run inside the dev image:  ros2_ws/scripts/dev.sh scripts/ci.sh
# Never uses ROS_DOMAIN_ID 0; tests use localhost-only domains 71-74.
set -euo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
repo="$(cd "$ws/.." && pwd)"
cd "$ws"

[[ "${ROS_DOMAIN_ID:-}" == "0" ]] && { echo "ci.sh: refusing ROS_DOMAIN_ID=0" >&2; exit 2; }
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}" ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$ws/src/ultron_b2_bringup/config/dds/cyclonedds_localhost.xml"
set +u; source /opt/ros/humble/setup.bash; set -u

echo "== [1/5] import pinned sources (deps.repos)"
scripts/import_deps.sh

echo "== [2/5] rosdep"
if [[ "$(id -u)" == "0" ]]; then
  rosdep install --from-paths src external/build --ignore-src -y -r --rosdistro humble
else
  # Non-root (dev.sh runs as the host user): the image must already satisfy every key.
  rosdep check --from-paths src external/build --ignore-src --rosdistro humble
fi

echo "== [3/5] colcon build"
colcon build --symlink-install --event-handlers console_cohesion- console_stderr+

echo "== [4/5] colcon test (ultron_b2_* incl. graph/launch tests)"
set +u; source install/setup.bash; set -u
colcon test --packages-select-regex '^ultron_b2_' --event-handlers console_direct-
colcon test-result --verbose

echo "== [5/5] repo unittest suite"
(cd "$repo" && PYTHONPATH=src python3 -m unittest discover -s tests)
echo "ci.sh: PASS"
