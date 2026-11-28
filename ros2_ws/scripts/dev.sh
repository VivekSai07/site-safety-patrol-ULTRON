#!/usr/bin/env bash
# Run a command in the ULTRON B2 dev container (ultron-b2-dev:humble) with this repo mounted
# at the same absolute path. Never uses ROS_DOMAIN_ID 0 and always sets ROS_LOCALHOST_ONLY=1.
#
#   ros2_ws/scripts/dev.sh bash -lc "colcon build --symlink-install"
#   ULTRON_MODE=replay ros2_ws/scripts/dev.sh bash -lc "ros2 launch ultron_b2_bringup replay.launch.py"
#   ros2_ws/scripts/dev.sh            # interactive shell
#
# Env knobs:
#   ULTRON_MODE      sim (domain 71, default) | replay (domain 72)
#   ULTRON_DEV_NET   bridge (default: private network namespace, so concurrent agents/containers
#                    cannot see each other's DDS traffic) | host (shares host loopback; only with
#                    ROS_LOCALHOST_ONLY=1, which is always set) | none
#   ULTRON_DEV_IMAGE image tag (default ultron-b2-dev:humble)
#   ULTRON_DEV_ARGS  extra `docker run` arguments
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
image="${ULTRON_DEV_IMAGE:-ultron-b2-dev:humble}"
mode="${ULTRON_MODE:-sim}"
net="${ULTRON_DEV_NET:-bridge}"

case "$mode" in
  sim) domain=71 ;;
  replay) domain=72 ;;
  *) echo "dev.sh: ULTRON_MODE must be sim or replay (live is never run from dev.sh)" >&2; exit 64 ;;
esac
case "$net" in
  bridge|host|none) ;;
  *) echo "dev.sh: ULTRON_DEV_NET must be bridge, host or none" >&2; exit 64 ;;
esac
if [[ "$domain" == "0" ]]; then echo "dev.sh: refusing ROS_DOMAIN_ID 0" >&2; exit 2; fi

docker image inspect "$image" >/dev/null 2>&1 || {
  echo "dev.sh: image $image missing; build it with:" >&2
  echo "  docker build -f $repo/ros2_ws/docker/Dockerfile.dev -t $image $repo/ros2_ws/docker" >&2
  exit 1; }

home_dir="$repo/ros2_ws/.dev_home"
mkdir -p "$home_dir"
mounts=(-v "$repo:$repo")
# Git worktrees keep their object store elsewhere; mount it read-only so `git rev-parse` works.
common="$(git -C "$repo" rev-parse --path-format=absolute --git-common-dir 2>/dev/null || true)"
if [[ -n "$common" && "$common" != "$repo/.git" && -d "$common" ]]; then mounts+=(-v "$common:$common:ro"); fi

tty=()
if [[ -t 0 && -t 1 ]]; then tty=(-it); fi
[[ $# -eq 0 ]] && set -- bash -l

# shellcheck disable=SC2086
exec docker run --rm "${tty[@]}" --init \
  --name "ultron-b2-dev-$(id -u)-$$" \
  --network "$net" --ipc private \
  --user "$(id -u):$(id -g)" \
  -e HOME="$home_dir" -e ROS_HOME="$home_dir/.ros" -e ROS_LOG_DIR="$repo/ros2_ws/log/ros" \
  -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI="file://$repo/ros2_ws/src/ultron_b2_bringup/config/dds/cyclonedds_localhost.xml" \
  -e ULTRON_MODE="$mode" -e PYTHONDONTWRITEBYTECODE=1 \
  "${mounts[@]}" -w "$repo/ros2_ws" ${ULTRON_DEV_ARGS:-} \
  "$image" "$@"
