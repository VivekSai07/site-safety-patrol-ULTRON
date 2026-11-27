#!/usr/bin/env bash
# Organizer simulator only; never use its standing script on physical hardware.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
kit_root="${ULTRON_ORGANIZER_KIT:-$(dirname -- "$repo_root")/ctrl-hackathon-2026-kit}"
expected=8f5de5806fcd432d339ca1dbb61a1f0f9ea1d6cd
actual="$(git -C "$kit_root" rev-parse HEAD)"
action="${1:-inspect}"
if [[ "$action" == inspect ]]; then
    printf 'Organizer kit: %s\nCommit: %s\nReviewed commit: %s\nSimulator: B2, loopback lo, DDS domain 1\n' "$kit_root" "$actual" "$expected"
    exit 0
fi
if [[ "$actual" != "$expected" ]]; then
    printf 'Kit revision changed; review it before using this wrapper.\n' >&2
    exit 2
fi
cd -- "$kit_root/unitree-mujoco"
export ROBOT=b2
case "$action" in
    config) exec docker compose config --quiet ;;
    start) exec docker compose up --build -d ;;
    stop) exec docker compose stop ;;
    shell) exec docker compose exec ros bash ;;
    watch) exec docker compose exec ros python3 /root/examples/watch_lowstate.py b2 ;;
    *) printf 'Usage: bash scripts/organizer-sim.sh [inspect|config|start|stop|shell|watch]\n' >&2; exit 2 ;;
esac
