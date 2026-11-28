#!/usr/bin/env bash
# Keep the desktop's working DISPLAY/XAUTHORITY; bound software-rendering threads.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export LP_NUM_THREADS=2
export ULTRON_VIEWER_FPS="${ULTRON_VIEWER_FPS:-5}"
export MUJOCO_GL=glfw
export PYTHONPATH="$(pwd)/src${PYTHONPATH:+:$PYTHONPATH}"
if [[ -z "${DISPLAY:-}" ]]; then
    printf 'DISPLAY is unset. Run this from the terminal where xclock works.\n' >&2
    exit 2
fi
if command -v xdpyinfo >/dev/null 2>&1; then
    if ! timeout 3 xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
        printf 'Cannot connect to DISPLAY=%s. Run this in the terminal where xclock works.\n' "$DISPLAY" >&2
        exit 2
    fi
fi
printf 'ULTRON: DISPLAY=%s, viewer=%s FPS, software raster threads=2\nKinematic patrol and oracle detections. Esc or Ctrl+C stops the run.\n' "$DISPLAY" "$ULTRON_VIEWER_FPS"
exec nice -n 10 python3 -m ultron demo --viewer --patrols 1 "$@"
