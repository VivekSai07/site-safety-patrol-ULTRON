#!/usr/bin/env bash
# One launch point for the dashboard, in one of two modes. Ctrl+C stops everything it started.
#
#   scripts/dashboard.sh demo   two simulated patrols on site "demo-site", replayed into a throwaway
#                               database (builds runs/dashboard-example first if missing, ~25 s).
#                               No robot link, no robot control.
#   scripts/dashboard.sh live   the working dashboard for a real or simulated robot: the patrol history
#                               in runs/dashboard.sqlite, robot control on (exploration, goals) and the
#                               ROS bridge (map, pose, Nav2). Needs ROS 2 with nav2_msgs sourced.
#                               The bridge reaches whatever robot is on this ROS network.
#   scripts/dashboard.sh live --sim
#                               live, fed by the MuJoCo patrol instead of a robot: the simulated B2
#                               walks its route (viewer window when a display is available) and is
#                               streamed in as site "sim-hall". Oracle detections, kinematic motion.
#
# Extra options go to `python -m ultron.dashboard`, e.g.: scripts/dashboard.sh live --port 8010 --lan
# The pipeline then posts to the printed URL: python -m ultron.pipeline patrol ... --dashboard URL
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
export PYTHONPATH="$(pwd)/src${PYTHONPATH:+:$PYTHONPATH}"

usage() { sed -n 2,17p "$0" | sed 's/^# \{0,1\}//'; }
mode=${1:-}
case $mode in
    demo|live) shift ;;
    -h|--help|'') usage; exit 0 ;;
    *) echo "Unknown mode '$mode'." >&2; usage >&2; exit 2 ;;
esac

PY=.venv/bin/python
[[ -x $PY ]] || { echo "No .venv here. Create it first (see README)." >&2; exit 2; }
$PY -c 'import fastapi, uvicorn' 2>/dev/null || { echo ".venv lacks FastAPI: .venv/bin/pip install -e '.[dashboard]'" >&2; exit 2; }

port=8000 sim=0 pass=()
while (($#)); do
    case $1 in
        --sim) [[ $mode == live ]] || { echo "--sim belongs to live mode." >&2; exit 2; }; sim=1 ;;
        --port) port=${2:?--port needs a number}; pass+=(--port "$2"); shift ;;
        --goals|--db|--media) echo "$1 is set by the mode; leave it out." >&2; exit 2 ;;
        *) pass+=("$1") ;;
    esac
    shift
done
url="http://127.0.0.1:$port"

DEMO_RUN=runs/dashboard-example
tmp=''
if [[ $mode == demo ]]; then
    if [[ ! -e $DEMO_RUN/manifest.json ]]; then
        [[ -e $DEMO_RUN ]] && { echo "$DEMO_RUN exists but is incomplete; move it away first." >&2; exit 2; }
        echo "Building the simulated demo run in $DEMO_RUN (about 25 s) ..."
        MUJOCO_GL=egl $PY -m ultron demo --out "$DEMO_RUN" --patrols 2 >/dev/null
    fi
    tmp=$(mktemp -d -t ultron-demo-XXXX)  # demo patrols never enter the real history
    pass+=(--db "$tmp/demo.sqlite" --media "$tmp/media")
else
    python3 -c 'import rclpy, nav2_msgs' 2>/dev/null \
        || { echo "live needs ROS 2 with nav2_msgs sourced in this shell (system python3)." >&2; exit 2; }
    pass+=(--goals)
fi

pids=()
cleanup() {
    trap - INT TERM EXIT
    if ((${#pids[@]})); then
        kill "${pids[@]}" 2>/dev/null || true
        for _ in $(seq 25); do kill -0 "${pids[@]}" 2>/dev/null || break; sleep 0.2; done
        kill -KILL "${pids[@]}" 2>/dev/null || true  # still running after 5 s
        wait 2>/dev/null || true
    fi
    if [[ -n $tmp ]]; then rm -rf -- "$tmp"; fi
    echo "Stopped."
}
trap cleanup INT TERM EXIT

$PY -m ultron.dashboard "${pass[@]}" &
pids+=($!)

for _ in $(seq 50); do  # up to 10 s for the server to answer
    curl -fs "$url/api/sites" >/dev/null 2>&1 && break
    kill -0 "${pids[0]}" 2>/dev/null || { echo "Dashboard exited during start-up (see above)." >&2; exit 1; }
    sleep 0.2
done
curl -fs "$url/api/sites" >/dev/null 2>&1 || { echo "Dashboard did not answer at $url" >&2; exit 1; }

if [[ $mode == demo ]]; then
    $PY -m ultron.dashboard.replay "$DEMO_RUN" --url "$url" &
    pids+=($!)
else
    python3 scripts/dashboard_bridge.py --ros-args -p dashboard_url:="$url" &
    pids+=($!)
fi
if ((sim)); then
    stamp=$(date +%Y%m%d-%H%M%S)
    sim_run=runs/sim-live-$stamp
    # the tailer polls for the run's files, so it can start first; its tag keeps patrol ids unique
    $PY -m ultron.dashboard.live_tail "$sim_run" --url "$url" --site sim-hall --tag="-$stamp" --idle-timeout 600 &  # "=": the value starts with a dash
    pids+=($!)
    if [[ -n ${DISPLAY:-} ]]; then  # viewer window, as scripts/launch-demo.sh runs it
        OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 LP_NUM_THREADS=2 MUJOCO_GL=glfw \
            nice -n 10 $PY -m ultron demo --out "$sim_run" --patrols 1 --viewer &
    else
        MUJOCO_GL=egl $PY -m ultron demo --out "$sim_run" --patrols 1 &
    fi
    pids+=($!)
fi

echo
echo "[$mode] Dashboard: $url   (live, /history, /ladder)"
if [[ $mode == demo ]]; then
    echo "Two simulated patrols arrive over the next few minutes (site demo-site); discarded on exit."
else
    echo "ROS bridge running; robot control on."
    ((sim)) && echo "MuJoCo patrol -> $sim_run, streamed in as site sim-hall (oracle detections, kinematic motion)."
    echo "Pipeline: python -m ultron.pipeline patrol ... --dashboard $url"
fi
echo "Ctrl+C stops all."
wait "${pids[0]}"
