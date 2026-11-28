#!/usr/bin/env bash
# Simulation ATE evaluation (dev tool; run INSIDE the dev container, ULTRON_MODE=sim -> domain 71):
#   ros2_ws/scripts/dev.sh bash -lc 'src/ultron_b2_localization/tools/eval_sim.sh <sim|mola|kiss> <out_dir> [seed]'
# Scripted hall survey loop from the validated sim bridge; estimator outputs vs logged ground truth.
set -u
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
SRC=${1:?odom source sim|mola|kiss}
OUT=$(realpath -m "${2:?output dir}")
SEED=${3:-7}
NS=/ultron/sim
HERE=$(cd "$(dirname "$0")" && pwd)
[ "${ROS_DOMAIN_ID:-}" = 71 ] || { echo "refusing: ROS_DOMAIN_ID must be 71 (sim)"; exit 2; }
mkdir -p "$OUT"
export ULTRON_MAP_ROOT=${ULTRON_MAP_ROOT:-$(dirname "$OUT")/maps}
pids=()
cleanup() {
  for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  for _ in $(seq 20); do
    alive=0; for p in "${pids[@]}"; do kill -0 -- "-$p" 2>/dev/null && alive=1; done
    [ $alive = 0 ] && break; sleep 0.5
  done
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
}
trap cleanup EXIT
setsid ros2 launch "$HERE/eval_sim.launch.py" odom_source:="$SRC" > "$OUT/launch.log" 2>&1 &
pids+=($!)
setsid python3 "$HERE/replay_eval.py" --out "$OUT" --namespace $NS > "$OUT/recorder.log" 2>&1 &
REC=$!
pids+=($REC)
sleep "${STARTUP_S:-8}"
OPENBLAS_NUM_THREADS=1 python3 "$HERE/eval_sim_bridge.py" --odom-source "$SRC" --out "$OUT/sim" --seed "$SEED" \
  > "$OUT/bridge.log" 2>&1
echo "bridge exit $?" >> "$OUT/bridge.log"
sleep 3
timeout 60 ros2 service call $NS/map/save ultron_b2_interfaces/srv/SaveMap \
  "{schema_version: ultron.save_map/v1, map_id: 'sim-$SRC-s$SEED', save_3d: false}" > "$OUT/save.log" 2>&1
kill -INT -- "-$REC"
for _ in $(seq 20); do kill -0 "$REC" 2>/dev/null || break; sleep 0.5; done
python3 "$HERE/sim_ate.py" "$OUT"
