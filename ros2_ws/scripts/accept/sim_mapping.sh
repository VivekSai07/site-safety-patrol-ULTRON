#!/usr/bin/env bash
# Integration acceptance: full sim stack mapping run (SIMULATION ONLY; evaluation ground truth, not hardware evidence).
# Run INSIDE the dev container from ros2_ws with ULTRON_MODE=sim (domain 71):
#   ULTRON_MODE=sim scripts/dev.sh bash -lc 'scripts/accept/sim_mapping.sh runs/integration/mapping_mola [mola|kiss|sim] [seed]'
# Starts scripts/ultron_launch.sh sim mode:=mapping with the scripted hall survey route (open-loop kinematic
# driver; EXTRA_ARGS appends launch args), records map->base_link + state/odom, saves the map through map/save, then computes ATE (sim_ate.py).
set -u
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
OUT=$(realpath -m "${1:?output dir}")
SRC=${2:-mola}
SEED=${3:-7}
NS=/ultron/sim
WS=$(cd "$(dirname "$0")/../.." && pwd)
REPO=$(dirname "$WS")
TOOLS=$WS/src/ultron_b2_localization/tools
[ "${ROS_DOMAIN_ID:-}" = 71 ] || { echo "refusing: ROS_DOMAIN_ID must be 71 (sim)"; exit 2; }
[ -e "$OUT" ] && { echo "refusing: $OUT exists"; exit 2; }
mkdir -p "$OUT"
export ULTRON_MAP_ROOT=${ULTRON_MAP_ROOT:-$OUT/maps}
MAP_ID=${MAP_ID:-hall-$SRC-s$SEED}
pids=()
cleanup() {
  for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  for _ in $(seq 30); do
    alive=0; for p in "${pids[@]}"; do kill -0 -- "-$p" 2>/dev/null && alive=1; done
    [ $alive = 0 ] && break; sleep 0.5
  done
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
}
trap cleanup EXIT
# The robot must not move before the estimator runs (a moving start offsets the whole odom frame), so the
# survey route is wrapped in a scenario that first waits START_WAIT_S of sim time.
python3 - "$REPO/config/autonomy/routes/hall_survey_loop.yaml" "$OUT/survey_scenario.yaml" "${START_WAIT_S:-15}" <<'PY'
import sys, yaml
route = yaml.safe_load(open(sys.argv[1]))
yaml.safe_dump({'schema': 'ultron.sim_scenario/v1', 'scene': route['scene'], 'start_pose': route['start_pose'],
                'steps': [{'wait_s': float(sys.argv[3])},
                          {'route': route['waypoints'], 'final_yaw': route['final_yaw']}]}, open(sys.argv[2], 'w'))
PY
CMD=(scripts/ultron_launch.sh sim mode:=mapping odom_source:="$SRC"
     sim_scenario:="$OUT/survey_scenario.yaml" sim_out:="$OUT/sim" sim_seed:="$SEED" ${EXTRA_ARGS:-})
echo "${CMD[*]}" > "$OUT/launch_cmd.txt"
setsid "${CMD[@]}" > "$OUT/launch.log" 2>&1 &
pids+=($!)
setsid python3 "$TOOLS/replay_eval.py" --out "$OUT" --namespace $NS > "$OUT/recorder.log" 2>&1 &
REC=$!
pids+=($REC)
sleep 20
for t in $NS/sensors/scan /tf $NS/sim/cmd_vel $NS/cmd_vel; do
  echo "== $t"; ros2 topic info "$t" -v
done > "$OUT/graph_publishers.txt" 2>&1
ros2 node list > "$OUT/nodes.txt" 2>&1
# The scripted route ends the bridge process (route done + 3 s hold).
for _ in $(seq "${ROUTE_TIMEOUT_S:-1500}"); do
  grep -q 'sim_contract_bridge-[0-9]*\]: process has' "$OUT/launch.log" && break
  sleep 1
done
grep 'sim_contract_bridge-[0-9]*\]: process has' "$OUT/launch.log" > "$OUT/bridge_exit.txt"
sleep 3
timeout 90 ros2 service call $NS/map/save ultron_b2_interfaces/srv/SaveMap \
  "{schema_version: ultron.save_map/v1, map_id: '$MAP_ID', save_3d: false}" > "$OUT/save.log" 2>&1
timeout 10 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable \
  $NS/state/localization > "$OUT/localization_state.txt" 2>&1
kill -INT -- "-$REC"
for _ in $(seq 20); do kill -0 "$REC" 2>/dev/null || break; sleep 0.5; done
python3 "$TOOLS/sim_ate.py" "$OUT" > "$OUT/ate_stdout.txt" 2>&1
echo "ate exit $?" >> "$OUT/ate_stdout.txt"
cleanup
trap - EXIT
grep -c 'TF_AUTHORITY_VIOLATION' "$OUT/launch.log" > "$OUT/tf_violations.txt"
find "$ULTRON_MAP_ROOT" -name 'map_meta.json' > "$OUT/saved_maps.txt"
echo "done: $OUT"
