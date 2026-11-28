#!/usr/bin/env bash
# Bag-replay evaluation of the localization stack (dev tool; run INSIDE the dev container, domain 72):
#   ULTRON_MODE=replay ULTRON_DEV_ARGS="-v <bag>:/bag:ro" ros2_ws/scripts/dev.sh \
#     bash -lc 'src/ultron_b2_localization/tools/eval_bag.sh <mola|kiss> <out_dir> [mapping|localization] [map_ref]'
# Plays only /scan_cloud (remapped to <ns>/sensors/lidar/points_filtered; bag /tf is NOT played: its
# world->imu_link is an IMU orientation frame, BAG_FACTS.md). The scan comes from the bringup's
# ultron_b2_preprocess cloud_to_scan (RELIABLE; the only sensors/scan publisher). The former eval-only
# tools/cloud_to_scan.py was removed at integration; pointcloud_to_laserscan (best effort) dropped 21-99 %.
# Mapping runs end with map/save (save_3d for MOLA). Outputs (CSV, summary.json, logs) go to <out_dir>.
set -u
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
SRC=${1:?odom source mola|kiss}
OUT=$(realpath -m "${2:?output dir}")
MODE=${3:-mapping}
MAP_REF=${4:-}
BAG=${BAG:-/bag}
RATE=${RATE:-1.0}
NS=/ultron/replay
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
export ULTRON_MAP_ROOT=${ULTRON_MAP_ROOT:-$(dirname "$OUT")/maps}
[ "${ROS_DOMAIN_ID:-}" = 72 ] || { echo "refusing: ROS_DOMAIN_ID must be 72 (replay)"; exit 2; }

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

extra=()
[ -n "$MAP_REF" ] && extra+=("map:=$MAP_REF")
setsid ros2 launch ultron_b2_bringup replay.launch.py mode:="$MODE" odom_source:="$SRC" "${extra[@]}" \
  > "$OUT/launch.log" 2>&1 &
pids+=($!)
setsid python3 "$HERE/replay_eval.py" --out "$OUT" --namespace $NS > "$OUT/recorder.log" 2>&1 &
REC=$!
pids+=($REC)
sleep "${STARTUP_S:-10}"
ros2 bag play "$BAG" --clock 100 --rate "$RATE" --read-ahead-queue-size 200 --topics /scan_cloud \
  --remap /scan_cloud:=$NS/sensors/lidar/points_filtered > "$OUT/play.log" 2>&1
sleep 4
if [ "$MODE" = mapping ]; then
  save3d=false; [ "$SRC" = mola ] && save3d=true
  timeout 60 ros2 service call $NS/map/save ultron_b2_interfaces/srv/SaveMap \
    "{schema_version: ultron.save_map/v1, map_id: '${MAP_ID:-bag-$SRC}', save_3d: $save3d}" > "$OUT/save.log" 2>&1
  echo "save exit $?" >> "$OUT/save.log"
fi
kill -INT -- "-$REC"
for _ in $(seq 20); do kill -0 "$REC" 2>/dev/null || break; sleep 0.5; done
echo "done: $OUT"
