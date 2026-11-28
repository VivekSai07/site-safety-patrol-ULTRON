#!/usr/bin/env bash
# Run inside an isolated replay container, from ros2_ws. No raw bag data is written to OUT.
# scripts/accept/replay_mapping.sh OUT /absolute/read-only/bag [mola|kiss]
# EXTRA_ARGS can set replay_rate, replay_delay, and replay_cameras. Output is local/private.
set -o pipefail
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
OUT=$(realpath -m "${1:?output directory required}")
BAG=$(realpath "${2:?bag directory required}")
SRC=${3:-mola}
WS=$(cd "$(dirname "$0")/../.." && pwd)
TOOLS=$WS/src/ultron_b2_localization/tools
NS=/ultron/replay
[[ ${ROS_DOMAIN_ID:-} == 72 && ${ROS_LOCALHOST_ONLY:-} == 1 ]] || {
  echo "refusing: replay requires domain 72 and ROS_LOCALHOST_ONLY=1" >&2; exit 2; }
case "$SRC" in mola|kiss) ;; *) echo "refusing: odom must be mola or kiss" >&2; exit 2 ;; esac
case "$BAG/" in "$WS"/*|"$(dirname "$WS")"/*) echo "refusing: bag inside repo" >&2; exit 2 ;; esac
opts=$(findmnt -no OPTIONS -T "$BAG") || { echo "cannot inspect bag mount" >&2; exit 2; }
[[ ,$opts, == *,ro,* ]] || { echo "refusing: bag is not mounted read-only" >&2; exit 2; }
mkdir "$OUT" || { echo "refusing: output exists or parent missing" >&2; exit 2; }
export ULTRON_MAP_ROOT="$OUT/maps"
MAP_ID=${MAP_ID:-replay-$SRC}
launch= recorder= audit=
cleanup() {
  for p in "$recorder" "$audit" "$launch"; do
    [[ -z $p ]] || kill -TERM "$p" 2>/dev/null || true
  done
  for p in "$recorder" "$audit" "$launch"; do
    [[ -z $p ]] || wait "$p" 2>/dev/null || true
  done
}
trap cleanup EXIT
# Intentional word splitting: EXTRA_ARGS is a list of launch arguments, never shell-evaluated.
read -r -a extra <<< "${EXTRA_ARGS:-replay_delay:=12 replay_rate:=0.7 replay_cameras:=false}"
CMD=(scripts/ultron_launch.sh replay mode:=mapping odom_source:="$SRC" viz:=foxglove bag:="$BAG" "${extra[@]}")
printf '%q ' "${CMD[@]}" > "$OUT/launch_cmd.txt"
printf '\n' >> "$OUT/launch_cmd.txt"
python3 "$TOOLS/replay_eval.py" --out "$OUT" --namespace "$NS" > "$OUT/recorder.log" 2>&1 &
recorder=$!
python3 "$TOOLS/replay_audit.py" --out "$OUT/audit.json" > "$OUT/audit.log" 2>&1 &
audit=$!
"${CMD[@]}" > "$OUT/launch.log" 2>&1 &
launch=$!
sleep 12
ros2 node list > "$OUT/nodes.txt" 2>&1
ros2 topic list -t > "$OUT/topics.txt" 2>&1
for t in "$NS/sensors/lidar/points" "$NS/sensors/scan" /clock /tf /tf_static \
         "$NS/bag/tf_xsens" "$NS/state/odom" "$NS/map" "$NS/state/localization"; do
  echo "== $t"; ros2 topic info "$t" -v
done > "$OUT/graph_publishers.txt" 2>&1
timeout 30 ros2 run ultron_b2_viz foxglove_probe --duration 10 --subscribe --out "$OUT/foxglove_probe.json" \
  > "$OUT/foxglove_probe.txt" 2>&1 || echo "probe failed" > "$OUT/probe_failure.txt"
finished=false
for _ in $(seq "${BAG_TIMEOUT_S:-180}"); do
  if grep -q 'bag_player-[0-9]*\]: process has finished cleanly' "$OUT/launch.log"; then finished=true; break; fi
  if ! kill -0 "$launch" 2>/dev/null; then break; fi
  sleep 1
done
printf '%s\n' "$finished" > "$OUT/player_finished.txt"
if [[ $finished == true ]]; then
  timeout 90 ros2 service call "$NS/map/save" ultron_b2_interfaces/srv/SaveMap \
    "{schema_version: ultron.save_map/v1, map_id: '$MAP_ID', save_3d: false}" > "$OUT/save.log" 2>&1 || true
fi
cleanup
trap - EXIT
python3 - "$OUT" <<'PY'
import json
import re
import sys
from pathlib import Path
from ultron_b2_viz.whitelist import foxglove_allows, is_denied

out = Path(sys.argv[1])
errors = []
def read_json(name):
    try:
        return json.loads((out / name).read_text())
    except (OSError, ValueError):
        errors.append(f'missing or invalid {name}')
        return {}
summary, audit, probe = (read_json(n) for n in ('summary.json', 'audit.json', 'foxglove_probe.json'))
if (out / 'player_finished.txt').read_text().strip() != 'true':
    errors.append('bag player did not finish cleanly')
log = (out / 'launch.log').read_text()
if 'TF_AUTHORITY_VIOLATION' in log:
    errors.append('TF authority violation')
# The preflight also exits cleanly; locate the player exit rather than the first clean exit.
player_exit = re.search(r'\[bag_player-\d+\]: process has finished cleanly', log)
if player_exit and 'process has died' in log[:player_exit.start()]:
    errors.append('a launch process died before playback completed')
if not summary.get('odom_msgs') or not summary.get('scans') or not summary.get('slam', {}).get('poses'):
    errors.append('missing odometry, scans, or map-frame poses')
if not audit.get('samples') or not audit.get('tf_diagnostic_messages'):
    errors.append('missing graph or TF authority diagnostics')
diag = audit.get('tf_diagnostics') or {}
frames = diag.get('frames', {})
if diag.get('level') != 0 or not frames or any('unknown_publisher_msgs=0' not in v for v in frames.values()):
    errors.append('TF diagnostics report violation or unknown publishers')
for child, parent in (('odom', 'map'), ('base_link', 'odom')):
    if f'parents={parent} dynamic_publishers=1 static_publishers=0' not in frames.get(child, ''):
        errors.append(f'missing expected TF authority {parent}->{child}')
if audit.get('forbidden_topics') or audit.get('forbidden_services'):
    errors.append('forbidden topic or service observed')
if any(not foxglove_allows(t) or is_denied(t) for t in probe.get('topics', [])):
    errors.append('Foxglove advertised a non-whitelisted or denied topic')
if probe.get('services') or 'clientPublish' in (probe.get('server_info') or {}).get('capabilities', []):
    errors.append('Foxglove exposed a service or client publishing')
if not probe.get('msgs', {}).get('/ultron/replay/map'):
    errors.append('Foxglove received no map')
if not re.search(r'success=True', (out / 'save.log').read_text() if (out / 'save.log').exists() else ''):
    errors.append('map save did not succeed')
(out / 'checks.json').write_text(json.dumps({'errors': errors, 'scope': 'replay observations only'}, indent=2) + '\n')
print('Replay checks:', 'FAILED: ' + '; '.join(errors) if errors else 'observed; review loss and localization state')
sys.exit(bool(errors))
PY
