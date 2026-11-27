#!/usr/bin/env bash
# ULTRON lidar SLAM simulation launcher. SIMULATION ONLY: isolated ROS domain, localhost
# only, no hardware topics. Requires .venv-ros (scripts/setup-ros-venv.sh).
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv="${ULTRON_ROS_VENV:-$repo_root/.venv-ros}"
[[ -x "$venv/bin/python" ]] || { echo "Missing $venv; run scripts/setup-ros-venv.sh" >&2; exit 2; }

set +u  # ROS setup scripts read unset variables
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="$repo_root/src${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1
domain="$("$venv/bin/python" -c 'from ultron.autonomy.config import load_sim_slam_config as l; print(l().ros["domain_id"])')"
export ROS_DOMAIN_ID="$domain" ROS_LOCALHOST_ONLY=1

action="${1:-help}"
shift || true
case "$action" in
    bridge)   # scripts/sim-slam.sh bridge [--route FILE] [--rate R] [--duration S] [--out DIR]
        exec "$venv/bin/python" -m ultron.autonomy.sim_bridge "$@" ;;
    survey)   # scripted mapping route with logs: scripts/sim-slam.sh survey OUT_DIR [--rate R]
        out="${1:?output folder}"; shift
        exec "$venv/bin/python" -m ultron.autonomy.sim_bridge \
            --route "$repo_root/config/autonomy/routes/hall_survey_loop.yaml" --out "$out" "$@" ;;
    map)      # scripted mapping + save + evaluate: scripts/sim-slam.sh map OUT_DIR [--rate R] [--record]
        exec "$venv/bin/python" -m ultron.autonomy.run_mapping "$@" ;;
    localize) # scripts/sim-slam.sh localize OUT --map MAP_RUN [--scenario FILE] [--offset DX DY DYAW_DEG] [--rate R]
        exec "$venv/bin/python" -m ultron.autonomy.run_localization "$@" ;;
    eval)     # re-evaluate a mapping run folder: scripts/sim-slam.sh eval OUT_DIR
        exec "$venv/bin/python" -m ultron.autonomy.eval_slam "$@" ;;
    record)   # scripts/sim-slam.sh record BAG_DIR  (new folder)
        bag="${1:?bag folder}"
        exec ros2 bag record -o "$bag" --use-sim-time /clock /tf /tf_static \
            /ultron/sim/lidar/points /ultron/sim/scan /ultron/sim/odom \
            /ultron/sim/ground_truth/pose /ultron/sim/bridge_status ;;
    rviz)     # scripts/sim-slam.sh rviz [map]   ("map" layout while SLAM runs)
        layout="$repo_root/config/autonomy/sim_slam.rviz"
        [[ "${1:-}" == map ]] && layout="$repo_root/config/autonomy/sim_slam_map.rviz"
        exec rviz2 -d "$layout" --ros-args -p use_sim_time:=true ;;
    env)      # print the environment for manual ros2 commands
        printf 'ROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=1\n' "$ROS_DOMAIN_ID" ;;
    exec)     # run any command inside the isolated environment
        exec "$@" ;;
    *)
        echo 'Usage: scripts/sim-slam.sh {bridge|survey|map|localize|eval|record|rviz|env|exec} [args]' >&2
        exit 2 ;;
esac
