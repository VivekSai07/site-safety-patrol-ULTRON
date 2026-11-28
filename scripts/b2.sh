#!/usr/bin/env bash
# One entry point on the B2 computer. Sources ROS itself; nothing to set up by hand.
#
#   scripts/b2.sh doctor                 read-only report: ROS, drivers, network, devices -> file
#   scripts/b2.sh check [--camera-only]  red/green table of sensor topics (camera, lidar, IMU)
#   scripts/b2.sh record [--camera-only|--force] [recorder options]
#                                        check, then record in tmux session "ultron-rec"
#   scripts/b2.sh relay LAPTOP_IP [--camera-only|--force] [relay options]
#                                        check, then relay to the patrol and record, in tmux "ultron-relay"
#
# Environment (all optional): ULTRON_ROS_SETUP="file1 file2" extra setup files to source after
# /opt/ros (e.g. a workspace with the lidar driver); ULTRON_BAGS (default ~/ultron-bags).
# Read-only towards the robot: nothing here publishes a topic or sends a command, edits
# .bashrc, installs packages or stops a running driver.
# no `set -u`: ROS setup files read unset variables
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAGS="${ULTRON_BAGS:-$HOME/ultron-bags}"

ros_env() {
    if [ -z "${ROS_DISTRO:-}" ]; then
        local setup=""
        for d in humble jazzy iron foxy; do
            [ -f "/opt/ros/$d/setup.bash" ] && { setup="/opt/ros/$d/setup.bash"; break; }
        done
        [ -z "$setup" ] && setup="$(ls /opt/ros/*/setup.bash 2>/dev/null | head -1)"
        [ -z "$setup" ] && { echo "No ROS found in /opt/ros." >&2; exit 2; }
        # shellcheck disable=SC1090
        source "$setup"
    fi
    for f in ${ULTRON_ROS_SETUP:-}; do
        # shellcheck disable=SC1090
        [ -f "$f" ] && source "$f" || echo "warning: ULTRON_ROS_SETUP file $f not found" >&2
    done
    if [ "${ROS_LOCALHOST_ONLY:-0}" = "1" ]; then
        echo "note: ROS_LOCALHOST_ONLY=1 is set; topics from other computers are invisible" >&2
    fi
}

doctor() {
    local out="$HOME/ultron-doctor-$(date +%Y%m%d-%H%M%S).txt"
    {
        echo "== host";      hostname; date; uname -a; (ip -4 -br addr 2>/dev/null || hostname -I)
        echo "== ros env";   ls /opt/ros 2>&1
        env | grep -E '^(ROS_|RMW_|CYCLONEDDS|FASTRTPS|AMENT_PREFIX)' | sed 's/\(AMENT_PREFIX_PATH=\).\{300\}.*/\1(long)/'
        ros_env   # the environment above is the login shell's; from here on ROS is sourced
        echo "== rmw implementations"; ls /opt/ros/*/lib 2>/dev/null | grep -oE '^librmw_[a-z_]+cpp' | sort -u
        echo "== drivers installed"
        timeout 30 ros2 pkg list 2>/dev/null | grep -iE 'realsense|rslidar|robosense|xsens|unitree|livox|velodyne|ouster|cyclonedds' || echo "(none of the expected packages)"
        echo "== colcon workspaces under ~"
        find "$HOME" -maxdepth 4 -path '*/install/setup.bash' 2>/dev/null | head -20
        echo "== driver configs/launch files mentioning the hall topics or drivers"
        timeout 30 grep -rlsE 'scan_cloud|lidar_link|RSHELIOS|camera_depth|xsens' "$HOME" --include='*.yaml' --include='*.yml' \
            --include='*.launch.py' --include='*.sh' --include='*.py' 2>/dev/null | grep -v '/ultron/' | head -30
        echo "== how drivers were started before (shell history)"
        grep -hE 'ros2 (launch|run)|rslidar|xsens|realsense|RMW_IMPLEMENTATION|CYCLONEDDS|ROS_DOMAIN_ID|setup\.(bash|sh)' \
            "$HOME/.bash_history" "$HOME/.zsh_history" 2>/dev/null | tail -40
        echo "== .bashrc ROS lines"; grep -nE 'ros|ROS_|RMW|CYCLONE|unitree|source' "$HOME/.bashrc" 2>/dev/null
        echo "== services and containers"
        systemctl list-units --type=service --no-pager 2>/dev/null | grep -iE 'ros|lidar|xsens|realsense|unitree|camera' || echo "(no matching services)"
        timeout 10 docker ps --format '{{.Names}}  {{.Image}}  {{.Status}}' 2>/dev/null || echo "(docker not available)"
        echo "== running driver processes"
        ps aux | grep -iE 'realsense|rslidar|xsens|unitree|ros2|launch|_node' | grep -v grep | cut -c1-220 | head -30
        echo "== devices"
        lsusb 2>/dev/null | grep -iE 'intel|xsens|2639|ftdi|silicon|prolific' || echo "(no RealSense/Xsens on USB)"
        ls -l /dev/ttyUSB* /dev/ttyACM* /dev/xsens* 2>/dev/null || echo "(no serial devices)"
        echo "== lidar on the network (RoboSense default 192.168.1.200)"
        ping -c1 -W1 192.168.1.200 >/dev/null 2>&1 && echo "192.168.1.200 answers" || echo "192.168.1.200 no answer"
        echo "== disk"; df -h "$HOME" | tail -1
        echo "== ROS graph"
        timeout 20 ros2 topic list --no-daemon 2>&1 | head -80
        echo "== preflight"
        timeout 30 python3 "$REPO/scripts/b2_check.py" --seconds 3 2>&1
    } 2>&1 | tee "$out"
    echo; echo "Report saved to $out  (send it to the team)"
}

check() { ros_env; python3 "$REPO/scripts/b2_check.py" "$@"; }

gate() {  # gate [--camera-only|--force]: run the preflight; stop on failure unless --force
    local mode="$1"
    [ "$mode" = "--force" ] && { echo "Skipping preflight (--force)."; return 0; }
    if ! check $([ "$mode" = "--camera-only" ] && echo --camera-only); then
        echo
        echo "Not starting. Fix the sensors above, or rerun with --camera-only (camera data only)"
        echo "or --force (record whatever is there)."
        exit 1
    fi
}

in_tmux() {  # in_tmux SESSION COMMAND...: run in a tmux session (survives SSH drops) and attach
    local session="$1"; shift
    if [ -n "${TMUX:-}" ] || ! command -v tmux >/dev/null; then "$@"; return; fi
    if tmux has-session -t "$session" 2>/dev/null; then
        echo "tmux session $session already exists: attaching (Ctrl-C there stops it)."
    else
        tmux new-session -d -s "$session" "$(printf '%q ' "$@"); echo; echo 'finished; press Enter'; read"
    fi
    echo "Detach with Ctrl-b d, come back with: tmux attach -t $session"
    sleep 1; tmux attach -t "$session"
}

split_mode() {  # first arg may be --camera-only / --force
    MODE=""
    case "${1:-}" in --camera-only|--force) MODE="$1";; esac
}

cmd="${1:-}"; shift || true
case "$cmd" in
    doctor) doctor ;;
    check)  check "$@" ;;
    record)
        split_mode "$@"; [ -n "$MODE" ] && shift
        gate "$MODE"
        in_tmux ultron-rec bash "$REPO/scripts/b2.sh" _record "$@" ;;
    relay)
        [ $# -ge 1 ] || { echo "usage: scripts/b2.sh relay LAPTOP_IP [--camera-only|--force] [relay options]"; exit 2; }
        target="$1"; shift
        split_mode "$@"; [ -n "$MODE" ] && shift
        gate "$MODE"
        in_tmux ultron-relay bash "$REPO/scripts/b2.sh" _relay "$target" "$@" ;;
    _record) ros_env; exec python3 "$REPO/scripts/record.py" --out "$BAGS" "$@" ;;
    _relay)  ros_env; t="$1"; shift; exec python3 "$REPO/scripts/ros_relay.py" --to "$t:5600" --record "$BAGS" "$@" ;;
    *) sed -n '2,15p' "$0"; exit 2 ;;
esac
