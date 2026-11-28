#!/bin/bash
# Inside a ROS 2 container with nav2_msgs (network off): dashboard + bridge + fake Nav2/SLAM + fake explorer,
# then scripts/b2_standin/explore_check.py walks the exploration contract against them. Needs the
# dashboard's Python packages for this interpreter on /pydeps (pip install --target, see README.md).
#   docker run --rm --network none -v "$PWD":/repo:ro -v DEPS:/pydeps:ro sitegen-ros:jazzy-4 \
#       bash /repo/scripts/b2_standin/explore_check.sh
source /opt/ros/*/setup.bash
export PYTHONUNBUFFERED=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
LOG=/tmp/explore-check && mkdir -p $LOG
cd /tmp
PYTHONPATH=/pydeps:/repo/src python3 -m ultron.dashboard --goals --db $LOG/d.sqlite --media $LOG/media --port 8000 \
    > $LOG/dashboard.log 2>&1 &
python3 /repo/scripts/b2_standin/fake_nav2.py --ros-args -p drive_s:=5.0 > $LOG/nav2.log 2>&1 &
echo $! > $LOG/nav2.pid
for _ in $(seq 50); do curl -fs http://127.0.0.1:8000/api/sites >/dev/null 2>&1 && break; sleep 0.2; done
PYTHONPATH=$PYTHONPATH:/pydeps python3 /repo/scripts/b2_standin/explore_check.py
code=$?
for f in $LOG/*.log; do echo "===== $f (tail)"; tail -n ${TAIL:-25} $f; done
pkill -INT -f dashboard_bridge.py; pkill -INT -f fake_; pkill -INT -f ultron.dashboard; sleep 2
exit $code
