#!/bin/bash
# Inside the stand-in container (network off): ros2 bag play -> relay -> pipeline over localhost,
# with the commands of RECORD_ON_B2.md section 6c. /run holds yoloe-11l-seg.pt and mobileclip_blt.ts.
source /opt/ros/humble/setup.bash
cd /run
export PYTHONUNBUFFERED=1 PYTHONPATH=/repo/src:$PYTHONPATH OPENAI_API_KEY=offline OPENAI_BASE_URL=http://127.0.0.1:9/v1
MAP="--sites /run/ultron-runs/sites --site ctrl-hall --map-id hall-start-A --data /run/ultron-runs/${PATROL:-patrol-1}"
/venv/bin/python -m ultron.pipeline survey --source relay:5600 --detector yolo:yoloe-11l-seg.pt --interval 0 $MAP \
    > /run/pipeline.log 2>&1 &
PIPE=$!
until grep -q "Waiting for the relay" /run/pipeline.log || ! kill -0 $PIPE 2>/dev/null; do sleep 1; done
python3 /repo/scripts/ros_relay.py --to 127.0.0.1:5600 --rate 8 > /run/relay.log 2>&1 &
RELAY=$!
sleep 3
ros2 bag play /bag > /run/play.log 2>&1
sleep 5
kill -INT $RELAY; wait $RELAY
wait $PIPE; echo "pipeline exit=$?"
/venv/bin/python -m ultron.pipeline status $MAP
