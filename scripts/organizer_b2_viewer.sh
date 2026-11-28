#!/bin/bash
set -eu

export DISPLAY=127.0.0.1:2 LIBGL_ALWAYS_SOFTWARE=1 PYTHONDONTWRITEBYTECODE=1
Xvfb :2 -screen 0 1600x900x24 -nolock -nolisten unix -listen tcp -ac &
sleep 2
x11vnc -display "$DISPLAY" -noshm -forever -shared -nopw -localhost -rfbport 5901 -quiet &
websockify --web /usr/share/novnc 127.0.0.1:6081 127.0.0.1:5901 &

robot_env=(env -i HOME=/root PATH=/opt/unitree-venv/bin:/usr/bin:/bin
  LD_LIBRARY_PATH=/opt/cyclonedds-0.10.2/lib CYCLONEDDS_HOME=/opt/cyclonedds-0.10.2
  PYTHONPATH=/opt/kit/robot PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
  DISPLAY=127.0.0.1:2 LIBGL_ALWAYS_SOFTWARE=1)
(sleep 5; exec "${robot_env[@]}" python /opt/kit/robot/stand_b2.py 71 lo) &
exec "${robot_env[@]}" python /opt/kit/robot/sim.py b2 71 lo 1
