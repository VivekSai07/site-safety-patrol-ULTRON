# ultron_b2_tools

**Owner: Agent B.**

- `bag_inspect` — characterise a rosbag2 without ROS (pip `rosbags`): topics, rates, gaps, TF tree,
  cloud fields, IMU, cameras. `ros2 run ultron_b2_tools bag_inspect <bag> --out runs/bag_inspect/<name>`
  or standalone `python3 ros2_ws/tools/bag_inspect.py ...`. Reports stay local (derived from private data).
- `stream_probe` — measure live/replayed topics over a wall-clock window: receive and header-stamp
  rates, message sizes, point counts, LaserScan finite beams/ranges and per-process CPU from `/proc`.
  Refuses `ROS_DOMAIN_ID` 0.

Planned (not yet): `bag_to_contract`, eval ports (`eval_slam`, `eval_localization`).
