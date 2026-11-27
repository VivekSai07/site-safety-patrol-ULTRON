# ROS 2 integration boundary (not implemented)

At kickoff, inspect the organizer starter and `ros2 topic list -t` with the operator. Record actual camera/depth/LiDAR topics, message types, frame IDs, QoS, calibration, image encoding and timestamps. Do not guess them.

Suggested first milestone: consume an organizer-approved rosbag offline, adapt detector/OCR findings to the documented observation contract, then replay through policy/reporting. Keep inference on your laptop or Jetson according to measured latency and event permissions. Avoid coupling motion control to model output.

Separate responsibilities: navigation/operator owns stopping distance and emergency stop; perception emits calibrated observations and unknowns; tracking associates repeated views; inspection maps the official protocol to evidence; reporting presents review actions. For real evidence, retain source image/crop reference, timestamp, frame and model provenance. The synthetic SVG renderer must be replaced, not used as camera evidence.

Any live hardware demo requires verified interfaces and an authorized operator. This directory is not a ROS package and publishes no commands.

## Dashboard bridge (simulation only)

`scripts/dashboard_bridge.py` is the one exception: it sends navigation goals. The dashboard map works like RViz's 2D Goal Pose. Each goal becomes a nav2 `NavigateToPose` action goal, and its result (ACCEPTED / SUCCEEDED / FAILED / CANCELED) shows on the map. The bridge also uploads `/map` and the TF pose `map -> base_frame` to the dashboard. Goals are refused unless the dashboard runs with `--goals`. Use it against the simulation only: anyone who can open the page can then move the robot.

    python3 -m ultron.dashboard --goals
    python3 scripts/dashboard_bridge.py --ros-args -p base_frame:=base_link   # with ROS sourced

Parameters: `dashboard_url`, `map_topic` (/map), `map_frame` (map), `base_frame` (base_link), `action_name` (navigate_to_pose), `pose_rate_hz` (5), `map_min_period_s` (2). Checked against a stand-in action server on ROS 2 Jazzy, not yet against real Nav2 or the Humble images.
