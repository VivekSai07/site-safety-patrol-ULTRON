# ROS 2 integration boundary

ROS visualization replay and the separate dashboard goal bridge exist; a
hardware integration is not verified. No hardware connection or motion is
authorized by these notes.

At kickoff, inspect the organizer starter and `ros2 topic list -t` with the operator. Record actual camera/depth/LiDAR topics, message types, frame IDs, QoS, calibration, image encoding and timestamps. Do not guess them.

Suggested first milestone: consume an organizer-approved rosbag offline, adapt detector/OCR findings to the documented observation contract, then replay through policy/reporting. Keep inference on your laptop or Jetson according to measured latency and event permissions. Avoid coupling motion control to model output.

Separate responsibilities: navigation/operator owns stopping distance and emergency stop; perception emits calibrated observations and unknowns; tracking associates repeated views; inspection maps the official protocol to evidence; reporting presents review actions. For real evidence, retain source image/crop reference, timestamp, frame and model provenance. The synthetic SVG renderer must be replaced, not used as camera evidence.

Any live hardware demo requires verified interfaces and an authorized operator. This directory is not a ROS package and publishes no commands.

## Dashboard bridge

`scripts/dashboard_bridge.py` is the one exception: it sends navigation goals.
Each dashboard goal becomes a Nav2 `NavigateToPose` action goal; the bridge
also uploads `/map` and the TF pose `map -> base_frame`. Goals are refused
unless the dashboard runs with `--goals`. The dashboard has no authentication,
and the bridge has no independent hardware-isolation guard: it can forward
goals to any reachable matching action server. Before using these examples, a
responsible operator must independently verify an isolated simulation with no
route to hardware. Localhost, a namespace or a ROS domain alone is not proof
of isolation.

    python3 -m ultron.dashboard --goals
    python3 scripts/dashboard_bridge.py --ros-args -p base_frame:=base_link   # with ROS sourced

Parameters: `dashboard_url`, `map_topic` (/map), `map_frame` (map), `base_frame` (base_link), `action_name` (navigate_to_pose), `pose_rate_hz` (5), `map_min_period_s` (2). A teammate reported a stand-in action-server check on ROS 2 Jazzy; the exact network isolation was not independently verified here. Real Nav2 and Humble remain untested. See [dashboard goal guidance](../../docs/dashboard_nav_goal.md).
