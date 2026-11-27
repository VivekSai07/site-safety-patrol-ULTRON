# ROS 2 integration boundary (not implemented)

At kickoff, inspect the organizer starter and `ros2 topic list -t` with the operator. Record actual camera/depth/LiDAR topics, message types, frame IDs, QoS, calibration, image encoding and timestamps. Do not guess them.

Suggested first milestone: consume an organizer-approved rosbag offline, adapt detector/OCR findings to the documented observation contract, then replay through policy/reporting. Keep inference on your laptop or Jetson according to measured latency and event permissions. Avoid coupling motion control to model output.

Separate responsibilities: navigation/operator owns stopping distance and emergency stop; perception emits calibrated observations and unknowns; tracking associates repeated views; inspection maps the official protocol to evidence; reporting presents review actions. For real evidence, retain source image/crop reference, timestamp, frame and model provenance. The synthetic SVG renderer must be replaced, not used as camera evidence.

Any live hardware demo requires verified interfaces and an authorized operator. This directory is not a ROS package and publishes no commands.
