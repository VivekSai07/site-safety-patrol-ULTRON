# ultron_b2_preprocess — STUB

**Owner: Agent B** (see `docs/b2_ros_stack/AGENT_B.md` and PLAN.md §4).

Planned contents: Self-filter crop box, range/height crop, voxel downsample, optional deskew; pointcloud_to_laserscan config. Python+numpy first, C++ later if throughput needs it.

Created empty-but-buildable by Agent A so the workspace builds. Keep relative topic names
(PLAN.md §3.2); the bringup sub-launch in `ultron_b2_bringup/launch/sub/` is the integration point.
