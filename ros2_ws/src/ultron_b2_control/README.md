# ultron_b2_control — STUB

**Owner: Agent D** (see `docs/b2_ros_stack/AGENT_D.md` and PLAN.md §4).

Planned contents: twist_mux (HOLD > teleop > autonomy), active_source_monitor, teleop, b2_sport_bridge (subscribes only cmd_vel; disabled by default), sim_driver adapter.

Created empty-but-buildable by Agent A so the workspace builds. Keep relative topic names
(PLAN.md §3.2); the bringup sub-launch in `ultron_b2_bringup/launch/sub/` is the integration point.
