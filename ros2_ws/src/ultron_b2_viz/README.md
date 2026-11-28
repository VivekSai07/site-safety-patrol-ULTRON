# ultron_b2_viz

**Owner: Agent F.** Read-only visualization for the ULTRON B2 stack. For the user guide, Wi-Fi
setup and measured bandwidth, see `docs/b2_ros_stack/VIZ_GUIDE.md`.

| Item | What |
|---|---|
| `ultron_b2_viz/whitelist.py` | Single source of truth for allowed topics (Foxglove regex, domain-42 bridge, zenoh) |
| `viz_throttle` (node) | `viz/points` (voxel + ≤2 Hz), `viz/map_cloud`, `viz/cam_head/color/image/compressed` (JPEG ≤5 Hz), rate-capped relays of map/costmaps/plan, `viz/throttle_status` JSON (bytes/s) |
| `launch/foxglove.launch.py` | `foxglove_bridge` on `127.0.0.1:8765`, topic whitelist only, no client publish/services/params |
| `launch/rviz_bridge.launch.py` | `domain_bridge` stack domain → 42 (`config/domain_bridge_<mode>.yaml`); `zenoh:=true` adds `zenoh-bridge-ros2dds` (UNRUN if not installed) |
| `config/zenoh_bridge_{orin,laptop}.json5` | zenoh allow-lists (UNRUN) |
| `rviz/ultron_viz42.rviz` | laptop RViz in domain 42 |
| `foxglove/<mode>/ultron_{mapping,nav,mission}.json` | Foxglove layouts |
| `topic_probe`, `foxglove_probe` | measurement tools: rate/bandwidth JSON for a domain, and a WebSocket client that lists the advertised channels |

The domain-bridge YAMLs, zenoh configs, RViz config and layouts are generated. After changing
`whitelist.py`, run `ros2 run ultron_b2_viz gen_viz_configs`; `test/test_viz_unit.py` fails if the
generated files are stale.

`viz_throttle` reads sensor inputs RELIABLE by default (`sensor_input_reliability`). Large
best-effort samples are dropped over loopback with the default 212 KB socket buffer. Set it to
`best_effort` only if a live driver publishes best-effort.
