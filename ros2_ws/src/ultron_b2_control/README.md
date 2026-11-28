# ultron_b2_control

**Owner: Agent D** (`docs/b2_ros_stack/AGENT_D.md`). Source selection and drivers behind the final motion gate.
The chain is described in `ultron_b2_navigation/README.md`.

| Node / file | Role |
|---|---|
| `config/twist_mux.yaml` | `control/in/teleop` (100) > `control/in/autonomy` (10); lock `ops/hold` (255); output `control/cmd_vel_muxed`. |
| `active_source_monitor` | `control/active_source` (`hold`/`teleop`/`autonomy`/`none`) from the same table and freshness (`sport_logic.select_source`). |
| `launch/control.launch.py` | twist_mux + active_source_monitor + collision_monitor + motion_gate + driver (`driver:=sim\|bridge\|none`). |
| `launch/teleop.launch.py` | joy / keyboard teleop → `control/in/teleop` only. |
| `sim_driver` | Subscribes to `cmd_vel` and publishes it to `sim/cmd_vel` (simulator input, Agent F). |
| `b2_sport_bridge` | Subscribes only to gated `cmd_vel` and publishes `unitree_api/msg/Request` on `/api/sport/request`: Move (1008, `{"x","y","z"}`) and StopMove (1003). API ids come from the pinned `unitree_ros2` @668d1ec. `hardware_enable` defaults to false, which means it is silent. It needs the `control/arm` service (armed state latches off on any fault). The watchdog (0.3 s) sends exactly one StopMove. Clamps and acceleration limits apply, non-finite input latches a fault, and a domain/target preflight refuses to start. `target:=robot` additionally needs domain 0, `/ultron/live` and a hardware token, and is **never** used by tests. |
| `mock_sport_server` | Records requests (JSONL) and simulates responses for tests. |

Tests (`colcon test --packages-select ultron_b2_control`) use domains 80 and 81 and never domain 0. The bridge has only
ever run against `mock_sport_server`; it has never been run against hardware.
