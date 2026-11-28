# Integrated simulation and hardware readiness

User requests live seven-section cloud assessment, frontend wiring and dynamic avoidance, with an honest hardware readiness audit. Existing full-system hardware gate remains in force; no robot connection is authorized.

Design: preserve Sahil's ROS branch and navigation contract. The known-map kinematic demo emits immutable evidence-job manifests during inspection. A single bounded background worker in the live dashboard tailer validates the existing seven-section Responses schema and publishes evidence-linked results. Cloud errors/refusals become review; model findings never certify safe use. The patrol makes no cloud calls and continues independently. Partial 360 coverage is explicit. Dynamic occupancy snapshots in the same map/time basis are required before every guarded motion step; stale, invalid or intersecting observations stop motion. A bounded replan may route around a fresh obstacle. Simulation injection is labelled and is not lidar validation or an independent safety controller.

1. Add failing tests for dynamic snapshot freshness, invalid input, swept-path clearance, stop/replan; implement `dynamic_obstacles.py`, integrate opt-in guarded demo and deterministic obstacle injection.
2. Add failing tests for evidence manifest binding, worker transport failure, hard-fail precedence and dashboard publication before run completion. Implement `patrol_assessment.py`, add opt-in tailer API options and patrol evidence-job events. Preserve immutable runs and default demo behavior.
3. Integrate target-relative three-band captures into the existing explicitly open A-frame orbit experiment. Keep blocked/full-ring cases incomplete, no physical pan/tilt claim.
4. Run focused unit tests, existing affected suites, representative headless integrated capture/dynamic cases. Attempt live API only if networking permits; record unavailable infrastructure accurately.
5. Write a per-layer readiness matrix, exact launch commands, test evidence and remaining owners. Required unrun ROS/physical cases block hardware readiness.
