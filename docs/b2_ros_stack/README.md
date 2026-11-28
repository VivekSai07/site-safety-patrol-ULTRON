# B2 ROS 2 stack — fleet pack

Plan and fleet assignments for the end-to-end Unitree B2 ROS 2 stack
(sim + replay + live; MOLA-LO + slam_toolbox + Nav2; ladder search; edge SLAM
with Foxglove / laptop RViz). **Plan only — no evidence, no hardware authority.**

| File | Purpose |
|---|---|
| [PLAN.md](PLAN.md) | Current state, architecture, frames/topics/domains, packages, OSS references, milestones, unknowns |
| [FLEET_COMMON.md](FLEET_COMMON.md) | Rules every agent follows (branching, isolation, evidence) |
| [AGENT_A.md](AGENT_A.md) | Foundation: workspace, interfaces, description, bringup, containers, CI |
| [AGENT_B.md](AGENT_B.md) | Real bag, drivers, replay, preprocessing |
| [AGENT_C.md](AGENT_C.md) | MOLA-LO / KISS-ICP odometry, slam_toolbox, localization, map manager |
| [AGENT_D.md](AGENT_D.md) | Nav2, safety chain, twist_mux, B2 sport bridge (disabled, mock-tested) |
| [AGENT_E.md](AGENT_E.md) | Exploration, ladder detection/tracking, mission manager, coverage |
| [AGENT_F.md](AGENT_F.md) | Sim wrapper, Foxglove/RViz, recording, integration tests, runbook, validation |
| [VALIDATION.md](VALIDATION.md) | Results, one section per agent (actual results only) |
| [CONTRACT_CHANGES.md](CONTRACT_CHANGES.md) | Shared-interface change log |

Order: A first (buildable skeleton in its first commits) → B, C, D, E, F in
parallel on stubs → C(sim) unblocks D → D(sim) unblocks E → F assembles M6.

To assign: tell each agent "On branch `om/b2-ros-stack`, read
`docs/b2_ros_stack/FLEET_COMMON.md` and `AGENT_<X>.md`, then follow its
kickoff prompt."
