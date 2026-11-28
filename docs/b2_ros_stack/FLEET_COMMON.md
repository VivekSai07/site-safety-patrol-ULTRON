# Fleet common rules — B2 ROS 2 stack

Every fleet agent reads this file, then [PLAN.md](PLAN.md), then its own brief
(`AGENT_A.md` … `AGENT_F.md`). Status of all of these: **plan, not evidence**.

## Read first (in order)

1. `AGENTS.md`, `CLAUDE.md` (repo root) — binding.
2. `docs/SIM_TO_HARDWARE.md`, `docs/HARDWARE_READINESS.md`.
3. `docs/SAHIL_HANDOFF.md`, `docs/SAHIL_SLAM_PLAN.md`, `docs/SAHIL_NAV_PLAN.md`,
   `docs/SAHIL_VALIDATION.md` — the existing, validated simulation SLAM/Nav2 work
   you are porting. Do not regress it.
4. `docs/b2_ros_stack/PLAN.md` §2–§4 — current state, architecture, frames,
   topic contract, package layout.

## Branching and Git

- Base branch: `om/b2-ros-stack`. Work on `om/b2-ros-stack-<letter>`
  (e.g. `om/b2-ros-stack-c`). Small, focused commits with the Copilot
  co-author trailer.
- Never push to or merge into `main`; never force-push; never rewrite others'
  history. Open PRs against `om/b2-ros-stack` only when the user asks.
- Commit **no** bags, maps, point clouds, weights, credentials, `runs/`,
  `build/ install/ log/`. Small text fixtures (≤ 100 KB) are fine.

## Environment

- ROS 2 Humble, Python 3.10 system interpreter for ROS nodes. The existing
  `.venv-ros` (`scripts/setup-ros-venv.sh`) is the reference for MuJoCo + rclpy.
- Workspace: `ros2_ws/` at repo root; `colcon build --symlink-install`.
  Third-party sources go in `ros2_ws/deps.repos` with pinned commits/tags
  (vcstool), not vendored copies, unless a licence review says otherwise.
- Prefer Humble binary packages (`ros-humble-*`) where available. Record every
  apt/pip install in your validation section.
- Existing fast suite must stay green:
  `PYTHONPATH=src python3 -m unittest discover -s tests -v`.

## Isolation (non-negotiable)

| Mode | Namespace | `ROS_DOMAIN_ID` | Network |
|---|---|---|---|
| sim | `/ultron/sim` | 71 | `ROS_LOCALHOST_ONLY=1` |
| replay | `/ultron/replay` | 72 | `ROS_LOCALHOST_ONLY=1`, no control nodes |
| live | `/ultron/live` | 0 (robot network) | **only with explicit user authorization** |
| laptop viz | — | 42 | domain_bridge / zenoh whitelist |

- No agent connects to the real B2, Orin, robot network or any hardware.
  No agent launches `live` mode against a real robot. Live code is written and
  tested against mocks/replay only.
- Command-path invariant (PLAN.md §topic contract): every velocity source
  (Nav2, teleop, mission, tests) enters `twist_mux`, then the common
  `collision_monitor`, then the common `motion_gate`; only `motion_gate`
  publishes `cmd_vel`, and the sim driver / B2 bridge subscribe to nothing
  else. No agent adds a shortcut past the monitor or gate.
- Real-motion prerequisites (outside any agent's authority): verified RC
  priority, independent watchdog, functional E-stop, measured environment,
  named operator, full-system sim acceptance record (M6) plus a separate,
  bounded, user-authorized hardware plan.

## Evidence rules

- Report PASS / FAIL / UNRUN with command, git revision, config hash, data
  provenance and artifact path (local, uncommitted). A planned test is not a pass.
- Every numeric threshold is **PROPOSED** and must be fixed *before* the run it
  judges. Simulation values are not hardware settings.
- Oracle detections, scripted findings and kinematic motion must be labelled as
  such; never presented as learned perception or validated walking.
- Ground truth may be used for evaluation only, never as estimator input; it
  never appears on `/tf`.
- `AUTOMATED_PASS` is scoped pre-screening, never safe-to-use. Hard fails
  dominate; unknown/conflicting evidence → MANUAL_REVIEW. Coverage gaps stay
  mission-level.
- Results go to `docs/b2_ros_stack/VALIDATION.md` under your agent's heading
  only (create the heading if absent; do not edit other agents' sections).

## Contract changes

`ultron_b2_interfaces` (Agent A) is shared. To change a message, open a short
note in `docs/b2_ros_stack/CONTRACT_CHANGES.md` (who, why, fields), coordinate
with A, bump the message's version field. Never silently redefine shared fields.

## Hand-back (end of every agent session)

Append to your VALIDATION.md section: what works (commands), what is stubbed,
what is blocked (exact missing input + owner), next command to run.
