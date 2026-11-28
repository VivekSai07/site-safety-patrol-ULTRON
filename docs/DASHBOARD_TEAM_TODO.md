# Dashboard: what it still needs from the team

Owner of the dashboard and `scripts/dashboard_bridge.py`: Aravind. Each item says who it is for, what
the dashboard side already does, and what is missing. Tick an item off when the answer or change is in.
Launch: `scripts/dashboard.sh demo` (simulated demo-site) or `scripts/dashboard.sh live` (robot).

## Exploration (Site map → Start exploration)

The button is on the live page but is a placeholder: it shows "not connected yet" and sends nothing.
The server side is in place and tested (`/api/exploration`: start / finish, states
IDLE → STARTING → EXPLORING → FINISHING → MAPPED / FAILED, confirmed only by the bridge; goals and
exploration exclude each other). Planned bridge behaviour (option B): Start launches the explore node,
Finish mapping publishes `explore/resume=false` then shuts the node down, Start again relaunches it and
continues the same SLAM map. Explorer: robo-friends/m-explore-ros2 (package `explore_lite`).

- [ ] **Launch command** for the explore node, with the team's params file. Placeholder until then:
      `ros2 launch explore_lite explore.launch.py` (its default `use_sim_time` is **true**: pass
      `use_sim_time:=false` on the real robot).
- [ ] **Namespace**, if the explore node runs in one (topics are relative: `explore/resume`, `explore/status`).
- [ ] **`explore_lite_msgs` installed** where the bridge runs, so it can read `explore/status`
      (`exploration_started`, `exploration_paused`, `exploration_complete`, `returning_to_origin`, ...).
      Not installed on Aravind's laptop today; only `slam_toolbox` is.
- [ ] **`return_to_init`**: keep it (robot drives back to where Start was last pressed when no frontiers
      are left) or turn it off.
- [ ] **Costmap topic** the explorer reads (`costmap_topic`; the package's params use `map`).
- [ ] Decide to wire the button for real (then: bridge step, and Set goal is refused while exploring).

## Map, pose and goals (live page)

The bridge already uploads `/map`, sends the robot pose from TF and forwards Set goal to Nav2.
Checked only against a stand-in action server on Jazzy.

- [ ] **Topic and frame names** on the robot stack: map topic (`/map`), frames (`map` → `base_link`),
      Nav2 action (`navigate_to_pose`). Bridge parameters exist for each.
- [ ] **SLAM map update interval** (slam_toolbox `map_update_interval`, default 5 s): sets how often the
      Site map refreshes while mapping. Shorter is smoother, at a CPU cost.
- [ ] A first run with **real Nav2 + SLAM** (`scripts/dashboard.sh live`), goal set from the page.
- [ ] Where the bridge runs (robot computer or the GPU machine) and the dashboard URL it should reach.

## Site checklists (history page, planned)

Catalogue of nesseler's five other checklists is in `src/ultron/dashboard/site_checklists.py`
(92 rows, 26 a robot can answer). Pages and downloads are next on the dashboard side.

- [ ] **Pipeline → dashboard**: send each non-ladder object's checklist rows and evidence photos
      (today only a one-line `CHECKLIST_DONE` event arrives). Owner of `ultron.pipeline`.
- [ ] **Detector prompts** for the kinds the checklists ask about that nothing detects yet: first-aid
      notice, escape plan, escape sign, escape route, emergency light, gas cylinder, scaffold, dust
      extractor, person. Until then those rows show "open, no detector". Owner of `pipeline/catalog.py`.
- [ ] Is there a **scaffold** in the hall? If not, the scaffold checklist is shown as not on this site.

## Real recordings (later)

- [ ] `ros2 bag info` of the new B2 recordings: topic names and message types, to check they match what
      `protocol/bag.py` reads (`/camera_depth/head/...`, `/scan_cloud`, raw not compressed images).
