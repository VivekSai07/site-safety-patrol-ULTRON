# Dashboard: what it still needs from the team

Owner of the dashboard and `scripts/dashboard_bridge.py`: Aravind. Each item says who it is for, what
the dashboard side already does, and what is missing. Tick an item off when the answer or change is in.
Launch: `scripts/dashboard.sh demo` (simulated demo-site) or `scripts/dashboard.sh live` (robot).

## Exploration (Site map → Start exploration)

Contract: [EXPLORATION_CONTRACT.md](EXPLORATION_CONTRACT.md) (its last section lists what was built and
where it deviates). Done on the dashboard and bridge side, checked by unit tests and by
`scripts/b2_standin/explore_check.sh` (real bridge, fake explorer, fake Nav2; Jazzy, network off):

- [x] `/api/exploration` start / pause / renew / resume / finish, states IDLE → STARTING → EXPLORING ⇄
      PAUSING → PAUSED → RESUMING → … → FINISHING → MAPPED / FAILED, confirmed only by the bridge.
      Pipeline pauses on a lease (60 s default, 180 s max) that the dashboard itself ends.
- [x] Goals carry `by` (operator / pipeline); operator > pipeline > explorer; 409 `exploring`, `pausing`,
      `operator goal underway`.
- [x] Live page: the button starts and finishes mapping for real; a second button pauses / resumes;
      the state shows next to the goal; the fence is drawn on the map.
- [x] Bridge: Start launches `explore_command` as a child process (stopped with the bridge), Pause / Resume
      publish `explore/resume`, Finish stops the node; explorer death or no answer → FAILED with why.
- [x] Fence: `GET/PUT/DELETE /api/fence`, `--fence FILE`; bridge publishes `/map_fenced` and `/keepout_mask`.
- [x] Obstacles: `GET/PUT /api/obstacles`; bridge publishes `/ultron/object_obstacles` (PointCloud2).
- [x] A goal with no Nav2 fails after 3 s with `navigation not available` instead of hanging.

Still open:

- [ ] **Launch command** for the explore node, with the team's params file, **pointing `costmap_topic` at
      `/map_fenced`**. The default (`ros2 launch explore_lite explore.launch.py use_sim_time:=false`) reads
      the package params, `costmap_topic: map`: **the unfenced map, so it would chase the space behind the
      windows.** A working example without a params file is in EXPLORATION_CONTRACT.md. Set it with
      `-p explore_command:="..."` on the bridge.
- [ ] **Namespace**, if the explore node runs in one (bridge parameters `explore_resume_topic`,
      `explore_status_topic`; defaults `explore/resume`, `explore/status`).
- [ ] **`explore_lite_msgs` installed** where the bridge runs, so it reads `explore/status`. Without it the
      bridge infers the state from the process and its `explore/resume` subscription (works, less exact).
- [ ] **`return_to_init`**: keep it or turn it off (the example command turns it off; in a small room a
      drive back is of little use).
- [ ] **The room's fence**: draw it (map frame, slightly inside the glass) and save it as a JSON file for
      `--fence`. There is no drawing tool on the page yet; `PUT /api/fence` with curl.
- [ ] **KeepoutFilter / obstacle layer** in the Nav2 params that read `/keepout_mask` and
      `/ultron/object_obstacles` (navigation stack, Sahil).
- [ ] A first run with **real explore_lite + Nav2 + SLAM**: start, pause, goal, resume, finish.

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
      Contract: `POST /api/missions/{id}/site-checks` (see `SiteCheck` in `dashboard/models.py`).
      Until then, after a patrol: `python -m ultron.dashboard.import_site_checks runs/pipeline --mission ID`
      reads the workspace's `objects/*/checklist.json` and posts them.
- [ ] **Detector prompts** for the kinds the checklists ask about that nothing detects yet: first-aid
      notice, escape plan, escape sign, escape route, emergency light, gas cylinder, scaffold, dust
      extractor, person. Until then those rows show "open, no detector". Owner of `pipeline/catalog.py`.
- [ ] Is there a **scaffold** in the hall? If not, the scaffold checklist is shown as not on this site.

## Real recordings (later)

- [ ] `ros2 bag info` of the new B2 recordings: topic names and message types, to check they match what
      `protocol/bag.py` reads (`/camera_depth/head/...`, `/scan_cloud`, raw not compressed images).
