# ultron_b2_perception

**Owner: Agent E** (`docs/b2_ros_stack/AGENT_E.md`). Started by `ultron_b2_bringup` `mode:=mission`.
All topic names are relative (mode namespace). Every numeric parameter is **PROPOSED** (`config/perception_sim.yaml`).

| Node | In | Out |
| --- | --- | --- |
| `ladder_detector` | `sensors/cam_head/color/image_raw`, `.../color/camera_info`; oracle only: `ground_truth/pose` (EVALUATION_ONLY) + `sensors/cam_head/depth/image_rect_raw`, or `sensors/cam_head/oracle/segmentation` | `perception/ladder_detections`, `perception/ladder_detector/status` |
| `ladder_localizer` | detections, `sensors/cam_head/depth/image_rect_raw` + `depth/camera_info`, TF map←camera **at the image stamp** | `perception/ladder_observations`, `perception/ladder_localizer/stats` |
| `ladder_tracker` | observations, `map`, `state/localization` | `perception/ladder_candidates` (latched; map-scoped) |

Backends (`backend` param):
* `oracle_sim` — simulation ground truth projected into the camera (`config/oracle_hall_ladders.yaml`,
  `source: SIMULATION_GROUND_TRUTH_ORACLE`). Refused unless `run_mode:=sim`. Every detection carries
  `is_oracle=true` and label `ORACLE_SIM:<type>`; candidates and registry rows keep the ORACLE label.
  **Oracle detections are not learned perception and validate nothing about real detection.**
* `yolo` / `owlv2` — load only if the libraries and local weights (`weights`, `model_dir`) exist. Weights
  are not in the repo; without them the detector reports `UNAVAILABLE` on its status topic and publishes nothing.

Localizer: drops observations whose TF at the image stamp is unavailable/extrapolated beyond `tf_timeout_s`
(stale TF), whose depth is missing or out of `max_range_m`, or whose image is older than `max_image_age_s`.
Tracker: association within `association_radius_m`; stationary re-observations keep the ID, new positions
create `NEW`, two-way-close matches stay `AMBIGUOUS` (never merged), assets unseen after
`missing_after_views` in-view checks become `MISSING`. Optional `memory_db` reads expected assets from
`src/ultron/memory.py`. Host tests: `python3 -m pytest test` (stale-TF test needs ROS).
