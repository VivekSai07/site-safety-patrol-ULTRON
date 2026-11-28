# ultron_b2_perception

**Owner: Agent E** (`docs/b2_ros_stack/AGENT_E.md`). Started by `ultron_b2_bringup` `mode:=mission`.
All topic names are relative (mode namespace). Every numeric parameter is **PROPOSED** (`config/perception_sim.yaml`).

| Node | In | Out |
| --- | --- | --- |
| `ladder_detector` | `sensors/cam_head/color/image_raw`, `.../color/camera_info`; simulation oracle: `sim/oracle/cam_head/ladder_labels` or ground-truth pose for projection | `perception/ladder_detections`, `perception/ladder_detector/status` |
| `ladder_localizer` | detections, `sensors/cam_head/depth/image_rect_raw` + `depth/camera_info`, TF map←camera **at the image stamp** | `perception/ladder_observations`, `perception/ladder_localizer/stats` |
| `ladder_tracker` | observations, `map`, `state/localization` | `perception/ladder_candidates` (latched; map-scoped) |

Backends (`backend` param):
* `oracle_sim` — simulation-only rendered instance labels matched to the exact color frame; the
  optional projection mode uses `config/oracle_hall_ladders.yaml` and can report occluded ladders
  without depth occlusion filtering. Instance IDs follow the simulator's name-sorted ladder labels.
  Refused unless `run_mode:=sim`. Every detection carries
  `is_oracle=true` and label `ORACLE_SIM:<type>`; candidates and registry rows keep the ORACLE label.
  **Oracle detections are not learned perception and validate nothing about real detection.**
  The simulated mission localizer also selects depth using exact-frame instance pixels. This
  privileged mask path is simulation-only; it rejects missing or ambiguous masks and never
  uses the simulator instance number as a persistent asset identity. Projection-oracle and
  learned backends need their own validated depth selection.
* `yolo` / `owlv2` — load only if the libraries and local weights (`weights`, `model_dir`) exist. Weights
  are not in the repo; without them the detector reports `UNAVAILABLE` on its status topic and publishes nothing.

Localizer: drops observations whose TF at the image stamp is unavailable/extrapolated beyond `tf_timeout_s`
(stale TF), whose depth is missing or out of `max_range_m`, or whose image is older than `max_image_age_s`.
It also rejects fusion with `DEPTH_NOT_REGISTERED` unless `depth_registered_to_color` is true. The sim
mission launch sets this for its co-located simulated RGB-D cameras; other modes default to false.
The recorded real camera has a color/depth baseline, so mapping boxes with intrinsics alone is not
registration. For non-simulation use, first supply a depth image registered to the color optical
center with matching camera information and verify calibration, timing and transforms; setting this
flag alone does not align images. This gate does not validate box depth or resolve candidate identity.
If any detection in a frame cannot be localized, the localizer suppresses all absence
checks for that frame; an unlocalized detection cannot justify marking a candidate missing.
Nonfinite or invalid camera matrices are rejected and clear the cached matrix until a valid
CameraInfo arrives; a prior calibration is not silently reused.
Tracker: association within `association_radius_m`; stationary re-observations keep the ID, new positions
create `NEW`, two-way-close matches stay `AMBIGUOUS` (never merged), assets unseen after
`missing_after_views` in-view checks become `MISSING`. Optional `memory_db` reads expected assets from
`src/ultron/memory.py`. Host tests: `python3 -m pytest test` (stale-TF test needs ROS).

Reproduce the deterministic rendered-scene **diagnostic**, in the dev container from `ros2_ws/`:

```bash
source install/setup.bash
MUJOCO_GL=egl PYTHONPATH="$PWD/../src:$PYTHONPATH" \
  python3 src/ultron_b2_perception/tools/eval_oracle_depth.py
```

It compares estimates against synthetic ladder centers only where both methods return a result.
Surface-to-center offsets and rejected detections remain; it is not a real-camera accuracy test.
