# Real-bag replay: local integration observations

2026-11-28. Replay only, base `3a4688ed60f9d5ba50254bf2470c68759a2210f8`.
No hardware connection, autonomous deployment, simulation acceptance, ground-truth
accuracy or hardware acceptance is claimed. Map geometry and calibration remain
unverified; localization thresholds and real sensor extrinsics are proposed.
The bag, derived maps and logs remain local and uncommitted.

## Reproduce mapping

With the specified private bag already present, build in the existing dev image.
The private network namespace has no network interface other than loopback; the
bag mount is read-only. Choose a new output directory for every run. No dependencies
were installed.

```bash
mkdir -p ros2_ws/.dev_home/replay-tmp
export BAG=/home/azut25r/workspace/bags/rosbag2_2026_11_28-00_48_42
export ULTRON_MODE=replay ULTRON_DEV_NET=none ULTRON_BAG_DIR="$BAG"
export ULTRON_DEV_ARGS="--cpus=4 --memory=8g -e TMPDIR=$PWD/ros2_ws/.dev_home/replay-tmp"
ros2_ws/scripts/dev.sh bash -lc \
  'source /opt/ros/humble/setup.bash; colcon build --symlink-install --parallel-workers 2'
ros2_ws/scripts/dev.sh bash -lc \
  'EXTRA_ARGS="replay_cameras:=true replay_delay:=12 replay_rate:=0.7" BAG_TIMEOUT_S=130 scripts/accept/replay_mapping.sh runs/NEW_OUTPUT /home/azut25r/workspace/bags/rosbag2_2026_11_28-00_48_42 mola'
```

The script refuses non-72 domains, non-localhost ROS, a non-read-only bag mount,
or an existing output directory. It samples graph names and TF diagnostics, probes
Foxglove, and saves the map. A zero exit means its **limited checks** succeeded,
not that the map or localization meets an accuracy threshold. Graph polling cannot
prove no short-lived endpoint or request ever existed. Inspect `launch.log` for
loss and `summary.json` for state transitions.

To relaunch against a saved map, use the same container settings and set
`ULTRON_MAP_ROOT` to the mapping run's `maps` directory inside the container:

```bash
ros2_ws/scripts/dev.sh bash -lc 'source /opt/ros/humble/setup.bash; source install/setup.bash;
export ULTRON_MAP_ROOT="$PWD/runs/replay_accept_final/maps";
scripts/ultron_launch.sh replay mode:=localization odom_source:=mola viz:=none map:=replay-mola:1 replay_cameras:=false replay_delay:=12 replay_rate:=0.7 bag:=/home/azut25r/workspace/bags/rosbag2_2026_11_28-00_48_42'
```

In the recorded reload run, the recorder and graph audit ran alongside that
launch; `/ultron/replay/map/load` was called after playback, before shutdown.
Humble's deserialize response has no success field, so service completion alone
does not prove correct localization.

## Observed

Local artifacts are in `ros2_ws/runs/replay_accept_final/`,
`ros2_ws/runs/replay_accept_final_reload/`, and
`ros2_ws/runs/replay_accept_no_backend/`; all are ignored, not committed.
Bag: 36.556 s, 364 recorded point clouds, 3,645 IMU messages; recorded raw lidar
packets were incomplete (~58%), so `/scan_cloud` is the input, not packet decoding.
No `/clock` was recorded: `ros2 bag play --clock` generates it. Recorded dynamic
`world -> imu_link` is remapped off `/tf` to `/ultron/replay/bag/tf_xsens`;
recorded RealSense `/tf_static` is passed through. Static robot mounts are proposed.

| Observation | Result |
|---|---|
| Mapping, 0.7x, cameras on | Player finished; recorder saw 363 scans, 363 odom messages, 359 map-frame poses. Odom path 15.851 m; no ground truth exists to assess error. |
| Map | Save succeeded; 415×445 at 0.05 m, 4,716 occupied, 63,755 free, 116,204 unknown cells. Metadata hash verification returned no errors. Unknown remains unknown on load. |
| Mapping monitor | 278 VALID, 77 DEGRADED, 24 INITIALIZING messages; last VALID. These are message counts, not time-weighted coverage or independent accuracy. |
| Reload, 0.7x, cameras off | Fresh localization launch loaded the pose graph. Recorder saw 364 scans, 364 odom messages, 359 map-frame poses; 242 VALID, 97 DEGRADED, 16 LOST, 24 INITIALIZING messages, last VALID. `map/load` returned success and the backend logged deserialization. |
| Missing backend | `map/load` now returns failure rather than announcing success when the deserialize service is absent. See `replay_accept_no_backend/load.log`. |
| TF | Mapping and reload diagnostics saw 14 children with one authority each and no unknown GIDs. `map -> odom` is slam_toolbox; `odom -> base_link` is MOLA's planarizer. |
| Replay graph | 122 mapping and 121 reload samples saw no cmd_vel, `/api`, or control topic/service. No robot network is reachable from these containers. This is sampled evidence, not proof of no transient messages. |
| Foxglove | 10 s subscribed probe: 11 map messages, 17 throttled clouds, 41 compressed camera frames, 70 scans, 70 odometry and 71 localization messages; 5.907 Mbit/s. Advertised topics matched the whitelist; no services/client-publish capability observed. |

`launch.log` reports slam_toolbox TF message-filter queue drops in both modes;
the log is throttled and does **not** establish an exact number. The camera-on
mapping recorder saw one fewer scan than the bag's 364 clouds; recorder counts and
its cloud/odom matching heuristic do not isolate where a loss happened. At 1x an
earlier run showed MOLA worker-busy drops (last reported 3.91%); using 0.7x
reduced resource pressure but is not a real-time throughput guarantee. The bag
offers reliable lidar/IMU and transient-local camera/static TF; the scan publisher
is reliable, while slam_toolbox and the recorder request best effort. Foxglove
limits some history depths to 5. See `graph_publishers.txt` and `launch.log`.
The scan's 0.35–12 m range differs from slam_toolbox's configured 0.3–20 m and
the backend emits range warnings; these are not calibrated field settings.

The final state is not a live freshness guarantee: after player exit `/clock`
stops, so ROS-time timers and age checks stop advancing. Localization was LOST
during reload and the queue drops, proposed extrinsics, unverified geometry and
no independent ground truth prevent claiming full replay mapping/localization
acceptance. Investigate timestamp/TF filter loss, clock-stop handling and
localization robustness; measure mounts and evaluate against independent truth.

## Configuration provenance

The image ID is `sha256:05db5c29d8c035dcfd4852bd1e49760c085bd64eb5ef5b803d59586585b53d91`.
Relevant SHA-256 values (files relative to `ros2_ws/src/`):

- `ultron_b2_localization/config/mola_lo.yaml`: `befe71910eb1f3efc3e4b83e91f169bf112a81d71581e93a29c1d5b1cef66457`
- `ultron_b2_localization/config/slam_toolbox_mapping.yaml`: `5622fd9df6a078e9955e627b5444eb5762c67fb267e176ab615fb7a5120848c0`
- `ultron_b2_localization/config/slam_toolbox_localization.yaml`: `a8cffa99d61e0aae198ae3dffc821a8892dec426a148733a382ba3da9f9038e3`
- `ultron_b2_localization/config/localization_monitor.yaml`: `0355603fe23c65312c5c349d41982ec2fb9678ba3211a06a068d00f9f7bb668a`
- `ultron_b2_preprocess/config/cloud_filter_real.yaml`: `c3795dbf83a549ab15862270e982b9ec13916d09d4a7a6952f6b8d8cc3d34508`
- `ultron_b2_preprocess/config/cloud_to_scan_real.yaml`: `3086c4738f82b1391c7a5f31524de42ed9a50d32e38eeade2d61c71be03e60bc`
- `ultron_b2_description/calibration/real.yaml`: `5bcc65876cc1da2e4529be89bd231c4d28f6339fd375f48e71bdfa03e23b31e5`
