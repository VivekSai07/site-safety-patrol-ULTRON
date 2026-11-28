# Record data on the real B2: quick start

Everything goes through one script on the robot computer, `scripts/b2.sh`. It sources ROS
itself, checks that the sensors are live before recording, and runs the recording in tmux so an
SSH drop doesn't stop it. No container, no `PYTHONPATH`, no `.bashrc` changes. Options and costs:
[docs/RECORDING.md](docs/RECORDING.md).

Robot computer: **`byte`, `icom@192.168.123.222`**. Wi-Fi and SSH passwords are **not** in this
repository; ask the team.

## 0. Get onto the robot network (laptop)

- **Ethernet cable to the robot (preferred)**, keeping university Wi-Fi for internet, or
- **the robot's 5G hotspot `RUT_B1D5_5G`** (replaces your normal Wi-Fi while connected).

IPv4 manual: address `192.168.123.190`, `.191`, `.192`, … **a different number for each
laptop**. Netmask `255.255.255.0`. **Gateway empty** (on Ubuntu tick *Routes → "Use this
connection only for resources on its network"*). Check: `ping 192.168.123.222`.

## 1. Copy the code to the robot (laptop, in this repo after `git pull`)

```bash
rsync -a --exclude .git --exclude runs --exclude datasets ./ icom@192.168.123.222:~/ultron/
ssh icom@192.168.123.222
cd ~/ultron
```

## 2. Check the sensors

```bash
scripts/b2.sh check
```

This prints a table and ends with `READY` or `NOT READY`. Required: **camera** (RealSense
`/camera_depth/head/...`), **lidar** (`/scan_cloud` or `/rslidar_points`) and **IMU**
(`/imu/data`). It also shows `/tf`, Unitree state and odometry when they exist. Each missing
sensor gets a line saying which driver provides it.

- **NOT READY, or almost no topics visible:** run `scripts/b2.sh doctor` and send the saved
  report (`~/ultron-doctor-<time>.txt`) to the team. It records what is installed and running:
  drivers, configs, network, devices, and how drivers were started before. It changes nothing.
- **Don't** change `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` or `.bashrc` to make topics appear.
  The drivers have to be started in the same setup they were configured for.
- Two drivers can't share one sensor. If a driver is already running somewhere, don't start
  a second copy.

## 3. Record

```bash
scripts/b2.sh record                    # refuses unless camera, lidar and IMU are live
scripts/b2.sh record --camera-only      # knowingly record camera data only
scripts/b2.sh record --force            # skip the check, record whatever is there
scripts/b2.sh record --note "hall run 2" --profile all    # extra options go to the recorder
```

Run the patrol and record at the same time (start the patrol on the laptop first, see README):

```bash
scripts/b2.sh relay LAPTOP_IP           # e.g. 192.168.123.191
```

It runs in tmux (`ultron-rec` or `ultron-relay`). **Ctrl-C stops it** and closes the bag
properly. `Ctrl-b d` detaches and leaves it running; `tmux attach -t ultron-rec` gets back.
Bags go to `~/ultron-bags/<YYYYmmdd-HHMMSS>/`, at about 7 GB per minute. Recording stops by
itself at 10 GB free disk.

## 4. Check the bag after 30 seconds (second SSH window)

```bash
source /opt/ros/humble/setup.bash
ros2 bag info $(ls -d ~/ultron-bags/*/bag | tail -1)
```

Expect about 30 colour images and 10 lidar clouds per second.

## 5. Copy the bags to the laptop

```bash
rsync -a --progress icom@192.168.123.222:~/ultron-bags/ ./b2-bags/
```

If a recorder is ever left running: `pkill -INT -f "ros2 bag record"` on the robot.

## 6. Run the pipeline on the robot itself (no GPU machine, no tunnel)

The B2 carries a Jetson Orin. The survey (detector, depth, object map, ids across patrols, moved
objects) runs there; the relay feeds it over localhost. Reading the inspection sheets needs a
vision model: with internet on the robot, set `OPENAI_API_KEY`; without it, read them afterwards on
a laptop (6e).

Tested in a stand-in container (ROS 2 Humble, Python 3.10, **network off**, the hall bag played as
live topics, `scripts/b2_standin/`): 5 objects, 5 found, nothing extra announced, 226 of 232 frames
processed (on an RTX 5090, so the Orin's speed is still unmeasured). Not yet run on the Jetson.

### 6a. Five-minute check (on the robot, read-only)

```bash
cat /etc/nv_tegra_release; free -h; df -h ~     # which Orin / JetPack, memory, disk
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
ping -c1 -W2 pypi.org && echo internet
```

### 6b. Install once (needs internet on the robot, or copy the wheels over)

```bash
cd ~/ultron
python3 -m venv --system-site-packages ~/ultron-venv      # keeps ROS's rclpy visible
~/ultron-venv/bin/pip install -U pip                       # Ubuntu 22.04's pip 22 silently skips the CLIP package
# PyTorch: NVIDIA's wheel for this JetPack (PyPI's torch has no Jetson GPU support), unless 6a showed it works
~/ultron-venv/bin/pip install -e '.[protocol,bags,detector]'
```

From the laptop, copy the detector weights and its text encoder (both are downloaded on first use,
which fails offline): `scp yoloe-11l-seg.pt mobileclip_blt.ts icom@192.168.123.222:~/ultron/`.
Use **yoloe-11l**: on the hall bag, 11m announced a ghost ladder and 11s called the hose cabinet an
extinguisher. On the workstation 11l takes 13 ms a frame; the pipeline needs 4 frames a second.

Faster on the Orin: build a TensorRT FP16 engine **on the robot** (an engine only runs on the GPU
and TensorRT version that built it; JetPack's TensorRT is visible through the venv's system site
packages). The prompts are baked in, so export after any `calibrate`:

```bash
~/ultron-venv/bin/python -m ultron.pipeline export --detector yolo:yoloe-11l-seg.pt --data ~/ultron-runs/patrol-1
# then run the survey with --detector yolo:yoloe-11l-seg.engine
```

On the hall bag the engine gave the same 5 objects, all confirmed, positions within 5 cm of
PyTorch. Workstation timing: model 1.6 ms, whole detector step (tracker, masks) 13 → 7 ms; the
rest is CPU work the engine does not speed up. With JetPack's TensorRT 10 FP16 builds directly;
TensorRT 11 would also need NVIDIA ModelOpt.

### 6c. Run (two tmux windows on the robot, in `~/ultron`)

```bash
# window 1: the pipeline (PYTHONUNBUFFERED shows its progress at once)
source /opt/ros/humble/setup.bash
export PYTHONPATH=src:$PYTHONPATH PYTHONUNBUFFERED=1 OPENAI_API_KEY=...   # any value if offline
~/ultron-venv/bin/python -m ultron.pipeline survey --source relay:5600 --detector yolo:yoloe-11l-seg.pt \
    --interval 0 --sites ~/ultron-runs/sites --site ctrl-hall --map-id hall-start-A --data ~/ultron-runs/patrol-1
# window 2: the relay (checks the sensors first, records too), once window 1 says "Waiting for the relay"
scripts/b2.sh relay 127.0.0.1
```

The survey needs the camera **and the lidar**: without lidar (and without SLAM) there is no pose,
so objects get no position on the floor and nothing can be matched across patrols. `b2.sh relay`
refuses to start when a sensor is missing; don't override it with `--camera-only` for a patrol.

Ctrl-C the relay to end the survey; the pipeline then settles the map (merges, moved objects) and
exits. List what it found:

```bash
~/ultron-venv/bin/python -m ultron.pipeline status --sites ~/ultron-runs/sites --site ctrl-hall \
    --map-id hall-start-A --data ~/ultron-runs/patrol-1
```

The object map is kept per site and map id under `~/ultron-runs/sites`, shared by every patrol with
the same `--map-id`; `--data` holds this patrol's events. Without SLAM the pose is lidar odometry from the start point: start every patrol of
a site from the same marked spot and pass the same `--map-id`, so patrols share one object map.

### 6d. Dashboard

Run `python -m ultron.dashboard --port 8010` on the robot and add
`--dashboard http://127.0.0.1:8010` to the survey; open `http://192.168.123.222:8010` on the laptop.

### 6e. Reading the sheets later (no internet on the robot)

```bash
rsync -a icom@192.168.123.222:~/ultron-runs/ ./ultron-runs/
python -m ultron.pipeline inspect --navigator passive --sites ./ultron-runs/sites --site ctrl-hall \
    --map-id hall-start-A --data ./ultron-runs/patrol-1        # with OPENAI_API_KEY set
```

This first lets the vision model confirm each object, then reads every sheet from the saved
keyframes. Tested on the stand-in's output: 5 confirmed, 5 inspected.

## What happened on 28 November

Today's bags (`20261128-14*`) have only camera data. Only the RealSense driver was running, so
there was no lidar, IMU or `/tf`. Last night's hall bag shows the drivers that were running then:
- `rslidar_sdk` publishing `/scan_cloud` in `lidar_link`, plus `/rslidar_packets`
- `xsens_mti_ros2_driver` publishing `/imu/*`, `/filter/*` and `/tf world→imu_link`
- RealSense as `camera_depth/head`

`b2.sh check` now catches a missing sensor before recording starts.
