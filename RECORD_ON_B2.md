# Record data on the real B2: quick start

Commands to record a ROS 2 bag on the robot, with or without the patrol running. No container
is needed: everything runs with the ROS already installed on the robot computer. Options,
costs and measurements: [docs/RECORDING.md](docs/RECORDING.md).

Wi-Fi, SSH and robot passwords are **not** in this repository. Ask the team or the robot operator.

## 0. Get onto the robot network (laptop)

Either option works:

- **Ethernet cable to the robot (preferred).** You can stay on university Wi-Fi at the same
  time, so internet and other sessions keep working.
- **The robot's 5G hotspot `RUT_B1D5_5G`.** This replaces your normal Wi-Fi while connected.

Set the robot connection's IPv4 to manual:

| Field | Value |
|---|---|
| Address | `192.168.123.190`, `.191`, `.192`, … **a different number for each laptop** (two laptops with the same address knock each other off) |
| Netmask | `255.255.255.0` |
| Gateway | **leave empty** (on Ubuntu also tick *Routes → "Use this connection only for resources on its network"*) |

Check: `ping 192.168.123.222` answers. That is the robot computer.

## 1. Copy the code to the robot (laptop, in this repo after `git pull`)

```bash
rsync -a --exclude .git --exclude runs --exclude datasets ./ unitree@192.168.123.222:~/ultron/
```

## 2. Check the sensor topics (on the robot)

```bash
ssh unitree@192.168.123.222
ls /opt/ros                                 # which ROS is installed; use it below
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E "camera|scan|tf"
df -h ~                                     # recording needs about 7 GB per minute
```

Nothing listed? Then the recorder has nothing to record. Possible causes: wrong computer, the
camera and lidar drivers aren't running, or a ROS setup file is missing (e.g.
`source ~/unitree_ros2/setup.sh`, matching `ROS_DOMAIN_ID`). Ask whoever recorded the hall bag
how they started the drivers.

## 3. Start recording (on the robot, inside tmux)

```bash
tmux new -s rec
source /opt/ros/humble/setup.bash
cd ~/ultron
```

Then **one** of:

```bash
# A. record only (no GPU machine needed)
PYTHONPATH=src:$PYTHONPATH python3 -m ultron.recording --out ~/ultron-bags --note "hall run 1"

# B. run the patrol and record at the same time
#    (start the patrol on the GPU machine/laptop first, see README)
python3 scripts/ros_relay.py --to LAPTOP_IP:5600 --record ~/ultron-bags
```

- If the camera topic names differ from `/camera_depth/head/...`, add `--profile all` (A) or
  `--record-profile all` (B) to record every topic.
- Disk tight? Use `--profile light` / `--record-profile light`, which drops raw images
  (about 10 MB/s).
- Recording stops by itself when free disk falls to 10 GB. The relay keeps running.
- `Ctrl-b d` detaches and leaves it running; `tmux attach -t rec` gets back.
- **Ctrl-C stops it** and closes the bag properly.

## 4. Check after 30 seconds (second SSH window on the robot)

```bash
source /opt/ros/humble/setup.bash
ros2 bag info $(ls -d ~/ultron-bags/*/bag | tail -1)
```

Expect about 30 colour images per recorded second. Near zero means the camera topic isn't being
recorded: see step 2, or use `--profile all`. Fewer than expected means the disk is too slow:
use `light`.

## 5. Copy the bags to the laptop

```bash
rsync -a --progress unitree@192.168.123.222:~/ultron-bags/ ./b2-bags/
```

Each run is one folder, `<YYYYmmdd-HHMMSS>/`, holding `bag/` and `recording.json` (what was
recorded, git commit, how it stopped). Use it in the pipeline with `--source bag:b2-bags/<time>/bag`.

If a recorder is ever left running: `pkill -INT -f "ros2 bag record"` (on the robot).

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
which fails offline): `scp yoloe-11l-seg.pt mobileclip_blt.ts unitree@192.168.123.222:~/ultron/`.
Use **yoloe-11l**: on the hall bag, 11m announced a ghost ladder and 11s called the hose cabinet an
extinguisher. On the workstation 11l takes 13 ms a frame; the pipeline needs 4 frames a second.

### 6c. Run (two tmux windows on the robot, in `~/ultron`)

```bash
# window 1: the pipeline (PYTHONUNBUFFERED shows its progress at once)
source /opt/ros/humble/setup.bash
export PYTHONPATH=src:$PYTHONPATH PYTHONUNBUFFERED=1 OPENAI_API_KEY=...   # any value if offline
~/ultron-venv/bin/python -m ultron.pipeline survey --source relay:5600 --detector yolo:yoloe-11l-seg.pt \
    --interval 0 --sites ~/ultron-runs/sites --site ctrl-hall --map-id hall-start-A --data ~/ultron-runs/patrol-1
# window 2: the relay, once window 1 says "Waiting for the relay"
source /opt/ros/humble/setup.bash
PYTHONPATH=src:$PYTHONPATH python3 scripts/ros_relay.py --to 127.0.0.1:5600 --rate 8 [--record ~/ultron-bags]
```

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
rsync -a unitree@192.168.123.222:~/ultron-runs/ ./ultron-runs/
python -m ultron.pipeline inspect --navigator passive --sites ./ultron-runs/sites --site ctrl-hall \
    --map-id hall-start-A --data ./ultron-runs/patrol-1        # with OPENAI_API_KEY set
```

This first lets the vision model confirm each object, then reads every sheet from the saved
keyframes. Tested on the stand-in's output: 5 confirmed, 5 inspected.
