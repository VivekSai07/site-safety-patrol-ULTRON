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
