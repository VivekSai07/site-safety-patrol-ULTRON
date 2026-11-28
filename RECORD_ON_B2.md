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

## What happened on 28 November

Today's bags (`20261128-14*`) have only camera data. Only the RealSense driver was running, so
there was no lidar, IMU or `/tf`. Last night's hall bag shows the drivers that were running then:
- `rslidar_sdk` publishing `/scan_cloud` in `lidar_link`, plus `/rslidar_packets`
- `xsens_mti_ros2_driver` publishing `/imu/*`, `/filter/*` and `/tf world→imu_link`
- RealSense as `camera_depth/head`

`b2.sh check` now catches a missing sensor before recording starts.
