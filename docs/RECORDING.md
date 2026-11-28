# Recording bags while the patrol runs (`ultron.recording`)

Each deployment on the B2 can also record a ROS 2 bag. Add `--record DIR` to the relay:

```bash
# robot side, ROS sourced; DIR on the robot computer's fastest disk (SSD/NVMe, not an SD card)
python3 scripts/ros_relay.py --to GPU_MACHINE:5600 --record /media/ssd/ultron-bags \
    --record-note "hall patrol, two ladders"
```

Or record without the relay:

```bash
PYTHONPATH=src:$PYTHONPATH python3 -m ultron.recording --out /media/ssd/ultron-bags
```

Ctrl-C stops both, and the bag is closed properly (`metadata.yaml` is written). Each run makes
`DIR/<YYYYmmdd-HHMMSS>/` holding `bag/` (rosbag2), `recording.json` (command, profile, git
commit, host, start and stop, exit code, size), `recorder.log` and, only with `--record-best-effort`,
`qos.yaml`. Read the bag on the GPU machine as usual: `--source bag:DIR/<time>/bag`.

| Option | Default | |
|---|---|---|
| `--record-profile` | `pipeline` | `pipeline`: colour, depth, both camera_infos, extrinsics, `/scan_cloud`, `/tf`, `/tf_static`, `/imu/data`, `/status`, plus odom, map, localisation, goal, plan, `/cmd_vel` and Unitree state if they are published. `light`: the same without raw images. `all`: every topic. |
| `--record-topics T ...` | none | extra topics |
| `--record-max-bag-gb` | 4 | split into files of this size |
| `--record-min-free-gb` | 10 | do not start below this much free disk, and stop recording if free space falls to it (the relay carries on) |
| `--record-storage` | distro default | `sqlite3` (Humble default) or `mcap` (Jazzy default, needs the plugin on Humble) |
| `--record-best-effort` | off | subscribe to images and clouds best-effort. **This loses frames** (see below); use it only if the camera driver stalls behind the recorder. |

Sahil's `/ultron/real/odom`, `/ultron/real/scan` and `/ultron/real/localization_state` are in
`pipeline` and `light`, and `/tf` is in every profile, so once localisation runs the bag records the robot's pose, which the
hall recording did not.

## What it costs

The recorder is `ros2 bag record` in a separate process under `nice -n 10` and `ionice -c2 -n7`.
It writes the serialized messages without decoding or compressing them. The patrol, YOLOE and the
vision model run on the GPU machine and are not affected at all. On the robot side the costs are:

- **Disk:** about **115 MB/s, 7 GB a minute** for the `pipeline` profile (4.27 GB for the 36 s hall
  recording; colour is 70 % of it). A 10-minute patrol is about 70 GB. The disk must sustain this
  write rate. If it cannot, the recorder falls behind and drops messages. The relay should not
  be affected, but that has only been measured on NVMe. `light` is about 10 MB/s.
- **CPU:** about 2.3 s of CPU for a one-minute run, roughly 4 % of one core on the workstation.
- **Memory:** about 95 MB resident.
- **Network:** run the recorder **on the computer that runs the camera driver**. There the extra
  subscriber gets a local copy. On another computer it pulls a second full camera stream over
  the link: about 0.9 Gbit/s, enough to fill a gigabit cable that also carries the relay.

### Measurement

Jazzy container on the workstation (32 cores, NVMe). Each run replayed the 36 s hall recording
with `ros2 bag play`, the relay sent to a TCP sink, and recordings were written to NVMe:

| Run | Relay frames sent | Relay CPU | Recorder CPU | Colour / depth / cloud messages recorded |
|---|---|---|---|---|
| relay only (2 runs) | 267, 233 | 14.0 s, 13.5 s | – | – |
| + recorder, reliable (3 runs, now the default) | 242, 252, 263 | 14.7 s, 14.1 s, 12.8 s | 2.8 s, 2.4 s, 2.3 s | **1090/1090, 1085/1085, 364/364** each run |
| + recorder, best-effort (2 runs) | 225, 248 | 14.0 s, 14.3 s | 2.1 s, 2.1 s | 564–738/1090, 755–939/1085, 212–292/364 |

The relay's frame count varies from 233 to 267 even with no recorder, and the reliable recorder
did not move it out of that range. Best-effort subscriptions lost a third to a half of the camera
frames in this container (Docker's small shared memory may contribute), so reliable is the default.

The workstation is much faster than the B2's onboard computer, so these numbers are a lower bound
on the cost. Test on the robot before relying on them. It takes two minutes:

1. Run the relay for 30 s without `--record` and note its `sent N frames` lines.
2. Run it again with `--record`, watching `top` (CPU) and `iostat -x 1` or `iotop` (disk).
3. Run `ros2 bag info DIR/<time>/bag` and compare the colour count against 30 Hz × duration.

If frames are missing, the disk is too slow. Write to a faster disk or use `--record-profile light`.

## Not verified

- Only containers have been tested: the full replay A/B in Jazzy, and a Humble (Python 3.10)
  smoke test. In Humble the recorder subscribed to a listed topic that appeared after it started,
  closed cleanly on stop and wrote sqlite3. Nothing has run on the B2 or its computer yet.
- If the relay is killed with SIGKILL, the recorder is sent SIGINT through Linux's parent-death
  signal. This path is not tested. If a recorder is ever left running, stop it with
  `pkill -INT -f "ros2 bag record"`.
- Unitree state topics (`/sportmodestate`, `/lowstate`) are recorded only if their message types
  are installed where the recorder runs.
- rosbag2 publishes `/events/write_split` when a file is split. The recorder publishes nothing
  else and sends no commands.
