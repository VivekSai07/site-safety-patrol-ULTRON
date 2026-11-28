"""Record a ROS 2 bag on the robot while the pipeline runs. Standard library only.

`scripts/ros_relay.py --record DIR` starts this next to the relay, so each deployment also
leaves a bag behind. On its own, in a shell with ROS sourced:

    PYTHONPATH=src:$PYTHONPATH python3 -m ultron.recording --out /media/ssd/ultron-bags

The bag is written by `ros2 bag record` in a child process, not by Python callbacks: it stores
the serialized messages without decoding them and never competes with the relay for the GIL.
The child runs under `nice`/`ionice`, subscribes with the publishers' own (reliable) QoS, and
compresses nothing (zstd at these rates would cost the robot a CPU core or more). Reliable is
the default because best-effort lost data: replaying the hall recording, a best-effort recorder
kept 52-68 % of the colour frames, a reliable one all 1090 and every depth image and cloud,
with no measurable change in the relay's frame rate or CPU. `best_effort=True` remains as an
option for a robot whose drivers stall behind a slow recorder.

Failures stay on this side: no `ros2` on PATH, too little disk or a recorder that dies only
log a warning; the relay keeps running. Recording stops by itself when free space falls below
`min_free_gb`. Each session gets its own folder:

    DIR/<YYYYmmdd-HHMMSS>/bag/          the rosbag2 folder (metadata.yaml + .db3 or .mcap)
    DIR/<YYYYmmdd-HHMMSS>/recording.json command, profile, git commit, start/stop, exit code, size
    DIR/<YYYYmmdd-HHMMSS>/recorder.log  the recorder's own output
    DIR/<YYYYmmdd-HHMMSS>/qos.yaml      only with best_effort: overrides for the heavy topics

Measured on the hall recording: the full camera stream is about 115 MB/s (7 GB a minute),
almost all of it raw colour and depth. Choose the profile and the disk accordingly.

Flags used are the ones `ros2 bag record` has in both Humble and Jazzy. The storage format is
left to the distro's default (Humble: sqlite3, Jazzy: mcap) unless `storage` is given; both are
read by `ultron.protocol.bag` through `rosbags`.
"""
import argparse
import ctypes
import datetime as _dt
import json
import logging
import os
import shutil
import signal
import socket
import subprocess
import threading
import time
from pathlib import Path

log = logging.getLogger("ultron.recording")

HEAD = "/camera_depth/head"
COLOR, COLOR_INFO = f"{HEAD}/color/image_raw", f"{HEAD}/color/camera_info"
DEPTH, DEPTH_INFO = f"{HEAD}/depth/image_rect_raw", f"{HEAD}/depth/camera_info"
EXTRINSICS = f"{HEAD}/extrinsics/depth_to_color"
CLOUD = "/scan_cloud"

# Small topics that make a recording usable later: frames, IMU, robot state, localisation and
# navigation output when those run. Topics that never appear are simply not recorded.
CONTEXT = [
    "/tf", "/tf_static", "/imu/data", "/status", "/rosout",
    "/ultron/real/odom", "/ultron/real/scan", "/ultron/real/localization_state", "/scan",
    "/odom", "/map", "/map_metadata", "/amcl_pose", "/pose", "/localization_pose",
    "/goal_pose", "/plan", "/cmd_vel", "/joint_states",
    "/sportmodestate", "/lf/sportmodestate", "/lowstate", "/lf/lowstate",
]
HEAVY = [COLOR, DEPTH, CLOUD, "/rslidar_points", "/rslidar_packets"]  # best-effort only if asked

PROFILES = {
    # What the pipeline reads (protocol/bag.py) plus pose sources: the default.
    "pipeline": [COLOR, COLOR_INFO, DEPTH, DEPTH_INFO, EXTRINSICS, CLOUD, "/rslidar_points", *CONTEXT],
    # Same without raw images: about 10 MB/s, for long runs or a slow disk.
    "light": [COLOR_INFO, DEPTH_INFO, EXTRINSICS, CLOUD, "/rslidar_points", *CONTEXT],
    # Everything published, including raw lidar packets and camera metadata.
    "all": None,
}


def default_command_prefix():
    """`nice`/`ionice` in front of the recorder where available, so the robot stack goes first."""
    prefix = []
    if shutil.which("ionice"):
        prefix += ["ionice", "-c2", "-n7"]
    if shutil.which("nice"):
        prefix += ["nice", "-n", "10"]
    return prefix


def build_command(bag_dir, profile="pipeline", topics=(), storage=None, max_bag_gb=4.0,
                  qos_path=None, prefix=()):
    """Argument list for `ros2 bag record`. Topics are positional: accepted by Humble and Jazzy."""
    if profile not in PROFILES:
        raise ValueError(f"unknown profile {profile!r}; choose from {', '.join(PROFILES)}")
    command = [*prefix, "ros2", "bag", "record", "-o", str(bag_dir)]
    if storage:
        command += ["-s", storage]
    if max_bag_gb:
        command += ["-b", str(int(max_bag_gb * 1e9))]
    if qos_path:
        command += ["--qos-profile-overrides-path", str(qos_path)]
    wanted = PROFILES[profile]
    if wanted is None:
        command.append("-a")
    else:
        command += list(dict.fromkeys([*wanted, *topics]))
    return command


def qos_overrides(topics=HEAVY):
    """YAML text: best-effort, volatile, keep-last subscriptions for the raw sensor topics."""
    return "".join(f"{topic}:\n  history: keep_last\n  depth: 10\n  reliability: best_effort\n"
                   f"  durability: volatile\n" for topic in topics)


def _child_setup():
    """In the child, before exec: make SIGINT work and arrive if the relay dies without stopping us.

    A process started in the background of a non-interactive shell (`relay &`, nohup) inherits
    SIGINT as ignored; the recorder would then only close the bag after the SIGTERM fallback.
    """
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGINT)  # PR_SET_PDEATHSIG
    except (OSError, AttributeError):
        pass


def _git_commit():
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], cwd=Path(__file__).resolve().parent,
                              capture_output=True, text=True, timeout=2).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def _size(path):
    return sum(f.stat().st_size for f in Path(path).rglob("*") if f.is_file()) if Path(path).exists() else 0


class BagRecorder:
    """One `ros2 bag record` child per session. `start()` and `stop()` never raise."""

    def __init__(self, root, profile="pipeline", topics=(), storage=None, max_bag_gb=4.0,
                 min_free_gb=10.0, check_every=5.0, best_effort=False, prefix=None, note=None):
        self.root, self.profile, self.topics, self.storage = Path(root), profile, list(topics), storage
        self.max_bag_gb, self.min_free_gb, self.check_every = max_bag_gb, min_free_gb, check_every
        self.best_effort, self.note = best_effort, note
        self.prefix = default_command_prefix() if prefix is None else list(prefix)
        self.process = self.session = self.info = None
        self._log_file = self._watcher = None
        self._lock = threading.Lock()
        self._stopped = threading.Event()

    @property
    def running(self):
        return self.process is not None and self.process.poll() is None

    def free_gb(self):
        return shutil.disk_usage(self.root).free / 1e9

    def start(self):
        """Start recording; returns the session folder, or None (with a warning) if it cannot."""
        try:
            return self._start()
        except Exception as error:  # never take the deployment down with us
            log.warning("recording not started: %s", error)
            return None

    def _start(self):
        if shutil.which("ros2") is None:
            log.warning("recording not started: no `ros2` on PATH (source ROS first)")
            return None
        self.root.mkdir(parents=True, exist_ok=True)
        free = self.free_gb()
        if free < self.min_free_gb:
            log.warning("recording not started: %.1f GB free in %s, need %.1f", free, self.root, self.min_free_gb)
            return None
        stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        session = self.root / stamp
        suffix = 1
        while session.exists():
            session, suffix = self.root / f"{stamp}-{suffix}", suffix + 1
        session.mkdir()
        qos_path = None
        if self.best_effort and self.profile != "light":
            qos_path = session / "qos.yaml"
            qos_path.write_text(qos_overrides())
        command = build_command(session / "bag", self.profile, self.topics, self.storage,
                                self.max_bag_gb, qos_path, self.prefix)
        self._log_file = open(session / "recorder.log", "ab")
        self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=self._log_file,
                                        stderr=subprocess.STDOUT, start_new_session=True,
                                        preexec_fn=_child_setup)
        self.session = session
        self.info = {"schema": "ultron.recording/v1", "host": socket.gethostname(),
                     "started": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
                     "profile": self.profile, "command": command, "git_commit": _git_commit(),
                     "free_gb_at_start": round(free, 1), "min_free_gb": self.min_free_gb,
                     "note": self.note, "pid": self.process.pid}
        self._write_info()
        self._stopped.clear()
        self._watcher = threading.Thread(target=self._watch, name="bag-watch", daemon=True)
        self._watcher.start()
        log.info("recording %s profile to %s (%.0f GB free)", self.profile, session / "bag", free)
        return session

    def _write_info(self):
        try:
            (self.session / "recording.json").write_text(json.dumps(self.info, indent=2) + "\n")
        except OSError as error:
            log.warning("could not write recording.json: %s", error)

    def _watch(self):
        while not self._stopped.wait(self.check_every):
            if self.process is None:
                return
            code = self.process.poll()
            if code is not None:
                log.warning("recorder exited by itself (code %s); see %s", code, self.session / "recorder.log")
                self._finish(code, "recorder exited")
                return
            try:
                free = self.free_gb()
            except OSError:
                continue
            if free < self.min_free_gb:
                log.warning("stopping recording: %.1f GB free, floor %.1f GB", free, self.min_free_gb)
                self.stop(reason="disk floor")
                return

    def stop(self, timeout=20.0, reason="stopped"):
        """SIGINT so the recorder closes the bag and writes metadata.yaml; escalate if it hangs."""
        with self._lock:
            process = self.process
            if process is None:
                return None
            self._stopped.set()
            code = process.poll()
            try:
                if code is None:
                    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
                        try:
                            os.killpg(process.pid, sig)
                        except ProcessLookupError:
                            break
                        try:
                            code = process.wait(wait)
                            break
                        except subprocess.TimeoutExpired:
                            log.warning("recorder did not stop on %s", signal.Signals(sig).name)
                            reason = f"{reason}; needed {signal.Signals(sig).name}"
                code = process.poll() if code is None else code
            except Exception as error:
                log.warning("stopping recorder: %s", error)
            self._finish(code, reason)
            return self.session

    def _finish(self, code, reason):
        if self.info is None or "stopped" in self.info:
            return
        bag = self.session / "bag"
        self.info.update({"stopped": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
                          "stop_reason": reason, "exit_code": code, "bag_bytes": _size(bag),
                          "metadata_written": (bag / "metadata.yaml").exists()})
        self._write_info()
        if self._log_file:
            self._log_file.close()
        log.info("recording closed: %s, %.2f GB, metadata %s", bag, self.info["bag_bytes"] / 1e9,
                 "written" if self.info["metadata_written"] else "MISSING")


def add_arguments(parser, prefix=""):
    """Recorder options, shared by this module's CLI and `scripts/ros_relay.py --record`."""
    p = f"--{prefix}" if prefix else "--"
    parser.add_argument(f"{p}profile", default="pipeline", choices=sorted(PROFILES),
                        help="pipeline: camera, depth, lidar, TF, IMU, pose (~115 MB/s); light: no raw "
                             "images; all: every topic")
    parser.add_argument(f"{p}topics", nargs="*", default=[], help="extra topics to record")
    parser.add_argument(f"{p}storage", default=None, choices=["sqlite3", "mcap"],
                        help="bag format (default: the distro's own)")
    parser.add_argument(f"{p}max-bag-gb", type=float, default=4.0, help="split the bag into files of this size")
    parser.add_argument(f"{p}min-free-gb", type=float, default=10.0,
                        help="do not start below, and stop recording at, this much free disk")
    parser.add_argument(f"{p}best-effort", action="store_true",
                        help="subscribe to images and clouds best-effort (loses frames; only if the "
                             "drivers stall behind the recorder)")
    parser.add_argument(f"{p}note", default=None, help="free text stored in recording.json")


def recorder_from_args(root, args, prefix=""):
    get = lambda name: getattr(args, (prefix + name).replace("-", "_"))
    return BagRecorder(root, profile=get("profile"), topics=get("topics"), storage=get("storage"),
                       max_bag_gb=get("max-bag-gb"), min_free_gb=get("min-free-gb"),
                       best_effort=get("best-effort"), note=get("note"))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Record the B2's topics into a ROS 2 bag until Ctrl-C.")
    parser.add_argument("--out", required=True, help="folder for recording sessions")
    add_arguments(parser)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    recorder = recorder_from_args(args.out, args)
    if recorder.start() is None:
        return 1

    def on_term(*_):
        raise KeyboardInterrupt

    previous = signal.signal(signal.SIGTERM, on_term)
    try:
        while recorder.running:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, previous)
        recorder.stop()
    return 0 if recorder.info and recorder.info.get("metadata_written") else 1


if __name__ == "__main__":
    raise SystemExit(main())
