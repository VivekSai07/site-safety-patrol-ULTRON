"""ultron.recording without ROS: a stub `ros2` stands in for `ros2 bag record`."""
import json
import os
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from ultron import recording
from ultron.recording import BagRecorder, build_command, qos_overrides

# Behaves like `ros2 bag record -o DIR ...`: creates DIR, writes metadata.yaml on SIGINT.
STUB = """#!/usr/bin/env python3
import signal, sys, time
from pathlib import Path
args = sys.argv[1:]
out = Path(args[args.index("-o") + 1])
mode = "{mode}"
if out.exists():
    sys.exit("output folder exists")
out.mkdir(parents=True)
(out / "args.txt").write_text(" ".join(args))
if mode == "crash":
    sys.exit(3)
def done(*_):
    if mode != "hang":
        (out / "metadata.yaml").write_text("rosbag2_bagfile_information: {{}}\\n")
        sys.exit(0)
if signal.getsignal(signal.SIGINT) is not signal.SIG_IGN:  # like the real recorder: an inherited
    signal.signal(signal.SIGINT, done)                       # "ignore" leaves it deaf to Ctrl-C
if mode == "hang":
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
while True:
    (out / "data_0.db3").open("ab").write(b"x" * 1000)
    time.sleep(0.05)
"""


class RecordingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.bin = self.tmp / "bin"
        self.bin.mkdir()
        self.path = mock.patch.dict(os.environ, {"PATH": f"{self.bin}:{os.environ['PATH']}"})
        self.path.start()
        self.addCleanup(self.path.stop)

    def stub(self, mode="ok"):
        ros2 = self.bin / "ros2"
        ros2.write_text(STUB.format(mode=mode))
        ros2.chmod(ros2.stat().st_mode | stat.S_IEXEC)

    def recorder(self, **kwargs):
        kwargs = {"min_free_gb": 0.0, "check_every": 0.1, "prefix": [], **kwargs}
        return BagRecorder(self.tmp / "bags", **kwargs)

    def test_command_uses_flags_common_to_humble_and_jazzy(self):
        command = build_command("/x/bag", "pipeline", ["/extra"], None, 2.0, "/x/qos.yaml", ["nice"])
        self.assertEqual(command[:6], ["nice", "ros2", "bag", "record", "-o", "/x/bag"])
        self.assertIn("-b", command)
        self.assertEqual(command[command.index("-b") + 1], "2000000000")
        self.assertNotIn("-s", command)  # distro default storage
        self.assertNotIn("--topics", command)  # Jazzy-only spelling
        for topic in (recording.COLOR, recording.DEPTH, recording.CLOUD, "/tf", "/tf_static", "/extra"):
            self.assertIn(topic, command)
        self.assertEqual(len(command), len(set(command)))
        self.assertIn("-a", build_command("/x", "all"))
        self.assertNotIn(recording.COLOR, build_command("/x", "light"))
        with self.assertRaises(ValueError):
            build_command("/x", "nope")

    def test_qos_overrides_make_heavy_topics_best_effort(self):
        text = qos_overrides()
        self.assertIn(f"{recording.COLOR}:\n", text)
        self.assertEqual(text.count("best_effort"), len(recording.HEAVY))
        self.assertNotIn("/tf_static", text)  # latched topic keeps the publisher's QoS

    def test_records_and_closes_bag_on_stop(self):
        self.stub()
        recorder = self.recorder(note="hall test")
        session = recorder.start()
        self.assertIsNotNone(session)
        time.sleep(0.4)
        self.assertTrue(recorder.running)
        recorder.stop()
        self.assertFalse(recorder.running)
        info = json.loads((session / "recording.json").read_text())
        self.assertEqual(info["exit_code"], 0)
        self.assertTrue(info["metadata_written"])
        self.assertGreater(info["bag_bytes"], 0)
        self.assertEqual(info["note"], "hall test")
        self.assertNotIn("--qos-profile-overrides-path", (session / "bag" / "args.txt").read_text())
        self.assertIsNone(recorder.stop() and None)  # second stop is harmless

    def test_best_effort_writes_qos_overrides(self):
        self.stub()
        recorder = self.recorder(best_effort=True)
        session = recorder.start()
        time.sleep(0.3)
        recorder.stop()
        self.assertIn("--qos-profile-overrides-path", (session / "bag" / "args.txt").read_text())
        self.assertIn("best_effort", (session / "qos.yaml").read_text())

    def test_stops_on_sigint_even_if_parent_ignores_it(self):
        # `relay &` in a script: SIGINT is ignored and would be inherited by the recorder.
        self.stub()
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            recorder = self.recorder()
            session = recorder.start()
        finally:
            signal.signal(signal.SIGINT, previous)
        time.sleep(0.3)
        recorder.stop(timeout=3.0)
        info = json.loads((session / "recording.json").read_text())
        self.assertEqual(info["stop_reason"], "stopped")
        self.assertTrue(info["metadata_written"])

    def test_two_sessions_never_share_a_folder(self):
        self.stub()
        first, second = self.recorder(), self.recorder()
        a, b = first.start(), second.start()
        first.stop(), second.stop()
        self.assertNotEqual(a, b)

    def test_missing_ros2_only_warns(self):
        with mock.patch.object(recording.shutil, "which", return_value=None):
            with self.assertLogs("ultron.recording", "WARNING"):
                self.assertIsNone(self.recorder().start())

    def test_too_little_disk_does_not_start(self):
        self.stub()
        with self.assertLogs("ultron.recording", "WARNING"):
            self.assertIsNone(self.recorder(min_free_gb=1e9).start())

    def test_disk_floor_stops_recording(self):
        self.stub()
        recorder = self.recorder()
        session = recorder.start()
        recorder.min_free_gb = 1e9  # disk "fills up"
        deadline = time.time() + 5
        while recorder.running and time.time() < deadline:
            time.sleep(0.05)
        self.assertFalse(recorder.running)
        time.sleep(0.1)
        info = json.loads((session / "recording.json").read_text())
        self.assertEqual(info["stop_reason"], "disk floor")
        self.assertTrue(info["metadata_written"])

    def test_recorder_crash_is_reported_not_raised(self):
        self.stub("crash")
        recorder = self.recorder()
        session = recorder.start()
        time.sleep(0.6)
        info = json.loads((session / "recording.json").read_text())
        self.assertEqual(info["exit_code"], 3)
        self.assertEqual(info["stop_reason"], "recorder exited")
        recorder.stop()  # still harmless

    def test_hung_recorder_is_killed(self):
        self.stub("hang")
        recorder = self.recorder()
        session = recorder.start()
        time.sleep(0.3)
        recorder.stop(timeout=0.5)
        self.assertFalse(recorder.running)
        info = json.loads((session / "recording.json").read_text())
        self.assertIn("SIGINT", info["stop_reason"])
        self.assertFalse(info["metadata_written"])

    def test_cli_exits_cleanly_on_sigterm(self):
        self.stub()
        env = {**os.environ, "PYTHONPATH": str(Path(__file__).resolve().parents[1] / "src")}
        cli = subprocess.Popen([sys.executable, "-m", "ultron.recording", "--out", str(self.tmp / "cli"),
                                "--min-free-gb", "0"], env=env)
        time.sleep(1.5)
        cli.terminate()
        self.assertEqual(cli.wait(30), 0)
        (session,) = (self.tmp / "cli").iterdir()
        self.assertTrue((session / "bag" / "metadata.yaml").exists())


if __name__ == "__main__":
    unittest.main()
