import copy
import importlib.util
import json
import os
from pathlib import Path
import shutil
import socket
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import uuid


path = Path(__file__).resolve().parents[1] / "scripts" / "teleop_launch.py"
spec = importlib.util.spec_from_file_location("teleop_launch", path)
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


class TeleopLauncherTests(unittest.TestCase):
    def setUp(self):
        self.output = Path(__file__).resolve().parent / f".teleop-launch-test-{uuid.uuid4().hex}"
        self.output.mkdir()
        self.image = "sha256:" + "a" * 64
        self.port = 18765
        self.run_id = "testing123"
        self.name, self.network_name = launcher.resource_names(self.run_id)
        self.labels = launcher.labels(self.run_id, self.image, self.port, self.output)
        self.network = {
            "Id": "network-id",
            "Name": self.network_name,
            "Labels": self.labels.copy(),
            "Driver": "bridge",
            "Internal": True,
            "EnableIPv6": False,
            "Attachable": False,
            "Containers": {"container-id": {}},
            "IPAM": {"Config": [{"Subnet": "172.20.0.0/16"}]},
        }
        bindings = {f"{self.port}/tcp": [{"HostIp": "127.0.0.1", "HostPort": str(self.port)}]}
        self.container = {
            "Id": "container-id",
            "Name": "/" + self.name,
            "Image": self.image,
            "State": {"Running": True},
            "Config": {
                "Labels": self.labels.copy(),
                "User": f"{os.getuid()}:{os.getgid()}",
                "Entrypoint": ["/opt/unitree-venv/bin/python"],
                "Env": [
                    "PYTHONDONTWRITEBYTECODE=1", "PYTHONPATH=/workspace/src",
                    "MUJOCO_GL=egl", "MESA_SHADER_CACHE_DISABLE=true", "HOME=/sessions",
                    "ULTRON_TELEOP_ISOLATED=1",
                ],
                "Cmd": [
                    "-u", "-m", "ultron.teleop", "--kit-root", "/opt/unitree_mujoco",
                    "--output", "/sessions/captures", "--host", "0.0.0.0",
                    "--port", str(self.port), "--width", "1280", "--height", "720",
                ],
            },
            "HostConfig": {
                "NetworkMode": self.network_name,
                "PortBindings": copy.deepcopy(bindings),
                "ReadonlyRootfs": True,
                "IpcMode": "none",
                "Privileged": False,
                "PidMode": "",
                "CapAdd": None,
                "CapDrop": ["ALL"],
                "Devices": [],
                "DeviceRequests": None,
                "SecurityOpt": ["no-new-privileges"],
            },
            "Mounts": [
                {"Type": "bind", "Source": str(launcher.SOURCE), "Destination": "/workspace/src", "RW": False},
                {"Type": "bind", "Source": str(self.output), "Destination": "/sessions", "RW": True},
            ],
            "NetworkSettings": {
                "Networks": {self.network_name: {"NetworkID": "network-id", "IPAddress": "172.20.0.2"}},
                "Ports": copy.deepcopy(bindings),
            },
        }

    def tearDown(self):
        shutil.rmtree(self.output, ignore_errors=True)

    def verify(self, container=None, **options):
        launcher.verify_container(
            container or self.container,
            name=self.name, network_name=self.network_name, network_id="network-id",
            image_id=self.image, port=self.port, output=self.output, expected=self.labels,
            require_running=True, **options,
        )

    def test_accepts_exact_local_only_isolation(self):
        launcher.verify_network(
            self.network, name=self.network_name, expected=self.labels, container_id="container-id"
        )
        self.verify()
        self.assertEqual(
            launcher._container_ip(self.container, self.network, self.network_name),
            "172.20.0.2",
        )

    def test_refuses_foreign_network_and_public_or_privileged_container(self):
        cases = (
            ("public_port", lambda c: c["HostConfig"]["PortBindings"][f"{self.port}/tcp"][0].update(HostIp="0.0.0.0")),
            ("host_network", lambda c: c["HostConfig"].update(NetworkMode="host")),
            ("capabilities", lambda c: c["HostConfig"].update(CapAdd=["SYS_ADMIN"])),
            ("device", lambda c: c["HostConfig"].update(Devices=[{"PathOnHost": "/dev/something"}])),
            ("device_cgroup", lambda c: c["HostConfig"].update(DeviceCgroupRules=["a *:* rwm"])),
            ("disabled_privilege_guard", lambda c: c["HostConfig"].update(SecurityOpt=["no-new-privileges:false"])),
            ("altered_pythonpath", lambda c: c["Config"]["Env"].append("PYTHONPATH=/elsewhere")),
            ("writable_source", lambda c: c["Mounts"][0].update(RW=True)),
            ("extra_mount", lambda c: c["Mounts"].append({"Destination": "/other"})),
            ("image_mismatch", lambda c: c.update(Image="sha256:" + "b" * 64)),
            ("foreign_label", lambda c: c["Config"]["Labels"].update({launcher.LABEL_PREFIX + "owner": "other"})),
            ("extra_network", lambda c: c["NetworkSettings"]["Networks"].update(other={"NetworkID": "other"})),
            ("runtime_public", lambda c: c["NetworkSettings"]["Ports"][f"{self.port}/tcp"][0].update(HostIp="0.0.0.0")),
        )
        for name, modify in cases:
            container = copy.deepcopy(self.container)
            modify(container)
            with self.subTest(case=name), self.assertRaises(ValueError):
                self.verify(container)
        foreign = copy.deepcopy(self.network)
        foreign["Containers"]["foreign-container"] = {}
        with self.assertRaisesRegex(ValueError, "foreign"):
            launcher.verify_network(foreign, name=self.network_name, expected=self.labels, container_id="container-id")

    def test_unpublished_port_requires_explicit_relay_and_private_target(self):
        container = copy.deepcopy(self.container)
        container["NetworkSettings"]["Ports"] = {"6081/tcp": None}
        with self.assertRaisesRegex(ValueError, "publication"):
            self.verify(container)
        self.verify(container, allow_unpublished=True)
        container["NetworkSettings"]["Networks"][self.network_name]["IPAddress"] = "8.8.8.8"
        with self.assertRaisesRegex(ValueError, "private"):
            launcher._container_ip(container, self.network, self.network_name)

    def test_rejects_retagged_or_volume_declaring_image(self):
        for image in (
            {"Id": "sha256:" + "b" * 64, "Config": {}},
            {"Id": self.image, "Config": {"Volumes": {"/extra": {}}}},
        ):
            with patch.object(launcher, "inspect", return_value=image), self.assertRaises(ValueError):
                launcher.verify_image("tag", self.image)

    def test_output_must_be_existing_user_owned_and_not_symlink(self):
        self.assertEqual(launcher.validate_output(self.output), self.output)
        with self.assertRaises(FileNotFoundError):
            launcher.validate_output(self.output / "missing")
        link = self.output / "link"
        link.symlink_to(self.output)
        with self.assertRaisesRegex(ValueError, "symlink"):
            launcher.validate_output(link)
        with self.assertRaisesRegex(ValueError, "source mount"):
            launcher.validate_output(launcher.SOURCE)

    def test_relay_identity_is_checked_before_signalling(self):
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        args = SimpleNamespace(run_id=self.run_id, image_id=self.image, port=port)
        target = "172.20.0.2"
        launcher.start_relay(args, self.output, target)
        try:
            state = launcher.verify_relay(args, self.output, target)
            state_file = launcher.relay_state_path(self.output, self.run_id)
            original = state_file.read_text()
            state["start_time"] = "incorrect"
            state_file.write_text(json.dumps(state))
            with self.assertRaisesRegex(ValueError, "identity"):
                launcher.stop_relay(args, self.output, target)
            state_file.write_text(original)
        finally:
            launcher.stop_relay(args, self.output, target)
        self.assertFalse(state_file.exists())

    def test_dead_owned_relay_state_can_be_cleaned_without_signalling_another_process(self):
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        args = SimpleNamespace(run_id=self.run_id, image_id=self.image, port=port)
        target = "172.20.0.2"
        state = launcher.start_relay(args, self.output, target)
        child = launcher._RELAY_CHILDREN[state["pid"]]
        child.terminate()
        child.wait(timeout=5)
        with self.assertRaisesRegex(ValueError, "no longer running"):
            launcher.verify_relay(args, self.output, target)
        launcher.stop_relay(args, self.output, target)
        self.assertFalse(launcher.relay_state_path(self.output, self.run_id).exists())


if __name__ == "__main__":
    unittest.main()
