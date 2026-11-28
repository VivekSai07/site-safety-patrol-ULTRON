import copy
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch


path = Path(__file__).resolve().parents[1] / "scripts" / "organizer_b2.py"
spec = importlib.util.spec_from_file_location("organizer_b2", path)
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)

headless_path = path.with_name("organizer_b2_headless.py")
headless_spec = importlib.util.spec_from_file_location("organizer_b2_headless", headless_path)
headless = importlib.util.module_from_spec(headless_spec)
headless_spec.loader.exec_module(headless)

viewer_path = path.with_name("organizer_b2_viewer.py")
viewer_spec = importlib.util.spec_from_file_location("organizer_b2_viewer", viewer_path)
viewer = importlib.util.module_from_spec(viewer_spec)
viewer_spec.loader.exec_module(viewer)


class OrganizerIsolationTests(unittest.TestCase):
    def setUp(self):
        self.config = {
            "services": {
                "robot": {
                    "image": "test:run",
                    "networks": {"simulation": None},
                    "ports": [{"host_ip": "127.0.0.1", "published": "6081", "target": 6081}],
                    "environment": {"IFACE": "lo", "DOMAIN": "71", "ROBOT": "b2"},
                    "command": ["run-sim"],
                },
                "ros": {
                    "image": "test:run",
                    "network_mode": "service:robot",
                    "environment": {"ROS_DOMAIN_ID": "71", "CYCLONEDDS_URI": "file:///opt/kit/cyclonedds-lo.xml"},
                    "command": ["sleep", "infinity"],
                },
            },
            "networks": {"simulation": {"internal": True}},
        }

    def test_isolated_config_accepted(self):
        launcher.validate_config(self.config, "test:run", 6081)

    def test_rejects_host_namespace_and_exposed_viewer(self):
        for update in ("host_network", "host_ipc", "extra_capability", "public_viewer", "external_network", "dds_interface"):
            config = copy.deepcopy(self.config)
            robot = config["services"]["robot"]
            if update == "host_network":
                robot["network_mode"] = "host"
            elif update == "host_ipc":
                robot["ipc"] = "host"
            elif update == "extra_capability":
                robot["cap_add"] = ["SYS_ADMIN"]
            elif update == "public_viewer":
                robot["ports"][0]["host_ip"] = "0.0.0.0"
            elif update == "external_network":
                config["networks"]["simulation"]["internal"] = False
            else:
                robot["environment"]["IFACE"] = "eth0"
            with self.subTest(update=update), self.assertRaises(ValueError):
                launcher.validate_config(config, "test:run", 6081)

    def test_refuses_foreign_project_containers(self):
        labels = {"com.docker.compose.project.config_files": "/elsewhere/compose.yaml",
                  "com.docker.compose.service": "robot"}
        class Result:
            def __init__(self, stdout):
                self.stdout = stdout
        import json
        with patch.object(launcher, "run", side_effect=[
                Result("abc\n"), Result(json.dumps([{"Config": {"Labels": labels}}]))]):
            with self.assertRaises(ValueError):
                launcher.owned_containers("ultron-b2-example")


class HeadlessIsolationTests(unittest.TestCase):
    def setUp(self):
        common = {"image": "test:run", "read_only": True, "ipc": "none", "cap_drop": ["ALL"],
                  "security_opt": ["no-new-privileges:true"]}
        self.config = {"services": {
            "robot": {**common, "network_mode": "none", "command": ["run-sim"],
                      "environment": {"ROBOT": "b2", "DOMAIN": "71", "IFACE": "lo", "VIEWER": "0", "STAND": "0"}},
            "ros": {**common, "network_mode": "service:robot", "command": ["sleep", "infinity"],
                    "environment": {"ROS_DOMAIN_ID": "71", "CYCLONEDDS_URI": "file:///opt/kit/cyclonedds-lo.xml",
                                    "PYTHONDONTWRITEBYTECODE": "1"}},
        }}

    def test_requires_readonly_root_loopback_and_no_stand(self):
        headless.validate_config(self.config, "test:run")
        for key, value in (("read_only", False), ("network_mode", "host"), ("ipc", "host"),
                           ("volumes", ["host:/tmp"]), ("ports", ["6081:6081"])):
            config = copy.deepcopy(self.config)
            config["services"]["robot"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                headless.validate_config(config, "test:run")
        config = copy.deepcopy(self.config)
        config["services"]["robot"]["environment"]["STAND"] = "1"
        with self.assertRaises(ValueError):
            headless.validate_config(config, "test:run")

    def test_runtime_rejects_writable_mounts(self):
        import json
        host = {"ReadonlyRootfs": True, "IpcMode": "none", "CapDrop": ["ALL"]}
        robot = {"Id": "123456789", "HostConfig": {**host, "NetworkMode": "none"},
                 "Config": {"Labels": {"com.docker.compose.service": "robot"}}, "Mounts": []}
        ros = {"Id": "987654321", "HostConfig": {**host, "NetworkMode": "container:123456789"},
               "Config": {"Labels": {"com.docker.compose.service": "ros"}}, "Mounts": []}
        class Result:
            stdout = json.dumps([robot, ros])
        with patch.object(headless, "verify_owned", return_value=["a", "b"]), patch.object(headless, "run", return_value=Result()):
            headless.verify_runtime("project")
            robot["Mounts"].append({"Destination": "/tmp"})
            with patch.object(headless, "run") as run_mock:
                run_mock.return_value.stdout = json.dumps([robot, ros])
                with self.assertRaises(ValueError):
                    headless.verify_runtime("project")


class ViewerIsolationTests(unittest.TestCase):
    def test_requires_readonly_private_viewer(self):
        common = {"image": "test:run", "read_only": True, "ipc": "none", "cap_drop": ["ALL"],
                  "security_opt": ["no-new-privileges:true"], "user": "0:0",
                  "working_dir": "/opt/kit"}
        config = {"services": {
            "robot": {**common, "network_mode": "none", "command": ["bash", "/opt/ultron/viewer.sh"],
                      "volumes": [{"type": "bind", "source": str(viewer.VIEWER),
                                   "target": "/opt/ultron/viewer.sh", "read_only": True},
                                  {"type": "bind", "source": "/project/cache",
                                   "target": "/var/lib/xkb"}]},
            "ros": {**common, "network_mode": "service:robot", "command": ["sleep", "infinity"],
                    "environment": {"ROS_DOMAIN_ID": "71", "CYCLONEDDS_URI": "file:///opt/kit/cyclonedds-lo.xml",
                                    "PYTHONDONTWRITEBYTECODE": "1"}},
        }}
        viewer.validate_config(config, "test:run", Path("/project/cache"))
        for key, value in (("read_only", False), ("network_mode", "host"), ("ipc", "host"),
                           ("ports", ["6081:6081"])):
            changed = copy.deepcopy(config)
            changed["services"]["robot"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                viewer.validate_config(changed, "test:run", Path("/project/cache"))
        changed = copy.deepcopy(config)
        changed["services"]["robot"]["volumes"][0]["read_only"] = False
        with self.assertRaises(ValueError):
            viewer.validate_config(changed, "test:run", Path("/project/cache"))


if __name__ == "__main__":
    unittest.main()
