#!/usr/bin/env python3
"""Isolated organizer B2 Compose lifecycle; no hardware interface is exposed."""

import argparse
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request


ROOT = Path(__file__).resolve().parents[1]
COMPOSE = ROOT / "config" / "organizer-b2.compose.yaml"


def run(command, **kwargs):
    return subprocess.run(command, check=True, text=True, **kwargs)


def validate_config(config, image, port):
    services = config["services"]
    if set(services) != {"robot", "ros"}:
        raise ValueError("unexpected services")
    robot, ros = services["robot"], services["ros"]
    for service in services.values():
        if service["image"] != image:
            raise ValueError("unexpected image")
        if any(key in service for key in ("container_name", "pid", "ipc", "cap_add", "privileged", "devices", "volumes", "security_opt", "sysctls")):
            raise ValueError("unsafe namespace, name, capability, or device")
    if robot.get("network_mode") or ros.get("network_mode") != "service:robot":
        raise ValueError("unexpected network mode")
    if set(robot["networks"]) != {"simulation"} or not config["networks"]["simulation"]["internal"]:
        raise ValueError("network must be private and internal")
    ports = robot["ports"]
    if len(ports) != 1 or ports[0]["host_ip"] != "127.0.0.1" or int(ports[0]["published"]) != port or int(ports[0]["target"]) != 6081:
        raise ValueError("viewer must be published only on the requested loopback port")
    if ros.get("ports") or robot["environment"]["IFACE"] != "lo":
        raise ValueError("DDS must stay in the shared private loopback namespace")
    if robot["environment"]["DOMAIN"] != "71" or ros["environment"]["ROS_DOMAIN_ID"] != "71":
        raise ValueError("inconsistent DDS domain")
    if (robot["environment"]["ROBOT"] != "b2"
            or ros["environment"]["CYCLONEDDS_URI"] != "file:///opt/kit/cyclonedds-lo.xml"
            or robot["command"] != ["run-sim"] or ros["command"] != ["sleep", "infinity"]
            or robot.get("entrypoint") or ros.get("entrypoint")):
        raise ValueError("unexpected simulator or ROS setup")


def owned_containers(project):
    ids = run(["docker", "ps", "-aq", "--filter", f"label=com.docker.compose.project={project}"], capture_output=True).stdout.split()
    if not ids:
        return []
    containers = json.loads(run(["docker", "inspect", *ids], capture_output=True).stdout)
    for container in containers:
        labels = container["Config"]["Labels"]
        if (labels.get("com.docker.compose.project.config_files") != str(COMPOSE)
                or labels.get("com.docker.compose.service") not in {"robot", "ros"}):
            raise ValueError("project includes containers not owned by this launcher")
    return ids


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kit", required=True, type=Path, help="path to organizer unitree-mujoco directory")
    parser.add_argument("--run-id", required=True, help="unique lowercase alphanumeric run ID")
    parser.add_argument("--port", required=True, type=int, help="unused host loopback viewer port")
    parser.add_argument("action", choices=("config", "build", "up", "check", "down"))
    args = parser.parse_args()
    if not re.fullmatch(r"[a-z0-9]{6,24}", args.run_id) or not 1024 <= args.port <= 65535:
        parser.error("run-id must be 6-24 lowercase alphanumeric characters and port 1024-65535")
    kit = args.kit.resolve()
    if args.action in {"config", "build", "up"} and not all((kit / name).is_file() for name in ("Dockerfile", "compose.yaml", "bin/run-sim", "robot/sim.py")):
        parser.error("kit must name the organizer unitree-mujoco directory")
    project = f"ultron-b2-{args.run_id}"
    image = f"ultron-organizer-b2:{args.run_id}"
    env = {key: value for key, value in os.environ.items() if not key.startswith("COMPOSE_")}
    env["COMPOSE_DISABLE_ENV_FILE"] = "1"
    env.update(ULTRON_KIT_IMAGE=image, ULTRON_VIEWER_PORT=str(args.port))
    compose = ["docker", "compose", "-p", project, "-f", str(COMPOSE)]
    result = run(compose + ["config", "--format", "json"], env=env, capture_output=True)
    validate_config(json.loads(result.stdout), image, args.port)

    if args.action == "config":
        print(result.stdout)
    elif args.action == "build":
        if subprocess.run(["docker", "image", "inspect", image], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
            raise ValueError("image tag already exists; use a fresh run-id")
        # Preserve upstream sources and pins while keeping explicit build scratch out of /tmp.
        dockerfile = (kit / "Dockerfile").read_text().replace("/tmp/", "/opt/ultron-build/")
        dockerfile = dockerfile.replace('SHELL ["/bin/bash", "-c"]', 'SHELL ["/bin/bash", "-c"]\nRUN mkdir -p /opt/ultron-build\nENV TMPDIR=/opt/ultron-build')
        run(["docker", "build", "--file", "-", "--tag", image, str(kit)], input=dockerfile)
    elif args.action == "up":
        if owned_containers(project):
            raise ValueError("project already has containers; use a fresh run-id")
        networks = run(["docker", "network", "ls", "--format", "{{.Name}}"], capture_output=True).stdout.splitlines()
        if f"{project}_simulation" in networks:
            raise ValueError("project network already exists; use a fresh run-id")
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", args.port))
        run(["docker", "image", "inspect", image], stdout=subprocess.DEVNULL)
        run(compose + ["up", "--no-build", "-d"], env=env)
    elif args.action == "check":
        if not owned_containers(project):
            raise ValueError("project has no containers")
        run(compose + ["ps"], env=env)
        run(compose + ["logs", "--no-color", "--tail", "200", "robot"], env=env, timeout=30)
        run(compose + ["exec", "-T", "robot", "cat", "/tmp/stand.log"], env=env, timeout=30)
        url = f"http://127.0.0.1:{args.port}/vnc.html"
        for attempt in range(20):
            try:
                with urllib.request.urlopen(url, timeout=2) as response:
                    print(f"viewer HTTP: {response.status} {url}", flush=True)
                    if response.status != 200:
                        raise ValueError("viewer HTTP response was not 200")
                break
            except (OSError, urllib.error.URLError):
                if attempt == 19:
                    raise
                time.sleep(1)
        shell = "source /opt/ros/humble/setup.bash && source /opt/unitree_ws/install/setup.bash && ros2 topic list -t"
        run(compose + ["exec", "-T", "ros", "bash", "-c", shell], env=env, timeout=30)
        probe = (ROOT / "scripts" / "organizer_b2_probe.py").read_text()
        shell = "source /opt/ros/humble/setup.bash && source /opt/unitree_ws/install/setup.bash && python3 -B -"
        run(compose + ["exec", "-T", "ros", "bash", "-c", shell], env=env, input=probe, timeout=30)
    else:
        if not owned_containers(project):
            raise ValueError("project has no containers")
        run(compose + ["down"], env=env)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        print(f"organizer B2: {exc}", file=sys.stderr)
        sys.exit(1)
