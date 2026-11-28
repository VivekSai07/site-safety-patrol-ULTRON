#!/usr/bin/env python3
"""Bounded B2 DDS-only smoke test; the containers have read-only root filesystems."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid


ROOT = Path(__file__).resolve().parents[1]
COMPOSE = ROOT / "config" / "organizer-b2-headless.compose.yaml"


def run(command, **kwargs):
    return subprocess.run(command, check=True, text=True, **kwargs)


def validate_config(config, image):
    services = config["services"]
    if set(services) != {"robot", "ros"}:
        raise ValueError("unexpected services")
    for service in services.values():
        if (service["image"] != image or not service.get("read_only")
                or service.get("ipc") != "none" or service.get("cap_drop") != ["ALL"]
                or service.get("security_opt") != ["no-new-privileges:true"] or service.get("entrypoint")
                or any(key in service for key in ("ports", "volumes", "devices", "pid", "privileged", "cap_add", "container_name"))):
            raise ValueError("unsafe image, mounts, capabilities or namespaces")
    robot, ros = services["robot"], services["ros"]
    if robot["network_mode"] != "none" or ros["network_mode"] != "service:robot":
        raise ValueError("DDS must use a private loopback-only network namespace")
    if robot["environment"] != {"ROBOT": "b2", "DOMAIN": "71", "IFACE": "lo", "VIEWER": "0", "STAND": "0"}:
        raise ValueError("unexpected robot environment")
    if (ros["environment"] != {"ROS_DOMAIN_ID": "71", "CYCLONEDDS_URI": "file:///opt/kit/cyclonedds-lo.xml",
                               "PYTHONDONTWRITEBYTECODE": "1"}
            or robot["command"] != ["run-sim"] or ros["command"] != ["sleep", "infinity"]):
        raise ValueError("unexpected simulation command or ROS configuration")


def verify_owned(project):
    ids = run(["docker", "ps", "-aq", "--filter", f"label=com.docker.compose.project={project}"], capture_output=True).stdout.split()
    if ids:
        containers = json.loads(run(["docker", "inspect", *ids], capture_output=True).stdout)
        for container in containers:
            labels = container["Config"]["Labels"]
            if (labels.get("com.docker.compose.project.config_files") != str(COMPOSE)
                    or labels.get("com.docker.compose.service") not in {"robot", "ros"}):
                raise ValueError("project has foreign containers")
    return ids


def verify_runtime(project):
    ids = verify_owned(project)
    if len(ids) != 2:
        raise ValueError("expected exactly two task-owned containers")
    containers = json.loads(run(["docker", "inspect", *ids], capture_output=True).stdout)
    services = {c["Config"]["Labels"]["com.docker.compose.service"]: c for c in containers}
    if set(services) != {"robot", "ros"}:
        raise ValueError("expected robot and ROS containers")
    for container in containers:
        host = container["HostConfig"]
        if (not host["ReadonlyRootfs"] or host["IpcMode"] != "none"
                or host["CapDrop"] != ["ALL"] or container["Mounts"]):
            raise ValueError("runtime does not enforce the read-only, mount-free isolation")
    network = services["ros"]["HostConfig"]["NetworkMode"]
    if (services["robot"]["HostConfig"]["NetworkMode"] != "none" or not network.startswith("container:")
            or not services["robot"]["Id"].startswith(network.removeprefix("container:"))):
        raise ValueError("runtime network namespace differs from the isolated configuration")
    print("runtime isolation: read-only roots, no mounts, IPC none, capabilities dropped, private shared loopback", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, help="already-built organizer kit image")
    args = parser.parse_args()
    image = json.loads(run(["docker", "image", "inspect", args.image], capture_output=True).stdout)[0]
    if image["Config"].get("Volumes"):
        raise ValueError("image declares writable volumes")
    project = "ultron-b2-headless-" + uuid.uuid4().hex[:12]
    env = {key: value for key, value in os.environ.items() if not key.startswith("COMPOSE_")}
    env.update(COMPOSE_DISABLE_ENV_FILE="1", ULTRON_KIT_IMAGE=args.image)
    compose = ["docker", "compose", "-p", project, "-f", str(COMPOSE)]
    config = json.loads(run(compose + ["config", "--format", "json"], env=env, capture_output=True).stdout)
    validate_config(config, args.image)
    if verify_owned(project):
        raise ValueError("project already exists")
    print(f"project={project} image={image['Id']}", flush=True)
    launched = False
    try:
        launched = True
        run(compose + ["up", "--no-build", "-d"], env=env, timeout=60)
        verify_runtime(project)
        for _ in range(60):
            logs = run(compose + ["logs", "--no-color", "robot"], env=env, capture_output=True, timeout=10).stdout
            if "robot up: b2" in logs:
                print(logs, flush=True)
                break
            time.sleep(1)
        else:
            print(logs, flush=True)
            raise RuntimeError("robot startup not observed within 60 seconds")
        probe = (ROOT / "scripts" / "organizer_b2_probe.py").read_text()
        shell = "source /opt/ros/humble/setup.bash && source /opt/unitree_ws/install/setup.bash && python3 -B -"
        run(compose + ["exec", "-T", "ros", "bash", "-c", shell], env=env, input=probe, timeout=30)
    finally:
        if launched and verify_owned(project):
            run(compose + ["down"], env=env, timeout=60)
            print(f"cleaned task containers: {project}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError, OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        print(f"organizer B2 headless: {exc}", file=sys.stderr)
        sys.exit(1)
