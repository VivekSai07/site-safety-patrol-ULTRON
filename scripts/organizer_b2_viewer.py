#!/usr/bin/env python3
"""Bounded isolated B2 dynamics, stand, DDS and noVNC smoke test."""

import argparse
import base64
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid


ROOT = Path(__file__).resolve().parents[1]
COMPOSE = ROOT / "config" / "organizer-b2-viewer.compose.yaml"
VIEWER = ROOT / "scripts" / "organizer_b2_viewer.sh"


def run(command, **kwargs):
    return subprocess.run(command, check=True, text=True, **kwargs)


def validate_config(config, image, xkb_dir):
    services = config["services"]
    if set(services) != {"robot", "ros"}:
        raise ValueError("unexpected services")
    for service in services.values():
        if (service["image"] != image or not service.get("read_only") or service.get("ipc") != "none"
                or service.get("cap_drop") != ["ALL"] or service.get("security_opt") != ["no-new-privileges:true"]
                or service.get("user") != "0:0" or service.get("working_dir") != "/opt/kit"
                or service.get("entrypoint") or any(key in service for key in ("ports", "devices", "pid", "privileged", "cap_add", "container_name"))):
            raise ValueError("unsafe image, ports, capabilities or namespaces")
    robot, ros = services["robot"], services["ros"]
    mounts = robot.get("volumes", [])
    mounts = {mount["target"]: mount for mount in mounts}
    if (len(robot.get("volumes", [])) != 2 or set(mounts) != {"/opt/ultron/viewer.sh", "/var/lib/xkb"}
            or mounts["/opt/ultron/viewer.sh"]["type"] != "bind"
            or Path(mounts["/opt/ultron/viewer.sh"]["source"]) != VIEWER
            or not mounts["/opt/ultron/viewer.sh"].get("read_only")
            or mounts["/var/lib/xkb"]["type"] != "bind"
            or Path(mounts["/var/lib/xkb"]["source"]) != xkb_dir
            or mounts["/var/lib/xkb"].get("read_only") or ros.get("volumes")):
        raise ValueError("unexpected viewer wrapper or persistent XKB cache mount")
    if robot["network_mode"] != "none" or ros["network_mode"] != "service:robot":
        raise ValueError("viewer and DDS must remain on private loopback")
    if (robot["command"] != ["bash", "/opt/ultron/viewer.sh"] or ros["command"] != ["sleep", "infinity"]
            or robot.get("environment")
            or ros["environment"] != {"ROS_DOMAIN_ID": "71", "CYCLONEDDS_URI": "file:///opt/kit/cyclonedds-lo.xml",
                                       "PYTHONDONTWRITEBYTECODE": "1"}):
        raise ValueError("unexpected simulator or ROS setup")


def inspect_owned(project):
    ids = run(["docker", "ps", "-aq", "--filter", f"label=com.docker.compose.project={project}"], capture_output=True).stdout.split()
    containers = json.loads(run(["docker", "inspect", *ids], capture_output=True).stdout) if ids else []
    for container in containers:
        labels = container["Config"]["Labels"]
        if (labels.get("com.docker.compose.project.config_files") != str(COMPOSE)
                or labels.get("com.docker.compose.service") not in {"robot", "ros"}):
            raise ValueError("project has foreign containers")
    return containers


def validate_runtime(containers, xkb_dir):
    services = {c["Config"]["Labels"]["com.docker.compose.service"]: c for c in containers}
    if len(containers) != 2 or set(services) != {"robot", "ros"}:
        raise ValueError("expected robot and ROS containers")
    for container in containers:
        host = container["HostConfig"]
        if not host["ReadonlyRootfs"] or host["IpcMode"] != "none" or host["CapDrop"] != ["ALL"]:
            raise ValueError("runtime does not enforce namespace and filesystem isolation")
    robot, ros = services["robot"], services["ros"]
    mounts = {mount["Destination"]: mount for mount in robot["Mounts"]}
    if (len(robot["Mounts"]) != 2 or set(mounts) != {"/opt/ultron/viewer.sh", "/var/lib/xkb"}
            or mounts["/opt/ultron/viewer.sh"]["Source"] != str(VIEWER) or mounts["/opt/ultron/viewer.sh"]["RW"]
            or mounts["/var/lib/xkb"]["Source"] != str(xkb_dir) or not mounts["/var/lib/xkb"]["RW"]
            or ros["Mounts"]):
        raise ValueError("unexpected writable or host mounts")
    network = ros["HostConfig"]["NetworkMode"]
    if (robot["HostConfig"]["NetworkMode"] != "none" or not network.startswith("container:")
            or not robot["Id"].startswith(network.removeprefix("container:"))):
        raise ValueError("runtime network isolation differs from configuration")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True)
    parser.add_argument("--out", required=True, help="new directory directly under runs/")
    args = parser.parse_args()
    output = Path(args.out)
    if output.is_absolute():
        parser.error("--out must be relative to the team repository")
    output = (ROOT / output).resolve()
    if output.parent != (ROOT / "runs").resolve():
        parser.error("--out must be a new directory directly under runs/")
    xkb_dir = output / "xkb-cache"
    image = json.loads(run(["docker", "image", "inspect", args.image], capture_output=True).stdout)[0]
    if image["Config"].get("Volumes"):
        raise ValueError("image declares writable volumes")
    project = "ultron-b2-viewer-" + uuid.uuid4().hex[:12]
    env = {key: value for key, value in os.environ.items() if not key.startswith("COMPOSE_")}
    env.update(COMPOSE_DISABLE_ENV_FILE="1", ULTRON_KIT_IMAGE=args.image, ULTRON_XKB_DIR=str(xkb_dir))
    compose = ["docker", "compose", "-p", project, "-f", str(COMPOSE)]
    config = json.loads(run(compose + ["config", "--format", "json"], env=env, capture_output=True).stdout)
    validate_config(config, args.image, xkb_dir)
    if inspect_owned(project):
        raise ValueError("project already exists")
    output.mkdir(mode=0o700)
    xkb_dir.mkdir(mode=0o777)
    xkb_dir.chmod(0o777)
    kit = ROOT.parent / "ctrl-hackathon-2026-kit"
    revision = run(["git", "-C", str(kit), "rev-parse", "HEAD"], capture_output=True).stdout.strip() if kit.is_dir() else None
    metadata = {"image": args.image, "image_id": image["Id"], "local_kit_checkout_revision": revision,
                "note": "The local checkout revision is not independently an image provenance attestation."}
    (output / "environment.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"project={project} image={image['Id']} evidence={output}", flush=True)
    launched = False
    try:
        launched = True
        run(compose + ["up", "--no-build", "-d"], env=env, timeout=60)
        validate_runtime(inspect_owned(project), xkb_dir)
        print("runtime isolation: read-only roots and wrapper, dedicated persistent XKB cache, no host ports, IPC none, capabilities dropped, private shared loopback", flush=True)
        for _ in range(60):
            logs = run(compose + ["logs", "--no-color", "robot"], env=env, capture_output=True, timeout=10).stdout
            if "robot up: b2" in logs and "Please ensure there are no obstacles" in logs:
                break
            time.sleep(1)
        else:
            raise RuntimeError("robot startup and stand-controller startup not both observed")
        time.sleep(5)
        logs = run(compose + ["logs", "--no-color", "robot"], env=env, capture_output=True, timeout=10).stdout
        (output / "robot.log").write_text(logs)
        print(logs, flush=True)
        http = "import urllib.request; r=urllib.request.urlopen('http://127.0.0.1:6081/vnc.html', timeout=5); print('viewer HTTP status:', r.status); assert r.status == 200"
        result = run(compose + ["exec", "-T", "robot", "python3", "-B", "-c", http], env=env, capture_output=True, timeout=15)
        print(result.stdout, end="", flush=True)
        (output / "viewer-http.log").write_text(result.stdout)
        websocket_probe = (ROOT / "scripts" / "organizer_b2_websocket_probe.py").read_text()
        result = run(compose + ["exec", "-T", "robot", "python3", "-B", "-"], env=env, input=websocket_probe, capture_output=True, timeout=20)
        print(result.stdout, end="", flush=True)
        (output / "viewer-websocket.log").write_text(result.stdout)
        probe = (ROOT / "scripts" / "organizer_b2_probe.py").read_text()
        shell = "source /opt/ros/humble/setup.bash && source /opt/unitree_ws/install/setup.bash && python3 -B - --require-lowcmd"
        result = run(compose + ["exec", "-T", "ros", "bash", "-c", shell], env=env, input=probe, capture_output=True, timeout=30)
        (output / "ros-probe.log").write_text(result.stdout + result.stderr)
        print(result.stdout + result.stderr, flush=True)
        capture = (ROOT / "scripts" / "organizer_b2_vnc_capture.py").read_text()
        result = run(compose + ["exec", "-T", "robot", "python3", "-B", "-"], env=env, input=capture, capture_output=True, timeout=30)
        png = base64.b64decode(result.stdout, validate=True)
        if not png.startswith(b"\x89PNG\r\n\x1a\n"):
            raise ValueError("VNC framebuffer was not PNG")
        (output / "viewer.png").write_bytes(png)
        print(f"viewer framebuffer: {len(png)} PNG bytes", flush=True)
    finally:
        if launched and inspect_owned(project):
            try:
                logs = run(compose + ["logs", "--no-color", "robot"], env=env, capture_output=True, timeout=10).stdout
                (output / "robot.log").write_text(logs)
                print(logs, flush=True)
            finally:
                run(compose + ["down"], env=env, timeout=60)
                print(f"cleaned task containers: {project}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError, OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        if isinstance(exc, subprocess.CalledProcessError) and exc.stderr:
            print(exc.stderr, file=sys.stderr)
        print(f"organizer B2 viewer: {exc}", file=sys.stderr)
        sys.exit(1)
