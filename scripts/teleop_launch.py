#!/usr/bin/env python3
"""Launch only the derived, simulated B2 capture UI on host loopback.

The image must already exist locally and match the supplied full image ID.
No DDS bridge, hardware devices, API credentials, or external network is used.
"""

from __future__ import annotations

import argparse
import http.client
import ipaddress
import json
import os
from pathlib import Path
import re
import select
import signal
import socket
import socketserver
import subprocess
import sys
import time
from typing import Any
import uuid


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src").resolve()
OWNER = "ultron-team-derived-capture-v1"
LABEL_PREFIX = "io.ultron.capture."
RUN_ID_PATTERN = re.compile(r"[a-z0-9]{6,24}\Z")
IMAGE_ID_PATTERN = re.compile(r"sha256:[0-9a-f]{64}\Z")
_RELAY_CHILDREN: dict[int, subprocess.Popen] = {}


def docker(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["docker", *args], check=check, text=True, capture_output=True, timeout=60
    )


def inspect(kind: str, name: str) -> dict[str, Any]:
    result = docker(kind, "inspect", name)
    data = json.loads(result.stdout)
    if not isinstance(data, list) or len(data) != 1:
        raise ValueError(f"unexpected Docker {kind} inspection result")
    return data[0]


def validate_output(path: Path) -> Path:
    absolute = Path(os.path.abspath(path))
    resolved = absolute.resolve(strict=True)
    if absolute != resolved:
        raise ValueError("output must not traverse symlinks")
    if not resolved.is_dir():
        raise ValueError("output must be an existing directory")
    if resolved.stat().st_uid != os.getuid() or not os.access(resolved, os.W_OK | os.X_OK):
        raise ValueError("output must be owned and writable by the current user")
    if resolved == SOURCE or SOURCE in resolved.parents or resolved in SOURCE.parents:
        raise ValueError("output must be separate from the source mount and its ancestors")
    if "," in str(resolved) or "," in str(SOURCE):
        raise ValueError("Docker bind paths may not contain commas")
    return resolved


def labels(run_id: str, image_id: str, port: int, output: Path) -> dict[str, str]:
    return {
        LABEL_PREFIX + "owner": OWNER,
        LABEL_PREFIX + "run": run_id,
        LABEL_PREFIX + "image": image_id,
        LABEL_PREFIX + "port": str(port),
        LABEL_PREFIX + "source": str(SOURCE),
        LABEL_PREFIX + "output": str(output),
        LABEL_PREFIX + "uid": str(os.getuid()),
    }


def verify_labels(actual: dict[str, str] | None, expected: dict[str, str]) -> None:
    if not isinstance(actual, dict) or any(actual.get(key) != value for key, value in expected.items()):
        raise ValueError("resource ownership labels do not match this run")


def verify_network(
    network: dict[str, Any], *, name: str, expected: dict[str, str],
    container_id: str | None = None, allow_attached: bool = True,
) -> None:
    verify_labels(network.get("Labels"), expected)
    if (
        network.get("Name") != name
        or network.get("Driver") != "bridge"
        or network.get("Internal") is not True
        or network.get("EnableIPv6") is True
        or network.get("Attachable") is True
    ):
        raise ValueError("network is not the expected private internal bridge")
    attached = set((network.get("Containers") or {}).keys())
    allowed = {container_id} if allow_attached and container_id else set()
    if attached - allowed:
        raise ValueError("task network has foreign attached containers; refusing lifecycle action")


def published_bindings(container: dict[str, Any]) -> dict[str, Any]:
    ports = (container.get("NetworkSettings") or {}).get("Ports")
    if not isinstance(ports, dict):
        raise ValueError("container runtime has no port binding information")
    return {key: value for key, value in ports.items() if value is not None}


def verify_container(
    container: dict[str, Any], *, name: str, network_name: str, network_id: str,
    image_id: str, port: int, output: Path, expected: dict[str, str],
    require_running: bool = False, allow_unpublished: bool = False,
) -> None:
    config = container.get("Config") or {}
    host = container.get("HostConfig") or {}
    verify_labels(config.get("Labels"), expected)
    bindings = {f"{port}/tcp": [{"HostIp": "127.0.0.1", "HostPort": str(port)}]}
    environment = dict(
        entry.split("=", 1) for entry in config.get("Env") or [] if "=" in entry
    )
    expected_environment = {
        "PYTHONDONTWRITEBYTECODE": "1", "PYTHONPATH": "/workspace/src",
        "MUJOCO_GL": "egl", "MESA_SHADER_CACHE_DISABLE": "true",
        "HOME": "/sessions", "ULTRON_TELEOP_ISOLATED": "1",
    }
    if (
        container.get("Name") != "/" + name
        or container.get("Image") != image_id
        or config.get("User") != f"{os.getuid()}:{os.getgid()}"
        or config.get("Entrypoint") != ["/opt/unitree-venv/bin/python"]
        or config.get("Cmd") != [
            "-u", "-m", "ultron.teleop", "--kit-root", "/opt/unitree_mujoco",
            "--output", "/sessions/captures", "--host", "0.0.0.0",
            "--port", str(port), "--width", "1280", "--height", "720",
        ]
        or host.get("NetworkMode") != network_name
        or host.get("PortBindings") != bindings
        or host.get("ReadonlyRootfs") is not True
        or host.get("IpcMode") != "none"
        or host.get("Privileged")
        or host.get("PidMode") not in ("", None)
        or host.get("CapAdd")
        or host.get("CapDrop") != ["ALL"]
        or host.get("Devices")
        or host.get("DeviceRequests")
        or host.get("DeviceCgroupRules")
        or host.get("GroupAdd")
        or set(host.get("SecurityOpt") or []) not in (
            {"no-new-privileges"}, {"no-new-privileges:true"},
        )
        or any(environment.get(key) != value for key, value in expected_environment.items())
    ):
        raise ValueError("container image, command, port or isolation settings do not match")
    mounts = container.get("Mounts") or []
    actual_mounts = {
        (item.get("Type"), item.get("Source"), item.get("Destination"), item.get("RW"))
        for item in mounts
    }
    if len(mounts) != 2 or actual_mounts != {
        ("bind", str(SOURCE), "/workspace/src", False),
        ("bind", str(output), "/sessions", True),
    }:
        raise ValueError("container mounts do not match the read-only source and explicit output")
    networks = (container.get("NetworkSettings") or {}).get("Networks") or {}
    running = (container.get("State") or {}).get("Running")
    allowed_network_ids = {network_id} if running else {"", network_id}
    if set(networks) != {network_name} or networks[network_name].get("NetworkID") not in allowed_network_ids:
        raise ValueError("container is not attached exclusively to its task network")
    if require_running:
        if not running:
            raise ValueError("task container is not running")
        ports = published_bindings(container)
        if ports != bindings and not (allow_unpublished and ports == {}):
            raise ValueError(f"runtime port publication is not loopback-only: {ports!r}")


def relay_state_path(output: Path, run_id: str) -> Path:
    return output / f".ultron-capture-{run_id}.relay.json"


def _process_start(pid: int) -> str:
    stat = Path(f"/proc/{pid}/stat").read_text()
    return stat[stat.rfind(")") + 2 :].split()[19]


def _container_ip(container: dict[str, Any], network: dict[str, Any], name: str) -> str:
    address = (container["NetworkSettings"]["Networks"][name]).get("IPAddress")
    try:
        ip = ipaddress.ip_address(address)
        subnets = [
            ipaddress.ip_network(item["Subnet"])
            for item in network["IPAM"]["Config"] if item.get("Subnet")
        ]
    except (ValueError, KeyError, TypeError) as exc:
        raise ValueError("private task network has no valid container address") from exc
    if ip.version != 4 or not ip.is_private or not any(ip in subnet for subnet in subnets):
        raise ValueError("relay target must be an IPv4 address on the private task network")
    return str(ip)


def start_relay(args: argparse.Namespace, output: Path, target: str) -> dict[str, Any]:
    state_path = relay_state_path(output, args.run_id)
    if state_path.exists():
        raise ValueError("relay state already exists; choose a fresh run ID")
    nonce = uuid.uuid4().hex
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", args.port))
        listener.listen(64)
        fd = listener.fileno()
        command = [
            sys.executable, "-B", str(Path(__file__).resolve()), "_relay",
            args.run_id, target, str(args.port), nonce, str(fd),
        ]
        process = subprocess.Popen(
            command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, close_fds=True, pass_fds=(fd,),
            start_new_session=True,
            env={"PATH": os.defpath, "HOME": str(output), "PYTHONDONTWRITEBYTECODE": "1"},
        )
        try:
            state = {
                "owner": OWNER, "run_id": args.run_id, "image_id": args.image_id,
                "port": args.port, "target": target, "pid": process.pid,
                "start_time": _process_start(process.pid), "nonce": nonce, "fd": fd,
                "python": sys.executable,
            }
            descriptor = os.open(state_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(state, stream)
            _RELAY_CHILDREN[process.pid] = process
            return state
        except Exception:
            process.terminate()
            process.wait(timeout=5)
            raise


def verify_relay(
    args: argparse.Namespace, output: Path, target: str, *, allow_dead: bool = False
) -> dict[str, Any]:
    state_path = relay_state_path(output, args.run_id)
    if state_path.is_symlink() or state_path.stat().st_uid != os.getuid():
        raise ValueError("relay state is not owned by the current user")
    state = json.loads(state_path.read_text(encoding="utf-8"))
    if any(
        state.get(key) != value
        for key, value in {
            "owner": OWNER, "run_id": args.run_id, "image_id": args.image_id,
            "port": args.port, "target": target,
        }.items()
    ):
        raise ValueError("relay ownership does not match the selected task")
    pid = state.get("pid")
    if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 1:
        raise ValueError("invalid relay PID")
    proc = Path(f"/proc/{pid}")
    expected_command = [
        state.get("python"), "-B", str(Path(__file__).resolve()), "_relay",
        args.run_id, target, str(args.port), state.get("nonce"), str(state.get("fd")),
    ]
    try:
        stat = proc.joinpath("stat").read_text()
        fields = stat[stat.rfind(")") + 2 :].split()
        start = fields[19]
        owner = proc.stat().st_uid
        if fields[0] == "Z" and start == state.get("start_time") and owner == os.getuid():
            if allow_dead:
                return {**state, "_dead": True}
            raise ValueError("task relay is no longer running")
        actual_command = proc.joinpath("cmdline").read_bytes().decode().rstrip("\0").split("\0")
    except FileNotFoundError:
        if allow_dead:
            return {**state, "_dead": True}
        raise ValueError("task relay is no longer running")
    if (
        owner != os.getuid()
        or start != state.get("start_time")
        or actual_command != expected_command
    ):
        raise ValueError("relay process identity does not match the task")
    return state


def stop_relay(args: argparse.Namespace, output: Path, target: str) -> None:
    state = verify_relay(args, output, target, allow_dead=True)
    if state.get("_dead"):
        child = _RELAY_CHILDREN.pop(state["pid"], None)
        if child is not None:
            child.wait(timeout=5)
        relay_state_path(output, args.run_id).unlink()
        return
    os.kill(state["pid"], signal.SIGTERM)
    proc = Path(f"/proc/{state['pid']}/stat")
    for _ in range(100):
        try:
            data = proc.read_text()
            fields = data[data.rfind(")") + 2 :].split()
            if fields[0] == "Z" or fields[19] != state["start_time"]:
                break
        except FileNotFoundError:
            break
        time.sleep(0.05)
    else:
        raise RuntimeError("task-owned relay did not stop; state retained")
    child = _RELAY_CHILDREN.pop(state["pid"], None)
    if child is not None:
        child.wait(timeout=5)
    relay_state_path(output, args.run_id).unlink()


def relay_main(run_id: str, target: str, port: int, nonce: str, fd: int) -> None:
    if not RUN_ID_PATTERN.fullmatch(run_id) or not re.fullmatch(r"[0-9a-f]{32}", nonce):
        raise ValueError("invalid relay identity")
    ip = ipaddress.ip_address(target)
    if ip.version != 4 or not ip.is_private or not 1024 <= port <= 65535:
        raise ValueError("invalid private relay destination")
    listener = socket.socket(fileno=fd)
    if listener.getsockname() != ("127.0.0.1", port):
        raise ValueError("relay listener is not bound exclusively to host loopback")

    class RelayHandler(socketserver.BaseRequestHandler):
        def handle(self) -> None:
            try:
                with socket.create_connection((target, port), timeout=5) as remote:
                    remote.settimeout(30)
                    self.request.settimeout(30)
                    peers = (self.request, remote)
                    while True:
                        ready, _, _ = select.select(peers, [], [], 30)
                        if not ready:
                            return
                        for source in ready:
                            data = source.recv(65536)
                            if not data:
                                return
                            (remote if source is self.request else self.request).sendall(data)
            except OSError:
                return

    class RelayServer(socketserver.ThreadingTCPServer):
        daemon_threads = True

    server = RelayServer(("127.0.0.1", port), RelayHandler, bind_and_activate=False)
    server.socket.close()
    server.socket = listener
    server.server_address = listener.getsockname()
    try:
        server.serve_forever()
    finally:
        server.server_close()


def verify_image(image: str, image_id: str) -> None:
    data = inspect("image", image)
    if data.get("Id") != image_id or (data.get("Config") or {}).get("Volumes"):
        raise ValueError("local image ID does not match the supplied pin or declares volumes")


def ui_ready(port: int) -> bool:
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
    try:
        connection.request("GET", "/")
        response = connection.getresponse()
        body = response.read(256 * 1024)
        return response.status == 200 and b"SIMULATED TELEOP" in body
    finally:
        connection.close()


def resource_names(run_id: str) -> tuple[str, str]:
    name = "ultron-capture-" + run_id
    return name, name + "-net"


def existing_names(kind: str) -> set[str]:
    if kind == "container":
        result = docker("ps", "-a", "--format", "{{.Names}}")
    else:
        result = docker("network", "ls", "--format", "{{.Name}}")
    return set(result.stdout.splitlines())


def checked_resources(args: argparse.Namespace, output: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    name, network_name = resource_names(args.run_id)
    expected = labels(args.run_id, args.image_id, args.port, output)
    network = inspect("network", network_name)
    container = inspect("container", name)
    verify_network(network, name=network_name, expected=expected, container_id=container["Id"])
    verify_container(
        container, name=name, network_name=network_name, network_id=network["Id"],
        image_id=args.image_id, port=args.port, output=output, expected=expected,
    )
    return container, network


def up(args: argparse.Namespace, output: Path) -> None:
    verify_image(args.image, args.image_id)
    name, network_name = resource_names(args.run_id)
    if name in existing_names("container") or network_name in existing_names("network"):
        raise ValueError("task name already exists; choose a fresh run ID")
    if relay_state_path(output, args.run_id).exists():
        raise ValueError("task relay state already exists; choose a fresh run ID")
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", args.port))
    expected = labels(args.run_id, args.image_id, args.port, output)
    label_args = [part for key, value in expected.items() for part in ("--label", f"{key}={value}")]
    network_id = None
    container_id = None
    relay_target = None
    try:
        network_id = docker(
            "network", "create", "--internal", "--driver", "bridge",
            *label_args, network_name,
        ).stdout.strip()
        network = inspect("network", network_id)
        verify_network(network, name=network_name, expected=expected)
        result = docker(
            "create", "--pull=never", "--name", name, *label_args,
            "--read-only", "--network", network_name,
            "--publish", f"127.0.0.1:{args.port}:{args.port}/tcp",
            "--ipc", "none", "--cap-drop", "ALL",
            "--security-opt", "no-new-privileges", "--pids-limit", "128",
            "--user", f"{os.getuid()}:{os.getgid()}",
            "-e", "PYTHONDONTWRITEBYTECODE=1", "-e", "PYTHONPATH=/workspace/src",
            "-e", "MUJOCO_GL=egl", "-e", "MESA_SHADER_CACHE_DISABLE=true",
            "-e", "HOME=/sessions", "-e", "ULTRON_TELEOP_ISOLATED=1",
            "--mount", f"type=bind,src={SOURCE},dst=/workspace/src,readonly",
            "--mount", f"type=bind,src={output},dst=/sessions",
            "--entrypoint", "/opt/unitree-venv/bin/python", args.image_id,
            "-u", "-m", "ultron.teleop", "--kit-root", "/opt/unitree_mujoco",
            "--output", "/sessions/captures", "--host", "0.0.0.0",
            "--port", str(args.port), "--width", "1280", "--height", "720",
        )
        container_id = result.stdout.strip()
        container = inspect("container", container_id)
        verify_container(
            container, name=name, network_name=network_name, network_id=network_id,
            image_id=args.image_id, port=args.port, output=output, expected=expected,
        )
        docker("start", container_id)
        container, network = checked_resources(args, output)
        verify_container(
            container, name=name, network_name=network_name, network_id=network["Id"],
            image_id=args.image_id, port=args.port, output=output, expected=expected,
            require_running=True, allow_unpublished=True,
        )
        ports = published_bindings(container)
        if ports == {}:
            relay_target = _container_ip(container, network, network_name)
            start_relay(args, output, relay_target)
            verify_relay(args, output, relay_target)
        for _ in range(60):
            try:
                if ui_ready(args.port):
                    break
            except OSError:
                pass
            if not (inspect("container", container_id).get("State") or {}).get("Running"):
                raise RuntimeError("teleop container exited before UI readiness")
            time.sleep(0.5)
        else:
            raise RuntimeError("loopback UI did not become ready in 30 seconds")
        container, network = checked_resources(args, output)
        verify_container(
            container, name=name, network_name=network_name, network_id=network["Id"],
            image_id=args.image_id, port=args.port, output=output, expected=expected,
            require_running=True, allow_unpublished=relay_target is not None,
        )
        if relay_target is not None:
            verify_relay(args, output, _container_ip(container, network, network_name))
        print(f"SIMULATED KINEMATIC POSITIONING — NOT WALKING\nhttp://127.0.0.1:{args.port}/")
        print(f"container={container_id} network={network_id} output={output}")
        if relay_target is not None:
            print("Docker did not publish its configured port; using a task-owned loopback relay.")
    except Exception:
        if relay_target and relay_state_path(output, args.run_id).exists():
            stop_relay(args, output, relay_target)
        if container_id:
            container = inspect("container", container_id)
            verify_labels((container.get("Config") or {}).get("Labels"), expected)
            docker("rm", "-f", container_id)
        if network_id:
            network = inspect("network", network_id)
            verify_network(network, name=network_name, expected=expected, allow_attached=False)
            docker("network", "rm", network_id)
        raise


def main(argv: list[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    if argv and argv[0] == "_relay":
        if len(argv) != 6:
            raise ValueError("invalid relay arguments")
        relay_main(argv[1], argv[2], int(argv[3]), argv[4], int(argv[5]))
        return 0
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("up", "status", "down"))
    parser.add_argument("--run-id", required=True, help="unique 6–24 lowercase alphanumeric characters")
    parser.add_argument("--image", default="ultron-organizer-b2:1127b2001", help="already-built local image tag")
    parser.add_argument("--image-id", required=True, help="full expected sha256 image ID")
    parser.add_argument("--port", required=True, type=int, help="host loopback port (1024–65535)")
    parser.add_argument("--output", required=True, type=Path, help="existing user-owned output directory")
    args = parser.parse_args(argv)
    if os.getuid() == 0:
        parser.error("run as a non-root user who owns the output directory")
    if not RUN_ID_PATTERN.fullmatch(args.run_id) or not IMAGE_ID_PATTERN.fullmatch(args.image_id):
        parser.error("run-id must be 6–24 lowercase alphanumeric characters and image-id a full sha256 ID")
    if not 1024 <= args.port <= 65535:
        parser.error("port must be between 1024 and 65535")
    try:
        output = validate_output(args.output)
        if args.action == "up":
            up(args, output)
        else:
            container, network = checked_resources(args, output)
            network_name = resource_names(args.run_id)[1]
            state_path = relay_state_path(output, args.run_id)
            relay_target = None
            if state_path.exists():
                relay_target = _container_ip(container, network, network_name)
                verify_relay(args, output, relay_target, allow_dead=args.action == "down")
            if args.action == "status":
                name, network_name = resource_names(args.run_id)
                verify_container(
                    container, name=name, network_name=network_name, network_id=network["Id"],
                    image_id=args.image_id, port=args.port, output=output,
                    expected=labels(args.run_id, args.image_id, args.port, output),
                    require_running=True, allow_unpublished=relay_target is not None,
                )
                if not ui_ready(args.port):
                    raise RuntimeError("loopback UI did not return the expected page")
                print(f"running: http://127.0.0.1:{args.port}/ (SIMULATED, NOT WALKING)")
            else:
                if relay_target is not None:
                    stop_relay(args, output, relay_target)
                docker("rm", "-f", container["Id"])
                network = inspect("network", network["Id"])
                verify_network(
                    network, name=resource_names(args.run_id)[1],
                    expected=labels(args.run_id, args.image_id, args.port, output),
                    allow_attached=False,
                )
                docker("network", "rm", network["Id"])
                print("removed only the validated task container and network")
    except (ValueError, OSError, RuntimeError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        detail = exc.stderr.strip() if isinstance(exc, subprocess.CalledProcessError) and exc.stderr else str(exc)
        print(f"teleop launcher: {detail}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
