# B2 stand-in container

Ubuntu 22.04 + ROS 2 Humble + system Python 3.10 with a `--system-site-packages` venv, like the
B2's Jetson, but x86 with a desktop GPU. It runs the hall bag as live topics through the relay into
the pipeline with the network off, using the commands of [RECORD_ON_B2.md](../../RECORD_ON_B2.md) §6.

```bash
docker build -f scripts/b2_standin/Dockerfile -t ultron-b2box:humble .   # sends only pyproject.toml
mkdir -p /tmp/b2run && cp yoloe-11l-seg.pt mobileclip_blt.ts /tmp/b2run/
docker run --rm --gpus all --network none -v "$PWD":/repo:ro -v BAG_DIR:/bag:ro -v /tmp/b2run:/run \
    ultron-b2box:humble bash /repo/scripts/b2_standin/inside.sh
```

What it does not show: the Orin's speed, NVIDIA's Jetson PyTorch wheel, and the robot's drivers.

## Exploration contract check (dashboard + bridge, no robot)

`explore_check.sh` runs the dashboard, the real `dashboard_bridge.py`, `fake_nav2.py` (stand-in
NavigateToPose server and `/map`) and `fake_explore.py` (stand-in explore_lite) in one container with the
network off, and walks [EXPLORATION_CONTRACT.md](../../docs/EXPLORATION_CONTRACT.md): manual goals, start,
pause, pipeline goal, resume, operator takeover, lease expiry, fence and obstacle topics, finish, explorer
crash, bridge restart, Nav2 absent. Needs an image with ROS 2 and `nav2_msgs` (`ultron-b2box:humble` has
no `nav2_msgs`; `sitegen-ros:jazzy-4` was used) and the dashboard's packages for that image's Python:

```bash
pip install --target /tmp/pydeps312 --python-version 3.12 --platform manylinux2014_x86_64 \
    --platform manylinux_2_28_x86_64 --implementation cp --only-binary=:all: \
    fastapi uvicorn "pydantic>=2" pypdf reportlab Pillow
docker run --rm --network none -v "$PWD":/repo:ro -v /tmp/pydeps312:/pydeps:ro sitegen-ros:jazzy-4 \
    bash /repo/scripts/b2_standin/explore_check.sh
```
