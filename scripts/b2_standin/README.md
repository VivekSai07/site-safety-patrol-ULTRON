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
