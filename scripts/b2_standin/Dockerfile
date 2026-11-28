# Stand-in for the B2's Jetson software: Ubuntu 22.04, ROS 2 Humble, system Python 3.10 and a venv
# with system site packages holding the pipeline's packages (x86 + CUDA PyTorch instead of Jetson's).
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip python3-venv git \
      libgl1 libglib2.0-0 ros-humble-rosbag2-storage-default-plugins ros-humble-tf2-ros-py \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /venv && /venv/bin/pip install --no-cache-dir -U pip && \
    /venv/bin/pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cu128
# The pipeline's extras protocol, bags and detector (pyproject.toml), installed from a copy of it.
COPY pyproject.toml /tmp/ultron/pyproject.toml
RUN mkdir -p /tmp/ultron/src/ultron && touch /tmp/ultron/src/ultron/__init__.py && \
    /venv/bin/pip install --no-cache-dir "/tmp/ultron[protocol,bags,detector]" opencv-python-headless
