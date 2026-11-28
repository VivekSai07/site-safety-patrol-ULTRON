#!/usr/bin/env bash
# Create .venv-ros: ROS 2 Humble's system Python 3.10 (rclpy via system site-packages)
# plus pinned simulation packages. ROS's own interpreter is never modified.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv="${ULTRON_ROS_VENV:-$repo_root/.venv-ros}"
python=/usr/bin/python3
hardware_preflight="${1:-}"
if [[ -n "$hardware_preflight" && "$hardware_preflight" != --hardware-preflight ]]; then
    echo "Usage: $0 [--hardware-preflight]" >&2
    exit 2
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble not found at /opt/ros/humble" >&2
    exit 2
fi
"$python" -c 'import sys; assert sys.version_info[:2]==(3,10), sys.version' \
    || { echo "Humble rclpy needs system Python 3.10" >&2; exit 2; }

packages=("mujoco==3.3.7" "numpy>=1.24,<2" "Pillow>=9" "PyYAML>=5.4")
if [[ "$hardware_preflight" == --hardware-preflight ]]; then
    packages+=("fastapi>=0.110,<1" "uvicorn>=0.29,<1" "pydantic>=2,<3" "httpx>=0.27,<1"
               "pypdf>=4,<7" "reportlab>=4,<5")
fi
if command -v uv >/dev/null; then
    # uv does not need the python3-venv/ensurepip system package.
    [[ -x "$venv/bin/python" ]] || uv venv --quiet --python "$python" --system-site-packages "$venv"
    uv pip install --quiet --python "$venv/bin/python" "${packages[@]}"
    [[ -n "$hardware_preflight" ]] || uv pip install --quiet --python "$venv/bin/python" --no-deps -e "$repo_root"
else
    [[ -x "$venv/bin/python" ]] || "$python" -m venv --system-site-packages "$venv"
    "$venv/bin/python" -m pip install --quiet "${packages[@]}"
    [[ -n "$hardware_preflight" ]] || "$venv/bin/python" -m pip install --quiet --no-deps -e "$repo_root"
fi

# shellcheck disable=SC1091
set +u  # ROS setup scripts read unset variables
source /opt/ros/humble/setup.bash
set -u
"$venv/bin/python" - <<'EOF'
import sys, mujoco, numpy, rclpy, yaml
print('python', sys.version.split()[0])
print('mujoco', mujoco.__version__, '| numpy', numpy.__version__, '| yaml', yaml.__version__)
print('rclpy', rclpy.__file__)
EOF
if [[ "$hardware_preflight" == --hardware-preflight ]]; then
    PYTHONPATH="$repo_root/src${PYTHONPATH:+:$PYTHONPATH}" "$venv/bin/python" -c         'from ultron.dashboard.app import create_app; from fastapi.testclient import TestClient; import httpx, uvicorn; print("Dashboard imports OK")'
fi
echo "Ready. Use: source /opt/ros/humble/setup.bash && source $venv/bin/activate"
