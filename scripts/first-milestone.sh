#!/usr/bin/env bash
# One command: isolate dependencies, start Aravind's app, run one live assessment.
set -euo pipefail
task_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$task_repo"
export PYTHONPATH="$task_repo/src"
export LP_NUM_THREADS=2 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
# Fail fast in restricted sessions before installing anything or spending credits.
python3 -m ultron.milestone_demo --check
if [[ "${1:-}" == --check ]]; then exit 0; fi
task_venv="$task_repo/runs/milestone-runtime/.venv"
if [[ ! -x "$task_venv/bin/python" ]]; then
    python3 -m venv --system-site-packages "$task_venv"
fi
# Let pip enforce declared versions, including pydantic>=2; imports alone are insufficient.
"$task_venv/bin/python" -m pip install --disable-pip-version-check -e "$task_repo[sim,dashboard]"
exec "$task_venv/bin/python" -m ultron.milestone_demo "$@"
