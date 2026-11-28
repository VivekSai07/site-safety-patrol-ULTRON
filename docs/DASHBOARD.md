# Dashboard

The dashboard can display supplied replay or integration data. A live B2 feed
and completed inspection are not established. It binds to localhost by
default, but has no user authentication: other local users can access it.
Use a trusted host and non-sensitive data; do not expose it on a shared LAN.

## What it does

- Shows supplied robot, map, camera and alert data when connected or replayed.
- Keeps **one report per ladder**; the optional PDF template is source material
  whose rights and applicability require separate review.
- Never marks a ladder safe. Completion and sign-off require a qualified
  person; generating a report does not constitute that sign-off.
- Goal sending is off unless started with `--goals`. The optional ROS bridge
  does **not** enforce simulation isolation and can send goals to a reachable
  real action server. Do not start it where hardware may be reachable. See
  [dashboard_nav_goal.md](dashboard_nav_goal.md).
- Planned: email all ladder reports to the site manager when the robot is back home.

## Run it

```bash
cd site-safety-patrol-ULTRON
.venv/bin/python -m pip install -e '.[sim,dashboard]'            # after creating .venv
PYTHONPATH=src MUJOCO_GL=egl .venv/bin/python -m ultron demo --out runs/dashboard-example
PYTHONPATH=src .venv/bin/python -m ultron.dashboard              # open http://127.0.0.1:8000
PYTHONPATH=src .venv/bin/python -m ultron.dashboard.replay runs/dashboard-example --speed 4   # second terminal: sim data
```

Create `.venv` first if needed, as described in the README. The demo output
path must be new; choose another name if it already exists.

The optional `python -m ultron.dashboard fetch-form` command downloads a
third-party PDF into ignored local data. Run it only after confirming the
applicable download and use rights; this reconciliation did not run it.
The report is not an official inspection certificate or qualified sign-off.

On Windows, set `$env:PYTHONPATH="src"` and use `python`.

Design mockup: Ladder Patrol Dashboard canvas, page "Redesign v2".
