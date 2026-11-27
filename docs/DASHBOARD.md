# Dashboard

Live view of the ladder patrol for the site manager. Runs on localhost.

## What it does

- Shows the patrol live: robot, map, front and depth camera, alerts.
- Keeps **one report per ladder**: nesseler's ladder form, one column per patrol, a new sheet every 5 patrols.
- Never marks a ladder safe. The qualified person signs each report.
- Sim only: shows the SLAM/Nav2 occupancy map and sends navigation goals from the map (click and drag, like RViz's 2D Goal Pose). Off unless started with `--goals`. See [dashboard_nav_goal.md](dashboard_nav_goal.md).
- Planned: email all ladder reports to the site manager when the robot is back home.

## Run it

```bash
cd site-safety-patrol
PYTHONPATH=src .venv/bin/python -m ultron.dashboard fetch-form   # once: nesseler's form
PYTHONPATH=src .venv/bin/python -m ultron.dashboard              # open http://127.0.0.1:8000
PYTHONPATH=src .venv/bin/python -m ultron.dashboard.replay runs/history-demo --speed 4   # second terminal: sim data
```

On Windows, set `$env:PYTHONPATH="src"` and use `python`.

Design mockup: Ladder Patrol Dashboard canvas, page "Redesign v2".
