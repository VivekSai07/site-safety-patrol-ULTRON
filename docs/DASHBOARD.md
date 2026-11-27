# Dashboard (brainstorm)

Status: idea stage, nothing built yet.

## Purpose
Show ladder inspection results live, for the site manager and the jury.

## Idea
- The robot side (sim now, ROS later) posts finished ladder records to a small server.
- The dashboard only displays them. It never re-judges a verdict.
- Results reach the browser live via Server-Sent Events.

## From the presentation feedback
- Robot replaces about 80% of the safety engineer's ladder work and assists with the other 20%
- So: live notifications during the mission to the responsible officers, on site and off site
- Ladder register per site mission, plus a site mission history page

## Requirements
- Runs on any team laptop: pip install + one start command
- No GPU, ROS or Docker needed
- Python 3.10+
- Team laptops: mostly Ubuntu and Windows
- Same start command on both, e.g. `python -m ultron.dashboard`

## Rough stack
- FastAPI + Pydantic + SQLite
- Plain HTML/JS page, no build step
- Exports: PDF / JSON / CSV

## Views (first guess)
- Site map with ladder pins coloured by status
- Ladder list
- Ladder detail: photos, reason, missing parts, pending tactile checks, history

## Open questions
- ~~nesseler's protocol template~~: answered. The PDF fills their Kontrollblatt Leitern und Tritte (BGI 694) directly
- ~~PDF on Windows~~: answered. pypdf + reportlab are pure Python, no WeasyPrint
- Who sees it during the demo: laptop, phone, jury screen?
- 

## Run it (Ubuntu or Windows, Python 3.10+)
```bash
pip install -e .[dashboard]                  # once
python -m ultron.dashboard fetch-form          # once: nesseler's ladder form for the PDF export
python -m ultron.dashboard --lan               # http://<laptop-ip>:8000 for phones and other laptops
python -m ultron.dashboard.replay runs/history-demo --speed 4   # play a sim run in live (needs the sim extras)
```
- Pages: `/` live mission, `/history` site mission history
- Exports per mission: nesseler form PDF (all ladders, or one ladder), CSV, JSON
- Phone push: copy `config/notify.example.json`, set an ntfy server and topics, start with `--notify config/notify.json`
