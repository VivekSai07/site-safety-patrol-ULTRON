# Autonomous Patrol + Live Dashboard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Patrol goals come from a coverage planner (no scripted route) with Sahil-compatible goal records, and a live tailer streams the running sim to Aravind's dashboard for an end-to-end site-manager view.

**Architecture:** New pure-python `CoveragePlanner` over the existing `Grid`; `run_demo(nav='auto')` swaps the fixed route for planner goals; a separate `live_tail` process reuses `replay.py`'s Api/mapping to stream JSONL files as they grow.

**Tech Stack:** stdlib only for sim-side; dashboard extra (fastapi/uvicorn/httpx/pydantic/pypdf/reportlab) installed for serving.

## Global Constraints

- Default `nav='scripted'` keeps existing behavior byte-identical; baseline suite 126 OK must stay green.
- Planner goals must pass `grid.free` and have a non-empty `grid.path`; termination guaranteed by `coverage_target` and `max_goals`.
- Events disclose autonomy honestly: goal records are `ultron.navigation_goal/v1`-shaped proposals, coverage is grid-cell observation, not certified area coverage.
- Run tests with system `python3`; render via `MUJOCO_GL=osmesa`.

---

### Task 1: `CoveragePlanner`

**Files:**
- Create: `src/ultron/patrol_planner.py`
- Test: `tests/test_patrol_planner.py`

**Interfaces:**
- Produces: `CoveragePlanner(grid, sense_range=3.0, coverage_target=0.92, max_goals=30, seed=7)` with `observe(pose)`, `coverage() -> float`, `next_goal(pose) -> tuple[float, float] | None`, `goals_issued` counter, and module helper `make_nav_goal(mission_id, x, y, yaw, reason, asset_id=None) -> dict` emitting the `ultron.navigation_goal/v1` fields (goal_id, mission_id, map_id='ultron_mock_site_v1', map_version=1, x, y, yaw, xy_tolerance_m=0.25, yaw_tolerance_rad=0.35, timeout_s=60.0, reason, asset_id, frame_id='map', schema).

- [ ] **Step 1: Write failing tests**

```python
# tests/test_patrol_planner.py
import unittest

from ultron.patrol_planner import CoveragePlanner, make_nav_goal
from ultron.planning import Grid


class CoveragePlannerTests(unittest.TestCase):
    def grid(self):
        return Grid((0., 0., 6., 6.), [(2.5, 2.5, 3.5, 3.5)], radius=.3, resolution=.3)

    def test_goals_are_free_reachable_and_coverage_grows(self):
        grid = self.grid()
        planner = CoveragePlanner(grid, sense_range=2.0, coverage_target=.8, max_goals=20)
        pose = (0.6, 0.6)
        planner.observe(pose)
        before = planner.coverage()
        goals = 0
        while (goal := planner.next_goal(pose)) is not None:
            self.assertTrue(grid.free(goal))
            self.assertTrue(grid.path(pose, goal))
            pose = goal
            planner.observe(pose)
            goals += 1
        self.assertGreater(goals, 0)
        self.assertGreater(planner.coverage(), before)
        self.assertTrue(planner.coverage() >= .8 or planner.goals_issued >= 20)

    def test_nav_goal_record_matches_contract_shape(self):
        record = make_nav_goal('PATROL-01', 1.0, 2.0, 0.5, 'coverage', asset_id=None)
        self.assertEqual(record['schema'], 'ultron.navigation_goal/v1')
        for key in ('goal_id', 'mission_id', 'map_id', 'map_version', 'x', 'y', 'yaw',
                    'xy_tolerance_m', 'yaw_tolerance_rad', 'timeout_s', 'reason', 'frame_id'):
            self.assertIn(key, record)


if __name__ == '__main__':
    unittest.main()
```

Run: `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_patrol_planner.py' -v` — Expected: FAIL (module missing)

- [ ] **Step 2: Implement `src/ultron/patrol_planner.py`**

```python
"""Coverage-driven autonomous goal selection on the bounded A* grid.

Goal selection only: paths remain Grid A*, base motion stays kinematic.
Goal records follow Sahil's ultron.navigation_goal/v1 contract shape so the
same mission logic can later target the Nav2 adapter unchanged.
"""
import math
import random
import uuid

GOAL_SCHEMA = 'ultron.navigation_goal/v1'


def make_nav_goal(mission_id, x, y, yaw, reason, asset_id=None):
    return {'goal_id': uuid.uuid4().hex, 'mission_id': mission_id,
            'map_id': 'ultron_mock_site_v1', 'map_version': 1,
            'x': float(x), 'y': float(y), 'yaw': float(yaw),
            'xy_tolerance_m': 0.25, 'yaw_tolerance_rad': 0.35, 'timeout_s': 60.0,
            'reason': reason, 'asset_id': asset_id, 'frame_id': 'map', 'schema': GOAL_SCHEMA}


class CoveragePlanner:
    def __init__(self, grid, sense_range=3.0, coverage_target=0.92, max_goals=30, seed=7):
        self.grid = grid
        self.sense_range = sense_range
        self.coverage_target = coverage_target
        self.max_goals = max_goals
        self.random = random.Random(seed)
        self.goals_issued = 0
        s = grid.resolution
        x0, y0, x1, y1 = grid.bounds
        self.cells = [(x0 + i * s, y0 + j * s)
                      for i in range(int((x1 - x0) / s) + 1)
                      for j in range(int((y1 - y0) / s) + 1)
                      if grid.free((x0 + i * s, y0 + j * s))]
        self.unseen = set(self.cells)

    def observe(self, pose):
        px, py = pose[0], pose[1]
        r = self.sense_range
        self.unseen = {c for c in self.unseen if (c[0] - px) ** 2 + (c[1] - py) ** 2 > r * r}

    def coverage(self):
        return 1.0 - len(self.unseen) / max(len(self.cells), 1)

    def _gain(self, cell):
        r = self.sense_range
        return sum(1 for c in self.unseen if (c[0] - cell[0]) ** 2 + (c[1] - cell[1]) ** 2 <= r * r)

    def next_goal(self, pose):
        if self.coverage() >= self.coverage_target or self.goals_issued >= self.max_goals:
            return None
        pool = list(self.unseen)
        candidates = pool if len(pool) <= 25 else self.random.sample(pool, 25)
        best, best_score = None, 0.0
        for cell in candidates:
            path = self.grid.path(pose[:2], cell)
            if not path:
                continue
            score = self._gain(cell) / (1.0 + len(path))
            if score > best_score:
                best, best_score = cell, score
        if best is None:
            return None
        self.goals_issued += 1
        return best
```

- [ ] **Step 3: Tests pass; commit**

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_patrol_planner.py' -v
git add src/ultron/patrol_planner.py tests/test_patrol_planner.py
git commit -m "Add coverage planner with Nav2-contract-shaped goal records"
```

---

### Task 2: Autonomous nav mode in the mission

**Files:**
- Modify: `src/ultron/mission.py`
- Modify: `src/ultron/__main__.py` (`--nav` flag)
- Test: `tests/test_mujoco_workflow.py` (gated auto-nav test)

**Interfaces:**
- Consumes: Task 1 planner.
- Produces: `run_demo(..., nav='scripted')`; `'auto'` emits `PATROL_GOAL_SELECTED` events with `goal` (nav-goal record) and `expected_gain`/`coverage_pct`; `MISSION_START` payload gains `zone_bounds`, `keepouts`, `base`; auto summary gains `observed_coverage_pct` and `coverage_note` override.

- [ ] **Step 1: Write failing gated test** (append inside `RenderedWorkflowTests`)

```python
    def test_auto_nav_patrol_selects_goals_and_returns_base(self):
        os.environ.setdefault('MUJOCO_GL','egl')
        from ultron.mission import run_demo
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'auto'
            with contextlib.redirect_stdout(io.StringIO()):
                result = run_demo(out, patrols=1, nav='auto')
            events = [json.loads(line) for line in (out / 'events.jsonl').read_text().splitlines()]
            goals = [e for e in events if e['type'] == 'PATROL_GOAL_SELECTED']
            self.assertTrue(goals)
            self.assertTrue(all(e['goal']['schema'] == 'ultron.navigation_goal/v1' for e in goals))
            start = next(e for e in events if e['type'] == 'MISSION_START')
            self.assertIn('zone_bounds', start)
            summary = result['missions'][0]
            self.assertIsNotNone(summary.get('observed_coverage_pct'))
```

Run (expected FAIL: unexpected keyword `nav`):
`ULTRON_RENDER_TESTS=1 MUJOCO_GL=osmesa LP_NUM_THREADS=2 PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_mujoco_workflow.py' -k auto_nav -v`

- [ ] **Step 2: Implement in `mission.py`**

1. Signature: append `, nav='scripted'` to `run_demo`.
2. After `grid=Grid(...)`, build the planner and swap the waypoint source:

```python
            planner=None
            if nav=='auto':
                from .patrol_planner import CoveragePlanner, make_nav_goal
                planner=CoveragePlanner(grid)
                planner.observe(sim.pose[:2])

            def waypoints():
                if planner is None:
                    yield from route
                    return
                while (goal:=planner.next_goal(sim.pose[:2])) is not None:
                    record=make_nav_goal(mid,goal[0],goal[1],math.pi/2,'coverage')
                    emit('PATROL_GOAL_SELECTED',goal=record,coverage_pct=round(100*planner.coverage(),1))
                    yield goal
```

3. Change `for waypoint in route:` to `for waypoint in waypoints():` and inside the `capture` closure add `planner and planner.observe(sim.pose[:2])` (observe at every capture).
4. Enrich mission start: `emit('MISSION_START',mission_mode=mode,operator_interventions=interventions,zone_bounds=list(grid.bounds),keepouts=grid.obstacles,base=list(sim.pose[:2]))`.
5. Summary: `planned_waypoints=len(route) if planner is None else planner.goals_issued`, keep `reached`/`skipped` counters as-is, and add `observed_coverage_pct=round(100*planner.coverage(),1) if planner else None` with `coverage_note='Planner grid-cell observation estimate, not certified area coverage' if planner else <existing note>`.
6. `__main__.py`: `demo.add_argument('--nav',choices=['scripted','auto'],default='scripted')`, pass `nav=args.nav`.

- [ ] **Step 3: Gated test passes; full suite green; commit**

```bash
ULTRON_RENDER_TESTS=1 MUJOCO_GL=osmesa LP_NUM_THREADS=2 PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_mujoco_workflow.py' -k auto_nav -v
PYTHONPATH=src python3 -m unittest discover -s tests 2>&1 | tail -3
git add src/ultron/mission.py src/ultron/__main__.py tests/test_mujoco_workflow.py
git commit -m "Add autonomous coverage-driven patrol mode (--nav auto)"
```

---

### Task 3: Live tailer

**Files:**
- Create: `src/ultron/dashboard/live_tail.py`
- Test: `tests/test_live_tail.py`

**Interfaces:**
- Consumes: `replay.Api`, `replay.convert_record`, `replay.depth_jpeg`; enriched `MISSION_START` from Task 2.
- Produces: `LiveTail(run_dir, api, site='demo-site', tag='')` with `.step() -> int` (drains newly appended telemetry/event lines, returns number of rows posted) and `.finish()` (posts inspections/photos/end from `protocol.json` when present); module `main(argv)` loop: poll every 0.5 s, exit after `finish()` or `--idle-timeout` (default 120 s) without new data. Posting rules identical to `replay.py`: telemetry row → pose + `front_rgb` frame (+ depth when present); event row → `/api/events` (same field filtering); `MISSION_START` additionally posts `/api/missions` using its `zone_bounds`/`keepouts`/`base`.

- [ ] **Step 1: Write failing test with a fake Api**

```python
# tests/test_live_tail.py
import json
from pathlib import Path
import tempfile
import unittest


class FakeApi:
    def __init__(self):
        self.posts = []
    def post(self, path, body=None, raw=None, content_type='application/json'):
        self.posts.append((path, body if body is not None else ('raw', len(raw or b''))))
        return 201, {}


class LiveTailTests(unittest.TestCase):
    def test_streams_rows_incrementally_and_starts_mission_from_event(self):
        from ultron.dashboard.live_tail import LiveTail
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run / 'frames').mkdir()
            (run / 'frames/00000.jpg').write_bytes(b'\xff\xd8jpg')
            api = FakeApi()
            tail = LiveTail(run, api, site='test-site')
            self.assertEqual(tail.step(), 0)  # nothing yet
            (run / 'events.jsonl').write_text(json.dumps(
                {'time': 0.0, 'mission_id': 'M1', 'type': 'MISSION_START', 'synthetic': True,
                 'zone_bounds': [0, 0, 5, 5], 'keepouts': [], 'base': [1, 1]}) + '\n')
            (run / 'telemetry.jsonl').write_text(json.dumps(
                {'index': 0, 'time': 0.1, 'mission_id': 'M1', 'state': 'PATROL',
                 'pose': [1, 1, 0], 'rgb': 'frames/00000.jpg'}) + '\n')
            posted = tail.step()
            self.assertGreaterEqual(posted, 2)
            paths = [p for p, _ in api.posts]
            self.assertIn('/api/missions', paths)
            self.assertIn('/api/missions/M1/pose', paths)
            self.assertIn('/api/missions/M1/frames/front_rgb', paths)
            self.assertTrue(any(p == '/api/events' for p in paths))
            # appending another event streams only the new row
            with (run / 'events.jsonl').open('a') as f:
                f.write(json.dumps({'time': 0.2, 'mission_id': 'M1', 'type': 'PATROL_GOAL_SELECTED',
                                    'goal': {'schema': 'ultron.navigation_goal/v1'}}) + '\n')
            before = len(api.posts)
            tail.step()
            self.assertGreater(len(api.posts), before)


if __name__ == '__main__':
    unittest.main()
```

Run: `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_live_tail.py' -v` — Expected: FAIL (module missing)

- [ ] **Step 2: Implement `live_tail.py`** — incremental file offsets (`_read_new(path, offset) -> (lines, new_offset)`); state: `started` mission ids, offsets per file, `t0` from first row; event rows posted with the exact replay field-filtering (drop `mission_id/type/time/asset_id/observation` into `data`, keep `observation.position`/`type` as map pin); `MISSION_START` → `/api/missions` with `{'mission_id', 'site_id', 'synthetic', 'map_version': 'sim', 'map': {'bounds', 'keepouts', 'base'}}` tolerating 409/non-201 by logging; `finish()` reads `protocol.json` if present and replays the `INSPECTION_COMMITTED` photo/inspection/end logic from `replay.py` (import and reuse `convert_record`); `main()` argparse: `run`, `--url` (default `http://127.0.0.1:8000`), `--site`, `--tag`, `--idle-timeout`.

- [ ] **Step 3: Tests pass; full suite green; commit**

```bash
PYTHONPATH=src python3 -m unittest discover -s tests 2>&1 | tail -3
git add src/ultron/dashboard/live_tail.py tests/test_live_tail.py
git commit -m "Stream a running simulation to the dashboard with a live tailer"
```

---

### Task 4: End-to-end live run + docs

**Files:**
- Modify: `docs/SITE_SAFETY.md` (live E2E section)
- Modify: `README.md` (one line pointing at it)

- [ ] **Step 1: Install dashboard extra** (conda python): `pip install "fastapi>=0.110,<1" "uvicorn>=0.29,<1" "pydantic>=2,<3" "httpx>=0.27,<1" "pypdf>=4,<7" "reportlab>=4,<5"`

- [ ] **Step 2: Start the dashboard as a persistent service** (survives the session; detached): `PYTHONPATH=src python3 -m ultron.dashboard --db runs/dashboard.sqlite --media runs/dashboard-media` (serves on 127.0.0.1:8000; verify with `curl -s http://127.0.0.1:8000/api/live | head -c 200`).

- [ ] **Step 3: Launch tailer + autonomous model-backed run**

```bash
PYTHONPATH=src python3 -m ultron.dashboard.live_tail runs/live-e2e --url http://127.0.0.1:8000 &   # waits for files
HF_HUB_DISABLE_IMPLICIT_TOKEN=1 PYTHONPATH=src MUJOCO_GL=osmesa LP_NUM_THREADS=2 python3 -m ultron demo \
  --out runs/live-e2e --patrols 1 --safety --nav auto \
  --detector yolo --weights weights/yoloe-11s-seg.pt \
  --inspector qwen --qwen Qwen/Qwen3-VL-2B-Instruct --allow-download
```

Verify live during the run: `curl -s http://127.0.0.1:8000/api/live` shows the mission, latest pose and recent events; after the run, the mission report page lists the review-pending ladder inspections and the `SAFETY_VIOLATION` events.

- [ ] **Step 4: Document** — add a "Live dashboard (end-to-end)" section to `docs/SITE_SAFETY.md` with the three commands above and what the site manager sees; one README line. Commit:

```bash
git add docs/SITE_SAFETY.md README.md
git commit -m "Document the live end-to-end dashboard stack test"
```
