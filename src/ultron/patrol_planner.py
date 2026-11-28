"""Coverage-driven autonomous goal selection on the bounded A* grid.

Goal selection only: paths remain Grid A*, base motion stays kinematic.
Goal records follow Sahil's ultron.navigation_goal/v1 contract shape so the
same mission logic can later target the Nav2 adapter unchanged.
"""
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
