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
