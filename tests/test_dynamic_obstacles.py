import unittest
from ultron.planning import Grid
from ultron.dynamic_obstacles import ObstacleSnapshot, MotionGuard


class DynamicObstacleTests(unittest.TestCase):
    def setUp(self):
        self.grid = Grid((0, 0, 10, 10), [], radius=.3)
        self.guard = MotionGuard(self.grid, 'map1', max_age=.3)

    def snapshot(self, stamp=1., rectangles=(), **kwargs):
        return ObstacleSnapshot(stamp, 'map1', tuple(rectangles), **kwargs)

    def test_missing_stale_future_wrong_map_and_localization_stop(self):
        for snap in [None, self.snapshot(.5), self.snapshot(2.),
                     ObstacleSnapshot(1., 'other', ()), self.snapshot(localization_valid=False),
                     self.snapshot(operator_enabled=False)]:
            self.assertFalse(self.guard.check((1, 1), (1.1, 1), .4, 1., snap)[0])

    def test_invalid_obstacle_and_nonfinite_data_stop(self):
        for snap in [self.snapshot(rectangles=[(2, 2, 1, 1)]),
                     self.snapshot(rectangles=[(2, 2, float('nan'), 3)]), self.snapshot(float('nan'))]:
            self.assertFalse(self.guard.check((1, 1), (1.1, 1), .4, 1., snap)[0])

    def test_swept_path_and_braking_margin_stop_then_replan(self):
        snap = self.snapshot(rectangles=[(3, .8, 3.2, 1.2)])
        self.assertFalse(self.guard.check((1, 1), (5, 1), .4, 1., snap)[0])
        self.assertFalse(self.guard.check((2.3, 1), (2.4, 1), .5, 1., snap)[0])
        route = self.guard.grid_for(snap, speed=.5).path((1, 1), (5, 1))
        self.assertTrue(route)
        self.assertTrue(any(y >= 1.8 for x, y in route))
        self.assertTrue(self.guard.check((1, 1), (1.1, 1), .4, 1., snap)[0])

    def test_out_of_order_snapshot_does_not_clear_obstacle(self):
        self.guard.check((1, 1), (1.1, 1), .1, 1.2, self.snapshot(1.2))
        self.assertFalse(self.guard.check((1, 1), (1.1, 1), .1, 1.2, self.snapshot(1.1))[0])
