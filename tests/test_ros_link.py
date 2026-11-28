import io
import math
import unittest

try:
    from PIL import Image
    from ultron.dashboard.ros_link import grid_png, iter_sse, quaternion_from_yaw, yaw_from_quaternion
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False


@unittest.skipUnless(HAVE_PIL, 'Pillow not installed')
class RosLinkTest(unittest.TestCase):
    def test_grid_png_flips_rows_and_colours_like_rviz(self):
        # 3x2 grid, row 0 (lowest y): unknown, free, occupied; row 1: 50 %, free, free
        png = grid_png(3, 2, [-1, 0, 100, 50, 0, 0])
        with Image.open(io.BytesIO(png)) as image:
            self.assertEqual(image.size, (3, 2))
            rows = [[image.getpixel((x, y)) for x in range(3)] for y in range(2)]
        self.assertEqual(rows[1], [205, 254, 0])   # lowest-y row is now the bottom image row
        self.assertEqual(rows[0], [127, 254, 254])

    def test_grid_png_rejects_wrong_size(self):
        with self.assertRaises(ValueError):
            grid_png(2, 2, [0, 0, 0])

    def test_yaw_quaternion_round_trip(self):
        for yaw in (0.0, 1.0, -2.5, math.pi - 1e-6):
            self.assertAlmostEqual(yaw_from_quaternion(*quaternion_from_yaw(yaw)), yaw, places=6)

    def test_sse_parser_skips_comments_and_keepalives(self):
        lines = [b': connected\n', b'\n', b'event: goal\n', b'data: {"seq": 1, "x": 2}\n', b'\n',
                 b': keepalive\n', b'\n', b'event: map\n', b'data: {"seq": 3}\n', b'\n']
        self.assertEqual(list(iter_sse(lines)), [('goal', {'seq': 1, 'x': 2}), ('map', {'seq': 3})])


if __name__ == '__main__':
    unittest.main()


class GoalToleranceTest(unittest.TestCase):
    """Coarse to fine: asking again where the robot already stands means "more precisely"."""

    def test_an_operator_goal_from_afar_is_coarse_and_one_within_reach_is_fine(self):
        from ultron.dashboard.ros_link import goal_tolerance
        robot = (1.0, 1.0, 0.0)
        self.assertEqual(goal_tolerance({'x': 4.0, 'y': 1.0}, robot), ('coarse', 0.4, 0.4))
        self.assertEqual(goal_tolerance({'x': 1.3, 'y': 1.1, 'by': 'operator'}, robot), ('fine', 0.1, 0.1))
        self.assertEqual(goal_tolerance({'x': 1.0, 'y': 1.0, 'yaw': 1.5}, robot)[0], 'fine')  # turn on the spot

    def test_pipeline_goals_and_an_unknown_robot_pose_stay_coarse(self):
        from ultron.dashboard.ros_link import goal_tolerance
        self.assertEqual(goal_tolerance({'x': 1.1, 'y': 1.0, 'by': 'pipeline'}, (1.0, 1.0, 0.0))[0], 'coarse')
        self.assertEqual(goal_tolerance({'x': 1.1, 'y': 1.0}, None)[0], 'coarse')
