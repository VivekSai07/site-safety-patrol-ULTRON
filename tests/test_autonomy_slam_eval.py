"""Phase 3: SLAM evaluation maths (no ROS) and an opt-in end-to-end mapping run."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

try:
    import numpy as np
    import yaml
    from PIL import Image
except ImportError:  # pragma: no cover
    np = None

if np is not None:
    from ultron.autonomy.eval_slam import compose, load_map, map_scores, trajectory_errors

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(np is None, 'NumPy/PyYAML/Pillow not installed')
class EvalMathTest(unittest.TestCase):
    start = (1.6, 1.5, math.pi / 2)

    def truth_and_estimate(self, offset=(0.0, 0.0), yaw_bias=0.0):
        truth, estimate = {}, {}
        for i in range(100):
            t = round(10 + 0.02 * i, 3)
            local = (0.05 * i, 0.0, 0.0)                      # map-frame motion along map +x
            truth[t] = compose(self.start, local)
            estimate[t] = (local[0] + offset[0], local[1] + offset[1], yaw_bias)
        return truth, estimate

    def test_start_anchoring_is_exact_for_a_perfect_estimate(self):
        truth, estimate = self.truth_and_estimate()
        result = trajectory_errors(estimate, truth, self.start)
        self.assertLess(result['rmse_m'], 1e-9)
        self.assertLess(result['yaw_rmse_deg'], 1e-9)

    def test_offsets_are_not_hidden_without_alignment(self):
        truth, estimate = self.truth_and_estimate(offset=(0.2, 0.0))
        anchored = trajectory_errors(estimate, truth, self.start)
        self.assertAlmostEqual(anchored['rmse_m'], 0.2, places=6)
        fitted = trajectory_errors(estimate, truth, self.start, align=True)
        self.assertLess(fitted['rmse_m'], 1e-6)  # best-fit hides it: reference only

    def test_no_common_stamps_rejected(self):
        with self.assertRaises(ValueError):
            trajectory_errors({1.0: (0, 0, 0)}, {2.0: (0, 0, 0)}, self.start)

    def test_map_scores(self):
        wall = np.column_stack((np.arange(0, 5, 0.01), np.zeros(500)))
        clutter = np.array([[2.0, 3.0]])
        perfect = map_scores(wall, wall, np.ones(len(wall), bool))
        self.assertEqual((perfect['precision'], perfect['recall_walls']), (1.0, 1.0))
        half = map_scores(np.vstack((wall[:250], clutter)), wall, np.ones(len(wall), bool))
        self.assertLess(half['recall_walls'], 0.6)
        self.assertLess(half['precision'], 1.0)
        shifted = map_scores(wall + [0, 0.3], wall, np.ones(len(wall), bool))
        self.assertEqual(shifted['precision'], 0.0)  # 0.3 m off is beyond 0.10 m tolerance

    def test_load_map_trinary(self):
        with tempfile.TemporaryDirectory() as tmp:
            pixels = np.full((4, 6), 254, np.uint8)
            pixels[0, 0] = 0       # occupied at top-left: highest row in the map
            pixels[3, 5] = 205     # unknown
            Image.fromarray(pixels).save(Path(tmp) / 'm.pgm')
            (Path(tmp) / 'm.yaml').write_text(yaml.safe_dump({'image': 'm.pgm', 'mode': 'trinary', 'resolution': 0.05,
                'origin': [-1.0, -2.0, 0.0], 'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}))
            grid = load_map(Path(tmp) / 'm.yaml')
            self.assertFalse(grid['unknown_reads_as_free'])
            # map_saver's default 0.25 would turn the unknown grey (0.196) into free space.
            (Path(tmp) / 'm.yaml').write_text((Path(tmp) / 'm.yaml').read_text().replace('0.196', '0.25'))
            self.assertTrue(load_map(Path(tmp) / 'm.yaml')['unknown_reads_as_free'])
        self.assertEqual((grid['occupied_cells'], grid['unknown_cells'], grid['free_cells']), (1, 1, 22))
        np.testing.assert_allclose(grid['occupied_xy'][0], [-1.0 + 0.025, -2.0 + 3.5 * 0.05])


@unittest.skipUnless(os.environ.get('ULTRON_SLAM_TESTS') == '1' and os.environ.get('ROS_LOCALHOST_ONLY') == '1',
                     'set ULTRON_SLAM_TESTS=1 and run inside scripts/sim-slam.sh exec (about 2 minutes)')
class MappingRunTest(unittest.TestCase):
    def test_scripted_mapping_meets_phase3_acceptance(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'run'
            proc = subprocess.run([sys.executable, '-m', 'ultron.autonomy.run_mapping', str(out), '--rate', '2'],
                                  cwd=ROOT, capture_output=True, text=True, timeout=600)
            self.assertEqual(proc.returncode, 0, proc.stdout[-2000:] + proc.stderr[-2000:])
            result = json.loads((out / 'eval.json').read_text())
            meta = json.loads((out / 'map_meta.json').read_text())
            self.assertEqual(result['verdict'], 'PASS', result['checks'])
            self.assertFalse(meta['ground_truth_used_by_estimator'])
            self.assertFalse(result['map']['unknown_reads_as_free'])
            for name in ('map.pgm', 'map.yaml', 'map.posegraph', 'map.data'):
                self.assertIn(name, meta['files_sha256'])
            print(f"\n  SLAM rmse {result['slam_start_anchored']['rmse_m']:.3f} m, "
                  f"wall recall {result['map']['recall_walls']:.3f}")


if __name__ == '__main__':
    unittest.main()
