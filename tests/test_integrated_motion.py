"""Rendered guard-to-command-log integration; no robot connection."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest


@unittest.skipUnless(os.environ.get('ULTRON_RENDER_TESTS')=='1' and importlib.util.find_spec('mujoco'),
                     'Enable rendered simulation tests')
class IntegratedMotionTests(unittest.TestCase):
    def test_stale_observations_stop_and_inhibit_all_later_commands(self):
        from ultron.mission import run_demo
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'stale'
            with contextlib.redirect_stdout(io.StringIO()):
                result=run_demo(out,patrols=1,variant='open-orbit',dynamic_obstacles='stale')
            summary=result['missions'][0]
            self.assertTrue(summary['motion_inhibited'])
            self.assertEqual(summary['status'],'COVERAGE_INCOMPLETE')
            commands=[json.loads(r) for r in (out/'motion_commands.jsonl').read_text().splitlines()]
            stops=[n for n,c in enumerate(commands) if c['operation']=='StopMove']
            self.assertTrue(stops)
            self.assertTrue(all(c['operation']=='StopMove' for c in commands[stops[0]:]))
            events=[json.loads(r) for r in (out/'events.jsonl').read_text().splitlines()]
            halt=next(e for e in events if e['type']=='MOTION_HALTED')
            self.assertEqual(halt['status'],'OBSTACLE_OBSERVATION_STALE_OR_REORDERED')
            self.assertEqual(halt['observation_source'],'SIMULATED_OCCUPANCY_INJECTION')
