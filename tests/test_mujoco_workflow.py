"""Opt-in rendered integration suite. Run with ULTRON_RENDER_TESTS=1."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest


@unittest.skipUnless(os.environ.get('ULTRON_RENDER_TESTS')=='1' and importlib.util.find_spec('mujoco'),
                     'Set ULTRON_RENDER_TESTS=1 with MuJoCo available')
class RenderedWorkflowTests(unittest.TestCase):
    def test_full_height_360_capture_has_24_valid_views_and_returns_base(self):
        from ultron.inspect360 import capture360
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/'full360'
            with contextlib.redirect_stdout(io.StringIO()):
                result=capture360(output)
            self.assertTrue(result['coverage']['complete'])
            self.assertEqual(result['coverage']['accepted_cells'],24)
            self.assertEqual(len(result['views']),24)
            self.assertEqual(len({v['sha256'] for v in result['views']}),24)
            self.assertTrue(all(v['visible_band_pixels_oracle']>=20 for v in result['views']))
            self.assertTrue(result['returned_to_base'])
            self.assertEqual(result['camera_control'],'SIMULATED_VIRTUAL_PAN_TILT_NOT_B2_ACTUATION')
            self.assertTrue((output/'keyframes.json').is_file())

    def test_two_patrols_reid_change_recovery_and_missing_asset(self):
        os.environ.setdefault('MUJOCO_GL','egl')
        from ultron.mission import run_demo
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/'run'
            with contextlib.redirect_stdout(io.StringIO()):
                result=run_demo(output,variant='missing')
            first,second=result['missions']
            self.assertEqual(first['found'],3)
            self.assertEqual(first['status'],'MISSION_COMPLETE')
            self.assertEqual(second['found'],2)
            self.assertEqual(second['status'],'COVERAGE_INCOMPLETE')
            self.assertEqual(second['missing_expected'],['LAD-003'])
            self.assertEqual(len(result['assets']),3)
            history=result['assets'][0]['history']
            self.assertEqual([e['payload']['status'] for e in history],['AUTOMATED_PASS','FLAGGED'])
            self.assertTrue(result['assets'][0]['status_changed'])
            aframe=first['records'][2]
            self.assertEqual(aframe['retry_count'],1)
            self.assertEqual(aframe['status'],'AUTOMATED_PASS')
            self.assertEqual(len(aframe['views']),3)
            rung=first['records'][1]
            self.assertEqual(rung['stop_reason'],'HARD_FAIL_EARLY_EXIT')
            self.assertEqual(rung['retry_count'],0)
            self.assertTrue((output/'preview.gif').is_file())
            for mission in result['missions']:
                self.assertEqual(mission['operator_interventions'],0)
                for record in mission['records']:
                    self.assertFalse(record['safe_to_use'])
                    for view in record['views']:
                        with Image.open(output/view['rgb']) as image:
                            self.assertEqual(image.size,(640,360))
            telemetry=[json.loads(line) for line in (output/'telemetry.jsonl').read_text().splitlines()]
            self.assertTrue(telemetry[0]['robot_geoms'])
            self.assertTrue(telemetry[0]['scene_geoms'])
            self.assertTrue(all(g['kind']!=0 for g in telemetry[0]['robot_geoms']))

    def test_open_site_executes_full_ring_and_returns_base(self):
        os.environ.setdefault('MUJOCO_GL','egl')
        from ultron.mission import run_demo
        with tempfile.TemporaryDirectory() as tmp:
            with contextlib.redirect_stdout(io.StringIO()):
                result=run_demo(Path(tmp)/'orbit',patrols=1,variant='open-orbit',view_policy='orbit',motion='legacy')
            mission=result['missions'][0]
            record=mission['records'][0]
            self.assertTrue(record['full_ring_executed'])
            self.assertEqual(len(set(record['orbit_angles'])),8)
            self.assertTrue(mission['returned_to_base'])
            self.assertEqual(len([v for v in record['views'] if v['state']=='INSPECT_ORBIT']),8)

    def test_blocked_closeup_never_passes_unseen_feet(self):
        os.environ.setdefault('MUJOCO_GL','egl')
        from ultron.mission import run_demo
        with tempfile.TemporaryDirectory() as tmp:
            with contextlib.redirect_stdout(io.StringIO()):
                result=run_demo(Path(tmp)/'blocked-closeup',patrols=1,variant='blocked-closeup')
            record=next(r for r in result['missions'][0]['records'] if r['oracle_key']=='ladder_a')
            self.assertEqual(record['status'],'MANUAL_REVIEW')
            self.assertIn('bottom_feet',record['missing'])
            self.assertEqual(record['stop_reason'],'NEXT_POSE_UNSAFE_BLOCKED')

    def test_blocked_inspection_cannot_emit_scoped_pass(self):
        os.environ.setdefault('MUJOCO_GL','egl')
        from ultron.mission import run_demo
        with tempfile.TemporaryDirectory() as tmp:
            with contextlib.redirect_stdout(io.StringIO()):
                result=run_demo(Path(tmp)/'blocked',patrols=1,variant='blocked')
            record=next(r for r in result['missions'][0]['records'] if r['ladder_type']=='A_FRAME_STEPLADDER')
            self.assertEqual(record['status'],'MANUAL_REVIEW')
            self.assertEqual(record['stop_reason'],'NEXT_POSE_UNSAFE_BLOCKED')

    def test_safety_sweep_confirms_props_flags_ppe_and_writes_manifest(self):
        os.environ.setdefault('MUJOCO_GL','egl')
        from ultron.assessment import evidence_bundle_id, review_proposal, validate_proposal
        from ultron.mission import run_demo
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'run'
            with contextlib.redirect_stdout(io.StringIO()):
                run_demo(out, patrols=1, safety=True)
            events = [json.loads(line) for line in (out / 'events.jsonl').read_text().splitlines()]
            types = [e['type'] for e in events]
            self.assertIn('SAFETY_ASSET_CONFIRMED', types)
            violations = [e for e in events if e['type'] == 'SAFETY_VIOLATION']
            self.assertTrue(any(e['violation'] == 'PPE_HELMET_MISSING' for e in violations))
            self.assertTrue(all(e['source'] == 'SIMULATION_ORACLE' for e in violations))
            self.assertTrue(all(e.get('asset_id') != 'person_ok' for e in violations))
            manifest = json.loads((out / 'safety_sweep.json').read_text())
            self.assertEqual(manifest['pack'], 'site_safety_v1')
            self.assertTrue(manifest['views'])
            validate_proposal(review_proposal(manifest, 'MANUAL REVIEW: offline'), manifest)
            evidence_bundle_id(manifest)

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
