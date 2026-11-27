import math
import unittest

from ultron.motion import motion_samples, Command, B2SdkTransport
from ultron.perception import validate_findings, evidence_gate
from ultron.viewpoints import candidate_views
from ultron.planning import Grid


class MotionTests(unittest.TestCase):
    def test_motion_is_bounded_and_reaches_goal_without_sideways_teleport(self):
        samples=list(motion_samples((1,1,0),[(1,1),(2,1),(2,2)],math.pi/2))
        self.assertTrue(samples)
        prior=(1,1,0)
        for pose,cmd in samples:
            self.assertLessEqual(math.hypot(pose[0]-prior[0],pose[1]-prior[1]),.051)
            self.assertLessEqual(abs(cmd.vx),.5)
            self.assertLessEqual(abs(cmd.vyaw),.8)
            self.assertEqual(cmd.vy,0.)
            prior=pose
        self.assertAlmostEqual(samples[-1][0][0],2,places=5)
        self.assertAlmostEqual(samples[-1][0][1],2,places=5)
        self.assertAlmostEqual(samples[-1][0][2],math.pi/2,places=5)
        self.assertEqual(samples[-1][1].operation,'StopMove')

    def test_profile_does_not_cross_inflated_obstacle_at_float_boundary(self):
        grid=Grid((.2,.2,11.8,8.6),[(4.2,2.45,5.,3.55)],radius=.65)
        start=(8.000000000000002,2.,20.420352248333657)
        path=grid.path(start,(1.5,1.5))
        self.assertTrue(path)
        for pose,_ in motion_samples(start,path,math.pi/2):
            self.assertTrue(grid.free(pose),str(pose))

    def test_sdk_adapter_stops_when_state_stale_and_never_initializes_network_itself(self):
        class Client:
            def __init__(self): self.moves=[];self.stops=0
            def Move(self,*args): self.moves.append(args);return 0
            def StopMove(self): self.stops+=1;return 0
        client=Client()
        transport=B2SdkTransport(client)
        transport.send(Command(.2,0.,0.,.1),state_age_s=0,operator_enabled=True)
        self.assertEqual(client.moves,[(.2,0.,0.)])
        with self.assertRaises(RuntimeError):
            transport.send(Command(.2,0.,0.,.1),state_age_s=1,operator_enabled=True)
        self.assertEqual(client.stops,1)
        with self.assertRaises(ValueError):
            transport.send(Command(float('nan'),0.,0.,.1),state_age_s=0,operator_enabled=True)
        self.assertEqual(len(client.moves),1)


class EvidenceTests(unittest.TestCase):
    def test_low_confidence_pass_and_unseen_parts_require_review(self):
        doc={'components':[{'component':'feet','state':'PASS','confidence':.99,'visible':False,'reason':'not seen'}], 'label_status':'VALID'}
        result=validate_findings(doc,['feet'],'view-1')
        self.assertEqual(result['checks']['feet'],'UNKNOWN')
        self.assertEqual(evidence_gate([result],['feet'])['status'],'MANUAL_REVIEW')
        doc['components'][0].update(visible=True,confidence=.4)
        self.assertEqual(validate_findings(doc,['feet'],'v')['checks']['feet'],'UNKNOWN')

    def test_repeated_low_confidence_views_do_not_manufacture_a_pass(self):
        result=validate_findings({'components':[{'component':'feet','state':'PASS','confidence':.5,'visible':True,'reason':'looks okay'}], 'label_status':'VALID'},['feet'],'v1')
        self.assertEqual(evidence_gate([result]*30,['feet'])['status'],'MANUAL_REVIEW')

    def test_fail_dominates_incomplete_and_pass_proposals_remain_uncertified(self):
        doc={'components':[{'component':'feet','state':'FAIL','confidence':.95,'visible':True,'reason':'missing foot cap'}], 'label_status':'UNREADABLE'}
        result=evidence_gate([validate_findings(doc,['feet','rails'],'v1')],['feet','rails'])
        self.assertEqual(result['status'],'FLAGGED')
        doc['components'][0]['state']='PASS';doc['label_status']='VALID'
        result=evidence_gate([validate_findings(doc,['feet'],'v1')],['feet'])
        self.assertEqual(result['status'],'MANUAL_REVIEW')
        self.assertFalse(result['safe_to_use'])
        self.assertIn('uncalibrated',result['reason'].lower())

    def test_duplicate_or_nonfinite_model_fields_are_rejected(self):
        item={'component':'feet','state':'PASS','confidence':float('nan'),'visible':True,'reason':'x'}
        for components in ([item],[dict(item,confidence=.9)]*2):
            with self.assertRaises(ValueError):
                validate_findings({'components':components,'label_status':'VALID'},['feet'],'v')

    def test_confirmed_failure_outside_template_still_flags(self):
        doc={'components':[
            {'component':'stiles','state':'PASS','confidence':.95,'visible':True,'reason':'intact'},
            {'component':'wheel_brakes_visual','state':'FAIL','confidence':.95,'visible':True,'reason':'broken'},
        ], 'label_status':'UNKNOWN'}
        view=validate_findings(doc,['stiles'],'v1')
        self.assertEqual(view['checks']['wheel_brakes_visual'],'FAIL')
        self.assertEqual(evidence_gate([view],['stiles'])['status'],'FLAGGED')

    def test_unconfirmed_outside_template_failure_does_not_flag(self):
        for confidence,visible in ((.84,True),(.95,False)):
            doc={'components':[
                {'component':'wheel_brakes_visual','state':'FAIL','confidence':confidence,
                 'visible':visible,'reason':'uncertain'},
            ], 'label_status':'UNKNOWN'}
            view=validate_findings(doc,['stiles'],'v1')
            self.assertNotEqual(view['checks'].get('wheel_brakes_visual'),'FAIL')
            self.assertEqual(evidence_gate([view],['stiles'])['status'],'MANUAL_REVIEW')

    def test_outside_template_pass_cannot_fill_required_evidence(self):
        doc={'components':[
            {'component':'wheel_brakes_visual','state':'PASS','confidence':.95,
             'visible':True,'reason':'appears intact'},
        ], 'label_status':'VALID'}
        result=evidence_gate([validate_findings(doc,['stiles'],'v1')],['stiles'])
        self.assertEqual(result['status'],'MANUAL_REVIEW')
        self.assertIn('stiles',result['missing'])
        doc['components'][0]['component']='unrecognized_part'
        with self.assertRaises(ValueError):
            validate_findings(doc,['stiles'],'v1')


class StreamTests(unittest.TestCase):
    def test_idle_follow_can_observe_terminal_events_without_another_frame(self):
        import tempfile,json
        from pathlib import Path
        from ultron.rerun_view import _frames,_events
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'telemetry.jsonl';path.write_text('{"time": 1}\n')
            events=Path(tmp)/'events.jsonl';events.write_text('')
            iterator=_frames(path,True)
            self.assertEqual(next(iterator),{'time':1})
            events.write_text(json.dumps({'type':'MISSION_FINISHED','time':2})+'\n')
            self.assertIsNone(next(iterator))
            self.assertEqual(_events(events)[0]['type'],'MISSION_FINISHED')
            iterator.close()


class ViewTests(unittest.TestCase):
    def test_leaning_ladder_never_requests_rear_orbit(self):
        grid=Grid((0,0,12,12),[])
        views=candidate_views((6,6),'LEANING_SINGLE',grid,full_orbit=True)
        self.assertTrue(views)
        self.assertTrue(all(v['position'][1]<6 for v in views))

    def test_blocked_views_are_rejected(self):
        grid=Grid((0,0,12,12),[(0,0,12,12)])
        self.assertEqual(candidate_views((6,6),'A_FRAME_STEPLADDER',grid,full_orbit=True,
                                         open_space_confirmed=True,rear_evidence_missing=True),[])

    def test_rear_views_require_a_frame_and_both_explicit_attestations(self):
        grid=Grid((0,0,12,12),[])
        defaults=candidate_views((6,6),'A_FRAME_STEPLADDER',grid,full_orbit=True)
        self.assertTrue(all(v['position'][1]<6 for v in defaults))
        for ladder_type,clear,missing in (
            ('A_FRAME_STEPLADDER',False,True),
            ('A_FRAME_STEPLADDER',True,False),
            ('PLATFORM_PODEST',True,True),
            ('LEANING_SINGLE',True,True),
        ):
            with self.subTest(ladder_type=ladder_type,clear=clear,missing=missing):
                views=candidate_views((6,6),ladder_type,grid,full_orbit=True,
                                      open_space_confirmed=clear,rear_evidence_missing=missing)
                self.assertTrue(views)
                self.assertTrue(all(v['position'][1]<6 for v in views))

    def test_confirmed_open_a_frame_with_missing_rear_evidence_allows_simulated_candidates(self):
        grid=Grid((0,0,12,12),[])
        views=candidate_views((6,6),'A_FRAME_STEPLADDER',grid,full_orbit=True,
                              open_space_confirmed=True,rear_evidence_missing=True)
        self.assertTrue(any(v['position'][1]>6 for v in views))
        self.assertTrue(all(v['validated_for']=='SIMULATION_GRID_ONLY' for v in views))
