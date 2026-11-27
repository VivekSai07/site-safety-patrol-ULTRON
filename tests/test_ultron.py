from datetime import date
from pathlib import Path
import tempfile
import unittest
from ultron.inspection import verdict, fuse
from ultron.memory import Registry
from ultron.planning import Grid


class InspectionTests(unittest.TestCase):
    def test_hard_fail_precedes_completeness(self):
        result=verdict({'feet':'FAIL'}, ['feet','rails'], 'UNREADABLE')
        self.assertEqual(result['status'],'FLAGGED')
        self.assertEqual(verdict({},['feet'],'OVERDUE')['status'],'FLAGGED')

    def test_pass_is_scoped_and_unknowns_abstain(self):
        self.assertEqual(verdict({'feet':'PASS'},['feet'],'VALID')['status'],'AUTOMATED_PASS')
        result=verdict({},['feet'],'VALID')
        self.assertEqual(result['status'],'MANUAL_REVIEW')
        self.assertIn('feet',result['missing'])
        self.assertFalse(result['safe_to_use'])

    def test_fusion_never_erases_fail(self):
        self.assertEqual(fuse({'feet':'FAIL'},{'feet':'PASS'})['feet'],'FAIL')
        self.assertEqual(fuse({'feet':'UNKNOWN'},{'feet':'PASS'})['feet'],'PASS')
        self.assertEqual(verdict({'feet':'UNKNOWN_CONFLICT'},['feet'],'VALID')['status'],'MANUAL_REVIEW')


class PlanningTests(unittest.TestCase):
    def test_path_avoids_inflated_obstacle_and_zone_edge(self):
        grid=Grid((0,0,10,8),[(4,2,6,6)],radius=.6,resolution=.2)
        path=grid.path((1,4),(9,4))
        self.assertTrue(path)
        self.assertTrue(all(grid.free(p) for p in path))
        self.assertTrue(any(p[1]<1.4 or p[1]>6.6 for p in path))
        self.assertFalse(grid.free((.1,4)))
        self.assertEqual(grid.path((1,4),(5,4)),[])

    def test_blocked_route_returns_no_path(self):
        grid=Grid((0,0,10,8),[(4,0,6,8)],radius=.6,resolution=.2)
        self.assertEqual(grid.path((1,4),(9,4)),[])


class MemoryTests(unittest.TestCase):
    def test_cross_patrol_history_reid_and_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'registry.sqlite'
            registry=Registry(path)
            m1=registry.start_mission('map-v1')
            asset,state=registry.associate((2,3,1),'LEANING_SINGLE','map-v1')
            self.assertEqual(state,'NEW')
            registry.append(m1,asset,{'status':'AUTOMATED_PASS','reason':'demo'})
            registry.close()
            registry=Registry(path)
            m2=registry.start_mission('map-v1')
            same,state=registry.associate((2.1,3.1,1),'LEANING_SINGLE','map-v1')
            self.assertEqual(same,asset)
            self.assertEqual(state,'REIDENTIFIED')
            registry.append(m2,same,{'status':'FLAGGED','reason':'defect'})
            self.assertEqual(len(registry.history(asset)),2)
            self.assertTrue(registry.assets()[0]['status_changed'])
            self.assertEqual(registry.missing(m2),[])
            m3=registry.start_mission('map-v1')
            self.assertEqual(registry.missing(m3),[asset])
            other,_=registry.associate((2,3,1),'LEANING_SINGLE','map-v2')
            self.assertNotEqual(other,asset)
            registry.close()

    def test_prior_flag_requires_explicit_human_resolution(self):
        with tempfile.TemporaryDirectory() as tmp:
            registry=Registry(Path(tmp)/'db')
            asset,_=registry.associate((2,3,1),'LEANING_SINGLE','m')
            first=registry.start_mission('m')
            registry.append(first,asset,{'status':'FLAGGED','reason':'Missing foot', 'checks':{'feet':'FAIL'}})
            second=registry.start_mission('m')
            registry.append(second,asset,{'status':'AUTOMATED_PASS','reason':'Current clean view', 'checks':{'feet':'PASS'}})
            latest=registry.history(asset)[-1]['payload']
            self.assertEqual(latest['status'],'FLAGGED')
            self.assertEqual(latest['checks']['feet'],'FAIL')
            self.assertEqual(latest['current_observation_status'],'AUTOMATED_PASS')
            registry.close()

    def test_export_copies_prior_run_evidence_with_colliding_filenames(self):
        from ultron.archive import export_history
        with tempfile.TemporaryDirectory() as tmp:
            old=Path(tmp)/'old'; new=Path(tmp)/'new'
            (old/'frames').mkdir(parents=True);(new/'frames').mkdir(parents=True)
            (old/'frames/00000.jpg').write_bytes(b'old evidence')
            (new/'frames/00000.jpg').write_bytes(b'new evidence')
            assets=[{'history':[{'payload':{'run_id':'old-run','evidence_root':str(old),
                         'views':[{'rgb':'frames/00000.jpg'}]}}]}]
            export_history(assets,new)
            copied=assets[0]['history'][0]['payload']['views'][0]['rgb']
            self.assertEqual((new/copied).read_bytes(),b'old evidence')
            self.assertEqual((new/'frames/00000.jpg').read_bytes(),b'new evidence')

    def test_ambiguous_neighbors_not_silently_merged(self):
        with tempfile.TemporaryDirectory() as tmp:
            registry=Registry(Path(tmp)/'db')
            first,_=registry.associate((2,3,1),'LEANING_SINGLE','m')
            second,_=registry.associate((3,3,1),'LEANING_SINGLE','m')
            third,state=registry.associate((2.5,3,1),'LEANING_SINGLE','m')
            self.assertEqual(state,'REID_AMBIGUOUS')
            self.assertNotIn(third,[first,second])
            registry.close()
