import tempfile
from pathlib import Path
import unittest
from ultron.supervisor import decide
from ultron.planning import Grid
from ultron.dataset import export_dataset
import json


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.grid=Grid((0,0,12,10),[])
        self.assets=[{'id':'LAD-001','type':'A_FRAME_STEPLADDER','x':5,'y':5,'z':1}]

    def test_selects_uninspected_target_then_returns_to_base(self):
        result=decide(self.assets,set(),(2,2),self.grid,survey_remaining=False,coverage_verified=True)
        self.assertEqual(result['action'],'INSPECT')
        self.assertEqual(result['asset_id'],'LAD-001')
        result=decide(self.assets,{'LAD-001'},(2,2),self.grid,survey_remaining=False,coverage_verified=True)
        self.assertEqual(result['action'],'RETURN_BASE')

    def test_unreachable_queue_produces_no_motion_review_outcome(self):
        from ultron.supervisor import inspection_queue
        grid=Grid((0,0,12,10),[(0,0,12,10)])
        detections={'ladder':{'type':'A_FRAME_STEPLADDER','position':[5,5,1]}}
        _,_,decision=next(inspection_queue(detections,lambda:(2,2),grid))
        self.assertEqual(decision['action'],'RECORD_UNREACHABLE')
        self.assertFalse(decision['motion_allowed'])

    def test_unknown_site_coverage_is_never_declared_complete(self):
        result=decide(self.assets,{'LAD-001'},(2,2),self.grid,survey_remaining=False,coverage_verified=False)
        self.assertEqual(result['action'],'RETURN_BASE_WITH_GAPS')
        self.assertIn('coverage',result['reason'].lower())

    def test_search_continues_and_unreachable_dock_requests_operator(self):
        self.assertEqual(decide([],set(),(2,2),self.grid,survey_remaining=True)['action'],'SURVEY')
        result=decide([],set(),(2,2),self.grid,survey_remaining=False,base=(20,20))
        self.assertEqual(result['action'],'AWAIT_OPERATOR')


class DatasetTests(unittest.TestCase):
    def test_exports_boxes_and_marks_damage_visibility_unannotated(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'run'; root.mkdir()
            Image.new('RGB',(100,100)).save(root/'frame.jpg')
            row={'rgb':'frame.jpg','frame_index':0,'objects':[{'box_xyxy':[10,20,50,80],
                 'asset_group':'site-1:ladder-a','ladder_type':'LEANING_SINGLE','defect_components':['rungs']}]}
            (root/'annotations.jsonl').write_text(json.dumps(row)+'\n')
            out=Path(tmp)/'dataset'
            export_dataset(root,out,split='train')
            self.assertEqual((out/'labels/train/000000.txt').read_text().strip(),'0 0.300000 0.500000 0.400000 0.600000')
            record=json.loads((out/'metadata.jsonl').read_text().splitlines()[0])
            self.assertEqual(record['damage_visibility'],'UNANNOTATED')
            self.assertTrue((out/'DATASET_CARD.md').is_file())
            self.assertNotIn('path: .', (out/'data.yaml').read_text())
