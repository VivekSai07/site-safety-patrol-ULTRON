import json
import math
from pathlib import Path
import tempfile
import unittest
from ultron.navigation import DimosNavigator
from ultron.training import validate_training_dataset
from ultron.observations import project_box, DetectionTracks


class NavigationTests(unittest.TestCase):
    def test_goal_polling_cancels_on_timeout(self):
        class Nav:
            def __init__(self): self.goal=None;self.cancelled=False
            def set_goal(self,g): self.goal=g;return True
            def is_goal_reached(self): return False
            def is_goal_cancelled(self): return False
            def cancel_goal(self,reason): self.cancelled=True;return True
        n=Nav();clock=iter([0.,0.,2.])
        adapter=DimosNavigator(n,pose_factory=lambda **kw:kw,clock=lambda:next(clock),sleep=lambda _:None)
        result=adapter.navigate((2,3),yaw=1.,timeout=1.)
        self.assertEqual(result['status'],'TIMEOUT')
        self.assertTrue(n.cancelled)
        self.assertEqual(n.goal['frame_id'],'map')

    def test_operator_stop_cancels_goal(self):
        class Nav:
            def set_goal(self,g): return True
            def is_goal_reached(self): return True
            def cancel_goal(self,reason): self.reason=reason;return True
        n=Nav()
        nav=DimosNavigator(n,pose_factory=lambda **kw:kw)
        self.assertEqual(nav.navigate((2,3),operator_enabled=lambda:False)['status'],'CANCELLED')


class TrainingTests(unittest.TestCase):
    def test_train_val_leakage_and_missing_validation_are_rejected(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            for split in ['train','val']:
                (p/'images'/split).mkdir(parents=True)
                (p/'labels'/split).mkdir(parents=True)
                Image.new('RGB',(20,20)).save(p/'images'/split/'one.jpg')
                (p/'labels'/split/'one.txt').write_text('0 .5 .5 .5 .5\n')
            entries=[{'image':f'images/{s}/one.jpg','split':s,'asset_group':'same-ladder'} for s in ['train','val']]
            (p/'metadata.jsonl').write_text('\n'.join(map(json.dumps,entries)))
            with self.assertRaises(ValueError): validate_training_dataset(p)
            entries[-1]['asset_group']='held-out-ladder'
            (p/'metadata.jsonl').write_text('\n'.join(map(json.dumps,entries)))
            self.assertEqual(validate_training_dataset(p)['images'],{'train':1,'val':1})
            bad=[dict(row,split='train',asset_group='same-ladder') for row in entries]
            (p/'metadata.jsonl').write_text('\n'.join(map(json.dumps,bad)))
            with self.assertRaises(ValueError): validate_training_dataset(p)
            (p/'metadata.jsonl').write_text('\n'.join(map(json.dumps,entries)))
            (p/'images/val/one.jpg').unlink()
            with self.assertRaises(ValueError): validate_training_dataset(p)


class ObservationTests(unittest.TestCase):
    def test_multiclass_detector_ignores_non_ladder_boxes(self):
        from types import SimpleNamespace
        from ultron.perception.models import YoloLadderDetector
        class Coordinates:
            def cpu(self): return self
            def tolist(self): return [1,2,3,4]
        result=SimpleNamespace(names={0:'person',1:'Ladder'},boxes=[
            SimpleNamespace(cls=[i],xyxy=[Coordinates()],conf=[.9]) for i in range(2)])
        detector=YoloLadderDetector.__new__(YoloLadderDetector)
        detector.model=SimpleNamespace(predict=lambda *a,**kw:[result]);detector.device='cpu';detector.weights='test'
        boxes=detector.detect('unused')
        self.assertEqual([b['label'] for b in boxes],['Ladder'])

    def test_runtime_detector_receives_rgb_pil_without_channel_swap(self):
        import numpy as np
        from PIL import Image
        from ultron.runtime_perception import RuntimePerception
        class Detector:
            def detect(self,image):
                self.pixel=image.getpixel((0,0))
                assert isinstance(image,Image.Image)
                return []
        runtime=RuntimePerception();runtime.detector=Detector()
        rgb=np.zeros((2,2,3),dtype=np.uint8);rgb[:,:,0]=255
        self.assertEqual(runtime.detections(None,{},rgb,None,None),[])
        self.assertEqual(runtime.detector.pixel,(255,0,0))

    def test_box_depth_projection_and_track_confirmation(self):
        import numpy as np
        frame={'intrinsics':[[100,0,50],[0,100,50],[0,0,1]],'camera_rotation':np.eye(3).tolist(),'camera_position':[0,0,0]}
        point=project_box([30,20,70,80],np.full((100,100),2.),frame)
        self.assertAlmostEqual(point[2],-2.)
        tracks=DetectionTracks()
        for _ in range(3):
            result=tracks.update([{'position':[1,2,1],'confidence':.9,'type':'UNKNOWN'}])
        self.assertEqual(result[0]['track_id'],'MODEL-001')
        self.assertEqual(result[0]['confirmation_frames'],3)
