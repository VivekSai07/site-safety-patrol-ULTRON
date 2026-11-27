"""Model observations connected to the patrol; oracle and model modes stay explicit."""
import json
import math
import numpy as np
from PIL import Image
from .observations import project_box,DetectionTracks
from .perception import validate_findings,evidence_gate


class RuntimePerception:
    def __init__(self,*,detector='oracle',weights=None,inspector='oracle',qwen=None,allow_download=False,device='cpu',family='yoloe'):
        self.detector=None;self.inspector=None;self.findings={}
        self.tracks=DetectionTracks()
        if detector=='yolo':
            if not weights: raise ValueError('--weights is required for model discovery')
            from .perception.models import YoloLadderDetector
            self.detector=YoloLadderDetector(weights,family=family,device=device,allow_download=allow_download)
        if inspector=='qwen':
            if not qwen: raise ValueError('--qwen is required for model inspection')
            from .perception.models import QwenInspector
            self.inspector=QwenInspector(qwen,allow_download=allow_download)

    def detections(self,sim,frame,rgb,depth,segmentation):
        if self.detector is None:
            return sim.oracle_detections(frame,depth,segmentation)
        observations=[]
        for box in self.detector.detect(Image.fromarray(rgb)):
            try:
                position=project_box(box['box_xyxy'],depth,frame)
            except ValueError:
                continue
            observations.append(dict(box,position=position,type='UNKNOWN'))
        return [dict(o,oracle_key=o['track_id'],source='MODEL_WITH_DEPTH_HEURISTIC',synthetic=True)
                for o in self.tracks.update(observations)]

    def reset(self,key):
        self.findings[key]=[]

    def read(self,sim,key,position,frame,rgb,depth,segmentation,views,required):
        if self.inspector is None:
            raise RuntimeError('No model inspector configured')
        box=None
        if self.detector is not None:
            candidates=[]
            for detection in self.detector.detect(Image.fromarray(rgb)):
                try: point=project_box(detection['box_xyxy'],depth,frame)
                except ValueError: continue
                distance=math.dist(point[:2],position[:2])
                if distance<=.65: candidates.append((distance,detection['box_xyxy']))
            if len(candidates)==1:
                box=candidates[0][1]
        else:
            # Explicit mixed mode: oracle crop selection, model component findings.
            body_ids=[i for i in range(sim.model.nbody) if sim.model.body(i).name==key]
            if body_ids:
                ids=np.where(sim.model.geom_bodyid==body_ids[0])[0]
                v,u=np.nonzero(np.isin(segmentation[:,:,0],ids))
                if len(u): box=[int(u.min()),int(v.min()),int(u.max())+1,int(v.max())+1]
        try:
            if box is None: raise ValueError('Target could not be uniquely identified in this view')
            x0,y0,x1,y1=[int(v) for v in box]
            height,width=rgb.shape[:2]
            x0,y0,x1,y1=max(0,x0),max(0,y0),min(width,x1),min(height,y1)
            if x1<=x0 or y1<=y0: raise ValueError('Invalid target crop')
            crop=f'frames/{frame["index"]:05}-inspection.jpg'
            Image.fromarray(rgb[y0:y1,x0:x1]).save(sim.output/crop)
            frame['inspection_crop']=crop
            paths=[sim.output/v['inspection_crop'] for v in views if 'inspection_crop' in v]
            text=self.inspector.inspect(paths,required)
            frame['model_response']=f'frames/{frame["index"]:05}-qwen.txt'
            (sim.output/frame['model_response']).write_text(text)
            finding=validate_findings(json.loads(text),required,str(frame['index']))
        except (ValueError,TypeError,KeyError,RuntimeError) as exc:
            finding={'checks':dict.fromkeys(required,'UNKNOWN'),'label_status':'UNKNOWN','proposals':[],
                     'source_view_id':str(frame['index']),'error':str(exc)}
        self.findings.setdefault(key,[]).append(finding)
        result=self.result(key,required)
        return result['checks'],self.findings[key][-1]['label_status'],result

    def result(self,key,required):
        return evidence_gate(self.findings.get(key,[]),required)
