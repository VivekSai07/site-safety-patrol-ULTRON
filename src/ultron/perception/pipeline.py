"""Image analysis tool; no detections or model findings can command the robot."""
import hashlib
import json
from pathlib import Path
from PIL import Image
from . import validate_findings,evidence_gate
from ..inspection import REQUIRED


def analyze(images,output,*,weights=None,family='yoloe',qwen=None,ladder_type='UNKNOWN',allow_download=False,device='cpu'):
    images=[Path(p) for p in images]
    if not images or any(not p.is_file() for p in images):
        raise ValueError('All source images must exist')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    detections=[]
    if weights:
        from .models import YoloLadderDetector
        detector=YoloLadderDetector(weights,family=family,device=device,allow_download=allow_download)
        for path in images:
            for detection in detector.detect(str(path)):
                detections.append(dict(detection,image=str(path.resolve())))
    views=[];raw=[];errors=[]
    if qwen:
        from .models import QwenInspector
        inspector=QwenInspector(qwen,allow_download=allow_download)
        try:
            text=inspector.inspect(images,REQUIRED[ladder_type])
            raw.append(text)
            document=json.loads(text)
            views.append(validate_findings(document,REQUIRED[ladder_type],'multi-view-001'))
        except (ValueError,TypeError,KeyError) as exc:
            errors.append('Invalid model response: '+str(exc))
    # The supplied image set must already show the same selected ladder across views.
    # Detector boxes do not establish same-asset association and are not damage checks.
    result=evidence_gate(views,REQUIRED[ladder_type])
    result.update(detector_results=detections,errors=errors,raw_model_outputs=raw,
                  source_images=[{'path':str(p.resolve()),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in images],
                  calibration_status='UNVALIDATED_MODEL_PROPOSALS',source='MODEL_ANALYSIS',
                  model=qwen,detector_checkpoint=str(weights) if weights else None,
                  next_action='Human review; request a collision-checked alternate view for named missing components')
    evidence=[]
    for index,path in enumerate(images):
        name=f'view-{index+1:02}.jpg'
        with Image.open(path) as image:
            image.convert('RGB').save(output/name,quality=92)
        evidence.append(name)
    result['evidence']=evidence
    (output/'analysis.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    from html import escape
    (output/'report.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>ULTRON model review</title>'
       '<style>body{font:17px system-ui;background:#111c2c;color:#e4edf8;padding:35px}img{width:30%;margin:1%}pre{white-space:pre-wrap}</style>'
       '<h1>ULTRON · model evidence review</h1><p>Uncalibrated model proposals; qualified human review required.</p>'
       +''.join(f'<img src="{name}" alt="Source inspection view">' for name in evidence)
       +'<pre>'+escape(json.dumps(result,indent=2))+'</pre></html>')
    return result
