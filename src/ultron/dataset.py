"""Export simulated detector labels plus conservative damage-annotation metadata."""
import json
from pathlib import Path
import shutil
from PIL import Image


def export_dataset(run,output,*,split='train'):
    if split not in ('train','val','test'):
        raise ValueError('split must be train, val or test')
    run=Path(run);output=Path(output)
    source=run/'annotations.jsonl'
    if not source.is_file():
        raise ValueError('This run has no synthetic annotation stream; generate a new demo first')
    rows=[json.loads(line) for line in source.read_text().splitlines()]
    output.mkdir(parents=True,exist_ok=False)
    for part in ('train','val','test'):
        (output/'images'/part).mkdir(parents=True)
        (output/'labels'/part).mkdir(parents=True)
    with (output/'metadata.jsonl').open('w') as metadata:
        for index,row in enumerate(rows):
            filename=f'{index:06}'
            with Image.open(run/row['rgb']) as image:
                width,height=image.size
                image.convert('RGB').save(output/'images'/split/(filename+'.jpg'),quality=92)
            labels=[]
            if not row['objects']:
                metadata.write(json.dumps({'image':f'images/{split}/{filename}.jpg','split':split,
                    'asset_group':row.get('group_prefix','unknown-scene')+':background','negative':True,
                    'synthetic':True,'damage_visibility':'UNANNOTATED'})+'\n')
            for obj in row['objects']:
                x0,y0,x1,y1=obj['box_xyxy']
                if not all(isinstance(v,(int,float)) for v in (x0,y0,x1,y1)) or not (0<=x0<x1<=width and 0<=y0<y1<=height):
                    raise ValueError('Invalid synthetic box')
                labels.append(f'0 {(x0+x1)/(2*width):.6f} {(y0+y1)/(2*height):.6f} {(x1-x0)/width:.6f} {(y1-y0)/height:.6f}')
                metadata.write(json.dumps(dict(obj,image=f'images/{split}/{filename}.jpg',split=split,
                    synthetic=True,damage_visibility='UNANNOTATED',
                    annotation_source='MUJOCO_SEGMENTATION_GROUND_TRUTH'))+'\n')
            (output/'labels'/split/(filename+'.txt')).write_text('\n'.join(labels)+'\n')
    (output/'data.yaml').write_text('train: images/train\nval: images/val\ntest: images/test\nnames:\n  0: ladder\n')
    (output/'DATASET_CARD.md').write_text('''# ULTRON synthetic ladder dataset

All images are MuJoCo renders and boxes come from visible semantic segmentation pixels. This is not real B2 camera data. One class: ladder. Scene defect metadata is simulator truth, NOT an annotation that the defect is visible in the image. Damage visibility and defect/component boxes require separate annotation before damage-model training.

This export populates only the requested split. Empty validation/test directories do not constitute a usable evaluation set. Build held-out independent scenes, physical ladder groups and capture sessions before reporting metrics. Repeated views of one ladder must stay in one split; do not randomly split neighboring frames. The current single mock scene cannot provide an independent real-world evaluation.

Real capture plan: record permitted RGB/aligned depth/calibration/poses, overview + close-up + reachable side views, stable physical ladder IDs and inspector-provided defect/unknown annotations. Include negative frames, occlusion, glare and intact examples. Preserve unresolved and tactile-only fields; never fabricate PASS. Keep real and synthetic data identifiable. Review privacy and rights before release. No license for future real captures is inferred here. Nothing is uploaded by this command.
''')
    return {'frames':len(rows),'split':split,'output':str(output),'synthetic':True}
