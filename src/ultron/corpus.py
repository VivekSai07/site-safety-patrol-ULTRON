"""Generate independent procedural scene groups for detector training/evaluation."""
import json
import math
from pathlib import Path
import random
import shutil
from .dataset import export_dataset


def generate_corpus(output,*,scenes=6,views=3,seed=42):
    if scenes<4 or views<1 or views>12:
        raise ValueError('Use at least 4 scene groups and 1..12 views per ladder')
    from .mujoco_sim import Simulation
    from .scene import scenarios
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    for split in ('train','val'):
        (output/'images'/split).mkdir(parents=True)
        (output/'labels'/split).mkdir(parents=True)
    metadata=[];counts={'train':0,'val':0}
    for index in range(scenes):
        rng=random.Random(seed+index)
        split='train' if index<max(2,int(scenes*.75)) else 'val'
        group=f'procedural-{seed}-{index:04}'
        run=output/'source_runs'/group;run.mkdir(parents=True)
        (run/'frames').mkdir()
        ladders=scenarios()
        for ladder in ladders:
            ladder.update(width_factor=rng.uniform(.8,1.25),height_factor=rng.uniform(.8,1.15))
            ladder['position'][1]+=rng.uniform(-.25,.25)
            ladder['defect']=rng.choice([None,None,'rungs','bottom_feet'])
        sim=Simulation(ladders,run,group_prefix=group)
        try:
            for ladder in ladders:
                x,y,_=ladder['position']
                for view in range(views):
                    angle=-math.pi/2+(-.45+.9*view/max(1,views-1))
                    radius=rng.uniform(2.7,3.2)
                    px,py=x+radius*math.cos(angle),y+radius*math.sin(angle)
                    sim.t+=1.;sim.place((px,py,math.atan2(y-py,x-px)))
                    sim.capture()
            sim.place((1.5,1.5,-math.pi/2));sim.capture()
        finally:
            sim.close()
        exported=run/'export'
        export_dataset(run,exported,split=split)
        mapping={}
        for image in sorted((exported/'images'/split).glob('*.jpg')):
            name=group+'-'+image.name
            shutil.copyfile(image,output/'images'/split/name)
            shutil.copyfile(exported/'labels'/split/(image.stem+'.txt'),output/'labels'/split/(Path(name).stem+'.txt'))
            mapping[f'images/{split}/{image.name}']=f'images/{split}/{name}'
            counts[split]+=1
        for line in (exported/'metadata.jsonl').read_text().splitlines():
            row=json.loads(line);row['image']=mapping[row['image']];metadata.append(row)
    (output/'metadata.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in metadata))
    (output/'data.yaml').write_text('train: images/train\nval: images/val\nnames:\n  0: ladder\n')
    card='''# ULTRON procedural detector corpus

Synthetic MuJoCo images only. Train and validation use disjoint scene/asset groups with varied ladder width, height, view angle and defects. Both splits share the procedural generator; validation measures synthetic generalization, not real-world safety. Visible ladder boxes come from segmentation. Scenario defect metadata does not establish visible damage; damage_visibility remains UNANNOTATED. No damage classifier is trained from these scene-level tags. Human-reviewed real captures remain necessary before accepting inspection performance.
'''
    (output/'DATASET_CARD.md').write_text(card)
    result={'scenes':scenes,'seed':seed,'images':counts,'synthetic':True}
    (output/'generation.json').write_text(json.dumps(result,indent=2))
    return result
