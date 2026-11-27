"""Validated foundation-detector training and evaluation; no fabricated training results."""
import json
from pathlib import Path


def validate_training_dataset(root):
    root=Path(root).resolve()
    counts={};groups={};image_groups={}
    manifest=root/'metadata.jsonl'
    if not manifest.is_file():
        raise ValueError('metadata.jsonl with physical asset groups is required')
    for line in manifest.read_text().splitlines():
        row=json.loads(line);split=row['split'];group=row['asset_group']
        image=(root/row['image']).resolve()
        try: relative=image.relative_to(root)
        except ValueError: raise ValueError('Image escapes dataset root')
        if len(relative.parts)!=3 or relative.parts[0]!='images' or relative.parts[1]!=split or not image.is_file():
            raise ValueError('Metadata split does not match actual image location')
        if split not in ('train','val','test') or not isinstance(group,str) or not group:
            raise ValueError('Invalid split or asset group')
        previous=groups.setdefault(group,split)
        if previous!=split:
            raise ValueError('Asset group leaks across splits: '+group)
        image_groups.setdefault(relative.as_posix(),set()).add(group)
    for split in ('train','val'):
        images=[p for p in (root/'images'/split).glob('*') if p.suffix.lower() in ('.jpg','.png','.jpeg')]
        if not images:
            raise ValueError('Nonempty independent '+split+' split is required')
        for image in images:
            label=root/'labels'/split/(image.stem+'.txt')
            if not label.is_file():
                raise ValueError('Missing label: '+str(label))
            if str(image.relative_to(root)) not in image_groups:
                raise ValueError('Image has no grouping metadata: '+str(image))
            for line in label.read_text().splitlines():
                if not line.strip(): continue
                fields=line.split()
                if len(fields)!=5 or fields[0]!='0':
                    raise ValueError('Expected one-class YOLO detection labels')
                values=list(map(float,fields[1:]))
                if not all(0<=v<=1 for v in values) or values[2]<=0 or values[3]<=0:
                    raise ValueError('Invalid normalized box')
        counts[split]=len(images)
    return {'root':str(root),'images':counts,'asset_groups':len(groups)}


def train_detector(root,output,*,weights,epochs=10,device='cpu',allow_download=False,dry_run=False):
    plan=validate_training_dataset(root)
    if epochs<1:
        raise ValueError('epochs must be positive')
    if not dry_run and not allow_download and not Path(weights).is_file():
        raise RuntimeError('Local pretrained checkpoint required; use --allow-download explicitly otherwise')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    config={**plan,'weights':str(weights),'epochs':epochs,'device':device,'executed':False,'checkpoint_available':Path(weights).is_file()}
    (output/'training-plan.json').write_text(json.dumps(config,indent=2))
    if dry_run:
        return config
    if not allow_download:
        import os
        os.environ['YOLO_AUTOINSTALL']='false'
        os.environ['HF_HUB_OFFLINE']='1'
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError('Install perception-yolo; training was not executed') from exc
    model=YOLO(str(weights))
    # Write an absolute training config to avoid Ultralytics cwd path ambiguity.
    yaml=output/'dataset.yaml'
    yaml.write_text(f'path: {json.dumps(plan["root"])}\ntrain: images/train\nval: images/val\nnames:\n  0: ladder\n')
    from contextlib import nullcontext
    from unittest.mock import patch
    # Dataset checking fetches fonts even with plots=False. No plots need fonts.
    font_guard=nullcontext() if allow_download else patch('ultralytics.data.utils.check_font',return_value=None)
    with font_guard:
        model.train(data=str(yaml.resolve()),epochs=epochs,imgsz=640,device=device,
                    project=str(output.resolve()),name='detector',exist_ok=False,seed=42,amp=False,plots=False)
        best=output/'detector/weights/best.pt'
        if not best.is_file():
            raise RuntimeError('Trainer did not produce the expected checkpoint')
        metrics=YOLO(str(best)).val(data=str(yaml.resolve()),split='val',device=device,plots=False,amp=False)
    config.update(executed=True,checkpoint=str(best.resolve()),metrics=metrics.results_dict)
    (output/'training-result.json').write_text(json.dumps(config,indent=2,default=float))
    return config


def train_vlm(manifest,output,*,model_path,epochs=1,allow_download=False,dry_run=False):
    """LoRA SFT of Qwen3-VL using explicitly reviewed, grouped component labels."""
    manifest=Path(manifest).resolve();rows=[json.loads(line) for line in manifest.read_text().splitlines()]
    splits={'train':[],'val':[]};groups={}
    from .perception import validate_findings
    for row in rows:
        if row.get('reviewed') is not True or row.get('split') not in splits:
            raise ValueError('Each VLM record requires reviewed=true and train/val split')
        group=row['asset_group'];split=row['split']
        if groups.setdefault(group,split)!=split:
            raise ValueError('VLM physical asset group leaks between splits')
        validate_findings(row['label'],row['required_components'],'annotation')
        paths=[(manifest.parent/p).resolve() for p in row['images']]
        if not paths or any(not p.is_file() for p in paths):
            raise ValueError('Missing VLM source image')
        splits[split].append(dict(row,image_paths=paths))
    if not all(splits.values()) or epochs<1:
        raise ValueError('Reviewed independent train/val records and positive epochs required')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    plan={'executed':False,'model':str(model_path),'epochs':epochs,
          'records':{k:len(v) for k,v in splits.items()},'method':'QWEN3_VL_LORA_SFT'}
    (output/'training-plan.json').write_text(json.dumps(plan,indent=2))
    if dry_run: return plan
    try:
        import torch
        from transformers import AutoProcessor,Qwen3VLForConditionalGeneration,Trainer,TrainingArguments
        from peft import LoraConfig,get_peft_model
    except ImportError as exc:
        raise RuntimeError('Install the training-vlm extra; no VLM training was executed') from exc
    from PIL import Image
    processor=AutoProcessor.from_pretrained(model_path,local_files_only=not allow_download)
    model=Qwen3VLForConditionalGeneration.from_pretrained(model_path,local_files_only=not allow_download,torch_dtype='auto')
    model=get_peft_model(model,LoraConfig(r=8,lora_alpha=16,target_modules=['q_proj','v_proj'],task_type='CAUSAL_LM'))

    def collate(batch):
        if len(batch)!=1: raise ValueError('VLM collator uses batch size 1')
        row=batch[0];images=[]
        for path in row['image_paths']:
            with Image.open(path) as img: images.append(img.convert('RGB').copy())
        question='Return only component evidence JSON for these required ladder components: '+json.dumps(row['required_components'])
        messages=[{'role':'user','content':[{'type':'image','image':im} for im in images]+[{'type':'text','text':question}]}]
        prefix=processor.apply_chat_template(messages,tokenize=True,add_generation_prompt=True,return_dict=True,return_tensors='pt')
        messages.append({'role':'assistant','content':[{'type':'text','text':json.dumps(row['label'])}]})
        inputs=processor.apply_chat_template(messages,tokenize=True,return_dict=True,return_tensors='pt')
        labels=inputs['input_ids'].clone();labels[:,:prefix['input_ids'].shape[-1]]=-100
        labels[inputs['attention_mask']==0]=-100
        inputs['labels']=labels
        return inputs

    args=TrainingArguments(output_dir=str(output/'checkpoints'),num_train_epochs=epochs,
        per_device_train_batch_size=1,per_device_eval_batch_size=1,gradient_accumulation_steps=8,
        learning_rate=2e-4,remove_unused_columns=False,report_to=[],save_strategy='epoch',logging_steps=1)
    trainer=Trainer(model=model,args=args,train_dataset=splits['train'],eval_dataset=splits['val'],data_collator=collate)
    trainer.train();metrics=trainer.evaluate()
    trainer.save_model(str(output/'adapter'));processor.save_pretrained(output/'adapter')
    plan.update(executed=True,metrics=metrics,adapter=str((output/'adapter').resolve()))
    (output/'training-result.json').write_text(json.dumps(plan,indent=2,default=float))
    return plan
