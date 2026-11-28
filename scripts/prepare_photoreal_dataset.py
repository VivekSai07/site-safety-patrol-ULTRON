"""Prepare/import a reference-informed synthetic dataset and bundled-imagegen CLI jobs.

Local files only. Never calls an API, loads a credential, or marks prompts as labels.
"""
import argparse
from collections import Counter,defaultdict
import hashlib
import json
from pathlib import Path
import re
import shutil

ROOT=Path(__file__).resolve().parents[1]
PACK=ROOT/'deliverables/ULTRON_inspection_prompt_pack_v1'
SECTIONS={f'S{i:02}':name for i,name in enumerate(
    ['holme','sprossen','spreiz','beschlag','fuesse','zubehoer','kennzeichnung'],1)}
STYLE='''Reference-informed photographic style (no reference photo is uploaded): ordinary phone/robot-camera inspection photographs at an unfinished German construction site, not polished product photography. Ribbed silver aluminium rails/steps, realistic rivets, dark or red rubber foot caps, red spreader bars/straps where the chosen ladder has them, plaster/concrete dust, unfinished concrete/block walls, occasional shoring tubes and scuffed flooring. Exterior cases may include a green site container and daylight glare. Include realistic manufacturer/data label strips, safety pictograms and small circular inspection stickers only when in frame. Label text must not encode the class or a safety verdict; generated text is unverified. Preserve plausible component geometry, ageing and modest exposure variation; no simulator/CAD/cartoon appearance, defect arrows, coloured answer overlays, dramatic horror damage or staged studio backgrounds. All condition variants of a base identity use comparable camera/lighting/background so appearance does not leak the label.
'''


def write_jsonl(path,rows):
    path.write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows))


def refresh(root,rows):
    """Only submit missing images; corrupt outputs require review before paid retry."""
    from PIL import Image
    root=Path(root);root.mkdir(parents=True,exist_ok=True);(root/'images').mkdir(exist_ok=True)
    seen=set();groups={};pending=[];records=[];hashes={};duplicates=[]
    for row in rows:
        row=dict(row);image_id=row['image_id'];group=row['asset_group'];split=row['split']
        if image_id in seen or not re.fullmatch(r'[A-Za-z0-9_.-]+',image_id):raise ValueError('Duplicate/unsafe image ID')
        seen.add(image_id)
        if groups.setdefault(group,split)!=split:raise ValueError('Physical/generation identity leaks across splits')
        filename=row['filename']
        if Path(filename).name!=filename or filename!=image_id+'.png':raise ValueError('Use a flat canonical image filename')
        p=root/'images'/filename
        row.update(image='images/'+filename,synthetic=True,reviewed=False,
            annotation_status='UNREVIEWED_INTENT_NOT_GROUND_TRUTH',pose_status='REQUESTED_NOT_MEASURED',
            cross_view_identity_verified=False)
        if p.exists():
            try:
                with Image.open(p) as im:
                    if im.format!='PNG':raise ValueError('Output is not PNG')
                    row['width'],row['height']=im.size;im.verify()
            except Exception as exc:raise ValueError('Corrupt output; preserve/quarantine before resubmitting: '+str(p)) from exc
            digest=hashlib.sha256(p.read_bytes()).hexdigest()
            if digest in hashes:duplicates.append([hashes[digest],image_id])
            hashes[digest]=image_id;row.update(file_status='PRESENT',sha256=digest)
        else:
            row['file_status']='PENDING'
            pending.append({'prompt':row['prompt'],'out':filename,'model':'gpt-image-2','n':1,
                'size':'1536x1024','quality':'medium','output_format':'png'})
        records.append(row)
    write_jsonl(root/'manifest.jsonl',records);write_jsonl(root/'jobs.pending.jsonl',pending)
    report={'requested_images':len(rows),'images_present':len(rows)-len(pending),'images_pending':len(pending),
        'reviewed_training_images':0,'asset_groups':len(groups),'duplicate_pixel_hashes':duplicates,
        'generation_complete':not pending,'network_generation_executed_by_preparer':False,
        'counts_by_section':dict(Counter(r.get('section','unknown') for r in rows)),
        'sections_by_split':{name:dict(Counter(r['split'] for r in rows if r.get('section')==name)) for name in SECTIONS.values()},
        'requested_model':'gpt-image-2','generation_provenance':'Requested model only; retain actual API receipts separately',
        'estimated_remaining_output_usd_historical':round(len(pending)*.041,3),
        'estimate_excludes':'Input text/image tokens, retries, model/current-price differences; NOT a spending cap',
        'pricing_reference':'https://developers.openai.com/api/docs/guides/image-generation#historical-output-costs'}
    (root/'status.json').write_text(json.dumps(report,indent=2))
    return report


def make_rows():
    original=[json.loads(line) for line in (PACK/'keyframe_prompts_360.jsonl').read_text().splitlines()]
    rows=[]
    for r in original:
        row=dict(r);row['section']=SECTIONS[row['asset_group'].split('-')[0]]
        row['original_pack_filename']=row['filename'];row['filename']=row['image_id']+'.png'
        group=row['asset_group'];number=int(group[:3][1:])
        # Preserve imported A-frame group; vary compatible other identities like the references.
        leaning=number in (1,2,4,7) and group!='S01-01' and int(group.split('-')[1])%2==0
        if leaning:
            row['prompt']=row['prompt'].replace('A-frame/platform stepladder','single leaning ladder resting against a narrow freestanding support')
            row['ladder_family']='leaning'
        else:row['ladder_family']='stepladder_or_platform'
        row['prompt']+='\n'+STYLE
        if leaning:
            row['prompt']+='From rear views, preserve realistic occlusion by the support; do not make it transparent or invent access through a wall. Requested azimuth is a generation target, not measured robot/camera pose.\n'
        rows.append(row)
    # Add only the missing section/split identity groups, keeping sibling variants together.
    # This fixes v1's accessory/marking holdout gaps rather than randomly splitting views.
    for section in SECTIONS.values():
        splits={r['split'] for r in rows if r['section']==section}
        template=next(r['asset_group'] for r in rows if r['section']==section)
        template_rows=[r for r in rows if r['asset_group']==template]
        for split in ['train','val','test']:
            if split in splits:continue
            for r in template_rows:
                new=dict(r);new_group=template+'-holdout-'+split
                new['asset_group']=new_group;new['split']=split
                new['image_id']=r['image_id'].replace(template,new_group,1)
                new['filename']=new['image_id']+'.png';new['scenario_id']=r['scenario_id'].replace(template,new_group,1)
                new['prompt']=r['prompt'].replace(template,new_group)
                new['prompt']+=f'\nNew held-out physical identity family {new_group}: change rung count, rail proportions, component details and background layout compared with the original template. Keep all views and condition variants of this identity consistent and in split {split}. This is a requested diversity change, not proof of independent identity; a reviewer must verify it.\n'
                new['supplemental_holdout_identity']=True
                rows.append(new)
    return rows


def prepare(output,seed_dir,reference_dir):
    from PIL import Image
    output=Path(output);output.mkdir(parents=True,exist_ok=False);(output/'images').mkdir()
    rows=make_rows();lookup={r['image_id']:r for r in rows}
    seed_dir=Path(seed_dir);meta=json.loads((seed_dir/'generation_metadata.json').read_text())
    imports=[]
    for item in meta['images']:
        if item['image_id'] not in lookup:raise ValueError('Unknown seed image')
        source=(seed_dir/item['filename']).resolve()
        if not source.is_relative_to(seed_dir.resolve()):raise ValueError('Seed image escapes root')
        digest=hashlib.sha256(source.read_bytes()).hexdigest()
        if digest!=item['sha256']:raise ValueError('Seed checksum mismatch')
        shutil.copyfile(source,output/'images'/lookup[item['image_id']]['filename'])
        imports.append(dict(item,source=str(source),imported_sha256=digest))
    (output/'imported_seed_provenance.json').write_text(json.dumps(imports,indent=2))
    ref=[]
    for p in sorted(Path(reference_dir).glob('*.jpg')):
        with Image.open(p) as im:width,height=im.size
        ref.append({'filename':p.name,'width':width,'height':height,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),
                    'usage':'Local style/reference review only; not uploaded or copied into generated dataset; not labeled defect ground truth'})
    (output/'reference_inventory.json').write_text(json.dumps(ref,indent=2))
    (output/'style_guide.txt').write_text(STYLE)
    write_jsonl(output/'plan.jsonl',rows)
    report=refresh(output,rows)
    (output/'REVIEW_FIRST_THREE.md').write_text('''# First three imported generated images

All three originals were copied byte-for-byte after matching their provided SHA-256 values. Source files and generation metadata remain untouched.

The 1536x1024 photographs resemble the unfinished concrete/scaffold site in the nesseler references. The middle view shows a localized bend in the viewer-right rail near a rung junction; its intended condition must still be reviewed against visible pixels. The lower/upper crops do not independently establish all rail defects. Apparent identity/background similarity is provisional, not verified 3D identity. No functional checks, inspection sticker dates or safe-use ground truth can be inferred from these three images. All annotations remain reviewed=false.
''')
    (output/'RUNBOOK.md').write_text('''# Complete photoreal synthetic dataset generation

This folder contains the complete plan, imported seed images and a pending job list. Read status.json: PENDING means no image exists, not a completed dataset. Reference photos were inspected locally; they are not uploaded or redistributed. Do not publish/merge this dataset without separate team authorization.

The full plan includes v1's 2,208 requested views plus 480 additional holdout-identity views, giving all seven sections train/val/test groups. The three existing images are preserved. All generated images remain unreviewed candidates; the requested condition/angle is not ground truth. Human-only function, grip, sharpness and lubrication fields cannot become image-only safe labels.

Direct API execution in the agent sandbox failed DNS resolution. OPENAI_API_KEY was not configured in the environment. No new images were generated or API credits spent. Use a network-enabled local terminal with the bundled imagegen CLI. Set a replacement key privately with read -s; do not paste it into command text or Git. Install openai in an isolated environment if the CLI reports it missing.

Before spending, review actual current pricing and the account balance. status.json's output estimate is historical, excludes inputs/retries and is NOT a spend cap. The bundled CLI does not expose a hard dollar cap or save usage receipts. Do not launch an unrestricted full batch against a $75 balance; use deliberately approved small batches and check billing. A max-request count is not an exact dollar limit.

Commands (from ULTRON root):

    read -r -s -p 'OpenAI API key: ' OPENAI_API_KEY; export OPENAI_API_KEY; printf '\\n'
    python3 -m pip install openai
    python3 scripts/prepare_photoreal_dataset.py --resume datasets/nesseler_photoreal_v2
    python3 /home/vivek/.codex/skills/.system/imagegen/scripts/image_gen.py generate-batch --input datasets/nesseler_photoreal_v2/jobs.pending.jsonl --out-dir datasets/nesseler_photoreal_v2/images --model gpt-image-2 --size 1536x1024 --quality medium --concurrency 2 --max-attempts 1 --fail-fast --no-augment

Run the API command only after deciding the spending budget and batch size. No automatic model downgrade is allowed. To resume, first run the preparer's --resume command, which rebuilds the pending file from decoded existing PNGs. Do NOT rerun an old pending list: the bundled CLI may charge before noticing an output file already exists. Corrupt files stop resume and must be preserved/quarantined before resubmission. Do not use --force.

Use the pending manifest to choose a balanced initial batch rather than spending only on the first rail category. Keep all sibling views/conditions in their original group split. Independent text-only image requests cannot guarantee consistent 360 identities; use reference-conditioned edits through the bundled edit command and image review where consistency is required. Do not claim calibrated geometry or a verified 360 sequence from these generation targets.

After each batch run --resume again, retain actual model/usage/generation receipts separately, review generated pixels for defect visibility, identity drift, impossible joints, annotation leakage and duplicate images, and add reviewer-attributed labels separately. Training/export is blocked until reviewed annotations exist. Keep real evaluation captures separate from generated images. Do not use photoreal synthetic quality as evidence of real-world damage-recognition accuracy.
''')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,default=ROOT/'datasets/nesseler_photoreal_v2')
    parser.add_argument('--resume',type=Path)
    parser.add_argument('--seed-dir',type=Path,default=PACK/'generated_images')
    parser.add_argument('--reference-dir',type=Path,default=ROOT.parent/'ctrl-hackathon-2026/site-safety-patrol/nesseler-data/photos')
    args=parser.parse_args()
    if args.resume:
        rows=[json.loads(line) for line in (args.resume/'plan.jsonl').read_text().splitlines()]
        report=refresh(args.resume,rows)
    else:report=prepare(args.out,args.seed_dir,args.reference_dir)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
