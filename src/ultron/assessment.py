"""Seven-section prompt builder and conservative adapter for Aravind's dashboard.

No network calls: construct a Responses API body, validate proposals, export a
dashboard record. Neither schema validity nor the 360 sampling grid proves safety.
"""
import base64
from datetime import date
import hashlib
import json
import math
from pathlib import Path
import re

ASSETS=Path(__file__).parent/'assets/inspection_v1'
SAFE_ID=re.compile(r'^[A-Za-z0-9_.-]{1,64}$')


def rubric():
    return json.loads((ASSETS/'rubric.json').read_text())


def _criteria():
    return {c['key']:c for s in rubric()['sections'] for c in s['criteria']}


def _identity(manifest):
    for key in ('mission_id','asset_id'):
        if not isinstance(manifest.get(key),str) or not SAFE_ID.fullmatch(manifest[key]):
            raise ValueError(f'Invalid {key}')
    ids=[v.get('image_id') for v in manifest.get('views',[])]
    if any(not isinstance(i,str) or not SAFE_ID.fullmatch(i) for i in ids) or len(set(ids))!=len(ids):
        raise ValueError('Distinct, safe image IDs are required')
    return set(ids)


def evidence_bundle_id(manifest):
    """Bind the immutable job context and pixel hashes to the exact prompt/schema."""
    _identity(manifest)
    context={k:manifest.get(k) for k in ('mission_id','asset_id','map_version','inspection_date',
        'ladder_type','position','verified_not_applicable','synthetic')}
    context['views']=manifest.get('views',[])
    context['prompt_fingerprints']={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(ASSETS.iterdir()) if p.suffix in ('.txt','.json')}
    raw=json.dumps(context,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    return hashlib.sha256(raw).hexdigest()


def assess_coverage(manifest):
    """Caller-attested target-relative sampling audit, not surface/defect completeness."""
    _identity(manifest)
    accepted=set();rejected=[];image_hashes=set()
    for v in manifest.get('views',[]):
        try:
            angle=v['azimuth_deg'];band=v['band']
            if isinstance(angle,bool) or not isinstance(angle,(int,float)) or not math.isfinite(angle):raise ValueError()
            sector=(int((angle%360+22.5)//45)*45)%360
            delta=abs((angle-sector+180)%360-180)
            if delta>15 or band not in ('lower','middle','upper'):raise ValueError()
            if not all(v.get(k) is True for k in ('pose_valid','same_asset_confirmed','quality_accepted','region_visible')):raise ValueError()
            digest=v.get('sha256')
            if not isinstance(digest,str) or not re.fullmatch('[a-f0-9]{64}',digest) or digest in image_hashes:raise ValueError()
            image_hashes.add(digest);accepted.add((sector,band))
        except (KeyError,ValueError,TypeError):
            rejected.append(v.get('image_id'))
    missing=[{'azimuth_deg':angle,'band':band} for angle in range(0,360,45)
             for band in ('lower','middle','upper') if (angle,band) not in accepted]
    return {'complete':not missing,'accepted_cells':len(accepted),'required_cells':24,
        'missing_cells':missing,'rejected_image_ids':rejected,
        'scope':'Caller-verified view sampling only; not structural, functional or safe-use certification'}


def build_request(manifest,root,model):
    """Read verified local JPEGs, attach explicit IDs and require all 23 criteria."""
    _identity(manifest)
    if not manifest.get('views'):raise ValueError('At least one keyframe is required')
    if len(manifest['views'])>32:raise ValueError('At most 32 selected keyframes per request')
    if not model or not isinstance(model,str):raise ValueError('An explicit tested model ID is required')
    root=Path(root).resolve();content=[];seen={}
    for v in manifest['views']:
        if v.get('same_asset_confirmed') is not True:raise ValueError('Only confirmed same-asset images may be combined')
        path=(root/v['path']).resolve()
        if not path.is_relative_to(root):raise ValueError('Image path escapes evidence root')
        data=path.read_bytes()
        if not data.startswith(b'\xff\xd8') or not 0<len(data)<5_000_000:raise ValueError('Use dashboard-compatible JPEGs under 5 MB')
        from PIL import Image
        with Image.open(path) as im:im.verify()
        digest=hashlib.sha256(data).hexdigest()
        if digest!=v.get('sha256'):raise ValueError('Keyframe checksum mismatch')
        seen[v['image_id']]=digest
        content.extend([{'type':'input_text','text':json.dumps({'image_id':v['image_id'],'band':v.get('band'),'azimuth_deg':v.get('azimuth_deg')})},
            {'type':'input_image','image_url':'data:image/jpeg;base64,'+base64.b64encode(data).decode(),'detail':'high'}])
    metadata={k:manifest.get(k) for k in ('asset_id','mission_id','map_version','ladder_type','inspection_date','verified_not_applicable')}
    metadata['bundle_id']=evidence_bundle_id(manifest)
    content.insert(0,{'type':'input_text','text':'Inspect only this evidence bundle: '+json.dumps(metadata)})
    prompt=(ASSETS/'system.txt').read_text()+'\n'+'\n'.join(p.read_text() for p in sorted(ASSETS.glob('section_*.txt')))
    return {'model':model,'store':False,'instructions':prompt,'input':[{'role':'user','content':content}],
        'text':{'format':{'type':'json_schema','name':'ultron_seven_sections_v1','strict':True,
                         'schema':json.loads((ASSETS/'response_schema.json').read_text())}}}


def validate_proposal(proposal,manifest):
    ids=_identity(manifest);criteria=_criteria()
    if not isinstance(proposal,dict) or set(proposal)!={'bundle_id','asset_id','criteria','label','summary'}:
        raise ValueError('Invalid proposal fields')
    if proposal['bundle_id']!=evidence_bundle_id(manifest):raise ValueError('Stale or mismatched evidence bundle')
    if proposal['asset_id']!=manifest['asset_id'] or not isinstance(proposal['summary'],str):raise ValueError('Wrong asset or summary')
    if not isinstance(proposal['criteria'],dict) or set(proposal['criteria'])!=set(criteria):raise ValueError('All 23 criterion keys required exactly once')
    verified_na=manifest.get('verified_not_applicable',[])
    if not isinstance(verified_na,list) or not set(verified_na)<=set(criteria):raise ValueError('Invalid caller applicability')
    for key,item in proposal['criteria'].items():
        if not isinstance(item,dict) or set(item)!={'state','reason','confidence','evidence_ids'}:raise ValueError('Invalid criterion fields')
        state=item['state'];confidence=item['confidence'];refs=item['evidence_ids']
        if state not in ('OK','DEFECT','UNKNOWN','NOT_CHECKED','N_A'):raise ValueError('Invalid criterion state')
        if not isinstance(item['reason'],str) or not item['reason'].strip():raise ValueError('Literal reasons required')
        if isinstance(confidence,bool) or not isinstance(confidence,(int,float)) or not math.isfinite(confidence) or not 0<=confidence<=1:raise ValueError('Invalid confidence')
        if not isinstance(refs,list) or any(not isinstance(i,str) or i not in ids for i in refs):raise ValueError('Unknown evidence ID')
        if state in ('OK','DEFECT') and not refs:raise ValueError('Observed states require evidence')
        if state=='OK' and criteria[key]['mode']=='person':raise ValueError('Human-only criterion cannot pass from images')
        if state=='N_A' and key not in verified_na:raise ValueError('Applicability must be caller-verified')
    label=proposal['label']
    if not isinstance(label,dict) or set(label)!={'raw_text','due_date_iso','due_date_explicit','evidence_ids'}:raise ValueError('Invalid label fields')
    if label['raw_text'] is not None and not isinstance(label['raw_text'],str):raise ValueError('Invalid raw label text')
    if type(label['due_date_explicit']) is not bool:raise ValueError('Invalid explicit-date flag')
    if not isinstance(label['evidence_ids'],list) or any(not isinstance(i,str) or i not in ids for i in label['evidence_ids']):raise ValueError('Unknown label evidence')
    if label['due_date_iso'] is not None:
        text=label['due_date_iso']
        if not isinstance(text,str) or not re.fullmatch(r'\d{4}-\d{2}(-\d{2})?',text):raise ValueError('Use explicit YYYY-MM or YYYY-MM-DD; do not invent date precision')
        date.fromisoformat(text+'-01' if len(text)==7 else text)
        if not label['due_date_explicit'] or not label['raw_text'] or not label['evidence_ids']:raise ValueError('Due date needs explicit readable evidence')
    return proposal


def dashboard_payload(proposal,manifest):
    """Exact current dashboard field names; model-only no-defect always needs review."""
    validate_proposal(proposal,manifest)
    photo_ids={p['image_id']:p['photo_id'] for p in photo_upload_plan(manifest)}
    position=manifest.get('position')
    if not isinstance(position,list) or len(position)!=3 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in position):
        raise ValueError('Real map position required; never fabricate [0,0,0]')
    items={key:item['state'] for key,item in proposal['criteria'].items()}
    for key,c in _criteria().items():
        if c['mode']=='person' and items[key] in ('UNKNOWN','NOT_CHECKED'):items[key]='NOT_CHECKED'
    checklist=dict(items)
    for s in rubric()['sections']:
        values={items[c['key']] for c in s['criteria']}
        checklist[s['key']]=next((v for v in ('DEFECT','UNKNOWN','OK','NOT_CHECKED') if v in values),'N_A')
    label=proposal['label'];overdue=False
    if label['due_date_iso']:
        due=label['due_date_iso'];today=date.fromisoformat(manifest['inspection_date'])
        overdue=(int(due[:4]),int(due[5:7]))<(today.year,today.month) if len(due)==7 else date.fromisoformat(due)<today
    label_status='OVERDUE' if overdue else ('VALID' if label['due_date_iso'] else 'UNREADABLE')
    status='FLAGGED' if 'DEFECT' in items.values() or overdue else 'MANUAL_REVIEW'
    coverage=assess_coverage(manifest)
    reason=proposal['summary']+f" | 360 view sampling: {coverage['accepted_cells']}/24; human completion pending"
    return {'mission_id':manifest['mission_id'],'asset_id':manifest['asset_id'],
        'ladder_type':manifest.get('ladder_type','UNKNOWN'),'status':status,'reason':reason,
        'checks':{},'checklist':checklist,
        'photos':[{'id':photo_ids[v['image_id']],'caption':f"{v['image_id']}: {v.get('azimuth_deg','?')} deg / {v.get('band','unknown')}",'camera':'inspection_rgb'} for v in manifest['views']],
        'item_photos':{k:[photo_ids[i] for i in v['evidence_ids']] for k,v in proposal['criteria'].items() if v['evidence_ids']},
        'ladder':{'sticker_status':label_status,'sticker_due':label['due_date_iso']},
        'missing':[k for k,v in items.items() if v=='UNKNOWN'],
        'label_status':label_status,'position':position,'safe_to_use':False,
        'tactile_checks_pending':True,'human_signoff_required':True}


def photo_upload_plan(manifest):
    """Mission-wide dashboard IDs bind the full asset ID and immutable pixel hash."""
    _identity(manifest);plan=[]
    for v in manifest.get('views',[]):
        digest=v.get('sha256')
        if not isinstance(digest,str) or not re.fullmatch('[a-f0-9]{64}',digest):raise ValueError('Photo publication needs verified checksums')
        binding=manifest['asset_id']+':'+digest
        photo_id='p-'+hashlib.sha256(binding.encode()).hexdigest()[:48]
        plan.append({'image_id':v['image_id'],'photo_id':photo_id,'path':v.get('path'),'sha256':digest})
    return plan


def review_proposal(manifest,reason):
    """Explicit conservative fallback for network/refusal/validation failure."""
    return {'bundle_id':evidence_bundle_id(manifest),'asset_id':manifest['asset_id'],'criteria':{key:{
        'state':'NOT_CHECKED' if c['mode']=='person' else 'UNKNOWN',
        'reason':reason,'confidence':0.,'evidence_ids':[]} for key,c in _criteria().items()},
        'label':{'raw_text':None,'due_date_iso':None,'due_date_explicit':False,'evidence_ids':[]},
        'summary':reason}


def parse_api_response(response,manifest):
    """Accept a Responses API JSON envelope; return review on refusal/incomplete/bad output.

    Caller retains the raw response alongside this result. This never calls OpenAI.
    """
    try:
        if response.get('status')!='completed':raise ValueError('API response incomplete or failed')
        texts=[]
        for output in response.get('output',[]):
            for item in output.get('content',[]):
                if item.get('type')=='refusal':raise ValueError('API refused assessment')
                if item.get('type')=='output_text':texts.append(item['text'])
        if len(texts)!=1:raise ValueError('Expected one structured assessment output')
        return validate_proposal(json.loads(texts[0]),manifest)
    except (ValueError,KeyError,TypeError,AttributeError) as exc:
        return review_proposal(manifest,'MANUAL REVIEW: '+str(exc))


def main():
    import argparse
    parser=argparse.ArgumentParser(description='Offline API prompt and dashboard payload preparation; no API calls')
    parser.add_argument('action',choices=['request','dashboard','coverage','uploads'])
    parser.add_argument('manifest',type=Path)
    parser.add_argument('--response',type=Path)
    parser.add_argument('--root',type=Path)
    parser.add_argument('--model')
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    manifest=json.loads(args.manifest.read_text())
    if args.action=='request':result=build_request(manifest,args.root or args.manifest.parent,args.model)
    elif args.action=='coverage':result=assess_coverage(manifest)
    elif args.action=='uploads':result=photo_upload_plan(manifest)
    else:
        if not args.response:parser.error('--response is required')
        result=dashboard_payload(json.loads(args.response.read_text()),manifest)
    with args.out.open('x') as f:json.dump(result,f,indent=2,ensure_ascii=False)
    print(args.out)


if __name__=='__main__':main()
