"""Per-ladder evidence jobs for the live tailer; no robot-control authority."""
from datetime import date
import hashlib
import json
from pathlib import Path
from .assessment import (assess_coverage, build_request, dashboard_payload,
                         parse_api_response, review_proposal, SAFE_ID, evidence_bundle_id, photo_upload_plan)


def record_manifest(record, root, *, pack='inspection_v1'):
    from .assessment import LADDER_PACKS
    if pack not in LADDER_PACKS:raise ValueError('Use a supported ladder assessment pack')
    root=Path(root).resolve()
    views=[]
    for n, frame in enumerate(record.get('views', [])):
        path=(root/frame['rgb']).resolve()
        if not path.is_relative_to(root):
            raise ValueError('Evidence outside run')
        data=path.read_bytes()
        view={k:frame[k] for k in ('azimuth_deg','band','pose_valid','same_asset_confirmed',
              'region_visible','quality_accepted','target_box_xyxy','pose','camera_position','camera_rotation','intrinsics') if k in frame}
        view.update(image_id=f'frame-{n:03}', path=str(path.relative_to(root)),
                    sha256=hashlib.sha256(data).hexdigest())
        # Only explicitly labelled fixture identity or caller-verified captures qualify.
        view.setdefault('same_asset_confirmed', bool(record.get('synthetic') and record.get('oracle_key')))
        views.append(view)
    manifest={k:record[k] for k in ('mission_id','asset_id','ladder_type','position')}
    manifest.update(map_version='ultron_mock_site_v1', inspection_date=date.today().isoformat(),
                    synthetic=record.get('synthetic',False), views=views, verified_not_applicable=[],
                    prior_status=record.get('status'), prior_checks=record.get('checks', {}),
                    identity_source='SIMULATION_ORACLE' if record.get('oracle_key') else 'CALLER_VERIFICATION_REQUIRED')
    if pack!='inspection_v1':manifest['pack']=pack
    manifest['coverage']=assess_coverage(manifest)
    return manifest


def assess_job(manifest, root, output, *, model, api_call, ledger=None, review_reason=None):
    """Exactly one attempt, validated structured response or explicit review fallback.

    No automatic paid retry. Persist the raw response and the full per-criterion
    reasons/confidences locally; model confidence is uncalibrated, never safety probability.
    """
    output=Path(output); output.mkdir(parents=True,exist_ok=False)
    def save(name, body):
        (output/name).write_text(json.dumps(body,indent=2,allow_nan=False))
    save('keyframes.json',manifest)
    state='REVIEW_INPUT_INVALID'
    try:
        if review_reason:
            state=review_reason
            raise ValueError('Review-only job')
        body=build_request(manifest,root,model)
        body['max_output_tokens']=6000
        save('request.json',body)
        save('api-attempt.json',{'attempts':1, 'model':model})
        state='API_TRANSPORT_FAILED'
        response=api_call(body)
        save('response.json',response)
        proposal=parse_api_response(response,manifest)
        state='MODEL_PROPOSAL' if not proposal['summary'].startswith('MANUAL REVIEW:') else 'REVIEW_RESPONSE_INVALID'
    except (OSError, ValueError, RuntimeError):
        proposal=review_proposal(manifest,'Manual review: '+state)
    save('proposal.json',proposal)
    payload=dashboard_payload(proposal,manifest)
    payload.update(criterion_details={k:{'reason':v['reason'],'confidence':v['confidence']}
                   for k,v in proposal['criteria'].items()},
                   assessment_source=state,view_coverage=assess_coverage(manifest))
    prior=manifest.get('prior_checks',{})
    if manifest.get('prior_status')=='FLAGGED' or 'FAIL' in prior.values():
        payload.update(status='FLAGGED', checks=prior)
        payload['reason']='Prior hard failure retained. '+payload['reason']
    if ledger is not None:
        ledger.retain(manifest,payload)
    save('result.json',payload)
    save('job-status.json',{'state':state,'safe_to_use':False, 'coverage':assess_coverage(manifest)})
    return payload


class AssessmentLedger:
    """Persistent conservative model-flag history, scoped to a shared map/asset registry.

    This is separate from the patrol's simulation-oracle registry. It cannot clear
    flags or grant motion authority. Use a new ledger when map identity changes.
    """
    def __init__(self,path):
        self.path=Path(path)

    def retain(self,manifest,payload):
        # Linux target: serialize read/merge/replace across tailer processes.
        import fcntl
        self.path.parent.mkdir(parents=True,exist_ok=True)
        with self.path.with_suffix(self.path.suffix+'.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            self._retain_locked(manifest,payload)

    def _retain_locked(self,manifest,payload):
        entries=json.loads(self.path.read_text()) if self.path.exists() else {}
        key=manifest['map_version']+':'+manifest['asset_id']
        previous=entries.get(key)
        defects={k for k,v in payload['checklist'].items() if v=='DEFECT'}
        photos={p['photo_id']:p for p in photo_upload_plan(manifest)}
        findings={}
        for criterion in defects:
            if criterion not in payload['criterion_details']:continue
            detail=payload['criterion_details'][criterion]
            findings[criterion]={**detail,'mission_id':manifest['mission_id'],
                'bundle_id':evidence_bundle_id(manifest),
                'photos':[photos[i] for i in payload['item_photos'].get(criterion,[]) if i in photos]}
        if previous:
            defects.update(previous['defects'])
            payload['status']='FLAGGED'
            payload['reason']='Unresolved assessment flag from '+previous['mission_id']+'. '+payload['reason']
            payload['retained_findings']=previous.get('findings',{})
            for criterion in previous['defects']:
                payload['checklist'][criterion]='DEFECT'
                # Never cite current clean images/scores as evidence for an older defect.
                payload['item_photos'].pop(criterion,None)
                old=previous.get('findings',{}).get(criterion)
                if criterion in payload['criterion_details']:
                    payload['criterion_details'][criterion]={
                        'reason':'Retained unresolved finding from '+previous['mission_id']+': '+(old['reason'] if old else 'Original detail unavailable; human review'),
                        'confidence':old['confidence'] if old else 0.}
            findings.update(previous.get('findings',{}))
        if payload['status']=='FLAGGED':
            entries[key]={'mission_id':previous['mission_id'] if previous else manifest['mission_id'],
                          'defects':sorted(defects),'findings':findings}
            self.path.parent.mkdir(parents=True,exist_ok=True)
            temporary=self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps(entries,indent=2))
            temporary.replace(self.path)


class LiveAssessmentWorker:
    """Single concurrent API call, bounded mission jobs, publication on the tailer thread."""
    def __init__(self, root, *, model, api_call, max_jobs=16, ledger_path=None):
        from concurrent.futures import ThreadPoolExecutor
        if max_jobs<1:raise ValueError('Positive job limit required')
        self.root=Path(root).resolve();self.model=model;self.api_call=api_call
        self.max_jobs=max_jobs;self.pending={};self.completed=set()
        self.ledger=AssessmentLedger(ledger_path or self.root/'api-assessments'/'unresolved-flags.json')
        self.reserved_calls=0
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='ultron-assessment')

    def submit(self, relative_path, mission_id, review_reason=None):
        path=(self.root/relative_path).resolve()
        if not path.is_relative_to(self.root):raise ValueError('Job manifest outside run')
        m=json.loads(path.read_text());m['mission_id']=mission_id
        if not SAFE_ID.fullmatch(m['asset_id']) or not SAFE_ID.fullmatch(mission_id):
            raise ValueError('Invalid job identity')
        identity=(mission_id,m['asset_id'])
        if identity in self.pending or identity in self.completed:return
        if self.reserved_calls>=self.max_jobs:review_reason='REVIEW_API_BUDGET_EXHAUSTED'
        if not review_reason:self.reserved_calls+=1
        output=self.root/'api-assessments'/mission_id/m['asset_id']
        if output.exists():
            from concurrent.futures import Future
            if json.loads((output/'keyframes.json').read_text())!=m:
                raise ValueError('Existing assessment job has different evidence')
            payload=json.loads((output/'result.json').read_text())
            future=Future();future.set_result(payload)
            self.pending[identity]=(m,future)
            return
        self.pending[identity]=(m,self.pool.submit(assess_job,m,self.root,output,
                                                model=self.model,api_call=self.api_call,ledger=self.ledger,review_reason=review_reason))

    def publish_ready(self, api, wait=False):
        count=0
        for identity,(m,future) in list(self.pending.items()):
            if not wait and not future.done():continue
            payload=future.result()
            from .assessment import photo_upload_plan
            for photo in photo_upload_plan(m):
                path=(self.root/photo['path']).resolve()
                if not path.is_relative_to(self.root) or hashlib.sha256(path.read_bytes()).hexdigest()!=photo['sha256']:
                    raise ValueError('Evidence changed before publication')
                status,_=api.post(f"/api/missions/{m['mission_id']}/photos/{photo['photo_id']}",
                                  raw=path.read_bytes(), content_type='image/jpeg')
                if status not in (200,201):raise RuntimeError('Photo publication failed')
            status,_=api.post('/api/inspections',payload)
            if status==409 and hasattr(api,'get'):
                code,stored=api.get('/api/missions/'+m['mission_id'])
                records=stored.get('inspections',[]) if code==200 and isinstance(stored,dict) else []
                from .dashboard.models import Inspection
                normalized=Inspection(**payload).model_dump(mode='json')
                if any(all(r.get(k)==v for k,v in normalized.items()) for r in records):status=201
            if status!=201:raise RuntimeError('Assessment publication failed: HTTP '+str(status))
            self.completed.add(identity);del self.pending[identity];count+=1
        return count

    def close(self):
        self.pool.shutdown(wait=True)
