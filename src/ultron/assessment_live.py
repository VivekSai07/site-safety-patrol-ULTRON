"""One assessment -> validated seven-section record -> existing dashboard API.

Standard-library HTTP only. No image generation, robot commands or automatic
billable retries. Images/responses stay in an ignored, unique local run folder.
"""
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

from .assessment import (assess_coverage,build_request,dashboard_payload,evidence_bundle_id,
                         parse_api_response,photo_upload_plan,review_proposal)


class TransportError(RuntimeError):
    pass


def load_settings(env_file='.env',environ=None):
    """Parse assignments as data; never source/execute .env or print credentials."""
    environ=os.environ if environ is None else environ
    values={};path=Path(env_file)
    if path.is_file():
        for line in path.read_text().splitlines():
            line=line.strip()
            if not line or line.startswith('#') or '=' not in line:continue
            name,value=line.split('=',1);name=name.removeprefix('export ').strip();value=value.strip()
            if len(value)>=2 and value[0]==value[-1] and value[0] in ('"',"'"):value=value[1:-1]
            values[name]=value
    key=environ.get('OPENAI_API_KEY') or environ.get('OpenAI_API') or values.get('OPENAI_API_KEY') or values.get('OpenAI_API')
    if key and any(c.isspace() for c in key):raise ValueError('API credential contains whitespace; check .env')
    return {'key':key,
        'model_explicit':bool(environ.get('ULTRON_OPENAI_MODEL') or values.get('ULTRON_OPENAI_MODEL')),
        'model':environ.get('ULTRON_OPENAI_MODEL') or values.get('ULTRON_OPENAI_MODEL') or 'gpt-6.1-terra',
        'dashboard_url':environ.get('ULTRON_DASHBOARD_URL') or values.get('ULTRON_DASHBOARD_URL') or 'http://127.0.0.1:8000'}


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self,*args,**kwargs):return None


def _http(url,*,body=None,raw=None,key=None,timeout=15):
    headers={};data=None
    if raw is not None:data=raw;headers['Content-Type']='image/jpeg'
    elif body is not None:data=json.dumps(body,allow_nan=False).encode();headers['Content-Type']='application/json'
    if key:headers['Authorization']='Bearer '+key
    request=urllib.request.Request(url,data=data,headers=headers,method='GET' if data is None else 'POST')
    try:
        with urllib.request.build_opener(_NoRedirect()).open(request,timeout=timeout) as response:
            data=response.read(8_000_001)
            if len(data)>8_000_000:raise TransportError('HTTP response exceeds limit')
            return response.status,json.loads(data) if data else {}
    except urllib.error.HTTPError as exc:
        # Do not retain server error bodies, request headers or echoed invalid keys.
        return exc.code,{}
    except (urllib.error.URLError,OSError,ValueError) as exc:
        raise TransportError('Network/response failure: '+type(exc).__name__) from None


class DashboardClient:
    def __init__(self,url,timeout=15):
        parts=urllib.parse.urlsplit(url)
        if parts.scheme not in ('http','https') or not parts.hostname or parts.username or parts.password or parts.query or parts.fragment:
            raise ValueError('Dashboard URL must be an HTTP(S) origin without credentials/query')
        self.url=url.rstrip('/');self.timeout=timeout
    def get(self,path):return _http(self.url+path,timeout=self.timeout)
    def post(self,path,body=None,raw=None):return _http(self.url+path,body=body,raw=raw,timeout=self.timeout)
    def get_binary(self,path):
        try:
            with urllib.request.build_opener(_NoRedirect()).open(self.url+path,timeout=self.timeout) as response:
                data=response.read(32_000_001)
                if len(data)>32_000_000:raise TransportError('Dashboard binary response exceeds limit')
                return response.status,data
        except urllib.error.HTTPError as exc:return exc.code,b''
        except (urllib.error.URLError,OSError):raise TransportError('Dashboard binary transport unavailable') from None


def openai_call(body,key,timeout=120):
    # Hardcoded official host: a dashboard/custom URL can never receive this key.
    code,result=_http('https://api.openai.com/v1/responses',body=body,key=key,timeout=timeout)
    if code!=200:raise TransportError('OpenAI HTTP '+str(code))
    return result


def _save(path,value):
    with path.open('x') as handle:json.dump(value,handle,indent=2,ensure_ascii=False,allow_nan=False)


def run_milestone(manifest_path,output,*,model,api_call,dashboard,site_id='ultron-sim',_live_mode=False):
    manifest_path=Path(manifest_path).resolve();source_root=manifest_path.parent
    manifest=json.loads(manifest_path.read_text());output=Path(output)
    evidence_bundle_id(manifest)  # Validate safe IDs before constructing output paths.
    if type(manifest.get('synthetic')) is not bool:raise ValueError('Explicit synthetic provenance is required')
    output.mkdir(parents=True,exist_ok=False)
    report={'started_at':datetime.now(timezone.utc).isoformat(),'source_manifest':str(manifest_path),
        'model_requested':model,'api_validated':False,'dashboard_published':False,'api_attempts':0,
        'transport_mode':'LIVE_HTTP' if _live_mode else 'INJECTED_TEST','stage':'DASHBOARD_PREFLIGHT'}
    started=time.monotonic()
    mission_created=False;mission_ended=False

    def post(path,body=None,raw=None,expect=(200,201,204)):
        code,result=dashboard.post(path,body,raw)
        if code not in expect:raise TransportError('Dashboard rejected '+path.split('/')[2]+' operation: HTTP '+str(code))
        return result

    try:
        code,state=dashboard.get('/api/live')
        if code!=200 or not isinstance(state,dict) or 'mission' not in state:raise TransportError('Dashboard live endpoint unavailable or incompatible')
        code,config=dashboard.get('/api/notify/config')
        if code!=200 or not isinstance(config,dict) or config.get('channels')!=['dashboard']:
            raise TransportError('Use a dedicated dashboard with external notification channels disabled')
        report['stage']='EVIDENCE_PREFLIGHT'
        (output/'evidence').mkdir()
        for view in manifest['views']:
            path=(source_root/view['path']).resolve()
            if not path.is_relative_to(source_root):raise ValueError('Evidence path escapes source root')
            data=path.read_bytes()
            if hashlib.sha256(data).hexdigest()!=view.get('sha256'):raise ValueError('Source evidence checksum changed')
            new_path='evidence/'+view['image_id']+'.jpg'
            (output/new_path).write_bytes(data);view['path']=new_path
        manifest['mission_id']='API-'+uuid.uuid4().hex[:16]
        coverage=assess_coverage(manifest)
        if not coverage['complete']:raise ValueError('360 sampling incomplete; required milestone has missing cells')
        _save(output/'keyframes.json',manifest)
        request=build_request(manifest,output,model);request['max_output_tokens']=6000
        if model=='gpt-6.1-terra':request['reasoning']={'effort':'low'}
        report.update(bundle_id=evidence_bundle_id(manifest),mission_id=manifest['mission_id'],coverage=coverage)
        _save(output/'request.json',request)
        uploads=photo_upload_plan(manifest);_save(output/'upload-plan.json',uploads)
        report['stage']='DASHBOARD_START'
        post('/api/missions',{'mission_id':manifest['mission_id'],'site_id':site_id,
            'map_version':manifest.get('map_version',''),'synthetic':manifest.get('synthetic',False)},expect=(201,))
        mission_created=True
        for kind,data in [('LADDER_CONFIRMED',{'position':manifest['position'],
                'ladder_type':manifest['ladder_type'],'source':manifest.get('identity_source','CALLER_CONFIRMED')}),
                ('INSPECTION_STARTED',{'bundle_id':report['bundle_id']}),('COVERAGE_360',coverage)]:
            post('/api/events',{'mission_id':manifest['mission_id'],'asset_id':manifest['asset_id'],
                'type':kind,'time':time.monotonic()-started,'data':data})
        report['stage']='EVIDENCE_UPLOAD'
        for item in uploads:
            data=(output/item['path']).read_bytes()
            if hashlib.sha256(data).hexdigest()!=item['sha256']:raise ValueError('Snapshot evidence changed')
            post(f"/api/missions/{manifest['mission_id']}/photos/{item['photo_id']}",raw=data,expect=(201,))
            post('/api/events',{'mission_id':manifest['mission_id'],'asset_id':manifest['asset_id'],
                'type':'EVIDENCE_CAPTURED','time':time.monotonic()-started,
                'data':{'photo_id':item['photo_id'],'image_id':item['image_id']}})
        first=(output/uploads[0]['path']).read_bytes()
        post(f"/api/missions/{manifest['mission_id']}/frames/inspection_rgb",raw=first)
        report['stage']='OPENAI_RESPONSE';report['api_attempts']=1
        # Persist an attempt marker BEFORE the non-idempotent billable POST. No auto retry.
        _save(output/'api-attempt.json',{'bundle_id':report['bundle_id'],'model':model,'attempts':1,
            'warning':'An interrupted/failed request may have been billed; inspect before repeating'})
        try:
            response=api_call(request);_save(output/'api-response.json',response)
            _save(output/'usage.json',response.get('usage') or {})
            proposal=parse_api_response(response,manifest)
            report['api_validated']=response.get('status')=='completed' and not proposal['summary'].startswith('MANUAL REVIEW:')
            report['model_returned']=response.get('model')
        except TransportError as exc:
            report['api_error']=str(exc)
            proposal=review_proposal(manifest,'MANUAL REVIEW: OpenAI request failed; evidence retained')
        _save(output/'proposal.json',proposal)
        inspection=dashboard_payload(proposal,manifest);_save(output/'inspection.json',inspection)
        report['screening_status']=inspection['status'];report['stage']='INSPECTION_PUBLISH'
        post('/api/inspections',inspection,expect=(201,))
        report['dashboard_published']=True
        post(f"/api/missions/{manifest['mission_id']}/end",{'status':'EVIDENCE_RECORDED_REVIEW_PENDING',
            'summary':{'asset_id':manifest['asset_id'],'api_validated':report['api_validated'],'coverage':coverage}})
        mission_ended=True
        report['stage']='COMPLETE' if report['api_validated'] else 'API_REVIEW_REQUIRED'
    except (TransportError,ValueError,KeyError,OSError) as exc:
        # Exception text originates in our validators/transport, never auth headers.
        report['error']=str(exc)
    finally:
        if mission_created and not mission_ended:
            cleanup={'mission_ended':False}
            try:
                final_status='EVIDENCE_RECORDED_REVIEW_PENDING' if report['dashboard_published'] else 'ASSESSMENT_ABORTED'
                code,_=dashboard.post(f"/api/missions/{manifest['mission_id']}/end",{
                    'status':final_status,'summary':{'failed_stage':report['stage'],
                    'dashboard_published':report['dashboard_published'],'error':report.get('error','Interrupted')}})
                cleanup['mission_ended']=code in (200,201,204)
                if not cleanup['mission_ended']:cleanup['error']='Dashboard cleanup HTTP '+str(code)
            except TransportError as exc:cleanup['error']=str(exc)
            report['failure_cleanup']=cleanup
        report['elapsed_s']=round(time.monotonic()-started,3)
        _save(output/'job-status.json',report)
    return report


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest',type=Path)
    parser.add_argument('--out',type=Path)
    parser.add_argument('--env-file',type=Path,default=Path('.env'))
    parser.add_argument('--model')
    parser.add_argument('--dashboard-url')
    parser.add_argument('--site-id',default='ultron-sim')
    args=parser.parse_args();settings=load_settings(args.env_file)
    if not settings['key']:parser.exit(2,'Set OPENAI_API_KEY or OpenAI_API in the ignored .env file\n')
    model=args.model or settings['model'];url=args.dashboard_url or settings['dashboard_url']
    out=args.out or Path('runs')/('api-milestone-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    report=run_milestone(args.manifest,out,model=model,
        api_call=lambda body:openai_call(body,settings['key']),dashboard=DashboardClient(url),site_id=args.site_id,_live_mode=True)
    print(json.dumps({'output':str(out),**report},indent=2))
    if report['stage']!='COMPLETE':raise SystemExit(2)


if __name__=='__main__':main()
