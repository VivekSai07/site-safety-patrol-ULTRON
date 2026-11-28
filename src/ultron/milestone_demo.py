"""Start Aravind's actual dashboard and run the first live assessment automatically."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid
import webbrowser

from .assessment_live import DashboardClient,TransportError,_http,load_settings,openai_call,run_milestone

ROOT=Path(__file__).resolve().parents[2]


def build_server_command(python,port):
    return [python,'-m','uvicorn','ultron.milestone_demo:create_milestone_app','--factory',
            '--host','127.0.0.1','--port',str(port)]


def choose_model(available,explicit=None):
    if explicit:return explicit
    # These model contracts were checked in official OpenAI documentation.
    for candidate in ('gpt-6.1-terra','gpt-4.1'):
        if candidate in available:return candidate
    raise ValueError('No documented vision/structured-output model found; set ULTRON_OPENAI_MODEL to an available tested model')


def select_manifest(requested):
    if requested is not None:
        path=Path(requested)
        if not path.is_file():raise ValueError('Explicit evidence manifest does not exist; no synthetic substitution is allowed')
        return path
    return ROOT/'runs/inspection-360-v1/keyframes.json'


def create_milestone_app():
    """Uvicorn factory; isolated data and no external notification channels."""
    from .dashboard.app import create_app
    from .dashboard.notify import load_config
    runtime=Path(os.environ['ULTRON_MILESTONE_SERVICE_DIR']);runtime.mkdir(parents=True,exist_ok=True)
    cfg=load_config();cfg['public_url']='http://127.0.0.1:'+os.environ['ULTRON_MILESTONE_PORT']
    return create_app(db_path=str(runtime/'dashboard.sqlite'),media_dir=str(runtime/'media'),
                      notify_config=cfg,channels=[])


def socket_check():
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
            s.bind(('127.0.0.1',0));return s.getsockname()[1]
    except OSError:
        raise TransportError('This runtime denies local TCP sockets. Run scripts/first-milestone.sh in the normal desktop terminal.') from None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path)
    parser.add_argument('--model')
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--no-wait',action='store_true')
    args=parser.parse_args()
    try:manifest=select_manifest(args.manifest)
    except ValueError as exc:parser.exit(2,str(exc)+'\n')
    settings=load_settings(ROOT/'.env')
    if not settings['key']:parser.exit(2,'Add OpenAI_API or OPENAI_API_KEY to the ignored .env first\n')
    try:port=socket_check()
    except TransportError as exc:parser.exit(2,str(exc)+'\n')
    if args.check:
        print('Credential loaded privately; loopback socket setup works. No API request made.');return
    runtime=ROOT/'runs'/('first-milestone-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    runtime.mkdir(parents=True,exist_ok=False)
    url=f'http://127.0.0.1:{port}';dashboard=DashboardClient(url,timeout=2)
    child_env=dict(os.environ,ULTRON_MILESTONE_SERVICE_DIR=str(runtime/'service'),ULTRON_MILESTONE_PORT=str(port))
    child_env.pop('OPENAI_API_KEY',None);child_env.pop('OpenAI_API',None)
    child_env['PYTHONPATH']=str(ROOT/'src')
    form=ROOT.parent/'ctrl-hackathon-2026/site-safety-patrol/nesseler-data/kontrollblatt_checkliste_leitern_tritte.pdf'
    if form.is_file():child_env.setdefault('ULTRON_NESSELER_FORM',str(form))
    server=None;log=(runtime/'dashboard-server.log').open('x');result_code=2
    try:
        server=subprocess.Popen(build_server_command(sys.executable,port),cwd=ROOT,env=child_env,
            stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            if server.poll() is not None:raise TransportError('Dashboard startup failed; see '+str(runtime/'dashboard-server.log'))
            try:
                code,state=dashboard.get('/api/live')
                if code==200 and 'mission' in state:break
            except TransportError:pass
            time.sleep(.2)
        else:raise TransportError('Dashboard did not become ready; see its server log')
        print('Dashboard ready:',url,flush=True)
        if not args.no_browser:webbrowser.open(url)
        explicit=args.model or (settings['model'] if settings['model_explicit'] else None)
        code,models=_http('https://api.openai.com/v1/models',key=settings['key'],timeout=15)
        if code!=200 or not isinstance(models,dict):raise TransportError('OpenAI authentication/model discovery failed: HTTP '+str(code))
        model=choose_model([item['id'] for item in models.get('data',[])],explicit=explicit)
        if not manifest.is_file():
            if args.manifest is not None:raise ValueError('Explicit evidence manifest disappeared; aborting without substitution')
            os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('LP_NUM_THREADS','2')
            from .inspect360 import capture360
            capture360(runtime/'capture');manifest=runtime/'capture/keyframes.json'
        print('Running ONE assessment request with model:',model,flush=True)
        report=run_milestone(manifest,runtime/'assessment',model=model,api_call=lambda body:openai_call(body,settings['key']),
                             dashboard=dashboard,_live_mode=True)
        persisted=False;pdf_saved=False
        if report.get('mission_id') and report['dashboard_published']:
            code,record=dashboard.get('/api/missions/'+report['mission_id'])
            persisted=code==200 and any(i['asset_id']==json.loads(manifest.read_text())['asset_id'] for i in record.get('inspections',[]))
        if persisted:
            asset=json.loads(manifest.read_text())['asset_id']
            code,data=dashboard.get_binary('/api/sites/ultron-sim/ladders/'+asset+'/report.pdf')
            if code==200 and data.startswith(b'%PDF'):
                (runtime/'assessment/protocol.pdf').write_bytes(data);pdf_saved=True
        summary={'dashboard_url':url,'runtime':str(runtime),'api_validated':report['api_validated'],
            'dashboard_published':report['dashboard_published'],'dashboard_readback_verified':persisted,
            'protocol_pdf_saved':pdf_saved,
            'success':report['api_validated'] and report['dashboard_published'] and persisted}
        (runtime/'milestone-result.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps(summary,indent=2),flush=True)
        result_code=0 if summary['success'] else 2
        print('Dashboard remains available until Ctrl+C. Evidence, responses, usage and DB stay in '+str(runtime),flush=True)
        if not args.no_wait:
            try:server.wait()
            except KeyboardInterrupt:pass
    except (TransportError,ValueError,OSError) as exc:
        (runtime/'bootstrap-failure.json').write_text(json.dumps({'error':str(exc),'dashboard_url':url,'success':False},indent=2))
        print(str(exc),file=sys.stderr)
    finally:
        if server is not None and server.poll() is None:
            server.terminate()
            try:server.wait(timeout=5)
            except subprocess.TimeoutExpired:server.kill();server.wait()
        log.close()
    raise SystemExit(result_code)


if __name__=='__main__':main()
