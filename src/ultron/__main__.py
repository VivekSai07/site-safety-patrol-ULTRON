import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import uuid


def main():
    parser=argparse.ArgumentParser(description='ULTRON B2 simulation and evidence memory')
    sub=parser.add_subparsers(dest='command',required=True)
    demo=sub.add_parser('demo')
    demo.add_argument('--out',type=Path)
    demo.add_argument('--database',type=Path)
    demo.add_argument('--patrols',type=int,choices=[1,2],default=2)
    demo.add_argument('--variant',choices=['normal','missing','blocked','blocked-closeup','intervention'],default='normal')
    demo.add_argument('--no-recovery',action='store_true')
    demo.add_argument('--viewer',action='store_true')
    demo.add_argument('--motion',choices=['smooth','legacy'],default='smooth')
    demo.add_argument('--rerun',action='store_true',help='Export/open Rerun after simulation')
    query=sub.add_parser('query')
    query.add_argument('database',type=Path)
    query.add_argument('--text',default='')
    query.add_argument('--near',nargs=2,type=float)
    query.add_argument('--radius',type=float,default=2.)
    rr=sub.add_parser('rerun')
    rr.add_argument('run',type=Path)
    rr.add_argument('--spawn',action='store_true')
    rr.add_argument('--follow',action='store_true',help='Tail a running simulation until Ctrl-C')
    dimos=sub.add_parser('dimos-ingest')
    dimos.add_argument('run',type=Path)
    dimos.add_argument('--database',type=Path,required=True)
    analysis=sub.add_parser('analyze',help='Analyze selected same-ladder images; no robot commands')
    analysis.add_argument('images',nargs='+',type=Path)
    analysis.add_argument('--out',type=Path,required=True)
    analysis.add_argument('--weights',help='Local YOLOE/YOLO-World checkpoint')
    analysis.add_argument('--family',choices=['yoloe','world'],default='yoloe')
    analysis.add_argument('--qwen',help='Local model directory or HF model ID with --allow-download')
    analysis.add_argument('--ladder-type',default='UNKNOWN')
    analysis.add_argument('--device',default='cpu')
    analysis.add_argument('--allow-download',action='store_true')
    dataset=sub.add_parser('export-dataset')
    dataset.add_argument('run',type=Path)
    dataset.add_argument('--out',type=Path,required=True)
    dataset.add_argument('--split',choices=['train','val','test'],default='train')
    args=parser.parse_args()
    try:
        if args.command=='demo':
            if not args.viewer:
                os.environ.setdefault('MUJOCO_GL','egl')
            from .mission import run_demo
            output=args.out or Path('runs')/(datetime.now().strftime('%Y%m%d-%H%M%S-')+uuid.uuid4().hex[:6])
            result=run_demo(output,args.database,args.patrols,args.variant,not args.no_recovery,args.viewer,args.motion)
            print(json.dumps({'output':str(output),'missions':[{k:m[k] for k in ['mission_id','status','found','missing_expected']} for m in result['missions']]},indent=2))
            if args.rerun:
                from .rerun_view import export_rerun
                export_rerun(output,spawn=True)
        elif args.command=='query':
            if not args.database.is_file():
                parser.error('Database does not exist')
            from .memory import Registry
            db=Registry(args.database)
            try:
                print(json.dumps(db.query(args.text,args.near,args.radius),indent=2))
            finally:
                db.close()
        elif args.command=='rerun':
            from .rerun_view import export_rerun
            export_rerun(args.run,args.spawn,follow=args.follow)
        elif args.command=='export-dataset':
            from .dataset import export_dataset
            print(json.dumps(export_dataset(args.run,args.out,split=args.split),indent=2))
        elif args.command=='analyze':
            from .inspection import REQUIRED
            if args.ladder_type not in REQUIRED:
                parser.error('Unknown ladder type; choose '+', '.join(REQUIRED))
            from .perception.pipeline import analyze
            result=analyze(args.images,args.out,weights=args.weights,family=args.family,qwen=args.qwen,
                           ladder_type=args.ladder_type,allow_download=args.allow_download,device=args.device)
            print(json.dumps({'status':result['status'],'report':str(args.out/'report.html')},indent=2))
        else:
            from .dimos_bridge import ingest
            ingest(args.run,args.database)
    except (RuntimeError,FileExistsError,ValueError) as exc:
        parser.exit(2,f'{exc}\n')


if __name__=='__main__':
    main()
