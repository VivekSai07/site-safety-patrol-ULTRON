import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import sys
import uuid


def main():
    parser=argparse.ArgumentParser(description='ULTRON B2 simulation and evidence memory')
    sub=parser.add_subparsers(dest='command',required=True)
    demo=sub.add_parser('demo')
    demo.add_argument('--out',type=Path)
    demo.add_argument('--database',type=Path)
    demo.add_argument('--patrols',type=int,choices=[1,2],default=2)
    demo.add_argument('--variant',choices=['normal','missing','blocked','blocked-closeup','intervention','open-orbit'],default='normal')
    demo.add_argument('--no-recovery',action='store_true')
    demo.add_argument('--viewer',action='store_true')
    demo.add_argument('--view-policy',choices=['bounded','orbit'],default='bounded')
    demo.add_argument('--detector',choices=['oracle','yolo'],default='oracle')
    demo.add_argument('--inspector',choices=['oracle','qwen'],default='oracle')
    demo.add_argument('--weights')
    demo.add_argument('--qwen')
    demo.add_argument('--allow-download',action='store_true')
    demo.add_argument('--device',default='cpu')
    demo.add_argument('--detector-family',choices=['yoloe','world','custom'],default='yoloe')
    demo.add_argument('--motion',choices=['smooth','legacy'],default='smooth')
    demo.add_argument('--safety',action='store_true',help='Also check fire/first-aid/escape/PPE props during patrol 0')
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
    analysis.add_argument('--family',choices=['yoloe','world','custom'],default='yoloe')
    analysis.add_argument('--qwen',help='Local model directory or HF model ID with --allow-download')
    analysis.add_argument('--ladder-type',default='UNKNOWN')
    analysis.add_argument('--device',default='cpu')
    analysis.add_argument('--allow-download',action='store_true')
    dataset=sub.add_parser('export-dataset')
    dataset.add_argument('run',type=Path)
    dataset.add_argument('--out',type=Path,required=True)
    dataset.add_argument('--split',choices=['train','val','test'],default='train')
    train=sub.add_parser('train-detector')
    train.add_argument('dataset',type=Path)
    train.add_argument('--out',type=Path,required=True)
    train.add_argument('--weights',required=True)
    train.add_argument('--epochs',type=int,default=10)
    train.add_argument('--device',default='cpu')
    train.add_argument('--allow-download',action='store_true')
    train.add_argument('--dry-run',action='store_true')
    corpus=sub.add_parser('generate-corpus')
    corpus.add_argument('--out',type=Path,required=True)
    corpus.add_argument('--scenes',type=int,default=6)
    corpus.add_argument('--views',type=int,default=3)
    vlm=sub.add_parser('train-vlm')
    vlm.add_argument('manifest',type=Path)
    vlm.add_argument('--out',type=Path,required=True)
    vlm.add_argument('--model',required=True)
    vlm.add_argument('--epochs',type=int,default=1)
    vlm.add_argument('--allow-download',action='store_true')
    vlm.add_argument('--dry-run',action='store_true')
    nesseler=sub.add_parser('export-nesseler',help='Export organizer checklist hand-off with pending human checks')
    nesseler.add_argument('run',type=Path)
    nesseler.add_argument('--out',type=Path,required=True)
    adaptive=sub.add_parser('adaptive-draft',help='Build a local evidence-linked Nesseler draft')
    adaptive.add_argument('manifest',type=Path,help='Local JSON manifest with views and manual/fixture observations')
    adaptive.add_argument('--sessions-root',type=Path,default=Path('runs/label-first-sessions'))
    adaptive.add_argument('--ladder-id',required=True)
    adaptive.add_argument('--operator-id')
    adaptive.add_argument('--session-id')
    sub.add_parser('teleop',help='Run the local simulated B2 teleoperation UI')
    if len(sys.argv) > 1 and sys.argv[1] == 'teleop':
        from .teleop import main as teleop_main
        return teleop_main(sys.argv[2:])
    args=parser.parse_args()
    try:
        if args.command=='demo':
            if not args.viewer:
                os.environ.setdefault('MUJOCO_GL','egl')
            from .mission import run_demo
            output=args.out or Path('runs')/(datetime.now().strftime('%Y%m%d-%H%M%S-')+uuid.uuid4().hex[:6])
            result=run_demo(output,args.database,args.patrols,args.variant,not args.no_recovery,args.viewer,args.motion,args.view_policy,args.detector,args.weights,args.inspector,args.qwen,args.allow_download,args.device,args.detector_family,safety=args.safety)
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
        elif args.command=='export-nesseler':
            from .nesseler import export_checklist
            print(json.dumps(export_checklist(args.run,args.out),indent=2))
        elif args.command=='generate-corpus':
            os.environ.setdefault('MUJOCO_GL','egl')
            from .corpus import generate_corpus
            print(json.dumps(generate_corpus(args.out,scenes=args.scenes,views=args.views),indent=2))
        elif args.command=='train-vlm':
            from .training import train_vlm
            print(json.dumps(train_vlm(args.manifest,args.out,model_path=args.model,epochs=args.epochs,allow_download=args.allow_download,dry_run=args.dry_run),indent=2))
        elif args.command=='train-detector':
            from .training import train_detector
            print(json.dumps(train_detector(args.dataset,args.out,weights=args.weights,epochs=args.epochs,device=args.device,allow_download=args.allow_download,dry_run=args.dry_run),indent=2))
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
        elif args.command=='adaptive-draft':
            from .adaptive_draft import build_adaptive_draft_from_file
            result=build_adaptive_draft_from_file(
                args.manifest,
                sessions_root=args.sessions_root,
                ladder_id=args.ladder_id,
                operator_id=args.operator_id,
                session_id=args.session_id,
            )
            print(json.dumps(result,indent=2))
        else:
            from .dimos_bridge import ingest
            ingest(args.run,args.database)
    except (RuntimeError,FileExistsError,ValueError) as exc:
        parser.exit(2,f'{exc}\n')


if __name__=='__main__':
    main()
