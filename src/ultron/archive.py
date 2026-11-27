"""Materialize history evidence into each export so run-relative names cannot collide."""
from pathlib import Path
import shutil


def export_history(assets, output):
    output=Path(output).resolve()
    for asset in assets:
        for event in asset['history']:
            payload=event['payload']
            root=Path(payload.get('evidence_root',output)).resolve()
            if root==output:
                payload['evidence_root']='.'
                continue
            run_id=payload['run_id']
            if Path(run_id).name!=run_id or run_id in ('.','..'):
                raise ValueError('Invalid history run ID')
            for view in payload.get('views',[]):
                for field in ('rgb','depth','overview','inspection_crop','model_response'):
                    if field not in view:
                        continue
                    relative=Path(view[field])
                    if relative.is_absolute() or '..' in relative.parts:
                        raise ValueError('History evidence must be a relative path')
                    destination=Path('history')/run_id/relative
                    target=output/destination
                    target.parent.mkdir(parents=True,exist_ok=True)
                    shutil.copyfile(root/relative,target)
                    view[field]=str(destination)
            payload['evidence_root']='.'
