"""Send a pipeline workspace's non-ladder results to the dashboard as site checks.

`python -m ultron.pipeline` keeps its state on disk (default runs/pipeline): objects.json, and per
inspected object objects/<id>/checklist.json with photo-N.jpg and evidence crops beleg-<row>.jpg.
This posts each found object of a kind nesseler's other checklists name, with its rows and photos,
to one patrol on the dashboard. Ladders are left to the pipeline's own publisher.

  python -m ultron.dashboard.import_site_checks runs/pipeline --mission PATROL-1
  python -m ultron.dashboard.import_site_checks runs/pipeline --mission HALL-2 --site ctrl-hall   # create it

Run it again after another inspection: an object's record is replaced, never duplicated.
"""
import argparse
import json
import re
from pathlib import Path

from .replay import Api
from .site_checklists import ROWS, kinds

SKIPPED_STATUS = {'REJECTED', 'MOVED'}   # the vision model said it is nothing, or its place was seen empty


def photo_id(object_id, file):
    return re.sub(r'[^A-Za-z0-9_.-]', '-', f'{object_id}-{Path(file).stem}')[:64]


def site_checks(workspace, log=print):
    """[(check body, [(photo id, path)])] for every object worth posting."""
    workspace = Path(workspace)
    objects = json.loads((workspace / 'objects.json').read_text())
    out = []
    for obj in objects:
        if obj['kind'] not in kinds() or obj.get('status') in SKIPPED_STATUS:
            continue
        folder = workspace / 'objects' / obj['id']
        sheet_path = folder / 'checklist.json'
        sheet = json.loads(sheet_path.read_text()) if sheet_path.exists() else {}
        files, rows = [], {}
        for file in sorted(folder.glob('photo-*.jpg')):
            files.append((photo_id(obj['id'], file), file))
        for row_id, row in (sheet.get('rows') or {}).items():
            if row_id not in ROWS or obj['kind'] not in ROWS[row_id].kinds:
                log(f"{obj['id']}: row {row_id} is not one a {obj['kind']} answers; skipped")
                continue
            evidence = row.get('evidence')
            if evidence and (folder / evidence).exists() and (folder / evidence) not in [f for _, f in files]:
                files.append((photo_id(obj['id'], evidence), folder / evidence))
            rows[row_id] = {'answer': row['answer'], 'remark': (row.get('remark') or '')[:1000],
                            'confidence': row.get('confidence'),
                            'photo': photo_id(obj['id'], evidence) if evidence else None}
        position = obj.get('position')
        body = {'object_id': obj['id'], 'kind': obj['kind'], 'verified': bool(obj.get('verified')),
                'position': list(position[:2]) if position else None, 'rows': rows,
                'photos': [p for p, _ in files], 'remarks': (sheet.get('remarks') or '')[:2000]}
        out.append((body, files))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('workspace', type=Path)
    parser.add_argument('--mission', required=True, help='patrol on the dashboard these objects belong to')
    parser.add_argument('--site', help='create the patrol on this site if the dashboard does not have it')
    parser.add_argument('--url', default='http://127.0.0.1:8000')
    args = parser.parse_args(argv)
    api = Api(args.url)
    if args.site:
        status, body = api.post('/api/missions', {'mission_id': args.mission, 'site_id': args.site, 'synthetic': False})
        if status not in (201, 409):
            raise SystemExit(f'could not create {args.mission}: {status} {body}')
    sent = 0
    for body, files in site_checks(args.workspace):
        for pid, path in files:
            status, response = api.post(f'/api/missions/{args.mission}/photos/{pid}', raw=path.read_bytes(),
                                        content_type='image/jpeg')
            if status == 404:
                raise SystemExit(f'the dashboard has no patrol {args.mission}; pass --site to create it')
        status, response = api.post(f'/api/missions/{args.mission}/site-checks', body)
        if status == 404:
            raise SystemExit(f'the dashboard has no patrol {args.mission}; pass --site to create it')
        print(f"{body['object_id']} ({body['kind']}): {len(body['rows'])} rows, {len(files)} photos -> HTTP {status}")
        sent += status == 201
    print(f'{sent} objects sent to {args.mission}')


if __name__ == '__main__':
    main()
