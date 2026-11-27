"""Regenerate site_safety_v1/response_schema.json from its rubric, cloning the inspection_v1 criterion shape."""
import json
from pathlib import Path

assets = Path(__file__).resolve().parents[1] / 'src/ultron/assets'
template = json.loads((assets / 'inspection_v1/response_schema.json').read_text())
rubric = json.loads((assets / 'site_safety_v1/rubric.json').read_text())
keys = [c['key'] for s in rubric['sections'] for c in s['criteria']]
criterion = next(iter(template['properties']['criteria']['properties'].values()))
template['properties']['criteria']['properties'] = {k: criterion for k in keys}
template['properties']['criteria']['required'] = keys
out = assets / 'site_safety_v1/response_schema.json'
out.write_text(json.dumps(template, indent=1, ensure_ascii=False))
print(f'wrote {out} with {len(keys)} criteria')
