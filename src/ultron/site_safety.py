"""Site-safety sweep vocabulary and manifest builder (fire/first-aid/escape/PPE).

Scene props and segmentation-oracle findings are simulation ground truth,
not visual inference; the VLM pack judges the captured frames separately.
"""
import hashlib
from pathlib import Path

KINDS = ('extinguisher', 'firstaid', 'escape_sign', 'person')
# Distinctive geom suffix per kind; the oracle keys off "<key>_<suffix>".
PRIMARY_GEOM = {'extinguisher': 'tank', 'firstaid': 'box', 'escape_sign': 'plate', 'person': 'head'}

# Criteria each station kind can supply evidence for (keys from assets/site_safety_v1/rubric.json).
KIND_CRITERIA = {
    'extinguisher': ('ext.vorhanden', 'ext.zugaenglich', 'ext.pruefplakette'),
    'firstaid': ('firstaid.aushang', 'firstaid.kasten'),
    'escape_sign': ('flucht.kennzeichnung', 'flucht.frei'),
    'person': ('psa.helm', 'psa.warnweste'),
}
# Open-vocabulary detector classes and the class that confirms each station kind.
YOLO_CLASS_FOR_KIND = {'extinguisher': 'fire extinguisher', 'firstaid': 'first aid kit',
                       'escape_sign': 'exit sign', 'person': 'person'}
SAFETY_CLASSES = ['fire extinguisher', 'first aid kit', 'exit sign', 'person',
                  'safety helmet', 'high visibility vest']


def sweep_manifest(mission_id, views, inspection_date, position, map_version='ultron_mock_site_v1'):
    """One site-visit evidence bundle; same_asset_confirmed means same confirmed site visit."""
    manifest = {'pack': 'site_safety_v1', 'mission_id': mission_id,
                'asset_id': f'SITE-SWEEP-{mission_id}', 'map_version': map_version,
                'inspection_date': inspection_date, 'position': list(position),
                'verified_not_applicable': [], 'synthetic': True, 'views': []}
    for view in views:
        digest = hashlib.sha256(Path(view['path_abs']).read_bytes()).hexdigest()
        manifest['views'].append({'image_id': view['image_id'], 'path': view['path'],
                                  'sha256': digest, 'band': 'middle', 'azimuth_deg': 0,
                                  'pose_valid': True, 'same_asset_confirmed': True,
                                  'quality_accepted': True, 'region_visible': True})
    return manifest
