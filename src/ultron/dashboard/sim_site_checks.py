"""Site checks from a simulated patrol's safety props (`python -m ultron demo --safety`).

The MuJoCo patrol finds fixture props with the segmentation oracle and emits SAFETY_ASSET_CONFIRMED,
SAFETY_EVIDENCE_CAPTURED and SAFETY_VIOLATION events. This turns them into the dashboard's site
checks, so demo and `live --sim` show nesseler's other checklists. It claims only what the oracle
saw: the object is there. Anything a photo would have to show (an inspection sticker) stays open,
and every remark says it is simulation ground truth, not inference.
"""

# Sim prop kind -> checklist kind, and the presence rows an oracle sighting answers.
SIM_KIND = {'extinguisher': 'fire_extinguisher', 'firstaid': 'first_aid_kit', 'escape_sign': 'escape_sign',
            'person': 'person'}
FOUND = 'Simulation: in der Szene gefunden (Segmentierungs-Oracle)'
ROWS = {
    'fire_extinguisher': {'BS-1': ('ja', FOUND), 'BS-2': ('offen', 'Simulation: Prüfplakette nicht modelliert')},
    'first_aid_kit': {'EH-1': ('ja', FOUND)},
    'escape_sign': {'EH-9': ('ja', FOUND)},
    'person': {},
}
VIOLATION = {'PPE_HELMET_MISSING': 'Helm fehlt', 'PPE_VEST_MISSING': 'Warnweste fehlt'}


class SimSiteChecks:
    """Feed it each posted event of one run; it posts photos and site checks as they become due.

    post(path, body=None, raw=None, content_type=...) -> (status, body), as replay.Api.post.
    """

    def __init__(self, run_dir, post, log=print):
        self.run, self.post, self.log = run_dir, post, log
        self.assets = {}   # (mission, asset_id) -> {'kind', 'position', 'photo', 'violations'}

    def on_event(self, mission_id, row):
        kind, asset = row.get('type'), row.get('asset_id')
        key = (mission_id, asset)
        if kind == 'SAFETY_ASSET_CONFIRMED' and row.get('asset_kind') in SIM_KIND:
            position = (row.get('observation') or {}).get('position')
            self.assets[key] = {'kind': SIM_KIND[row['asset_kind']], 'position': position[:2] if position else None,
                                'photo': None, 'violations': []}
        elif key not in self.assets:
            return
        elif kind == 'SAFETY_EVIDENCE_CAPTURED' and row.get('rgb'):
            photo_id = f'site-{asset}'
            path = self.run / row['rgb']
            if path.exists():
                status, _ = self.post(f'/api/missions/{mission_id}/photos/{photo_id}', raw=path.read_bytes(),
                                      content_type='image/jpeg')
                if status == 201:
                    self.assets[key]['photo'] = photo_id
            self._send(mission_id, asset)
        elif kind == 'SAFETY_VIOLATION' and row.get('violation') in VIOLATION:
            self.assets[key]['violations'].append(VIOLATION[row['violation']])
            self._send(mission_id, asset)   # replaces the record sent with the photo

    def _send(self, mission_id, asset):
        a = self.assets[(mission_id, asset)]
        rows = {row_id: {'answer': answer, 'remark': remark, 'photo': a['photo']}
                for row_id, (answer, remark) in ROWS[a['kind']].items()}
        remarks = '; '.join(a['violations'])
        body = {'object_id': asset, 'kind': a['kind'], 'position': a['position'], 'rows': rows,
                'photos': [a['photo']] if a['photo'] else [],
                'remarks': f'Simulation (Szenen-Ground-Truth): {remarks}' if remarks else 'Simulation'}
        status, response = self.post(f'/api/missions/{mission_id}/site-checks', body)
        if status != 201:
            self.log(f'{mission_id}: site check {asset} rejected: {status} {response}')
