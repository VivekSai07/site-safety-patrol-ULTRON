"""Deterministic mission intelligence; model text never grants motion authority."""
import math
from .viewpoints import candidate_views


def decide(assets,inspected_ids,position,grid,*,survey_remaining,coverage_verified=False,
           missing_expected=(),base=(1.5,1.5)):
    pending=[a for a in assets if a['id'] not in inspected_ids]
    reachable=[]
    for asset in pending:
        if asset['id'] in missing_expected and not survey_remaining:
            continue
        views=candidate_views((asset['x'],asset['y']),asset['type'],grid,start=position,radius=2.8)
        if views:
            view=views[0]
            reachable.append((math.dist(position[:2],view['position']),asset,view))
    if reachable:
        _,asset,view=min(reachable,key=lambda x:x[0])
        return {'action':'INSPECT','asset_id':asset['id'],'goal':view['position'],'yaw':view['yaw'],
                'reason':'Uninspected known asset with reachable observation pose',
                'confidence_semantics':'DETERMINISTIC_TASK_STATE_NOT_SAFETY_PROBABILITY'}
    if survey_remaining:
        return {'action':'SURVEY','reason':'Continue bounded search; unobserved space may contain more ladders'}
    if not grid.path(position,base):
        return {'action':'AWAIT_OPERATOR','reason':'Base is unreachable; preserve evidence and stop'}
    gaps=bool(pending or missing_expected or not coverage_verified)
    return {'action':'RETURN_BASE_WITH_GAPS' if gaps else 'RETURN_BASE','goal':list(base),
            'reason':'Unreachable/pending assets, expected missing assets or unverified search coverage' if gaps else 'All known assets inspected and search coverage verified',
            'pending_ids':[a['id'] for a in pending],'missing_expected':list(missing_expected)}


def inspection_queue(detections,position,grid):
    """Re-evaluate target order after each inspection using the latest robot position."""
    pending=dict(detections)
    while pending:
        assets=[{'id':key,'type':value['type'],'x':value['position'][0],'y':value['position'][1]}
                for key,value in pending.items()]
        decision=decide(assets,set(),position(),grid,survey_remaining=True)
        if decision['action']!='INSPECT':
            decision={'action':'RECORD_UNREACHABLE','motion_allowed':False,
                      'reason':'No reachable inspection view; preserve review outcome without motion',
                      'supervisor_action':decision['action']}
        key=decision.get('asset_id',next(iter(pending)))
        detection=pending.pop(key)
        yield key,detection,decision
