"""Validated model proposals; confidence is not a calibrated safety probability."""
import math
from ..inspection import REQUIRED,fuse,verdict


def validate_findings(document,required,view_id,threshold=.85):
    if not isinstance(document,dict) or not isinstance(document.get('components'),list):
        raise ValueError('Expected a components list')
    checks=dict.fromkeys(required,'UNKNOWN')
    known_components=set(required) | {component for template in REQUIRED.values() for component in template}
    seen=set(); proposals=[]
    for item in document['components']:
        if not isinstance(item,dict):
            raise ValueError('Invalid component proposal')
        name=item.get('component');state=item.get('state');confidence=item.get('confidence')
        if (not isinstance(name,str) or name not in known_components or name in seen
                or state not in ('PASS','FAIL','UNKNOWN','UNKNOWN_CONFLICT')):
            raise ValueError('Unknown, duplicate or invalid component')
        if isinstance(confidence,bool) or not isinstance(confidence,(int,float)) or not math.isfinite(confidence) or not 0<=confidence<=1:
            raise ValueError('Invalid component confidence')
        if type(item.get('visible')) is not bool or not isinstance(item.get('reason'),str) or not item['reason'].strip():
            raise ValueError('Visibility and reason are required')
        seen.add(name)
        if item['visible'] and confidence>=threshold and (name in required or state=='FAIL'):
            checks[name]=state
        proposals.append(dict(item,source_view_id=view_id))
    label=document.get('label_status','UNKNOWN')
    if label not in ('VALID','OVERDUE','UNREADABLE','UNKNOWN'):
        raise ValueError('Invalid label status')
    # A model's label verdict is a proposal, not a parsed/verified inspection date.
    return {'checks':checks,'label_status':label,'proposals':proposals,'source_view_id':view_id,
            'model_confidence_calibrated':False}


def evidence_gate(views,required):
    checks={};label='UNKNOWN'
    for view in views:
        checks=fuse(checks,view['checks'])
        if view['label_status']=='OVERDUE':
            label='OVERDUE'
        elif label!='OVERDUE' and view['label_status']=='VALID':
            label='VALID'
    result=verdict(checks,required,label)
    if result['status']=='AUTOMATED_PASS':
        result.update(status='MANUAL_REVIEW',reason='Uncalibrated model proposals require human confirmation')
    result.update(checks=checks,model_confidence_calibrated=False,
                  evidence_coverage=sum(checks.get(c)=='PASS' for c in required)/len(required) if required else 0.,
                  views=views)
    return result
