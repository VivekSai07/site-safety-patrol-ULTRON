"""Nesseler checklist hand-off: coarse screening cannot complete individual criteria."""
import json
from pathlib import Path

# Organizer's three-page form, not a claim of conformity with a newer standard.
SECTIONS = (
    ('Holme', ('Verformung','Beschädigung (z.B. Risse)','Scharfe Kanten, Splitter, Grat','Abnutzung','Schutzbehandlung (bei Holz)'), ('stiles','rails')),
    ('Sprossen/Stufen/Plattform', ('Verformung','Beschädigung','Scharfe Kanten, Splitter, Grat','Verbindung zum Holm','Abnutzung (Trittfläche, Plattformauflage)'), ('rungs','steps','platform','platform_deck')),
    ('Spreizsicherung', ('Vollständigkeit/Befestigung','Funktionsfähigkeit','Beschädigung'), ('spreader',)),
    ('Beschlagteile', ('Beschädigung/Korrosion','Vollständigkeit/Befestigung','Funktionsfähigkeit','Abnutzung','Schmierung (mechanische Teile)'), ('hinges','locks','fittings','hinge_visual','guide_brackets_visual')),
    ('Leiter-/Trittfüße/Rollen', ('Vollständigkeit/Befestigung','Abnutzung/Beschädigung','Funktionsfähigkeit'), ('bottom_feet','front_feet','rear_feet','feet','rollers','wheel_brakes_visual')),
    ('Zubehör', ('Vollständigkeit/Befestigung',), ('accessories','wall_support','stabilizer')),
    ('Kennzeichnung', ('Betriebsanleitung (z.B. Piktogramm)',), ('markings','pictograms')),
)


def checklist_record(inspection):
    """Preserve evidence and pending fields; no inferred applicability or sign-off."""
    checks=inspection.get('checks',{})
    evidence=[{k:v[k] for k in ('rgb','depth','inspection_crop','model_response','view_angle_deg') if k in v}
              for v in inspection.get('views',[])]
    sections=[]
    for number,(name,criteria,components) in enumerate(SECTIONS,1):
        sections.append({'number':number,'name_de':name,
            'criteria':[{'name_de':c,'status':'PLEASE_CHECK','human_completion_required':True} for c in criteria],
            'observations':{k:checks[k] for k in components if k in checks},
            'evidence':evidence,'evidence_scope':'Inspection-level references; per-criterion visibility is unverified'})
    return {'asset_id':inspection.get('asset_id'),'mission_id':inspection.get('mission_id'),
        'inventory':{'inventory_number':inspection.get('asset_id'),'inventory_number_source':'ULTRON map identity; confirm physical inventory',
            'location':inspection.get('position'),'ladder_type_proposal':inspection.get('ladder_type'),
            **{k:None for k in ('material','rung_count','length','manufacturer','article_type_number','purchase_date','withdrawal_date','qualified_inspector')}},
        'label_reading':inspection.get('label_reading'),
        'source':inspection.get('source','UNSPECIFIED'),'synthetic':inspection.get('synthetic',False),
        'sections':sections,'raw_checks':dict(checks),'model_evidence':inspection.get('model_evidence',[]),
        'result':{'screening_status':inspection.get('status','MANUAL_REVIEW'),
            'screening_reason':inspection.get('reason'),'human_decision':'PENDING',
            'allowed_human_decisions':['Leiter i. O. und verwendungsfähig','Reparatur notwendig','Leiter sofort aussondern'],
            'safe_to_use':False,'remarks':None,'next_inspection_month_year':None,'inspection_date':None,
            'functional_tactile_checks':'PENDING','signature':None}}


def export_checklist(run, output):
    run=Path(run);protocol=json.loads((run/'protocol.json').read_text())
    result={'schema':'ultron.nesseler-handoff.v1','template':'kontrollblatt_checkliste_leitern_tritte.pdf (pages 1–3)',
        'scope':'Automated screening hand-off; qualified human completion required',
        'evidence_base':str(run.resolve()),
        'inspections':[checklist_record(r) for m in protocol['missions'] for r in m['records']]}
    with Path(output).open('x') as f:
        json.dump(result,f,indent=2,ensure_ascii=False)
    return {'output':str(output),'inspections':len(result['inspections'])}
