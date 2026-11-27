"""nesseler's ladder checklist (Kontrollblatt Leitern und Tritte, based on BGI 694).

Each item says whether a camera can screen it (`visual`) or whether it always needs a
person (`person`: function, sharp edges, lubrication, ...). Group keys line up with the
form's numbered sections so the PDF filler can tick the right row.
"""
from enum import Enum


class ItemResult(str, Enum):
    OK = 'OK'                    # no defect seen in the evidence photo
    DEFECT = 'DEFECT'            # defect seen
    UNKNOWN = 'UNKNOWN'          # required, but not visible or not classified
    NOT_CHECKED = 'NOT_CHECKED'  # outside automated scope: a person checks it
    N_A = 'N_A'                  # does not apply to this ladder type


# Form ascent types (page 1, "Aufstiegsart") and the robot's ladder types that map to them.
ASCENT_TYPES = ['Anlegeleiter', 'Mehrzweckleiter', 'Schiebeleiter', 'Podestleitern', 'Seilzugleiter',
                'Steckleiter', 'Stehleiter', 'Tritt', 'Sonstige']
MATERIALS = ['Aluminium', 'Stahl', 'Kunststoff', 'Edelstahl', 'Holz']
SIM_TYPE_TO_ASCENT = {'LEANING_SINGLE': 'Anlegeleiter', 'A_FRAME_STEPLADDER': 'Stehleiter'}

# (group key, German title, English title, components, applies to ascent types or None=all, items)
# item = (key, German label, English label, mode)
GROUPS = [
    ('holme', '1. Holme', 'Rails', ['stiles'], None, [
        ('verformung', 'Verformung', 'Bent', 'visual'),
        ('beschaedigung', 'Beschädigung (z.B. Risse)', 'Damage (e.g. cracks)', 'visual'),
        ('kanten', 'Scharfe Kanten, Splitter, Grat', 'Sharp edges, splinters, burrs', 'person'),
        ('abnutzung', 'Abnutzung', 'Wear', 'visual'),
        ('schutz', 'Schutzbehandlung (bei Holz)', 'Protective treatment (wood)', 'person'),
    ]),
    ('sprossen', '2. Sprossen/Stufen/Plattform', 'Rungs / steps / platform', ['rungs'], None, [
        ('verformung', 'Verformung', 'Bent', 'visual'),
        ('beschaedigung', 'Beschädigung', 'Damage', 'visual'),
        ('kanten', 'Scharfe Kanten, Splitter, Grat', 'Sharp edges, splinters, burrs', 'person'),
        ('verbindung', 'Verbindung zum Holm', 'Joint to the rail', 'visual'),
        ('abnutzung', 'Abnutzung (z.B. Trittfläche, Plattformauflage)', 'Wear (treads, platform)', 'visual'),
    ]),
    ('spreiz', '3. Spreizsicherung', 'Spreader', ['spreader'], ['Stehleiter', 'Mehrzweckleiter', 'Podestleitern', 'Tritt'], [
        ('vollstaendigkeit', 'Vollständigkeit/Befestigung', 'Complete and fixed', 'visual'),
        ('funktion', 'Funktionsfähigkeit', 'Works', 'person'),
        ('beschaedigung', 'Beschädigung', 'Damage', 'visual'),
    ]),
    ('beschlag', '4. Beschlagteile', 'Fittings', ['hinge_visual'], None, [
        ('beschaedigung', 'Beschädigung/Korrosion', 'Damage / rust', 'visual'),
        ('vollstaendigkeit', 'Vollständigkeit/Befestigung', 'Complete and fixed', 'visual'),
        ('funktion', 'Funktionsfähigkeit', 'Works', 'person'),
        ('abnutzung', 'Abnutzung', 'Wear', 'visual'),
        ('schmierung', 'Schmierung (mechanische Teile)', 'Greased (moving parts)', 'person'),
    ]),
    ('fuesse', '5. Leiter-/Trittfüße/Rollen', 'Feet / rollers', ['bottom_feet', 'front_feet', 'rear_feet'], None, [
        ('vollstaendigkeit', 'Vollständigkeit/Befestigung', 'Complete and fixed', 'visual'),
        ('abnutzung', 'Abnutzung/Beschädigung', 'Worn / damaged', 'visual'),
        ('funktion', 'Funktionsfähigkeit', 'Works', 'person'),
    ]),
    ('zubehoer', '6. Zubehör', 'Accessories', ['accessories'], None, [
        ('vollstaendigkeit', 'Vollständigkeit/Befestigung', 'Complete and fixed', 'visual'),
    ]),
    ('kennzeichnung', '7. Kennzeichnung', 'Marking', ['pictograms'], None, [
        ('betriebsanleitung', 'Betriebsanleitung (z.B. Piktogramm)', 'Instructions / pictograms', 'visual'),
    ]),
]

GROUP_KEYS = [g[0] for g in GROUPS]
ITEM_KEYS = [f'{g[0]}.{i[0]}' for g in GROUPS for i in g[5]]
ALL_KEYS = GROUP_KEYS + ITEM_KEYS


def describe():
    """Checklist structure for the web page."""
    return [{'key': g, 'de': de, 'en': en, 'items': [
        {'key': f'{g}.{k}', 'de': ide, 'en': ien, 'mode': mode} for k, ide, ien, mode in items]}
        for g, de, en, _, _, items in GROUPS]


def from_sim_checks(checks, required, ascent_type):
    """Map the simulation's coarse component checks onto the nesseler items.

    The simulation reports one PASS/FAIL/UNKNOWN per component, not per defect kind,
    so a failed component marks the group DEFECT and leaves its visual items UNKNOWN
    rather than guessing which kind of defect it was.
    """
    out = {}
    for group, _, _, components, applies, items in GROUPS:
        if applies is not None and ascent_type not in applies:
            out[group] = ItemResult.N_A
            for key, *_ in items:
                out[f'{group}.{key}'] = ItemResult.N_A
            continue
        seen = [checks[c] for c in components if c in checks]
        in_scope = any(c in required for c in components) or bool(seen)
        if not in_scope:
            group_result = ItemResult.NOT_CHECKED
        elif 'FAIL' in seen:
            group_result = ItemResult.DEFECT
        elif 'UNKNOWN' in seen or len(seen) < len([c for c in components if c in required]):
            group_result = ItemResult.UNKNOWN
        else:
            group_result = ItemResult.OK
        out[group] = group_result
        for key, _, _, mode in items:
            if mode == 'person' or group_result is ItemResult.NOT_CHECKED:
                result = ItemResult.NOT_CHECKED
            elif group_result is ItemResult.OK:
                result = ItemResult.OK
            else:
                result = ItemResult.UNKNOWN
            out[f'{group}.{key}'] = result
    return out
