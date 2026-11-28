"""nesseler's other site checklists (BG BAU / AMS BAU templates), row by row, for the site history.

Source: ctrl-hackathon-2026/site-safety-patrol/nesseler-data/checklists/*.pdf. Row texts are quoted
from the forms; only the checkbox rows are listed, not the posted fire-safety order (Brandschutzordnung
Teil A/B) that fills pages 3 to 5 of the first-aid form.

Each row says how a patrol can answer it:
    presence  the robot looks for an object of `kinds`: found -> ja, not found -> "nicht gefunden"
              (not "nein": a patrol that did not see something has not shown it is missing)
    visual    a camera judges an object of `kinds` that was found (sticker date, guardrail, ...)
    derived   filled from another result of the same patrol (`source`, e.g. the ladder records)
    person    paperwork or a work process: a person checks it, the robot never answers it

Row ids BS-1, BS-2, EH-1 and EH-2 are the ones `ultron.pipeline.checklists` already answers;
tests/test_site_checklists.py keeps the two in step. Kinds named here that the pipeline's catalog
does not detect yet are listed in NO_DETECTOR, so their rows show as open, not as absent.
"""
from dataclasses import dataclass

PRESENCE, VISUAL, DERIVED, PERSON = 'presence', 'visual', 'derived', 'person'
MODES = (PRESENCE, VISUAL, DERIVED, PERSON)

# Kinds these checklists need that ultron.pipeline.catalog has no detector prompt for yet.
NO_DETECTOR = {
    'first_aid_notice': 'Aushang "Anleitung zur Ersten Hilfe"',
    'escape_plan': 'Flucht- und Rettungsplan (Aushang)',
    'escape_sign': 'Fluchtweg-Kennzeichnung (grünes Schild)',
    'escape_route': 'Flucht- und Rettungsweg',
    'emergency_light': 'Notbeleuchtung',
    'gas_cylinder': 'Flüssiggasflasche',
    'scaffold': 'Gerüst',
    'dust_extractor': 'Entstauber / Bausauger',
    'person': 'Person auf der Baustelle',
}


@dataclass(frozen=True)
class Row:
    id: str
    section: str
    text: str            # German, verbatim from the form
    english: str
    mode: str
    kinds: tuple = ()    # object kinds that answer a presence or visual row
    source: str = ''     # for derived rows: where the answer comes from


@dataclass(frozen=True)
class Checklist:
    key: str
    title: str           # the form's own title
    english: str
    document: str        # PDF in nesseler-data/checklists
    rows: tuple

    @property
    def robot_rows(self):
        return [r for r in self.rows if r.mode != PERSON]


def _rows(section, *rows):
    return tuple(Row(r[0], section, r[1], r[2], r[3], *r[4:]) for r in rows)


FIRST_AID_FIRE = Checklist(
    'erste_hilfe_brandschutz', 'Checkliste Erste Hilfe und Brandschutz', 'First aid and fire protection',
    'erste_hilfe_brandschutz.pdf',
    _rows('Erste Hilfe',
          ('EH-3', 'Aushang Anleitung zur Ersten Hilfe ist ausgehängt und ausgefüllt.',
           'First-aid instructions are posted and filled in', PRESENCE, ('first_aid_notice',)),
          ('EH-4', 'Ersthelferinnen und Ersthelfer in ausreichender Anzahl sind vorhanden.',
           'Enough first aiders are on site', PERSON),
          ('EH-5', 'Die Namen der Ersthelferinnen und Ersthelfer sind bekannt gemacht (z. B. durch Aushang, '
                   'Kennzeichnung am Helm).', "First aiders' names are made known", PERSON),
          ('EH-1', 'Erste-Hilfe-Material (Verbandkasten) ist vorhanden. Haltbarkeit beachten!',
           'First-aid material (kit) is present', PRESENCE, ('first_aid_kit',)),
          ('EH-6', 'Ein elektronisches Verbandbuch wird geführt.', 'An electronic first-aid log is kept', PERSON),
          ('EH-7', 'Die Namen nahegelegener Durchgangsärztinnen und Durchgangsärzte (D-Ärzte) sind bekannt.',
           'Nearby accident doctors are known', PERSON),
          ('EH-8', 'Flucht- und Rettungsplan ist erstellt und bekannt gemacht.',
           'Escape and rescue plan is made and posted', PRESENCE, ('escape_plan',)),
          ('EH-9', 'Flucht- und Rettungswege sind gekennzeichnet.', 'Escape routes are marked',
           PRESENCE, ('escape_sign',)),
          ('EH-10', 'Flucht- und Rettungswege sind freigehalten.', 'Escape routes are kept clear',
           VISUAL, ('escape_route',)),
          ('EH-11', 'Notbeleuchtung ist vorhanden.', 'Emergency lighting is present', PRESENCE, ('emergency_light',)),
          ('EH-2', 'Meldeeinrichtungen sind vorhanden.', 'Alarm devices are present', PRESENCE, ('fire_alarm',))) +
    _rows('Bei Baustellen mit mehr als 50 bzw. 100 Beschäftigten einschließlich MiUs',
          ('EH-12', 'Sanitätsraum/Sanitätscontainer ist vorhanden (bei mehr als 50 Beschäftigten einschließlich MiUs).',
           'First-aid room or container (more than 50 workers)', PERSON),
          ('EH-13', 'Betriebssanitäterin/Betriebssanitäter ist vorhanden (bei mehr als 100 Beschäftigten '
                    'einschließlich MiUs).', 'Company paramedic (more than 100 workers)', PERSON),
          ('EH-14', 'Eine Zustimmung der BG Bau liegt vor.', 'BG BAU consent is on file', PERSON),
          ('EH-15', 'Rettungsorganisation in unmittelbarer Nähe (Hilfsfrist 10-15 Minuten wird deutlich '
                    'unterschritten!) übernimmt Aufgaben des Betriebssanitäters (z.B. Krankenhaus, Feuerwache)',
           'A nearby rescue service takes over the paramedic duties', PERSON),
          ('EH-16', 'Eine Abstimmung mit der Feuerwehr und dem Rettungsdienst hat stattgefunden (z. B. bezüglich der '
                    'Zufahrt auf die Baustelle oder der erreichbaren Höhe der vorhandenen Drehleitern).',
           'Agreed with fire brigade and rescue service', PERSON),
          ('EH-17', 'Vorgespräch zur Umsetzung mit Rettungsorganisationen und Aufsichtsperson hat stattgefunden',
           'Preliminary meeting with rescue services held', PERSON),
          ('EH-18', 'Schriftliche Bestätigung der Rettungsorganisation liegt vor und ist an die zuständige '
                    'Aufsichtsperson weitergeleitet', 'Written confirmation from the rescue service', PERSON),
          ('EH-19', 'Ortskunde der Rettungsorganisation wurde durch Begehungen sichergestellt',
           'Rescue service knows the site from walk-throughs', PERSON),
          ('EH-20', 'Anfahrtswege/Zugänge sind geschaffen und unterhalten (z. B. für Feuerwehr, Rettungsdienste, '
                    'Werksfeuerwehr, Höhenrettungstrupp).', 'Access routes for rescue services exist', PERSON),
          ('EH-21', 'Rettungsgeräte und Transportmittel (s. Arbeitsstätten Richtlinie) sind vorhanden.',
           'Rescue and transport equipment is present', PERSON),
          ('EH-22', 'Höhenrettungsmaßnahmen sind vorbereitet (z. B. Höhenrettungsgerät vorhanden).',
           'Height rescue is prepared', PERSON),
          ('EH-23', 'Lotsinnen/Lotsen für den Rettungsdienst sind eingeteilt.', 'Guides for the rescue service assigned',
           PERSON),
          ('EH-24', 'Auffallend gekennzeichneter Treffpunkt ist zwischen Lotsinnen/Lotsen und Rettungsorganisation '
                    'abgestimmt', 'Marked meeting point agreed', PERSON),
          ('EH-25', 'Rettungsübungen sind durchgeführt.', 'Rescue drills have been held', PERSON),
          ('EH-26', 'Hervorhebung der Ersthelfer aller MiUs z.B. durch Aufkleber am Helm',
           'First aiders of all contractors marked', PERSON),
          ('EH-27', 'Einweisung aller MiUs in die Rettungskette ist erfolgt', 'All contractors briefed on the rescue chain',
           PERSON)) +
    _rows('Brandschutz',
          ('BS-1', 'Feuerlöscheinrichtungen/Feuerlöscher sind für die Baustelle festgelegt und vorhanden.',
           'Fire-fighting equipment is present', PRESENCE, ('fire_extinguisher', 'fire_hydrant')),
          ('BS-2', 'Die Prüffrist der Feuerlöscher/Feuerlöscheinrichtungen ist nicht abgelaufen.',
           'Inspection date on the sticker has not passed', VISUAL, ('fire_extinguisher', 'fire_hydrant')),
          ('BS-3', 'Für Feuer-, Schweiß- und Schneidarbeiten in brandgefährdeten Bereichen liegt eine schriftliche '
                   'Erlaubnis vor.', 'Written permit for hot work in fire-risk areas', PERSON),
          ('BS-4', 'Flüssiggasflaschen und andere leicht entflammbare Stoffe und Gegenstände lagern nicht in '
                   'Unterkünften.', 'No gas bottles or flammables stored in site huts', VISUAL, ('gas_cylinder',))) +
    _rows('Sonderfälle Brandschutz',
          ('BS-5', 'Es gibt beim Bauen im Bestand z.B. in Werken eine Brandschutzordnung durch den Auftraggeber.',
           "Client's fire-safety order exists (work in existing buildings)", PERSON),
          ('BS-6', 'Absprachen mit Brandschutzbeauftragten des Auftraggebers sind erforderlich/ haben stattgefunden '
                   'oder ein Brandschutzbeauftragter ist wegen einer erhöhten Brandgefahr erforderlich oder wird '
                   'durch Auflagen der Behörden gefordert.', "Agreed with the client's fire-safety officer", PERSON),
          ('BS-7', 'Brandschutzhelferinnen und Brandschutzhelfern sind benannt.', 'Fire wardens are named', PERSON)))

SCAFFOLD = Checklist(
    'geruest', 'Prüfprotokoll für Gerüstbenutzende', 'Scaffold check before use', 'geruest_pruefprotokoll.pdf',
    _rows('Verwendungszweck',
          ('GR-1', 'Geeignet z. B. für Maurerarbeiten, Stuck- und Putzarbeiten, Malerarbeiten.',
           'Suitable for the intended work', PERSON),
          ('GR-2', 'Ist das Gerüst an sichtbarer Stelle (z. B. Aufstieg) gekennzeichnet? – Arbeitsgerüst und/oder '
                   'Schutzgerüst nach DIN EN 12811/DIN 4420 – Lastklasse und Nutzlast, Breitenklasse – Gerüstersteller',
           'Scaffold tag posted at a visible place', VISUAL, ('scaffold',)),
          ('GR-3', 'Ist das Gerüst augenscheinlich ausreichend verankert? (Liegt ein Auszugsprotokoll vor?)',
           'Visibly anchored (pull-out test record?)', PERSON)) +
    _rows('Arbeits- und Betriebssicherheit',
          ('GR-4', 'Sind sichere Zugänge oder Aufstiege, wie z. B. Treppenlauf oder Treppentürme, vorhanden?',
           'Safe access (stairs, stair towers)', VISUAL, ('scaffold',)),
          ('GR-5', 'Ist jede genutzte Gerüstlage vollflächig mit Belägen (z. B. Rahmentafeln oder Bohlen) ausgelegt?',
           'Every level in use fully decked', VISUAL, ('scaffold',)),
          ('GR-6', 'Sind die Gerüstbeläge und -bohlen so verlegt, dass sie weder wippen noch ausweichen können und '
                   'sind sie gegen Abheben gesichert?', 'Decking does not rock or shift, secured against lifting',
           PERSON),
          ('GR-7', 'Ist bei der Einrüstung einer Bauwerksecke der Belag in voller Breite herumgeführt?',
           'Full-width decking around building corners', VISUAL, ('scaffold',)),
          ('GR-8', 'Sind Belag-Elemente augenscheinlich unbeschädigt, z. B. nicht eingerissen, eingeschnitten?',
           'Decking visibly undamaged', VISUAL, ('scaffold',)),
          ('GR-9', 'Sind Gerüstlagen bei mehr als 2,00 m Absturzhöhe mit einem 3-teiligen Seitenschutz '
                   '(Geländerholm, Zwischenholm und Bordbrett) versehen?',
           '3-part side protection above 2 m fall height', VISUAL, ('scaffold',)),
          ('GR-10', 'Ist der 3-teilige Seitenschutz auch an Stirnseiten und Öffnungen angebracht?',
           'Side protection also at ends and openings', VISUAL, ('scaffold',)),
          ('GR-11', 'Ist ein maximaler Wandabstand von 0,30 m eingehalten?', 'Wall gap at most 0.30 m',
           VISUAL, ('scaffold',))) +
    _rows('Anforderungen an Fang- und Dachfanggerüste',
          ('GR-12', 'Ist bei Dachfanggerüsten die Belagfläche mindestens 0,60 m breit?',
           'Roof catch scaffold decking at least 0.60 m wide', PERSON),
          ('GR-13', 'Liegt der Belag des Dachfanggerüstes nicht tiefer als 1,50 m unter der Traufkante?',
           'Roof catch decking at most 1.50 m below the eaves', PERSON),
          ('GR-14', 'Beträgt der Abstand zwischen Schutzwand und Traufkante mindestens 0,70 m?',
           'Protective wall at least 0.70 m from the eaves', PERSON),
          ('GR-15', 'Ist die Schutzwand aus Schutznetzen oder Geflechten ordnungsgemäß am Gerüst befestigt?',
           'Protective net or mesh properly fixed', PERSON),
          ('GR-16', 'Ist bei Fanggerüsten die Belagfläche mindestens 0,90 m breit?',
           'Catch scaffold decking at least 0.90 m wide', PERSON),
          ('GR-17', 'Liegt der Belag des Fanggerüstes nicht tiefer als 2,00 m unter der Absturzkante?',
           'Catch decking at most 2.00 m below the edge', PERSON)) +
    _rows('Sonstige Anforderungen',
          ('GR-18', 'Sind spannungsführende Leitungen und/oder Geräte im Gerüstbereich abgeschaltet, abgedeckt oder '
                    'abgeschrankt?', 'Live lines near the scaffold switched off, covered or fenced', PERSON),
          ('GR-19', 'Ist die Beleuchtung zur Sicherung des öffentlichen Verkehrs gewährleistet?',
           'Lighting for public traffic', PERSON),
          ('GR-20', 'Ist am Gerüst beim Einsatz im öffentlichen Bereich ein Schutzdach vorhanden?',
           'Protective roof where the scaffold is in a public area', PERSON)))

LOW_DUST = Checklist(
    'staubarme_baustelle', 'Checkliste staubarme Baustelle', 'Low-dust site', 'staubarme_baustelle.pdf',
    _rows('Maßnahmen',
          ('ST-1', 'Wird bei der Herstellung von Ortbeton, Fertigteilen und Mauerwerk die Lehrrohrplanung und die '
                   'Schlitz- und Durchbruchsplanung berücksichtig?', 'Conduit, slot and opening planning considered',
           PERSON),
          ('ST-2', 'Setzen die eigenen Mitarbeiter Maschinen mit Absaugungen ein und sind die Mitarbeiter unterwiesen?',
           'Own staff use machines with extraction and are trained', PERSON),
          ('ST-3', 'Werden Trennschleifer im Nassschneiderverfahren mit Wasseranschluss eingesetzt?',
           'Cut-off grinders used wet, with water supply', PERSON),
          ('ST-4', 'Wird die Baustellenkreissäge mit Absaugung zur Verfügung gestellt?',
           'Site circular saw provided with extraction', PERSON),
          ('ST-5', 'Werden zugelassene, geprüfte und funktionsfähige Entstauber/Staubsauger eingesetzt?',
           'Approved, tested dust extractors in use', PRESENCE, ('dust_extractor',)),
          ('ST-6', 'Werden staubarme Produkte eingesetzt (wie z.B. staubreduzierende Granulate von Fliesenklebemörtel '
                   'und Spachtelmassen)?', 'Low-dust products used', PERSON),
          ('ST-7', 'Werden staubarme Arbeitsverfahren eingesetzt? Wird beim Umfüllen der Sackware z.B. die Fallhöhe '
                   'berücksichtigt? Können ggf. Silos statt Sackware eingesetzt werden?', 'Low-dust work methods used',
           PERSON),
          ('ST-8', 'Wird die Persönliche Schutzausrüstung getragen, wenn technische Schutzmaßnahmen nicht umgesetzt '
                   'werden können?', 'PPE worn where technical measures are not possible', VISUAL, ('person',)),
          ('ST-9', 'Erfolgt eine regelmäßige Reinigung von Arbeitsplätzen (nicht trocken kehren, kein Abblasen von '
                   'Stäuben oder Reinigern von Kleidung mit Druckluft)? Wird die Arbeitskleidung regelmäßig gereinigt?',
           'Workplaces and work clothes cleaned regularly', PERSON),
          ('ST-10', 'Entsteht eine Gefahr durch mineralische Stäube verursacht durch Fremdfirmen und wurde das Thema im '
                    'Rahmen der Einweisung der Fremdfirma berücksichtigt?',
           'Dust from other contractors covered in their briefing', PERSON)))

EFFECTIVENESS = Checklist(
    'wirksamkeitskontrolle', 'Wirksamkeitskontrolle der Gefährdungsbeurteilung Hochbau',
    'Check that the risk-assessment measures are in place', 'wirksamkeitskontrolle_gefaehrdungsbeurteilung.pdf',
    _rows('Themen',
          ('WK-1', 'Allgemeine Organisation auf der Baustelle (Erste Hilfe A 004, Arbeitsmedizinische Vorsorge D 513, '
                   'Sozialräume A 025, Verkehrswege A 026)', 'General site organisation', DERIVED, (),
           'erste_hilfe_brandschutz'),
          ('WK-2', 'Umgang mit elektrischen Betriebsmitteln auf Baustellen (Bohrmaschine B 266, Handtrennschleifmaschinen '
                   'B 274, Schlagbohr- und Stemmgeräte B 206, Baustellenkreissägen/ Handkreissägen B 265, '
                   'Mischmaschinen B201, Handkettensägen B 259, Baustromverteiler B 171)', 'Electrical equipment',
           PERSON),
          ('WK-3', 'Bohren und Sägen von Beton und Asphalt (C 365)', 'Drilling and sawing concrete and asphalt', PERSON),
          ('WK-4', 'Umgang mit Gefahrstoffen (A 042)', 'Hazardous substances', PERSON),
          ('WK-5', 'Umgang mit Leitern (B 131, B 132)', 'Ladders', DERIVED, (), 'ladders'),
          ('WK-6', 'Anschlagen von Lasten (B 164, B 161)', 'Slinging loads', PERSON),
          ('WK-7', 'Arbeiten mit dem Bagger (B 181)', 'Excavators', PERSON),
          ('WK-8', 'Verbau von Gruben und Gräben (C 469, C 470)', 'Shoring of pits and trenches', PERSON),
          ('WK-9', 'Umgang mit Kränen (B 213, B 214, B 215, B 164, B161)', 'Cranes', PERSON),
          ('WK-10', 'Absturzsicherungen auf Baustellen (B 100)', 'Fall protection', PERSON),
          ('WK-11', 'Umgang mit Gerüsten (B111, B 113, B 119, B 120, B 121, B 122)', 'Scaffolds', DERIVED, (),
           'geruest'),
          ('WK-12', 'Schweißarbeiten (C 423, C 424)', 'Welding', PERSON),
          ('WK-13', 'Transport von Baumaschinen (A 069), Transport von Gasflaschen (A 065), Transport allgemein, '
                    'Transport von Gefahrgütern in geringen Mengen (A 071)', 'Transport', PERSON)))

CONTRACTOR_BRIEFING = Checklist(
    'einweisung_nachunternehmen', 'Einweisung von Mitunternehmen in Gefahren und Schutzmaßnahmen ihres Arbeitsbereiches',
    'Safety briefing for subcontractors (a signed record per person: nothing for a robot to see)',
    'einweisung_nachunternehmen.pdf',
    _rows('Allgemeine Einweisungsinhalte',
          ('EW-1', 'Ort des Arbeitsplatzes und sein gefahrloses Erreichen', 'Workplace and safe way there', PERSON),
          ('EW-2', 'Besondere Gefahren der Tätigkeit und des Arbeitsplatzes', 'Particular hazards', PERSON),
          ('EW-3', 'Betriebliche Regelungen und Verbote', 'Site rules and bans', PERSON),
          ('EW-4', 'Alarmierung, Fluchtwege, Feuerlöscher, Alarmpläne, Verhalten im Notfall, Notrufnummer 112',
           'Alarms, escape routes, extinguishers, emergency', PERSON),
          ('EW-5', 'Ersthelfer und Erste-Hilfe-Material', 'First aiders and first-aid material', PERSON),
          ('EW-6', 'Zuständiger Vorgesetzter und Ansprechpartner bei Fragen', 'Supervisor and contact', PERSON),
          ('EW-7', 'Persönliche Schutzausrüstung - welche, wann, wie?', 'PPE: which, when, how', PERSON),
          ('EW-8', 'Inhalte der SiGeKo-Unterlagen', 'Health and safety coordinator documents', PERSON)) +
    _rows('Baustellenspezifische Einweisungsinhalte',
          ('EW-9', 'Flurförderzeuge (z. B. Regelung zur Verkehrsführung, Fußgängerwege, Wege für Flurförderzeuge)',
           'Industrial trucks and traffic routes', PERSON),
          ('EW-10', 'Schweißgeräte (z. B. Arbeiten nur mit Schweißerlaubnisschein)', 'Welding equipment', PERSON),
          ('EW-11', 'Krane (z. B. Vorfahrtsregelungen, Festlegung von Lagerplätzen)', 'Cranes', PERSON),
          ('EW-12', 'Elektrische Anlagen und Betriebsmittel (z. B. Regelungen zur Stromversorgung auf der Baustelle)',
           'Electrical installations', PERSON),
          ('EW-13', 'Kraftbetriebene Arbeitsmittel (z. B. Gegenseitige Gefährdungen ausschließen)',
           'Powered work equipment', PERSON),
          ('EW-14', 'Hebebühne (z. B. Absprach über Nutzung mit GU)', 'Lifting platform', PERSON),
          ('EW-15', 'Staubarme Baustelle (Arbeitsverfahren, Einsatz von Staubsaugern, Verhalten)', 'Low-dust site',
           PERSON)))

CHECKLISTS = {c.key: c for c in (FIRST_AID_FIRE, SCAFFOLD, LOW_DUST, EFFECTIVENESS, CONTRACTOR_BRIEFING)}
ROWS = {row.id: row for checklist in CHECKLISTS.values() for row in checklist.rows}


def kinds():
    """Every object kind a robot row asks about."""
    return sorted({k for c in CHECKLISTS.values() for r in c.rows for k in r.kinds})


# Kinds the pipeline detects today, named as on the site; with NO_DETECTOR this names every kind above.
DETECTED_LABELS = {
    'fire_extinguisher': 'Feuerlöscher',
    'fire_hydrant': 'Wandhydrant / Löschschlauchschrank',
    'fire_alarm': 'Meldeeinrichtung (Brandmelder)',
    'first_aid_kit': 'Verbandkasten',
}
KIND_LABELS = {**DETECTED_LABELS, **NO_DETECTOR}

# What a patrol says about one row. The robot's own answers use the form's words (ja / nein / offen);
# the rest say why there is no answer, so a gap is never shown as "nein".
YES, NO, OPEN = 'ja', 'nein', 'offen'
NOT_FOUND = 'nicht_gefunden'   # the patrol did not see such an object: not proof it is missing
NO_DETECTOR_YET = 'kein_detektor'
BY_PERSON = 'person'
ROBOT_ANSWERS = (YES, NO, OPEN)
ANSWER_ORDER = (NO, OPEN, NOT_FOUND, NO_DETECTOR_YET, YES, BY_PERSON)  # worst first, for a checklist's headline


def _worst(answers):
    for answer in ANSWER_ORDER:
        if answer in answers:
            return answer
    return NOT_FOUND


def _derived(row, ladders, checklist_answers):
    """WK rows filled from this patrol's other results. A robot never answers "measures sufficient":
    a defect found is a "nein", anything else stays open for the person signing the form."""
    if row.source == 'ladders':
        if not ladders:
            return NOT_FOUND, 'keine Leiter auf dieser Patrouille geprüft'
        flagged = sum(1 for l in ladders if l.get('status') == 'FLAGGED')
        note = f'{len(ladders)} Leiter(n) geprüft, {flagged} mit Mangel'
        return (NO if flagged else OPEN), note
    answers = checklist_answers.get(row.source, [])
    robot = [a for a in answers if a in ROBOT_ANSWERS]
    if not robot:
        return NOT_FOUND, f'nichts zu {CHECKLISTS[row.source].title} gesehen'
    return (NO if NO in robot else OPEN), f'aus {CHECKLISTS[row.source].title}'


def patrol_summary(objects, ladders=()):
    """Every checklist row for one patrol, plus which object kinds it found.

    objects: the site checks posted for this patrol, each {'object_id', 'kind', 'rows': {row_id:
    {'answer', 'remark', 'photo', ...}}, ...}. ladders: this patrol's ladder inspections (status only).
    """
    by_kind = {}
    for obj in objects:
        by_kind.setdefault(obj['kind'], []).append(obj)

    def evaluate(row):
        if row.mode == PERSON:
            return {'answer': BY_PERSON, 'objects': [], 'remark': '', 'photo': None}
        seen = [o for k in row.kinds for o in by_kind.get(k, [])]
        if not seen:
            missing = all(k in NO_DETECTOR for k in row.kinds)
            return {'answer': NO_DETECTOR_YET if missing else NOT_FOUND, 'objects': [], 'remark': '', 'photo': None}
        given = [(o, o.get('rows', {}).get(row.id)) for o in seen]
        answers = [g['answer'] for _, g in given if g]
        if row.mode == PRESENCE:
            # found is enough for "there"; an object reported missing, empty or damaged makes it "nein"
            answer = NO if NO in answers else YES
        else:
            answer = _worst(answers) if answers else OPEN
        cited = next(((o, g) for o, g in given if g and g['answer'] == answer), (seen[0], None))
        return {'answer': answer, 'objects': [o['object_id'] for o in seen],
                'remark': (cited[1] or {}).get('remark', ''), 'photo': (cited[1] or {}).get('photo'),
                'photo_object': cited[0]['object_id']}

    results, checklist_answers = [], {}
    order = sorted(CHECKLISTS.values(), key=lambda c: any(r.mode == DERIVED for r in c.rows))  # sources first
    for checklist in order:
        rows = []
        for row in checklist.rows:
            if row.mode == DERIVED:
                answer, remark = _derived(row, list(ladders), checklist_answers)
                result = {'answer': answer, 'objects': [], 'remark': remark, 'photo': None}
            else:
                result = evaluate(row)
            rows.append({'id': row.id, 'section': row.section, 'text': row.text, 'english': row.english,
                         'mode': row.mode, **result})
        answers = [r['answer'] for r in rows]
        checklist_answers[checklist.key] = answers
        robot = [a for a in answers if a != BY_PERSON]
        results.append({'key': checklist.key, 'title': checklist.title, 'english': checklist.english,
                        'document': checklist.document, 'headline': _worst(robot) if robot else BY_PERSON,
                        'counts': {a: answers.count(a) for a in ANSWER_ORDER if a in answers}, 'rows': rows})
    results.sort(key=lambda c: list(CHECKLISTS).index(c['key']))

    presence = [{'kind': kind, 'label': KIND_LABELS[kind], 'detector': kind not in NO_DETECTOR,
                 'found': len(by_kind.get(kind, [])), 'objects': [o['object_id'] for o in by_kind.get(kind, [])]}
                for kind in kinds()]
    return {'checklists': results, 'presence': presence}


def describe():
    """JSON-ready structure for the dashboard pages and the exports."""
    return [{'key': c.key, 'title': c.title, 'english': c.english, 'document': c.document,
             'rows': [{'id': r.id, 'section': r.section, 'text': r.text, 'english': r.english, 'mode': r.mode,
                       'kinds': list(r.kinds), 'source': r.source, 'detector': all(k not in NO_DETECTOR for k in r.kinds)}
                      for r in c.rows]}
            for c in CHECKLISTS.values()]
