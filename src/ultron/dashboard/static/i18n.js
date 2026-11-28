// German / English labels. German follows nesseler's form wording where the form has a term.
const I18N = {
  de: {
    'nav.live': 'Live', 'nav.ladders': 'Leitern', 'nav.history': 'Verlauf',
    'sim': 'SIMULATION', 'live': 'LIVE', 'no_mission': 'Keine Mission', 'ended': 'Beendet',
    'theme.light': 'Hell', 'theme.dark': 'Dunkel', 'theme.toggle': 'Hell/Dunkel umschalten', 'lang.toggle': 'Sprache',
    'progress': 'Fortschritt', 'home': 'Rückkehr', 'home.done': 'Zurück', 'home.next': 'Danach',
    'now': 'In Prüfung', 'last': 'Zuletzt geprüft', 'photos': '{n} Fotos', 'result_pending': 'Ergebnis folgt',
    'ladders': 'Leitern', 'report': 'Bericht', 'map': 'Lageplan',
    'goal.set': 'Ziel setzen', 'goal.pick': 'Klicken und in Blickrichtung ziehen', 'goal.failed': 'Ziel nicht gesendet',
    'goal.SENT': 'Ziel gesendet', 'goal.ACCEPTED': 'Ziel angenommen', 'goal.SUCCEEDED': 'Ziel erreicht',
    'goal.FAILED': 'Ziel fehlgeschlagen', 'goal.CANCELED': 'Ziel abgebrochen',
    // patrol page, site checklists
    'p.back': '← Verlauf', 'p.none': 'Keine Patrouille gewählt', 'p.map': 'Route und Fundorte', 'p.downloads': 'Downloads',
    'p.ladders': 'Leitern', 'p.objects': 'Gefundene Objekte', 'p.checklists': 'Checklisten (nesseler)',
    'p.legend': 'Ja/Nein nur, wo der Roboter es gesehen hat. „Nicht gefunden“ heißt: auf dieser Patrouille nicht gesehen, nicht „fehlt“.',
    'p.footer': 'Sichtvorprüfung durch den Roboter. Jede Checkliste unterschreibt die verantwortliche Person.',
    'p.found': 'Objekte gefunden', 'p.rows_no': 'Punkte mit Nein', 'p.rows_open': 'Punkte offen',
    'p.legend_map': 'Linie: Route · Punkt: Leiter · Raute: weiteres Objekt', 'p.no_route': 'Keine Route aufgezeichnet · Punkt: Leiter · Raute: Objekt',
    'p.dl_checklists': 'Checklisten-Punkte', 'p.dl_objects': 'Objekte (gefunden / nicht)', 'p.dl_ladders': 'Leitern', 'p.dl_all': 'Alles',
    'p.no_photo': 'Kein Foto', 'p.confirmed': 'bestätigt', 'p.unconfirmed': 'unbestätigt',
    'p.no_objects': 'Keine weiteren Objekte gefunden.', 'p.not_found_kinds': 'Nicht gefunden:',
    'p.row': 'Nr.', 'p.answer': 'Antwort', 'p.objects_short': 'Objekte', 'p.all_person': 'Diese Checkliste prüft nur eine Person.',
    'p.person_rows': '{n} Punkte prüft eine Person', 'p.form': 'Formular',
    'ans.ja': 'Ja', 'ans.nein': 'Nein', 'ans.offen': 'Offen', 'ans.nicht_gefunden': 'Nicht gefunden', 'ans.kein_detektor': 'Kein Detektor',
    'ans.person': 'Person',
    'kind.fire_extinguisher': 'Feuerlöscher', 'kind.fire_hydrant': 'Wandhydrant', 'kind.fire_alarm': 'Meldeeinrichtung',
    'kind.first_aid_kit': 'Verbandkasten', 'kind.first_aid_notice': 'Aushang Erste Hilfe', 'kind.escape_plan': 'Flucht- und Rettungsplan',
    'kind.escape_sign': 'Fluchtwegschild', 'kind.escape_route': 'Fluchtweg', 'kind.emergency_light': 'Notbeleuchtung',
    'kind.gas_cylinder': 'Gasflasche', 'kind.scaffold': 'Gerüst', 'kind.dust_extractor': 'Entstauber', 'kind.person': 'Person',
    'h.open_patrol': 'Patrouille öffnen', 'h.checks': 'Checklisten', 'h.objects': 'Objekte',
    'h.objects_matrix': 'Objekte über alle Patrouillen', 'h.objects_dl': 'Als CSV',
    'cl.erste_hilfe_brandschutz': 'Erste Hilfe & Brandschutz', 'cl.geruest': 'Gerüst', 'cl.staubarme_baustelle': 'Staubarm',
    'cl.wirksamkeitskontrolle': 'Wirksamkeit', 'cl.einweisung_nachunternehmen': 'Einweisung', 'h.kind': 'Objektart',
    'explore.start': 'Erkundung starten', 'explore.pending': 'Erkundung noch nicht angebunden',
    'cam.front': 'Frontkamera', 'cam.depth': 'Tiefenkamera', 'cam.wait': 'Warte auf Bild',
    'robot': 'Roboter', 'robot.state': 'Zustand', 'robot.pos': 'Position', 'robot.mode': 'Modus', 'robot.takeovers': 'Eingriffe',
    'mode.auto_sim': 'Autonom (Sim)', 'mode.auto': 'Autonom',
    'delivery': 'Berichtsversand', 'delivery.planned': 'E-Mail an Bauleitung bei Rückkehr · noch nicht eingerichtet',
    'alerts': 'Meldungen', 'alerts.none': 'Keine Meldungen', 'ack': 'Quittieren', 'acked': 'Quittiert', 'open': 'Offen',
    'action.FLAGGED': 'Sofort aussondern', 'action.MANUAL_REVIEW': 'Vor Ort prüfen', 'action.ROBOT': 'Roboter prüfen',
    'log': 'Protokoll', 'empty': '–',
    'status.FLAGGED': 'Mangel', 'status.MANUAL_REVIEW': 'Prüfen', 'status.AUTOMATED_PASS': 'Ohne Befund', 'status.INSPECTING': 'In Prüfung',
    'result.OK': 'Ohne Befund', 'result.DEFECT': 'Mangel', 'result.UNKNOWN': 'Nicht sichtbar', 'result.NOT_CHECKED': 'Sachkundiger',
    'result.N_A': 'Entfällt', 'result.PENDING': 'Ausstehend',
    'reason.defect': 'Mangel: {x}', 'reason.sticker': 'Plakette abgelaufen', 'reason.unseen': 'Nicht sichtbar: {x}', 'reason.clear': 'Kein Befund',
    'group.holme': 'Holme', 'group.sprossen': 'Sprossen', 'group.spreiz': 'Spreizsicherung', 'group.beschlag': 'Beschläge',
    'group.fuesse': 'Füße', 'group.zubehoer': 'Zubehör', 'group.kennzeichnung': 'Kennzeichnung',
    'role.site_safety_officer': 'Sifa vor Ort', 'role.safety_engineer': 'Sicherheitsingenieur', 'role.site_manager': 'Bauleitung',
    'role.robot_operator': 'Roboterbediener',
    'footer': 'Sichtvorprüfung, keine Freigabe. Freigabe nur durch den Sachkundigen.',
    'ev.MISSION_START': 'Start', 'ev.LADDER_CONFIRMED': 'Leiter erkannt', 'ev.SUPERVISOR_TARGET_SELECTED': 'Ziel gewählt',
    'ev.EVIDENCE_CAPTURED': 'Foto', 'ev.SECOND_LOOK_REQUESTED': 'Zweiter Blick', 'ev.INSPECTION_COMMITTED': 'Ergebnis',
    'ev.ROUTE_RESUME': 'Weiter', 'ev.PATROL_WAYPOINT': 'Wegpunkt', 'ev.SUPERVISOR_DECISION': 'Entscheidung',
    'ev.BASE_REACHED': 'Zurück an Basis', 'ev.MISSION_FINISHED': 'Ende', 'ev.OPERATOR_INTERVENTION': 'Eingriff',
    'asc.Anlegeleiter': 'Anlegeleiter', 'asc.Stehleiter': 'Stehleiter', 'asc.Mehrzweckleiter': 'Mehrzweckleiter',
    'asc.Schiebeleiter': 'Schiebeleiter', 'asc.Podestleitern': 'Podestleiter', 'asc.Seilzugleiter': 'Seilzugleiter',
    'asc.Steckleiter': 'Steckleiter', 'asc.Tritt': 'Tritt', 'asc.Sonstige': 'Sonstige',
    // detail dialog and history page
    'close': 'Schließen', 'sticker': 'Prüfplakette', 'not_read': 'nicht gelesen', 'due': 'fällig', 'item': 'Prüfpunkt',
    'sheet': 'Blatt {n} · Prüfung {a}–{b}', 'latest': 'Aktuell', 'photos.title': 'Fotonachweis', 'proposal': 'Aussondern (Vorschlag)',
    'f.ascent': 'Aufstiegsart', 'f.sticker': 'Prüfplakette', 'f.inventory': 'Inventar-Nr.', 'f.position': 'Standort', 'f.patrols': 'Prüfungen',
    'release': 'Ergebnis und Freigabe: nur Sachkundiger', 'pick_ladder': 'Keine Leiter gewählt',
    'h.title': 'Verlauf', 'h.patrols': 'Patrouillen', 'h.ladders': 'Leitern', 'h.open': 'Offene Mängel', 'h.changed': 'Verändert',
    'h.patrol': 'Patrouille', 'h.date': 'Datum', 'h.flagged': 'Mängel', 'h.alerts': 'Meldungen', 'h.data': 'Daten', 'h.change': 'Änderung',
    'h.worse': 'Verschlechtert', 'h.still': 'Weiter offen', 'h.same': 'Unverändert', 'h.first': 'Erstprüfung', 'site': 'Baustelle',
    'h.matrix': 'Leitern über alle Patrouillen', 'h.running': 'Läuft', 'h.none': 'Keine Leiter in dieser Patrouille geprüft.',
    'h.empty': 'Noch keine Patrouille auf dieser Baustelle.', 'h.type': 'Art', 'h.reason': 'Befund', 'h.ladder': 'Leiter',
    'result': 'Ergebnis', 'photo': 'Foto', 'person_only': '„Leiter i. O.“ setzt nur der Sachkundige.',
  },
  en: {
    'nav.live': 'Live', 'nav.ladders': 'Ladders', 'nav.history': 'History',
    'sim': 'SIMULATION', 'live': 'LIVE', 'no_mission': 'No mission', 'ended': 'Finished',
    'theme.light': 'Light', 'theme.dark': 'Dark', 'theme.toggle': 'Switch light/dark', 'lang.toggle': 'Language',
    'progress': 'Progress', 'home': 'Return', 'home.done': 'Home', 'home.next': 'Next',
    'now': 'Inspecting', 'last': 'Last inspected', 'photos': '{n} photos', 'result_pending': 'Result pending',
    'ladders': 'Ladders', 'report': 'Report', 'map': 'Site map',
    'goal.set': 'Set goal', 'goal.pick': 'Click and drag toward the heading', 'goal.failed': 'Goal not sent',
    'goal.SENT': 'Goal sent', 'goal.ACCEPTED': 'Goal accepted', 'goal.SUCCEEDED': 'Goal reached',
    'goal.FAILED': 'Goal failed', 'goal.CANCELED': 'Goal canceled',
    // patrol page, site checklists
    'p.back': '← History', 'p.none': 'No patrol selected', 'p.map': 'Route and finds', 'p.downloads': 'Downloads',
    'p.ladders': 'Ladders', 'p.objects': 'Objects found', 'p.checklists': 'Checklists (nesseler)',
    'p.legend': 'Yes/No only where the robot saw it. "Not found" means not seen on this patrol, not "missing".',
    'p.footer': 'Visual pre-check by the robot. The responsible person signs every checklist.',
    'p.found': 'Objects found', 'p.rows_no': 'Rows with No', 'p.rows_open': 'Rows open',
    'p.legend_map': 'Line: route · dot: ladder · diamond: other object', 'p.no_route': 'No route recorded · dot: ladder · diamond: object',
    'p.dl_checklists': 'Checklist rows', 'p.dl_objects': 'Objects (found / not)', 'p.dl_ladders': 'Ladders', 'p.dl_all': 'Everything',
    'p.no_photo': 'No photo', 'p.confirmed': 'confirmed', 'p.unconfirmed': 'unconfirmed',
    'p.no_objects': 'No other objects found.', 'p.not_found_kinds': 'Not found:',
    'p.row': 'No.', 'p.answer': 'Answer', 'p.objects_short': 'Objects', 'p.all_person': 'Only a person checks this checklist.',
    'p.person_rows': '{n} rows a person checks', 'p.form': 'Form',
    'ans.ja': 'Yes', 'ans.nein': 'No', 'ans.offen': 'Open', 'ans.nicht_gefunden': 'Not found', 'ans.kein_detektor': 'No detector',
    'ans.person': 'Person',
    'kind.fire_extinguisher': 'Fire extinguisher', 'kind.fire_hydrant': 'Hose cabinet', 'kind.fire_alarm': 'Alarm device',
    'kind.first_aid_kit': 'First-aid kit', 'kind.first_aid_notice': 'First-aid notice', 'kind.escape_plan': 'Escape plan',
    'kind.escape_sign': 'Escape sign', 'kind.escape_route': 'Escape route', 'kind.emergency_light': 'Emergency light',
    'kind.gas_cylinder': 'Gas cylinder', 'kind.scaffold': 'Scaffold', 'kind.dust_extractor': 'Dust extractor', 'kind.person': 'Person',
    'h.open_patrol': 'Open patrol', 'h.checks': 'Checklists', 'h.objects': 'Objects',
    'h.objects_matrix': 'Objects across patrols', 'h.objects_dl': 'As CSV',
    'cl.erste_hilfe_brandschutz': 'First aid & fire', 'cl.geruest': 'Scaffold', 'cl.staubarme_baustelle': 'Low dust',
    'cl.wirksamkeitskontrolle': 'Effectiveness', 'cl.einweisung_nachunternehmen': 'Briefing', 'h.kind': 'Object kind',
    'explore.start': 'Start exploration', 'explore.pending': 'Exploration not connected yet',
    'cam.front': 'Front camera', 'cam.depth': 'Depth camera', 'cam.wait': 'Waiting for frames',
    'robot': 'Robot', 'robot.state': 'State', 'robot.pos': 'Position', 'robot.mode': 'Mode', 'robot.takeovers': 'Takeovers',
    'mode.auto_sim': 'Autonomous (sim)', 'mode.auto': 'Autonomous',
    'delivery': 'Report delivery', 'delivery.planned': 'Email to site manager on return · not set up yet',
    'alerts': 'Alerts', 'alerts.none': 'No alerts', 'ack': 'Acknowledge', 'acked': 'Acknowledged', 'open': 'Open',
    'action.FLAGGED': 'Take out of use', 'action.MANUAL_REVIEW': 'Check in person', 'action.ROBOT': 'Check the robot',
    'log': 'Log', 'empty': '–',
    'status.FLAGGED': 'Flagged', 'status.MANUAL_REVIEW': 'Review', 'status.AUTOMATED_PASS': 'No defect seen', 'status.INSPECTING': 'Inspecting',
    'result.OK': 'No defect', 'result.DEFECT': 'Defect', 'result.UNKNOWN': 'Not visible', 'result.NOT_CHECKED': 'Person',
    'result.N_A': 'N/A', 'result.PENDING': 'Pending',
    'reason.defect': 'Defect: {x}', 'reason.sticker': 'Sticker expired', 'reason.unseen': 'Not visible: {x}', 'reason.clear': 'No defect seen',
    'group.holme': 'Rails', 'group.sprossen': 'Rungs', 'group.spreiz': 'Spreader', 'group.beschlag': 'Fittings',
    'group.fuesse': 'Feet', 'group.zubehoer': 'Accessories', 'group.kennzeichnung': 'Marking',
    'role.site_safety_officer': 'Safety officer', 'role.safety_engineer': 'Safety engineer', 'role.site_manager': 'Site manager',
    'role.robot_operator': 'Robot operator',
    'footer': 'Visual pre-check, not a release. Only the qualified person releases a ladder.',
    'ev.MISSION_START': 'Start', 'ev.LADDER_CONFIRMED': 'Ladder found', 'ev.SUPERVISOR_TARGET_SELECTED': 'Target chosen',
    'ev.EVIDENCE_CAPTURED': 'Photo', 'ev.SECOND_LOOK_REQUESTED': 'Second look', 'ev.INSPECTION_COMMITTED': 'Result',
    'ev.ROUTE_RESUME': 'Resume', 'ev.PATROL_WAYPOINT': 'Waypoint', 'ev.SUPERVISOR_DECISION': 'Decision',
    'ev.BASE_REACHED': 'Back home', 'ev.MISSION_FINISHED': 'End', 'ev.OPERATOR_INTERVENTION': 'Takeover',
    'asc.Anlegeleiter': 'Leaning ladder', 'asc.Stehleiter': 'Stepladder', 'asc.Mehrzweckleiter': 'Multi-purpose ladder',
    'asc.Schiebeleiter': 'Extension ladder', 'asc.Podestleitern': 'Platform ladder', 'asc.Seilzugleiter': 'Rope ladder',
    'asc.Steckleiter': 'Sectional ladder', 'asc.Tritt': 'Step stool', 'asc.Sonstige': 'Other',
    'close': 'Close', 'sticker': 'Inspection sticker', 'not_read': 'not read', 'due': 'due', 'item': 'Item',
    'sheet': 'Sheet {n} · inspections {a}–{b}', 'latest': 'Latest', 'photos.title': 'Evidence', 'proposal': 'Take out (proposal)',
    'f.ascent': 'Type', 'f.sticker': 'Inspection sticker', 'f.inventory': 'Inventory no.', 'f.position': 'Location', 'f.patrols': 'Inspections',
    'release': 'Result and release: qualified person only', 'pick_ladder': 'No ladder selected',
    'h.title': 'History', 'h.patrols': 'Patrols', 'h.ladders': 'Ladders', 'h.open': 'Open flags', 'h.changed': 'Changed',
    'h.patrol': 'Patrol', 'h.date': 'Date', 'h.flagged': 'Flagged', 'h.alerts': 'Alerts', 'h.data': 'Data', 'h.change': 'Change',
    'h.worse': 'Got worse', 'h.still': 'Still open', 'h.same': 'No change', 'h.first': 'First inspection', 'site': 'Site',
    'h.matrix': 'Ladders across all patrols', 'h.running': 'Running', 'h.none': 'No ladder was inspected on this patrol.',
    'h.empty': 'No patrol on this site yet.', 'h.type': 'Type', 'h.reason': 'Finding', 'h.ladder': 'Ladder',
    'result': 'Result', 'photo': 'Photo', 'person_only': 'Only the qualified person can mark a ladder OK for use.',
  },
};

function store(key, value) {
  try { if (value === undefined) return localStorage.getItem(key); localStorage.setItem(key, value); } catch (e) { return null; }
  return null;
}

let LANG = store('ultron-lang') || ((navigator.language || 'de').toLowerCase().startsWith('de') ? 'de' : 'en');

function t(key, vars) {
  let text = (I18N[LANG] && I18N[LANG][key]) ?? I18N.en[key] ?? key;
  for (const [k, v] of Object.entries(vars || {})) text = text.replace(`{${k}}`, v);
  return text;
}

// Static markup: <span data-i18n="key">, aria labels via data-i18n-aria.
function applyI18n(root = document) {
  document.documentElement.lang = LANG;
  root.querySelectorAll('[data-i18n]').forEach(n => { n.textContent = t(n.dataset.i18n); });
  root.querySelectorAll('[data-i18n-aria]').forEach(n => n.setAttribute('aria-label', t(n.dataset.i18nAria)));
}

function currentTheme() {
  const set = document.documentElement.dataset.theme;
  if (set) return set;
  return window.matchMedia && matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
}

// Wire the language and theme switches in the header; onChange re-renders dynamic content.
function initChrome(onChange) {
  const saved = store('ultron-theme');
  if (saved) document.documentElement.dataset.theme = saved;
  const langButtons = document.querySelectorAll('[data-lang]');
  const themeButton = document.getElementById('theme');
  const sync = () => {
    langButtons.forEach(b => b.setAttribute('aria-pressed', String(b.dataset.lang === LANG)));
    if (themeButton) {
      const dark = currentTheme() === 'dark';
      themeButton.setAttribute('aria-pressed', String(dark));
      themeButton.querySelector('[data-theme-label]').textContent = dark ? t('theme.dark') : t('theme.light');
    }
    applyI18n();
  };
  langButtons.forEach(b => b.addEventListener('click', () => { LANG = b.dataset.lang; store('ultron-lang', LANG); sync(); onChange && onChange(); }));
  if (themeButton) themeButton.addEventListener('click', () => {
    const next = currentTheme() === 'dark' ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    store('ultron-theme', next);
    sync();
    onChange && onChange();
  });
  sync();
}
