// Site mission history: missions, per-ladder history across missions, open items, per-mission register.
const $ = id => document.getElementById(id);
let checklist = [];
let missions = [];

async function loadSite(site) {
  const list = await getJSON(`/api/missions?site_id=${encodeURIComponent(site)}`);
  missions = await Promise.all(list.map(async m => {
    const [full, alerts] = await Promise.all([getJSON(`/api/missions/${m.id}`), getJSON(`/api/alerts?mission_id=${m.id}`)]);
    return {...full, alerts};
  }));
  const history = await getJSON(`/api/sites/${encodeURIComponent(site)}/ladders`);
  renderTiles(history); renderMissions(); renderMatrix(history); renderOpen(history);
  if (missions.length) selectMission(missions[0].id);
}

function counts(m) {
  const c = s => m.inspections.filter(i => i.status === s).length;
  return {flagged: c('FLAGGED'), review: c('MANUAL_REVIEW'), pass: c('AUTOMATED_PASS')};
}

function changed(entries) {
  return entries.length > 1 && entries[entries.length - 1].status !== entries[entries.length - 2].status;
}

function renderTiles(history) {
  const ladders = Object.entries(history);
  const open = ladders.filter(([, h]) => h[h.length - 1].status !== 'AUTOMATED_PASS');
  const tiles = [['Missions on this site', missions.length, missions.map(m => m.id).slice(0, 3).join(', '), 'var(--ink)'],
    ['Ladders on register', ladders.length, 'Same ID across patrols', 'var(--ink)'],
    ['Open flags', open.length, open.map(([a]) => a).join(', ') || 'None', open.length ? 'var(--flag-fg)' : 'var(--ink)'],
    ['Changed since last mission', ladders.filter(([, h]) => changed(h)).length, 'Status differs from previous patrol', 'var(--accent)'],
    ['Alerts sent', missions.reduce((n, m) => n + m.alerts.filter(a => a.severity !== 'info').length, 0), 'Flagged and review alerts', 'var(--ink)']];
  fill($('tiles'), tiles.map(([label, value, note, color]) => el('div', {class: 'tile'},
    el('div', {class: 'label'}, label), el('div', {class: 'value', style: `color:${color}`}, value), el('div', {class: 'note'}, note))));
}

function renderMissions() {
  if (!missions.length) { fill($('missions'), el('tr', {}, el('td', {colspan: 9, class: 'empty'}, 'No missions on this site yet.'))); return; }
  fill($('missions'), missions.map((m, i) => {
    const c = counts(m);
    return el('tr', {},
      el('td', {}, el('button', {class: 'link', onclick: () => selectMission(m.id)}, m.id), i === 0 ? el('div', {class: 'sub'}, 'Latest') : null),
      el('td', {}, new Date(m.started).toLocaleString()), el('td', {}, m.status), el('td', {}, m.inspections.length),
      el('td', {style: 'color:var(--flag-fg);font-weight:700'}, c.flagged), el('td', {style: 'color:var(--rev-fg);font-weight:700'}, c.review),
      el('td', {style: 'color:var(--pass-fg);font-weight:700'}, c.pass), el('td', {}, m.alerts.filter(a => a.severity !== 'info').length),
      el('td', {}, el('a', {href: `/api/missions/${m.id}/protocol.pdf`, target: '_blank'}, 'PDF'), ' · ',
        el('a', {href: `/api/missions/${m.id}/export.csv`}, 'CSV'), ' · ',
        el('a', {href: `/api/missions/${m.id}/export.json`, target: '_blank'}, 'JSON')));
  }));
}

function renderMatrix(history) {
  const order = missions.slice().reverse().map(m => m.id);
  const head = el('thead', {}, el('tr', {}, el('th', {}, 'Ladder'), order.map(id => el('th', {}, id)), el('th', {}, 'What changed')));
  const rows = Object.entries(history).sort().map(([asset, entries]) => {
    const byMission = Object.fromEntries(entries.map(e => [e.mission_id, e]));
    const last = entries[entries.length - 1], prev = entries[entries.length - 2];
    let note = 'First inspection';
    if (prev) note = changed(entries) ? `${STATUS_LABEL[prev.status]} → ${STATUS_LABEL[last.status]}: ${last.reason}`
      : last.status === 'AUTOMATED_PASS' ? 'No change' : `Still open: ${last.reason}`;
    return el('tr', {class: changed(entries) ? 'flagged' : ''}, el('td', {}, el('strong', {}, asset)),
      order.map(id => el('td', {}, byMission[id] ? statusPill(byMission[id].status) : '–')), el('td', {}, note));
  });
  fill($('matrix'), head, el('tbody', {}, rows));
}

function renderOpen(history) {
  const open = Object.entries(history).filter(([, h]) => h[h.length - 1].status !== 'AUTOMATED_PASS');
  if (!open.length) { fill($('open'), el('p', {class: 'empty'}, 'Nothing open.')); return; }
  fill($('open'), open.map(([asset, h]) => {
    const since = [...h].reverse().find((e, i, arr) => !arr[i + 1] || arr[i + 1].status === 'AUTOMATED_PASS');
    const last = h[h.length - 1];
    return el('div', {class: 'rule'}, el('strong', {}, `${asset} · ${STATUS_LABEL[last.status]}`), el('span', {}, last.reason),
      el('span', {class: 'sub'}, `Open since ${since ? since.mission_id : last.mission_id}`));
  }));
}

function selectMission(id) {
  const m = missions.find(x => x.id === id);
  $('reg-title').textContent = `Ladder register · ${m.id}`;
  fill($('reg-exports'), el('a', {class: 'btn', href: `/api/missions/${m.id}/export.csv`}, 'CSV'),
    el('a', {class: 'btn primary', href: `/api/missions/${m.id}/protocol.pdf`, target: '_blank'}, 'Protocol PDF'));
  fill($('register'), m.inspections.map(rec => el('tr', {class: rec.status === 'FLAGGED' ? 'flagged' : ''},
    el('td', {}, el('button', {class: 'link', onclick: () => openLadder($('ladder'), rec, checklist, m.id)}, rec.asset_id)),
    el('td', {}, ascentLabel(rec)), el('td', {}, statusPill(rec.status)), el('td', {}, rec.reason),
    el('td', {}, (rec.ladder || {}).sticker_status || rec.label_status || '–'),
    el('td', {}, el('a', {href: `/api/missions/${m.id}/ladders/${rec.asset_id}/protocol.pdf`, target: '_blank'}, 'PDF')))));
}

(async () => {
  checklist = await getJSON('/api/checklist');
  const sites = await getJSON('/api/sites');
  fill($('site'), sites.length ? sites.map(s => el('option', {value: s}, s)) : [el('option', {}, 'No sites yet')]);
  $('site').addEventListener('change', e => loadSite(e.target.value));
  if (sites.length) await loadSite(sites[0]);
})();
