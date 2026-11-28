// Site history: ladders across patrols and the list of patrols.
const $ = id => document.getElementById(id);
const H = {site: new URLSearchParams(location.search).get('site'), missions: [], ladders: {}};

async function boot() {
  const sites = await getJSON('/api/sites');
  if (!H.site) H.site = sites[0];
  $('site').textContent = H.site || '';
  fill($('site-select'), sites.map(s => el('option', {value: s, selected: s === H.site}, s)));
  $('site-select').onchange = e => { location.search = new URLSearchParams({site: e.target.value}).toString(); };
  if (!H.site) return render();
  const list = await getJSON(`/api/missions?site_id=${encodeURIComponent(H.site)}`);
  H.missions = await Promise.all(list.map(async m => {
    const [full, alerts] = await Promise.all([getJSON(`/api/missions/${m.id}`), getJSON(`/api/alerts?mission_id=${m.id}`)]);
    return {...full, alerts: alerts.filter(a => a.severity !== 'info')};
  }));
  H.ladders = await getJSON(`/api/sites/${encodeURIComponent(H.site)}/ladders`);
  render();
}

function change(h) {
  if (h.length < 2) return t('h.first');
  const [prev, last] = h.slice(-2);
  if (prev.status !== last.status) return last.status === 'FLAGGED' || prev.status === 'AUTOMATED_PASS' ? t('h.worse') : `${t(`status.${prev.status}`)} → ${t(`status.${last.status}`)}`;
  return last.status === 'AUTOMATED_PASS' ? t('h.same') : t('h.still');
}

function render() {
  const ladders = Object.entries(H.ladders).sort();
  const open = ladders.filter(([, h]) => h[h.length - 1].status !== 'AUTOMATED_PASS').length;
  const changed = ladders.filter(([, h]) => h.length > 1 && h[h.length - 1].status !== h[h.length - 2].status).length;
  fill($('tiles'), [[t('h.patrols'), H.missions.length, 'var(--ink)'], [t('h.ladders'), ladders.length, 'var(--ink)'],
    [t('h.open'), open, open ? 'var(--flag-fg)' : 'var(--ink)'], [t('h.changed'), changed, changed ? 'var(--accent)' : 'var(--ink)']]
    .map(([label, value, color]) => el('div', {class: 'tile'}, el('div', {class: 'value', style: `color:${color}`}, value), el('div', {class: 'label'}, label))));

  const order = H.missions.slice().reverse().map(m => m.id);
  fill($('matrix'),
    el('thead', {}, el('tr', {}, el('th', {}, t('h.ladders')), order.map(id => el('th', {class: 'mono'}, id)), el('th', {}, t('h.change')), el('th', {}))),
    el('tbody', {}, ladders.map(([id, h]) => {
      const by = Object.fromEntries(h.map(e => [e.mission_id, e]));
      const last = h[h.length - 1];
      return el('tr', {class: h.length > 1 && last.status !== h[h.length - 2].status ? 'flagged' : ''},
        el('td', {}, el('a', {class: 'mono', style: 'font-weight:700', href: `/ladder?${new URLSearchParams({site: H.site, id})}`}, id)),
        order.map(mid => el('td', {}, by[mid] ? statusPill(by[mid].status) : '–')),
        el('td', {}, change(h)),
        el('td', {}, el('a', {href: `/api/sites/${encodeURIComponent(H.site)}/ladders/${id}/report.pdf`, target: '_blank'}, t('report'))));
    })));

  // One card per patrol, newest first (the API's order), with the ladders it inspected underneath.
  // The newest card starts open; which cards are open survives a re-render (language or theme switch).
  if (!H.open) H.open = new Set(H.missions.slice(0, 1).map(m => m.id));
  fill($('patrols'), H.missions.length ? H.missions.map(m => patrolCard(m, H.open.has(m.id))) : el('p', {class: 'sub'}, t('h.empty')));
}

const SEVERITY = {FLAGGED: 0, MANUAL_REVIEW: 1, INSPECTING: 2, AUTOMATED_PASS: 3};

// What changed for this ladder on this patrol, compared with its previous patrol on the site.
function changeAt(assetId, missionId) {
  const h = H.ladders[assetId] || [];
  const at = h.findIndex(e => e.mission_id === missionId);
  return at < 0 ? '' : change(h.slice(0, at + 1));
}

function patrolCard(m, open) {
  const flagged = m.inspections.filter(i => i.status === 'FLAGGED').length;
  const started = new Date(m.started).toLocaleString(LANG === 'de' ? 'de-DE' : 'en-GB', {dateStyle: 'short', timeStyle: 'short'});
  const ladders = m.inspections.slice().sort((a, b) =>
    (SEVERITY[a.status] ?? 9) - (SEVERITY[b.status] ?? 9) || a.asset_id.localeCompare(b.asset_id));
  const counts = [[t('h.ladders'), m.inspections.length, ''], [t('h.flagged'), flagged, flagged ? 'color:var(--flag-fg);font-weight:700' : ''],
    [t('h.alerts'), m.alerts.length, '']];
  return el('details', {class: `card patrol${flagged ? ' has-flag' : ''}`, open: open || null,
    ontoggle: e => { if (e.target.open) H.open.add(m.id); else H.open.delete(m.id); }},
    el('summary', {},
      el('span', {class: 'mono', style: 'font-weight:700'}, m.id),
      el('span', {class: 'sub'}, started),
      m.status === 'RUNNING' ? el('span', {class: 'pill s-INSPECTING'}, t('h.running')) : null,
      el('span', {class: 'counts'}, counts.map(([label, n, style]) => el('span', {style}, `${n} ${label}`)))),
    ladders.length ? el('div', {class: 'table-box'}, el('table', {},
      el('thead', {}, el('tr', {}, [t('h.ladder'), t('h.type'), t('result'), t('h.reason'), t('h.change'), ''].map(x => el('th', {}, x)))),
      el('tbody', {}, ladders.map(r => el('tr', {class: r.status === 'FLAGGED' ? 'flagged' : ''},
        el('td', {}, el('a', {class: 'mono', style: 'font-weight:700', href: `/ladder?${new URLSearchParams({site: H.site, id: r.asset_id})}`}, r.asset_id)),
        el('td', {}, ascentLabel(r)),
        el('td', {}, statusPill(r.status)),
        el('td', {}, reasonText(r)),
        el('td', {}, changeAt(r.asset_id, m.id)),
        el('td', {}, el('a', {href: reportUrl(H.site, r), target: '_blank'}, t('report')))))))) : el('p', {class: 'sub'}, t('h.none')),
    el('div', {class: 'sub'}, `${t('h.data')}: `, el('a', {href: `/api/missions/${m.id}/export.csv`}, 'CSV'), ' · ',
      el('a', {href: `/api/missions/${m.id}/export.json`, target: '_blank'}, 'JSON')));
}

initChrome(render);
boot();
