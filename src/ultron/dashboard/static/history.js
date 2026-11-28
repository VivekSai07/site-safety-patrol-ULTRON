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

  fill($('patrols'),
    el('thead', {}, el('tr', {}, [t('h.patrol'), t('h.date'), t('h.ladders'), t('h.flagged'), t('h.alerts'), t('h.data')].map(x => el('th', {}, x)))),
    el('tbody', {}, H.missions.map(m => el('tr', {},
      el('td', {class: 'mono', style: 'font-weight:600'}, m.id),
      el('td', {}, new Date(m.started).toLocaleString(LANG === 'de' ? 'de-DE' : 'en-GB', {dateStyle: 'short', timeStyle: 'short'})),
      el('td', {}, m.inspections.length),
      el('td', {style: 'color:var(--flag-fg);font-weight:700'}, m.inspections.filter(i => i.status === 'FLAGGED').length),
      el('td', {}, m.alerts.length),
      el('td', {}, el('a', {href: `/api/missions/${m.id}/export.csv`}, 'CSV'), ' · ', el('a', {href: `/api/missions/${m.id}/export.json`, target: '_blank'}, 'JSON'))))));
}

initChrome(render);
boot();
