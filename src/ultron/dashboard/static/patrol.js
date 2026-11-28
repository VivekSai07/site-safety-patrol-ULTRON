// One patrol's record: route, ladders, the other objects it found and nesseler's other checklists.
// Read only. Answers come from /api/missions/{id}/site-summary; nothing here decides a result.
const $ = id => document.getElementById(id);
const SVG = 'http://www.w3.org/2000/svg';
const P = {id: new URLSearchParams(location.search).get('id'), mission: null, summary: null, checks: [], route: null, open: null};
const LADDER_DOT = {FLAGGED: 'var(--flag-dot)', MANUAL_REVIEW: 'var(--rev-dot)', AUTOMATED_PASS: 'var(--pass-dot)', INSPECTING: 'var(--live-dot)'};
const LADDER_ORDER = {FLAGGED: 0, MANUAL_REVIEW: 1, INSPECTING: 2, AUTOMATED_PASS: 3};

function svg(tag, attrs, style) {
  const node = document.createElementNS(SVG, tag);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  if (style) node.setAttribute('style', style);
  return node;
}
const media = photo => `/media/${encodeURIComponent(P.id)}/${encodeURIComponent(photo)}.jpg`;
const rowText = row => (LANG === 'de' ? row.text : row.english || row.text);
const kindLabel = kind => t(`kind.${kind}`);

async function boot() {
  if (!P.id) return fill($('title'), t('p.none'));
  const [mission, summary, checks, route] = await Promise.all([
    getJSON(`/api/missions/${encodeURIComponent(P.id)}`), getJSON(`/api/missions/${encodeURIComponent(P.id)}/site-summary`),
    getJSON(`/api/missions/${encodeURIComponent(P.id)}/site-checks`), getJSON(`/api/missions/${encodeURIComponent(P.id)}/route`)]);
  Object.assign(P, {mission, summary, checks, route});
  // problems first: a checklist with a "nein" or an open robot row starts unfolded
  P.open = new Set(summary.checklists.filter(c => ['nein', 'offen'].includes(c.headline)).map(c => c.key));
  render();
}

function render() {
  const m = P.mission;
  if (!m) return;
  const history = `/history?${new URLSearchParams({site: m.site_id})}`;
  $('back').href = $('nav-history').href = history;
  $('site').textContent = m.site_id;
  $('sim-badge').hidden = !m.synthetic;
  $('title').textContent = m.id;
  $('when').textContent = new Date(m.started).toLocaleString(LANG === 'de' ? 'de-DE' : 'en-GB', {dateStyle: 'medium', timeStyle: 'short'});
  fill($('state'), m.status === 'RUNNING' ? el('span', {class: 'pill s-INSPECTING'}, t('h.running')) : null);
  renderTiles(); renderMap(); renderDownloads(); renderLadders(); renderObjects(); renderChecklists();
}

function renderTiles() {
  const ladders = P.mission.inspections, rows = P.summary.checklists.flatMap(c => c.rows);
  const flagged = ladders.filter(l => l.status === 'FLAGGED').length;
  const no = rows.filter(r => r.answer === 'nein').length, open = rows.filter(r => r.answer === 'offen').length;
  const found = P.summary.presence.reduce((n, p) => n + p.found, 0);
  fill($('tiles'), [[t('h.ladders'), ladders.length, ''], [t('h.flagged'), flagged, flagged ? 'var(--flag-fg)' : ''],
    [t('p.found'), found, ''], [t('p.rows_no'), no, no ? 'var(--flag-fg)' : ''], [t('p.rows_open'), open, open ? 'var(--rev-fg)' : '']]
    .map(([label, value, color]) => el('div', {class: 'tile'}, el('div', {class: 'value', style: color ? `color:${color}` : ''}, value),
      el('div', {class: 'label'}, label))));
}

// Route on the occupancy map the patrol drove on (or the mission's zone), ladders and objects pinned.
function renderMap() {
  const map = $('map'), grid = P.route.grid, m = P.mission;
  const zone = (m.map && m.map.bounds) || null;
  const pins = [...m.inspections.map(l => l.position), ...P.checks.map(c => c.position).filter(Boolean)];
  const poses = P.route.poses;
  let b = grid ? [grid.origin[0], grid.origin[1], grid.origin[0] + grid.width * grid.resolution, grid.origin[1] + grid.height * grid.resolution] : zone;
  if (!b) {
    const xs = [...poses.map(p => p[1]), ...pins.map(p => p[0])], ys = [...poses.map(p => p[2]), ...pins.map(p => p[1])];
    b = xs.length ? [Math.min(...xs) - 1, Math.min(...ys) - 1, Math.max(...xs) + 1, Math.max(...ys) + 1] : [0, 0, 10, 8];
  }
  const pad = grid ? 0 : 0.4, w = b[2] - b[0] + 2 * pad, h = b[3] - b[1] + 2 * pad;
  const X = x => x - b[0] + pad, Y = y => b[3] + pad - y;
  map.setAttribute('viewBox', `0 0 ${w} ${h}`);
  map.replaceChildren();
  if (grid) map.append(svg('image', {href: `/api/missions/${encodeURIComponent(P.id)}/map.png`, x: 0, y: 0, width: w, height: h, preserveAspectRatio: 'none'}));
  else map.append(svg('rect', {x: X(b[0]), y: Y(b[3]), width: b[2] - b[0], height: b[3] - b[1], 'stroke-width': 0.04}, 'fill: var(--surface); stroke: var(--map-line)'));
  for (const k of (!grid && m.map && m.map.keepouts) || []) {
    map.append(svg('rect', {x: X(k[0]), y: Y(k[3]), width: k[2] - k[0], height: k[3] - k[1], 'stroke-width': 0.03, 'stroke-dasharray': '0.1 0.08'}, 'fill: var(--hatch); stroke: var(--muted)'));
  }
  if (poses.length > 1) map.append(svg('polyline', {points: poses.map(p => `${X(p[1])},${Y(p[2])}`).join(' '), fill: 'none', 'stroke-width': 0.06, 'stroke-linejoin': 'round'}, 'stroke: var(--live-dot); opacity: .85'));
  const label = (x, y, text) => {
    const node = svg('text', {x, y: y - 0.32, 'text-anchor': 'middle', 'font-size': 0.26, 'font-weight': 700, 'stroke-width': 0.08, 'paint-order': 'stroke'}, 'fill: var(--ink); stroke: var(--surface)');
    node.textContent = text;
    return node;
  };
  for (const l of m.inspections) {
    const cx = X(l.position[0]), cy = Y(l.position[1]);
    map.append(svg('circle', {cx, cy, r: 0.2, 'stroke-width': 0.05}, `fill: ${LADDER_DOT[l.status]}; stroke: var(--surface)`), label(cx, cy, l.asset_id));
  }
  for (const c of P.checks.filter(c => c.position)) {
    const cx = X(c.position[0]), cy = Y(c.position[1]), r = 0.17;
    map.append(svg('rect', {x: cx - r, y: cy - r, width: 2 * r, height: 2 * r, 'stroke-width': 0.05, transform: `rotate(45 ${cx} ${cy})`}, 'fill: var(--accent); stroke: var(--surface)'), label(cx, cy, c.object_id));
  }
  $('map-info').textContent = `${(b[2] - b[0]).toFixed(1)} × ${(b[3] - b[1]).toFixed(1)} m`;
  $('map-legend').textContent = poses.length > 1 ? t('p.legend_map') : t('p.no_route');
}

function renderDownloads() {
  const id = encodeURIComponent(P.id);
  const link = (href, label, note) => el('a', {class: 'btn', href, download: ''}, label, el('span', {class: 'sub'}, note));
  fill($('downloads'),
    link(`/api/missions/${id}/site-checks.csv`, t('p.dl_checklists'), 'CSV'),
    link(`/api/missions/${id}/objects.csv`, t('p.dl_objects'), 'CSV'),
    link(`/api/missions/${id}/export.csv`, t('p.dl_ladders'), 'CSV'),
    el('a', {class: 'btn', href: `/api/missions/${id}/export.json`, target: '_blank'}, t('p.dl_all'), el('span', {class: 'sub'}, 'JSON')));
}

function renderLadders() {
  const ladders = P.mission.inspections.slice().sort((a, b) => (LADDER_ORDER[a.status] ?? 9) - (LADDER_ORDER[b.status] ?? 9) || a.asset_id.localeCompare(b.asset_id));
  fill($('ladders'), ladders.length ? el('div', {class: 'table-box'}, el('table', {},
    el('thead', {}, el('tr', {}, [t('h.ladder'), t('h.type'), t('result'), t('h.reason'), ''].map(x => el('th', {}, x)))),
    el('tbody', {}, ladders.map(r => el('tr', {class: r.status === 'FLAGGED' ? 'flagged' : ''},
      el('td', {}, el('a', {class: 'mono', style: 'font-weight:700', href: `/ladder?${new URLSearchParams({site: P.mission.site_id, id: r.asset_id})}`}, r.asset_id)),
      el('td', {}, ascentLabel(r)), el('td', {}, statusPill(r.status)), el('td', {}, reasonText(r)),
      el('td', {}, el('a', {href: reportUrl(P.mission.site_id, r), target: '_blank'}, t('report')))))))) : el('p', {class: 'sub'}, t('h.none')));
}

// Found objects, and the kinds the checklists ask about that this patrol did not find.
function renderObjects() {
  const cards = P.checks.map(c => el('figure', {class: 'object-card'},
    c.photos.length ? el('a', {href: media(c.photos[0]), target: '_blank'}, el('img', {src: media(c.photos[0]), alt: `${c.object_id} ${kindLabel(c.kind)}`, loading: 'lazy'}))
      : el('div', {class: 'no-photo'}, t('p.no_photo')),
    el('figcaption', {},
      el('div', {style: 'display:flex;justify-content:space-between;gap:8px;align-items:baseline'},
        el('strong', {}, kindLabel(c.kind)), el('span', {class: `pill ${c.verified ? 'r-OK' : 'r-N_A'}`}, t(c.verified ? 'p.confirmed' : 'p.unconfirmed'))),
      el('div', {class: 'sub mono'}, c.object_id, c.position ? ` · ${c.position[0].toFixed(1)} / ${c.position[1].toFixed(1)} m` : ''),
      c.remarks ? el('div', {class: 'sub'}, c.remarks) : null)));
  const missing = P.summary.presence.filter(p => !p.found);
  fill($('objects'),
    cards.length ? cards : el('p', {class: 'sub'}, t('p.no_objects')),
    missing.length ? el('div', {class: 'missing'}, el('span', {class: 'sub'}, t('p.not_found_kinds')),
      missing.map(p => el('span', {class: `pill ${p.detector ? 'r-N_A' : 'r-NOT_CHECKED'}`, title: p.detector ? t('ans.nicht_gefunden') : t('ans.kein_detektor')},
        kindLabel(p.kind), p.detector ? '' : ` · ${t('ans.kein_detektor')}`))) : null);
}

function rowLine(r) {
  const objects = r.objects.length ? r.objects.join(', ') : '';
  return el('tr', {class: r.answer === 'nein' ? 'flagged' : ''},
    el('td', {class: 'mono'}, r.id),
    el('td', {}, rowText(r), r.remark ? el('div', {class: 'sub'}, r.remark) : null),
    el('td', {}, answerPill(r.answer)),
    el('td', {class: 'mono'}, objects),
    el('td', {}, r.photo ? el('a', {href: media(r.photo), target: '_blank'}, el('img', {class: 'thumb', src: media(r.photo), alt: `${r.id} ${t('photo')}`, loading: 'lazy'})) : ''));
}

function renderChecklists() {
  fill($('checklists'), P.summary.checklists.map(c => {
    const robot = c.rows.filter(r => r.mode !== 'person'), person = c.rows.filter(r => r.mode === 'person');
    const counts = Object.entries(c.counts).map(([answer, n]) => el('span', {class: `pill ${ANSWER_CLASS[answer]}`}, `${n} ${t(`ans.${answer}`)}`));
    return el('details', {class: `card checklist-card${c.headline === 'nein' ? ' has-flag' : ''}`, open: P.open.has(c.key) || null,
      ontoggle: e => { if (e.target.open) P.open.add(c.key); else P.open.delete(c.key); }},
      el('summary', {},
        el('strong', {}, LANG === 'de' ? c.title : c.english), answerPill(c.headline),
        el('span', {class: 'counts'}, counts)),
      robot.length ? el('div', {class: 'table-box'}, el('table', {},
        el('thead', {}, el('tr', {}, [t('p.row'), t('item'), t('p.answer'), t('p.objects_short'), t('photo')].map(x => el('th', {}, x)))),
        el('tbody', {}, robot.map(rowLine)))) : el('p', {class: 'sub'}, t('p.all_person')),
      person.length ? el('details', {class: 'person-rows'},
        el('summary', {class: 'sub'}, t('p.person_rows', {n: person.length})),
        el('ol', {}, person.map(r => el('li', {}, el('span', {class: 'mono'}, `${r.id} `), rowText(r))))) : null,
      el('div', {class: 'sub'}, `${t('p.form')}: ${c.document}`));
  }));
}

initChrome(render);
boot().catch(error => fill($('title'), `${P.id || ''} – ${error.message}`));
