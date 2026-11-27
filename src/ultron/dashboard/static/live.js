// Live page: loads the current mission, then follows the server's event stream.
const S = {mission: null, inspections: new Map(), confirmed: new Map(), photos: new Map(), order: [], alerts: new Map(),
  events: [], pose: null, trail: [], checklist: [], history: {}, clock: null};
const $ = id => document.getElementById(id);
const SVG = 'http://www.w3.org/2000/svg';
const DOT = {FLAGGED: 'var(--flag-dot)', MANUAL_REVIEW: 'var(--rev-dot)', AUTOMATED_PASS: 'var(--pass-dot)', INSPECTING: 'var(--live-dot)'};

function svg(tag, attrs, style) {
  const node = document.createElementNS(SVG, tag);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  if (style) node.setAttribute('style', style);
  return node;
}

function noteEvent(e) {
  if (!e.asset_id) return;
  if (e.type === 'LADDER_CONFIRMED') {
    if (!S.confirmed.has(e.asset_id)) S.order.push(e.asset_id);
    S.confirmed.set(e.asset_id, e);
  }
  if (e.type === 'EVIDENCE_CAPTURED') S.photos.set(e.asset_id, (S.photos.get(e.asset_id) || 0) + 1);
}

async function loadHistory() {
  if (S.mission) S.history = await getJSON(`/api/sites/${encodeURIComponent(S.mission.site_id)}/ladders`).catch(() => ({}));
}

async function load() {
  const live = await getJSON('/api/live');
  S.mission = live.mission;
  S.inspections = new Map((live.inspections || []).map(i => [i.asset_id, i]));
  S.alerts = new Map((live.alerts || []).map(a => [a.id, a]));
  S.confirmed = new Map(); S.photos = new Map(); S.order = [];
  for (const e of live.events || []) noteEvent(e);
  for (const i of live.inspections || []) if (!S.order.includes(i.asset_id)) S.order.push(i.asset_id);
  S.events = (live.events || []).slice().reverse();
  S.pose = live.pose;
  S.trail = live.trail || [];
  S.clock = S.pose ? S.pose.time : (live.events || []).reduce((m, e) => Math.max(m, e.time), 0);
  await loadHistory();
  for (const cam of Object.keys(live.cameras || {})) showFrame(cam);
  render();
}

const statusOf = asset => (S.inspections.get(asset) || {}).status || 'INSPECTING';

function render() {
  const m = S.mission;
  $('site').textContent = m ? m.site_id : '–';
  $('sim-badge').hidden = !(m && m.synthetic);
  const running = m && m.status === 'RUNNING';
  $('live-chip').className = `live-chip${running ? '' : ' idle'}`;
  $('live-chip').textContent = running ? t('live') : m ? t('ended') : t('no_mission');
  $('mission-clock').textContent = m ? `${m.id} · ${mmss(S.clock)}` : '';
  renderBanner(); renderSteps(); renderNow(); renderLadders(); renderMap(); renderAlerts(); renderRobot(); renderLog();
}

async function ack(id) {
  const response = await fetch(`/api/alerts/${id}/ack?by=dashboard`, {method: 'POST'});
  if (response.ok) { const alert = await response.json(); S.alerts.set(alert.id, alert); render(); }
}

function renderBanner() {
  const open = [...S.alerts.values()].filter(a => a.severity === 'critical' && !a.acked_at).sort((a, b) => b.id - a.id);
  fill($('banner'), open.map(a => el('div', {class: 'item', role: 'alert'},
    el('strong', {}, `${a.asset_id || t('robot')} · ${a.rule === 'FLAGGED' ? t('status.FLAGGED') : t('robot')} — ${t(`action.${a.rule === 'FLAGGED' ? 'FLAGGED' : 'ROBOT'}`)}`),
    el('button', {onclick: () => ack(a.id)}, t('ack')))));
}

function renderSteps() {
  const ended = S.mission && S.mission.status !== 'RUNNING';
  const steps = S.order.map(asset => {
    const rec = S.inspections.get(asset);
    const status = statusOf(asset);
    return el('li', {class: status}, el('span', {class: 'id mono'}, asset), el('strong', {style: 'font-size:12.5px'}, t(`status.${status}`)),
      el('span', {class: 'sub mono'}, rec && rec.time !== null && rec.time !== undefined ? mmss(rec.time) : t('photos', {n: S.photos.get(asset) || 0})));
  });
  steps.push(el('li', {class: ended ? 'AUTOMATED_PASS' : 'WAIT'}, el('span', {class: 'id'}, t('home')),
    el('strong', {style: 'font-size:12.5px'}, ended ? t('home.done') : t('home.next'))));
  fill($('steps'), steps);
}

function groupResults(asset) {
  const rec = S.inspections.get(asset);
  if (rec) return Object.fromEntries(GROUP_KEYS.map(k => [k, (rec.checklist || {})[k] || 'N_A']));
  const ascent = SIM_ASCENT[((S.confirmed.get(asset) || {}).data || {}).ladder_type];
  return Object.fromEntries(GROUP_KEYS.map(k => [k, k === 'spreiz' && ascent === 'Anlegeleiter' ? 'N_A' : 'PENDING']));
}

function renderNow() {
  const asset = S.order[S.order.length - 1];
  $('now').hidden = !asset;
  if (!asset) return;
  const rec = S.inspections.get(asset);
  const results = groupResults(asset);
  const relevant = GROUP_KEYS.filter(k => results[k] !== 'N_A');
  const done = relevant.filter(k => results[k] !== 'PENDING').length;
  const ascent = rec ? ascentLabel(rec) : t(`asc.${SIM_ASCENT[((S.confirmed.get(asset) || {}).data || {}).ladder_type] || 'Sonstige'}`);
  $('now-label').textContent = rec ? t('last') : t('now');
  $('now-title').textContent = `${asset} · ${ascent}`;
  $('now-count').textContent = rec ? t(`status.${rec.status}`) : `${t('photos', {n: S.photos.get(asset) || 0})} · ${t('result_pending')}`;
  $('now-bar').style.width = `${relevant.length ? Math.round(100 * done / relevant.length) : 0}%`;
  fill($('now-groups'), GROUP_KEYS.map(k => el('div', {class: 'group'}, el('strong', {}, t(`group.${k}`)), resultPill(results[k]))));
}

function renderLadders() {
  const site = S.mission && S.mission.site_id;
  fill($('ladders'), S.order.map(asset => {
    const rec = S.inspections.get(asset);
    const status = statusOf(asset);
    const past = (S.history[asset] || []).filter(h => !rec || h.mission_id !== rec.mission_id);
    const trend = [...past.map(h => h.status), status];
    const first = trend.length - Math.min(5, trend.length);
    return el('article', {class: `ladder edge-${status}`},
      el('div', {class: 'row'},
        rec ? el('button', {class: 'link mono', style: 'font-size:15px', onclick: () => openLadder($('ladder'), rec, S.checklist, site)}, asset)
          : el('strong', {class: 'mono', style: 'font-size:15px'}, asset),
        statusPill(status)),
      el('span', {class: 'sub'}, rec ? ascentLabel(rec) : t('result_pending')),
      rec ? el('span', {}, reasonText(rec)) : null,
      el('div', {class: 'row'},
        el('div', {class: 'trend'}, trend.slice(-5).map((s, i) => el('span', {class: `s-${s}`, title: t(`status.${s}`)}, String(first + i + 1)))),
        rec ? el('a', {href: reportUrl(site, rec), target: '_blank', style: 'font-weight:600'}, t('report')) : null));
  }));
}

function renderAlerts() {
  const alerts = [...S.alerts.values()].filter(a => a.severity !== 'info').sort((a, b) => b.id - a.id);
  if (!alerts.length) { fill($('alerts'), el('p', {class: 'empty', style: 'margin:0'}, t('alerts.none'))); return; }
  fill($('alerts'), alerts.map(a => {
    const ladderRule = a.rule === 'FLAGGED' || a.rule === 'MANUAL_REVIEW';
    return el('div', {class: 'alert-row'},
      el('div', {style: 'display:flex;gap:8px;align-items:center;flex-wrap:wrap'},
        el('span', {class: `pill ${ladderRule ? 's-' + a.rule : 'sev-critical'}`}, ladderRule ? t(`status.${a.rule}`) : t('robot')),
        el('strong', {class: 'mono'}, a.asset_id || ''),
        el('span', {class: 'sub'}, `→ ${(a.recipients || []).map(r => t(`role.${r.role}`)).join(', ')}`)),
      a.acked_at ? el('span', {class: 'pill acked'}, t('acked')) : el('button', {class: 'warn', onclick: () => ack(a.id)}, t('ack')));
  }));
}

function renderRobot() {
  const p = S.pose;
  const items = [[t('robot.state'), p && p.state ? p.state.replaceAll('_', ' ').toLowerCase() : '–'],
    [t('robot.pos'), p ? `${p.x.toFixed(1)} / ${p.y.toFixed(1)} m` : '–'],
    [t('robot.mode'), S.mission ? (S.mission.synthetic ? t('mode.auto_sim') : t('mode.auto')) : '–'],
    [t('robot.takeovers'), String(S.events.filter(e => e.type === 'OPERATOR_INTERVENTION').length)]];
  fill($('robot'), items.map(([k, v]) => el('div', {}, el('span', {}, k), el('strong', {}, v))));
}

function renderLog() {
  const shown = S.events.filter(e => e.type !== 'PATROL_WAYPOINT' && e.type !== 'ROUTE_RESUME').slice(0, 60);
  fill($('log'), shown.map(e => {
    const label = t(`ev.${e.type}`);
    return el('li', {}, el('time', {class: 'mono'}, mmss(e.time)),
      el('span', {}, label === `ev.${e.type}` ? e.type.toLowerCase() : label, e.asset_id ? el('span', {class: 'mono'}, ` ${e.asset_id}`) : null));
  }));
}

function renderMap() {
  const map = $('map');
  const m = S.mission;
  const b = (m && m.map && m.map.bounds) || [0, 0, 12, 9];
  const pad = 0.4, w = b[2] - b[0] + 2 * pad, h = b[3] - b[1] + 2 * pad;
  const X = x => x - b[0] + pad, Y = y => b[3] + pad - y;
  map.setAttribute('viewBox', `0 0 ${w} ${h}`);
  map.replaceChildren();
  map.append(svg('rect', {x: X(b[0]), y: Y(b[3]), width: b[2] - b[0], height: b[3] - b[1], 'stroke-width': 0.04}, 'fill: var(--surface); stroke: var(--map-line)'));
  for (const k of (m && m.map && m.map.keepouts) || []) {
    map.append(svg('rect', {x: X(k[0]), y: Y(k[3]), width: k[2] - k[0], height: k[3] - k[1], 'stroke-width': 0.03, 'stroke-dasharray': '0.1 0.08'}, 'fill: var(--hatch); stroke: var(--muted)'));
  }
  const base = m && m.map && m.map.base;
  if (base) map.append(svg('rect', {x: X(base[0]) - 0.2, y: Y(base[1]) - 0.2, width: 0.4, height: 0.4, 'stroke-width': 0.05}, 'fill: var(--surface); stroke: var(--ink)'));
  if (S.trail.length > 1) {
    map.append(svg('polyline', {points: S.trail.map(([x, y]) => `${X(x)},${Y(y)}`).join(' '), fill: 'none', 'stroke-width': 0.06, 'stroke-linejoin': 'round'}, 'stroke: var(--live-dot); opacity: .85'));
  }
  for (const asset of S.order) {
    const rec = S.inspections.get(asset);
    const conf = S.confirmed.get(asset);
    const pos = rec ? rec.position : conf && conf.data && conf.data.position;
    if (!pos) continue;
    const status = statusOf(asset);
    const cx = X(pos[0]), cy = Y(pos[1]), r = 0.22;
    const fillStyle = `fill: ${DOT[status]}; stroke: var(--surface)`;
    const g = svg('g', {}, rec ? 'cursor: pointer' : '');
    if (status === 'FLAGGED') g.append(svg('rect', {x: cx - r, y: cy - r, width: 2 * r, height: 2 * r, 'stroke-width': 0.06}, fillStyle));
    else if (status === 'MANUAL_REVIEW') g.append(svg('rect', {x: cx - r, y: cy - r, width: 2 * r, height: 2 * r, 'stroke-width': 0.06, transform: `rotate(45 ${cx} ${cy})`}, fillStyle));
    else g.append(svg('circle', {cx, cy, r, 'stroke-width': 0.06}, fillStyle));
    const label = svg('text', {x: cx, y: cy - 0.36, 'text-anchor': 'middle', 'font-size': 0.3, 'font-weight': 700, 'stroke-width': 0.08, 'paint-order': 'stroke'}, 'fill: var(--ink); stroke: var(--surface)');
    label.textContent = asset;
    g.append(label);
    if (rec) g.addEventListener('click', () => openLadder($('ladder'), rec, S.checklist, m.site_id));
    map.append(g);
  }
  if (S.pose) {
    const robot = svg('g', {transform: `translate(${X(S.pose.x)} ${Y(S.pose.y)}) rotate(${-S.pose.yaw * 180 / Math.PI})`});
    robot.append(svg('rect', {x: -0.35, y: -0.2, width: 0.7, height: 0.4, rx: 0.08, 'stroke-width': 0.08}, 'fill: var(--surface); stroke: var(--live-dot)'));
    robot.append(svg('path', {d: 'M0.18,-0.12 L0.42,0 L0.18,0.12 Z'}, 'fill: var(--live-dot)'));
    map.append(robot);
  }
  $('map-info').textContent = `${(b[2] - b[0]).toFixed(1)} × ${(b[3] - b[1]).toFixed(1)} m`;
}

// Camera frames: fetch the newest JPEG when told one exists; never queue old frames.
const camBusy = {};
function showFrame(camera) {
  const box = $(`cam-${camera}`);
  if (!box || camBusy[camera]) return;
  camBusy[camera] = true;
  let img = box.querySelector('img');
  if (!img) { img = el('img', {alt: t(camera === 'front_depth' ? 'cam.depth' : 'cam.front')}); box.removeAttribute('data-i18n'); box.replaceChildren(img); }
  img.onload = img.onerror = () => { camBusy[camera] = false; };
  img.src = `/api/cameras/${camera}/latest.jpg?t=${Date.now()}`;
}

let queued = false;
function scheduleRender() {
  if (queued) return;
  queued = true;
  requestAnimationFrame(() => { queued = false; render(); });
}

function connect() {
  const source = new EventSource('/api/stream');
  const on = (kind, fn) => source.addEventListener(kind, msg => fn(JSON.parse(msg.data)));
  const mine = x => S.mission && x.mission_id === S.mission.id;
  on('mission', m => { if (!S.mission || m.id !== S.mission.id || m.status !== S.mission.status) load(); });
  on('event', e => { if (!mine(e)) return; S.events.unshift(e); noteEvent(e); S.clock = Math.max(S.clock || 0, e.time); scheduleRender(); });
  on('inspection', i => {
    if (!mine(i)) return;
    S.inspections.set(i.asset_id, i);
    if (!S.order.includes(i.asset_id)) S.order.push(i.asset_id);
    loadHistory().then(scheduleRender);
  });
  on('alert', a => { if (mine(a)) { S.alerts.set(a.id, a); scheduleRender(); } });
  on('pose', p => { if (!mine(p)) return; S.pose = p; S.trail.push([p.x, p.y]); S.clock = Math.max(S.clock || 0, p.time); scheduleRender(); });
  on('frame', f => { if (mine(f)) showFrame(f.camera); });
  source.onerror = () => { $('live-chip').className = 'live-chip idle'; $('live-chip').textContent = '…'; };
  source.onopen = () => load();
}

(async () => {
  initChrome(render);
  S.checklist = await getJSON('/api/checklist');
  await load();
  connect();
})();
