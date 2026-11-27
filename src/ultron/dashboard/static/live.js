// Live mission page: loads the current mission, then follows the server's event stream.
const S = {mission: null, inspections: new Map(), confirmed: new Map(), alerts: new Map(), events: [],
  pose: null, trail: [], checklist: [], config: null, clock: null};
const SVG = 'http://www.w3.org/2000/svg';
const $ = id => document.getElementById(id);

function svg(tag, attrs) {
  const node = document.createElementNS(SVG, tag);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  return node;
}

async function load() {
  const live = await getJSON('/api/live');
  S.mission = live.mission;
  S.inspections = new Map((live.inspections || []).map(i => [i.asset_id, i]));
  S.alerts = new Map((live.alerts || []).map(a => [a.id, a]));
  S.events = (live.events || []).slice().reverse();
  S.confirmed = new Map();
  for (const e of live.events || []) noteConfirmed(e);
  S.pose = live.pose;
  S.trail = live.trail || [];
  S.clock = S.pose ? S.pose.time : (live.events || []).reduce((t, e) => Math.max(t, e.time), 0);
  for (const cam of Object.keys(live.cameras || {})) showFrame(cam);
  render();
}

function noteConfirmed(e) {
  if (e.type === 'LADDER_CONFIRMED' && e.asset_id && e.data && e.data.position) S.confirmed.set(e.asset_id, e);
}

function current(asset) {
  if (S.inspections.has(asset)) return S.inspections.get(asset).status;
  return 'INSPECTING';
}

function render() {
  const m = S.mission;
  $('site').textContent = m ? m.site_id : '–';
  $('mission').textContent = m ? m.id : '–';
  $('sim-badge').hidden = !(m && m.synthetic);
  const running = m && m.status === 'RUNNING';
  $('live-dot').className = `live-dot${running ? '' : ' idle'}`;
  $('live-dot').textContent = running ? 'LIVE' : m ? `ENDED · ${m.status}` : 'NO MISSION';
  $('clock').textContent = mmss(S.clock);
  renderTiles(); renderAlerts(); renderRegister(); renderEvents(); renderMap(); renderRobot(); renderExports();
}

function renderTiles() {
  const list = [...S.inspections.values()];
  const count = s => list.filter(i => i.status === s).length;
  const found = new Set([...S.inspections.keys(), ...S.confirmed.keys()]).size;
  const open = [...S.alerts.values()].filter(a => a.severity !== 'info' && !a.acked_at).length;
  const tiles = [
    ['Ladders found', found, `${found - list.length} being inspected`, 'var(--ink)'],
    ['Flagged', count('FLAGGED'), 'Take out of use', 'var(--flag-fg)'],
    ['Manual review', count('MANUAL_REVIEW'), 'Check in person', 'var(--rev-fg)'],
    ['Automated pass', count('AUTOMATED_PASS'), 'Touch checks pending', 'var(--pass-fg)'],
    ['Alerts open', open, open ? 'Waiting for acknowledgement' : 'All acknowledged', open ? 'var(--accent)' : 'var(--ink)'],
    ['Area coverage', 'Unverified', 'Waypoints ≠ area covered', 'var(--grey-fg)'],
  ];
  fill($('tiles'), tiles.map(([label, value, note, color]) => el('div', {class: 'tile'},
    el('div', {class: 'label'}, label), el('div', {class: 'value', style: `color:${color}`}, value), el('div', {class: 'note'}, note))));
  const link = $('alert-count');
  link.textContent = open ? `${open} alert${open > 1 ? 's' : ''} not acknowledged` : 'No open alerts';
  link.className = `alert-count${open ? '' : ' zero'}`;
}

async function ack(id) {
  const response = await fetch(`/api/alerts/${id}/ack?by=dashboard`, {method: 'POST'});
  if (response.ok) { const alert = await response.json(); S.alerts.set(alert.id, alert); render(); }
}

function deliveryPills(recipient) {
  return el('span', {class: 'ch'}, (recipient.deliveries || []).map(d =>
    el('span', {class: `pill d-${d.state}`, title: d.error || ''}, `${d.channel}: ${d.state}`)));
}

function renderAlerts() {
  const alerts = [...S.alerts.values()].sort((a, b) => b.id - a.id);
  const critical = alerts.filter(a => a.severity === 'critical' && !a.acked_at);
  fill($('banner'), critical.map(a => el('div', {class: 'item', role: 'alert'},
    el('div', {}, el('strong', {}, a.title), el('div', {}, a.action || a.message)),
    el('button', {onclick: () => ack(a.id)}, 'Acknowledge'))));
  if (!alerts.length) { fill($('alert-list'), el('p', {class: 'empty'}, 'No notifications yet.')); return; }
  fill($('alert-list'), alerts.map(a => el('article', {class: `alert ${a.severity}`},
    el('div', {class: 'row'},
      el('div', {style: 'display:flex;gap:8px;align-items:center;flex-wrap:wrap'},
        el('span', {class: `pill sev-${a.severity}`}, a.severity.toUpperCase()), el('strong', {}, a.title)),
      el('span', {class: 'sub'}, new Date(a.created).toLocaleTimeString())),
    el('div', {}, a.message),
    a.action ? el('div', {}, el('strong', {}, 'Action requested: '), a.action) : null,
    el('div', {class: 'recips'}, [...(a.recipients || []), ...(a.escalation || []).map(r => ({...r, title: `Escalated → ${r.title}`}))]
      .map(r => el('div', {class: 'recip'}, el('span', {}, el('strong', {}, r.title), ` · ${r.name} · ${r.where}`), deliveryPills(r)))),
    el('div', {class: 'row'},
      a.acked_at ? el('span', {class: 'pill acked'}, `Acknowledged by ${a.acked_by} · ${new Date(a.acked_at).toLocaleTimeString()}`)
        : a.severity === 'info' ? el('span', {class: 'sub'}, 'Information only') : el('span', {class: 'pill sev-warning'}, 'Not acknowledged'),
      !a.acked_at && a.severity !== 'info' ? el('button', {class: 'warn', onclick: () => ack(a.id)}, 'Acknowledge') : null))));
}

function renderRules() {
  const cfg = S.config;
  if (!cfg) return;
  const titles = Object.fromEntries(cfg.recipients.map(r => [r.role, r.title]));
  const text = {FLAGGED: 'Immediately', MANUAL_REVIEW: 'Immediately', ROBOT_STOPPED: 'E-stop, takeover or link lost',
    MISSION_SUMMARY: 'When the mission ends', ESCALATION: `If a critical alert is not acknowledged in ${Math.round(cfg.escalate_after_s / 60)} min`};
  const cls = {FLAGGED: 's-FLAGGED', MANUAL_REVIEW: 's-MANUAL_REVIEW', ROBOT_STOPPED: 's-INSPECTING', MISSION_SUMMARY: 'sev-info', ESCALATION: 'sev-critical'};
  fill($('rules'), Object.entries(cfg.rules).map(([rule, roles]) => el('div', {class: 'rule'},
    el('span', {class: `pill ${cls[rule] || 'sev-info'}`}, rule.replace('_', ' ')),
    el('span', {}, roles.map(r => titles[r] || r).join(', ')), el('span', {class: 'sub'}, text[rule] || ''))));
  $('channels').textContent = `Channels: ${cfg.channels.join(', ')}${cfg.channels.length === 1 ? ' (phone push not configured)' : ''}`;
}

function renderRegister() {
  const assets = [...new Set([...S.confirmed.keys(), ...S.inspections.keys()])].sort();
  if (!assets.length) { fill($('register'), el('tr', {}, el('td', {colspan: 6, class: 'empty'}, 'No ladders yet.'))); return; }
  fill($('register'), assets.map(asset => {
    const rec = S.inspections.get(asset);
    if (!rec) return el('tr', {}, el('td', {}, el('strong', {}, asset)), el('td', {}, '…'), el('td', {}, statusPill('INSPECTING')),
      el('td', {}, 'Inspection in progress'), el('td', {}, '–'), el('td', {}, '–'));
    const sticker = (rec.ladder || {}).sticker_status || rec.label_status || '–';
    return el('tr', {class: rec.status === 'FLAGGED' ? 'flagged' : ''},
      el('td', {}, el('button', {class: 'link', onclick: () => openLadder($('ladder'), rec, S.checklist, S.mission.id)}, asset)),
      el('td', {}, ascentLabel(rec)), el('td', {}, statusPill(rec.status)), el('td', {}, rec.reason),
      el('td', {}, sticker), el('td', {}, mmss(rec.time)));
  }));
  const m = S.mission;
  $('coverage').textContent = m && m.summary && m.summary.coverage_note ? `Coverage: ${m.summary.coverage_note}.` :
    'Coverage: area coverage is reported separately from waypoint completion and is not verified yet.';
}

function renderEvents() {
  fill($('events'), S.events.slice(0, 80).map(e => el('li', {}, el('time', {}, mmss(e.time)),
    el('span', {}, el('strong', {}, e.type.replaceAll('_', ' ').toLowerCase()), e.asset_id ? ` · ${e.asset_id}` : ''))));
}

function renderRobot() {
  const p = S.pose;
  const items = [['State', p ? p.state.replaceAll('_', ' ').toLowerCase() || '–' : '–'],
    ['Position', p ? `${p.x.toFixed(1)}, ${p.y.toFixed(1)} m` : '–'],
    ['Mode', S.mission ? (S.mission.synthetic ? 'Autonomous (simulation)' : 'Autonomous') : '–'],
    ['Takeovers', String(S.events.filter(e => e.type === 'OPERATOR_INTERVENTION').length)]];
  fill($('robot'), items.map(([k, v]) => el('div', {}, el('span', {}, k), el('strong', {}, v))));
}

function renderExports() {
  const m = S.mission;
  fill($('exports'), m ? [
    el('a', {class: 'btn', href: `/api/missions/${m.id}/export.csv`}, 'CSV'),
    el('a', {class: 'btn', href: `/api/missions/${m.id}/export.json`, target: '_blank'}, 'JSON'),
    el('a', {class: 'btn primary', href: `/api/missions/${m.id}/protocol.pdf`, target: '_blank'}, 'Protocol PDF')] : []);
}

function renderMap() {
  const map = $('map');
  const m = S.mission;
  const b = (m && m.map && m.map.bounds) || [0, 0, 12, 9];
  const pad = 0.4, w = b[2] - b[0] + 2 * pad, h = b[3] - b[1] + 2 * pad;
  const X = x => x - b[0] + pad, Y = y => b[3] + pad - y;
  map.setAttribute('viewBox', `0 0 ${w} ${h}`);
  map.replaceChildren();
  map.append(svg('rect', {x: X(b[0]), y: Y(b[3]), width: b[2] - b[0], height: b[3] - b[1], fill: '#f7f8f5', stroke: '#8a9099', 'stroke-width': 0.04}));
  for (const k of (m && m.map && m.map.keepouts) || []) {
    map.append(svg('rect', {x: X(k[0]), y: Y(k[3]), width: k[2] - k[0], height: k[3] - k[1], fill: '#d4d7d1', stroke: '#6b7079', 'stroke-width': 0.03, 'stroke-dasharray': '0.1 0.08'}));
  }
  const base = m && m.map && m.map.base;
  if (base) map.append(svg('rect', {x: X(base[0]) - 0.2, y: Y(base[1]) - 0.2, width: 0.4, height: 0.4, fill: '#fff', stroke: '#17191c', 'stroke-width': 0.05}));
  if (S.trail.length > 1) {
    map.append(svg('polyline', {points: S.trail.map(([x, y]) => `${X(x)},${Y(y)}`).join(' '), fill: 'none', stroke: '#2563a8', 'stroke-width': 0.06, 'stroke-linejoin': 'round', opacity: 0.8}));
  }
  const assets = new Set([...S.confirmed.keys(), ...S.inspections.keys()]);
  for (const asset of assets) {
    const rec = S.inspections.get(asset);
    const pos = rec ? rec.position : S.confirmed.get(asset).data.position;
    const status = current(asset);
    const color = {FLAGGED: '#c62828', MANUAL_REVIEW: '#b7791f', AUTOMATED_PASS: '#2e7d4f', INSPECTING: '#2563a8'}[status];
    const cx = X(pos[0]), cy = Y(pos[1]), r = 0.22;
    const group = svg('g', {cursor: rec ? 'pointer' : 'default'});
    if (status === 'FLAGGED') group.append(svg('rect', {x: cx - r, y: cy - r, width: 2 * r, height: 2 * r, fill: color, stroke: '#fff', 'stroke-width': 0.06}));
    else if (status === 'MANUAL_REVIEW') group.append(svg('rect', {x: cx - r, y: cy - r, width: 2 * r, height: 2 * r, fill: color, stroke: '#fff', 'stroke-width': 0.06, transform: `rotate(45 ${cx} ${cy})`}));
    else group.append(svg('circle', {cx, cy, r, fill: color, stroke: '#fff', 'stroke-width': 0.06}));
    const label = svg('text', {x: cx, y: cy - 0.38, 'text-anchor': 'middle', 'font-size': 0.32, 'font-weight': 700, fill: '#17191c', 'paint-order': 'stroke', stroke: '#fff', 'stroke-width': 0.08});
    label.textContent = asset;
    group.append(label);
    if (rec) group.addEventListener('click', () => openLadder($('ladder'), rec, S.checklist, S.mission.id));
    map.append(group);
  }
  if (S.pose) {
    const cx = X(S.pose.x), cy = Y(S.pose.y), deg = -S.pose.yaw * 180 / Math.PI;
    const robot = svg('g', {transform: `translate(${cx} ${cy}) rotate(${deg})`});
    robot.append(svg('rect', {x: -0.35, y: -0.2, width: 0.7, height: 0.4, rx: 0.08, fill: '#fff', stroke: '#2563a8', 'stroke-width': 0.08}));
    robot.append(svg('path', {d: 'M0.18,-0.12 L0.42,0 L0.18,0.12 Z', fill: '#2563a8'}));
    map.append(robot);
  }
  $('map-info').textContent = `${(b[2] - b[0]).toFixed(1)} × ${(b[3] - b[1]).toFixed(1)} m · trail ${S.trail.length} poses`;
}

// Camera frames: fetch the newest JPEG when told a new one exists, never queue up old ones.
const camBusy = {};
function showFrame(camera) {
  const box = $(`cam-${camera}`);
  if (!box || camBusy[camera]) return;
  camBusy[camera] = true;
  let img = box.querySelector('img');
  if (!img) { img = el('img', {alt: `Live ${camera.replace('_', ' ')} frame`}); box.replaceChildren(img); }
  img.onload = img.onerror = () => { camBusy[camera] = false; };
  img.src = `/api/cameras/${camera}/latest.jpg?t=${Date.now()}`;
}

let renderQueued = false;
function scheduleRender() {
  if (renderQueued) return;
  renderQueued = true;
  requestAnimationFrame(() => { renderQueued = false; render(); });
}

function connect() {
  const source = new EventSource('/api/stream');
  const on = (kind, fn) => source.addEventListener(kind, msg => fn(JSON.parse(msg.data)));
  on('mission', m => { if (!S.mission || m.id !== S.mission.id || m.status !== S.mission.status) load(); });
  on('event', e => {
    if (!S.mission || e.mission_id !== S.mission.id) return;
    S.events.unshift(e); noteConfirmed(e); S.clock = Math.max(S.clock || 0, e.time); scheduleRender();
  });
  on('inspection', i => { if (S.mission && i.mission_id === S.mission.id) { S.inspections.set(i.asset_id, i); scheduleRender(); } });
  on('alert', a => { if (S.mission && a.mission_id === S.mission.id) { S.alerts.set(a.id, a); scheduleRender(); } });
  on('pose', p => {
    if (!S.mission || p.mission_id !== S.mission.id) return;
    S.pose = p; S.trail.push([p.x, p.y]); S.clock = Math.max(S.clock || 0, p.time); scheduleRender();
  });
  on('frame', f => { if (S.mission && f.mission_id === S.mission.id) showFrame(f.camera); });
  source.onerror = () => { $('live-dot').className = 'live-dot idle'; $('live-dot').textContent = 'RECONNECTING'; };
  source.onopen = () => load();
}

(async () => {
  [S.checklist, S.config] = await Promise.all([getJSON('/api/checklist'), getJSON('/api/notify/config')]);
  renderRules();
  await load();
  connect();
})();
