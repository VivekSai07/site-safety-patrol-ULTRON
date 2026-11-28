// Shared DOM helpers. All text goes through textContent (no HTML injection) and t() (German/English).
function el(tag, attrs, ...children) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs || {})) {
    if (value === null || value === undefined || value === false) continue;
    if (key === 'class') node.className = value;
    else if (key.startsWith('on')) node.addEventListener(key.slice(2), value);
    else node.setAttribute(key, value === true ? '' : value);
  }
  for (const child of children.flat()) {
    if (child === null || child === undefined || child === false) continue;
    node.append(child instanceof Node ? child : document.createTextNode(String(child)));
  }
  return node;
}

const SIM_ASCENT = {LEANING_SINGLE: 'Anlegeleiter', A_FRAME_STEPLADDER: 'Stehleiter'};
const GROUP_KEYS = ['holme', 'sprossen', 'spreiz', 'beschlag', 'fuesse', 'zubehoer', 'kennzeichnung'];

function statusPill(status) { return el('span', {class: `pill s-${status}`}, t(`status.${status}`)); }
function resultPill(result) { return el('span', {class: `pill r-${result || 'N_A'}`}, result ? t(`result.${result}`) : '–'); }
function mmss(seconds) {
  if (seconds === null || seconds === undefined) return '–';
  const s = Math.max(0, Math.floor(seconds));
  return `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
}
function ascentOf(record) { return (record.ladder || {}).ascent_type || SIM_ASCENT[record.ladder_type] || null; }
function ascentLabel(record) { const a = ascentOf(record); return a ? t(`asc.${a}`) : record.ladder_type; }
function groupNames(keys) { return keys.map(k => t(`group.${k}`)).join(', '); }

// Short, translated reason built from the structured result instead of the robot's English sentence.
function reasonText(record) {
  const items = record.checklist || {};
  const defects = GROUP_KEYS.filter(k => items[k] === 'DEFECT');
  const unseen = GROUP_KEYS.filter(k => items[k] === 'UNKNOWN');
  const sticker = (record.ladder || {}).sticker_status || record.label_status;
  const parts = [];
  if (defects.length) parts.push(t('reason.defect', {x: groupNames(defects)}));
  if (sticker === 'OVERDUE') parts.push(t('reason.sticker'));
  if (record.status === 'MANUAL_REVIEW' && unseen.length) parts.push(t('reason.unseen', {x: groupNames(unseen)}));
  if (!parts.length && record.status === 'AUTOMATED_PASS') parts.push(t('reason.clear'));
  return parts.length ? parts.join(' · ') : record.reason;
}

async function getJSON(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${url}: HTTP ${response.status}`);
  return response.json();
}
function fill(node, ...children) { node.replaceChildren(...children.flat().filter(c => c !== null && c !== undefined)); }

function reportUrl(site, record) {
  return `/api/sites/${encodeURIComponent(site)}/ladders/${record.asset_id}/report.pdf?mission=${encodeURIComponent(record.mission_id)}`;
}

// Ladder detail dialog: nesseler checklist, evidence photos, report link.
function openLadder(dialog, record, checklist, site) {
  const items = record.checklist || {};
  const photoIndex = Object.fromEntries((record.photos || []).map((p, i) => [p.id, `F${i + 1}`]));
  const de = LANG === 'de';
  const rows = [];
  for (const group of checklist) {
    rows.push(el('tr', {}, el('td', {class: 'grp'}, de ? group.de : group.en), el('td', {class: 'grp'}, resultPill(items[group.key])), el('td', {class: 'grp'}, '')));
    for (const item of group.items) {
      const refs = ((record.item_photos || {})[item.key] || []).map(id => photoIndex[id]).filter(Boolean).join(', ');
      const detail = (record.criterion_details || {})[item.key];
      const old = (record.retained_findings || {})[item.key];
      const prior = old ? (old.photos || []).map(p => el('a', {href: `/media/${encodeURIComponent(old.mission_id)}/${encodeURIComponent(p.photo_id)}.jpg`, target: '_blank'}, `${de ? 'Früher' : 'Prior'} ${old.mission_id} `)) : [];
      const explanation = detail ? el('div', {class: 'sub'}, `${detail.reason} · ${de ? 'Modellwert (unkalibriert)' : 'Model score (uncalibrated)'}: ${detail.confidence.toFixed(2)}`) : null;
      rows.push(el('tr', {}, el('td', {style: 'padding-left:22px'}, de ? item.de : item.en, explanation), el('td', {}, resultPill(items[item.key])), el('td', {class: 'mono'}, refs, ...prior)));
    }
  }
  const ladder = record.ladder || {};
  fill(dialog, el('div', {class: 'dlg'},
    el('div', {class: 'dlg-head'},
      el('div', {}, el('h2', {style: 'font-size:22px'}, `${record.asset_id} · ${ascentLabel(record)}`),
        el('span', {class: 'sub mono'}, `${record.mission_id} · ${record.position[0].toFixed(1)} / ${record.position[1].toFixed(1)} m`)),
      el('div', {style: 'display:flex;gap:8px;align-items:center;flex-wrap:wrap'}, statusPill(record.status),
        el('a', {class: 'btn primary', href: reportUrl(site, record), target: '_blank'}, `${t('report')} (PDF)`),
        el('button', {onclick: () => dialog.close()}, t('close')))),
    el('div', {class: `reason s-${record.status}`}, reasonText(record)),
    el('div', {class: 'sub'}, `${t('sticker')}: ${ladder.sticker_status || t('not_read')}${ladder.sticker_due ? ` · ${t('due')} ${ladder.sticker_due}` : ''}`),
    record.assessment_source ? el('div', {class:'sub'}, `${record.assessment_source} · 360°: ${(record.view_coverage || {}).accepted_cells || 0}/24 (${de ? 'Blickwinkel, keine Sicherheitsfreigabe' : 'view sampling, not safety certification'})`) : null,
    el('div', {class: 'photos'}, (record.photos || []).map((p, i) => el('figure', {},
      el('img', {src: `/media/${record.mission_id}/${p.id}.jpg`, alt: `F${i + 1} ${record.asset_id}`}),
      el('figcaption', {}, `F${i + 1}`)))),
    el('div', {class: 'table-box'}, el('table', {class: 'checklist'},
      el('thead', {}, el('tr', {}, el('th', {}, t('item')), el('th', {}, t('result')), el('th', {}, t('photo')))),
      el('tbody', {}, rows))),
    el('p', {class: 'sub', style: 'margin:0'}, t('person_only'))));
  dialog.showModal();
}
