// Small DOM helpers shared by both pages. All text goes through textContent (no HTML injection).
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

const STATUS_LABEL = {FLAGGED: 'FLAGGED', MANUAL_REVIEW: 'MANUAL REVIEW', AUTOMATED_PASS: 'AUTOMATED PASS', INSPECTING: 'INSPECTING'};
const RESULT_LABEL = {OK: 'No defect seen', DEFECT: 'Defect', UNKNOWN: 'Not visible', NOT_CHECKED: 'Person checks', N_A: 'Does not apply'};
const ASCENT_EN = {Anlegeleiter: 'Leaning ladder', Stehleiter: 'Stepladder (A-frame)', Mehrzweckleiter: 'Multi-purpose ladder',
  Schiebeleiter: 'Extension ladder', Podestleitern: 'Platform ladder', Seilzugleiter: 'Rope-operated ladder',
  Steckleiter: 'Sectional ladder', Tritt: 'Step stool', Sonstige: 'Other'};

function statusPill(status) { return el('span', {class: `pill s-${status}`}, STATUS_LABEL[status] || status); }
function resultPill(result) { return el('span', {class: `pill r-${result}`}, RESULT_LABEL[result] || result || '–'); }
function mmss(seconds) {
  if (seconds === null || seconds === undefined) return '–';
  const s = Math.max(0, Math.floor(seconds));
  return `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
}
function ascentLabel(record) {
  const a = (record.ladder || {}).ascent_type;
  return a ? `${ASCENT_EN[a] || a} (${a})` : record.ladder_type;
}
async function getJSON(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${url}: HTTP ${response.status}`);
  return response.json();
}
function fill(node, ...children) { node.replaceChildren(...children.flat().filter(c => c !== null && c !== undefined)); }

// Ladder detail dialog: nesseler checklist, evidence photos and PDF link.
function openLadder(dialog, record, checklist, mission) {
  const items = record.checklist || {};
  const photoIndex = Object.fromEntries((record.photos || []).map((p, i) => [p.id, `F${i + 1}`]));
  const rows = [];
  for (const group of checklist) {
    rows.push(el('tr', {}, el('td', {class: 'grp'}, `${group.de}`, el('span', {class: 'sub'}, `  ${group.en}`)),
      el('td', {class: 'grp'}, resultPill(items[group.key])), el('td', {class: 'grp'}, '')));
    for (const item of group.items) {
      const refs = ((record.item_photos || {})[item.key] || []).map(id => photoIndex[id]).filter(Boolean).join(', ');
      rows.push(el('tr', {}, el('td', {}, item.de, el('div', {class: 'sub'}, item.en + (item.mode === 'person' ? ' · needs a person' : ''))),
        el('td', {}, resultPill(items[item.key])), el('td', {}, refs)));
    }
  }
  const ladder = record.ladder || {};
  const reasonClass = record.status === 'FLAGGED' ? 's-FLAGGED' : record.status === 'MANUAL_REVIEW' ? 's-MANUAL_REVIEW' : 's-AUTOMATED_PASS';
  fill(dialog, el('div', {class: 'dlg'},
    el('div', {class: 'dlg-head'},
      el('div', {}, el('h2', {}, `${record.asset_id} · ${ascentLabel(record)}`),
        el('div', {class: 'sub'}, `Mission ${record.mission_id} · map (${record.position[0].toFixed(1)}, ${record.position[1].toFixed(1)}) m`)),
      el('div', {style: 'display:flex;gap:8px;align-items:center;flex-wrap:wrap'}, statusPill(record.status),
        el('a', {class: 'btn primary', href: `/api/missions/${mission}/ladders/${record.asset_id}/protocol.pdf`, target: '_blank'}, 'nesseler form PDF'),
        el('button', {onclick: () => dialog.close()}, 'Close'))),
    el('div', {class: `reason ${reasonClass}`}, el('strong', {}, 'Reason: '), record.reason,
      record.status === 'AUTOMATED_PASS' ? ' · Visual pre-screen only. Function and touch checks are still pending; not a release for use.' : ''),
    el('div', {class: 'kv'},
      el('div', {}, el('span', {}, 'Inspection sticker (Prüfplakette)'), el('strong', {}, `${ladder.sticker_status || 'not read'}${ladder.sticker_due ? ' · due ' + ladder.sticker_due : ''}`)),
      el('div', {}, el('span', {}, 'Inventory no. / manufacturer'), el('strong', {}, `${ladder.inventory_no || 'not read'} / ${ladder.manufacturer || 'not read'}`))),
    el('div', {class: 'photos'}, (record.photos || []).map((p, i) => el('figure', {},
      el('img', {src: `/media/${record.mission_id}/${p.id}.jpg`, alt: `Evidence photo F${i + 1} of ${record.asset_id}`}),
      el('figcaption', {}, `F${i + 1} · ${p.caption}`)))),
    el('div', {class: 'table-box'}, el('table', {class: 'checklist'},
      el('thead', {}, el('tr', {}, el('th', {}, 'nesseler checklist item'), el('th', {}, 'Result'), el('th', {}, 'Photo'))),
      el('tbody', {}, rows))),
    el('p', {class: 'sub'}, 'Only the qualified person (Sachkundiger) can set "Leiter i. O. und verwendungsfähig" and sign the form.')));
  dialog.showModal();
}
