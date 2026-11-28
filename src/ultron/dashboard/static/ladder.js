// Ladder report page: one ladder's nesseler sheet, one column per patrol, five columns per sheet.
const $ = id => document.getElementById(id);
const PER_SHEET = 5;
const Q = new URLSearchParams(location.search);
const P = {site: Q.get('site'), id: Q.get('id'), sheet: Q.get('sheet') ? Number(Q.get('sheet')) : null,
  checklist: [], ladders: {}, detail: null};

function go(params) {
  const q = new URLSearchParams({site: P.site, ...params});
  location.search = q.toString();
}

async function boot() {
  P.checklist = await getJSON('/api/checklist');
  const sites = await getJSON('/api/sites');
  if (!P.site) P.site = sites[0];
  $('site').textContent = P.site || '';
  fill($('site-select'), sites.map(s => el('option', {value: s, selected: s === P.site}, s)));
  $('site-select').onchange = e => { location.search = new URLSearchParams({site: e.target.value}).toString(); };
  if (!P.site) return render();
  P.ladders = await getJSON(`/api/sites/${encodeURIComponent(P.site)}/ladders`);
  if (!P.id) P.id = Object.keys(P.ladders).sort()[0];
  if (P.id) P.detail = await getJSON(`/api/sites/${encodeURIComponent(P.site)}/ladders/${encodeURIComponent(P.id)}`).catch(() => null);
  render();
}

function render() {
  fill($('ladder-list'), Object.entries(P.ladders).sort().map(([id, h]) => {
    const last = h[h.length - 1];
    return el('a', {href: `/ladder?${new URLSearchParams({site: P.site, id})}`, class: `ladder edge-${last.status}`,
      style: `text-decoration:none;color:var(--ink);${id === P.id ? 'outline:2px solid var(--ink)' : ''}`},
      el('div', {class: 'row'}, el('strong', {class: 'mono'}, id), statusPill(last.status)),
      el('span', {class: 'sub'}, `${t(`asc.${SIM_ASCENT[last.ladder_type] || 'Sonstige'}`)} · ${h.length}×`));
  }));
  const d = P.detail;
  if (!d) { fill($('report'), el('p', {class: 'empty'}, t('pick_ladder'))); return; }
  const sheets = d.sheets;
  const sheetNo = P.sheet && P.sheet <= sheets.length ? P.sheet : sheets.length;
  const sheet = sheets[sheetNo - 1];
  const cols = d.inspections.slice(sheet.first - 1, sheet.last);
  const latest = cols[cols.length - 1];
  const ladder = latest.ladder || {};
  const sticker = ladder.sticker_status || latest.label_status;

  const head = el('section', {class: `card edge-${latest.status}`},
    el('div', {class: 'card-head'},
      el('div', {}, el('h1', {style: 'font-size:26px'}, `${d.asset_id} · ${ascentLabel(latest)}`),
        el('span', {class: 'sub'}, t('sheet', {n: sheetNo, a: sheet.first, b: sheet.first + PER_SHEET - 1}))),
      el('div', {style: 'display:flex;gap:8px;flex-wrap:wrap;align-items:center'},
        sheets.length > 1 ? el('select', {'aria-label': 'Sheet', onchange: e => go({id: d.asset_id, sheet: e.target.value})},
          sheets.map(s => el('option', {value: s.sheet, selected: s.sheet === sheetNo}, t('sheet', {n: s.sheet, a: s.first, b: s.first + PER_SHEET - 1})))) : null,
        el('a', {class: 'btn primary', href: sheet.url, target: '_blank'}, `${t('report')} (PDF)`))),
    el('div', {class: `reason s-${latest.status}`}, el('strong', {}, `${t('latest')} (${latest.mission_id}): `), reasonText(latest)),
    el('div', {class: 'kv', style: 'grid-template-columns:repeat(auto-fit,minmax(170px,1fr))'},
      [[t('f.ascent'), ascentLabel(latest)],
       [t('f.sticker'), sticker ? `${sticker}${ladder.sticker_due ? ' · ' + ladder.sticker_due : ''}` : t('not_read')],
       [t('f.inventory'), ladder.inventory_no || t('not_read')],
       [t('f.position'), `${latest.position[0].toFixed(1)} / ${latest.position[1].toFixed(1)} m`],
       [t('f.patrols'), `${cols.length} / ${PER_SHEET}`]].map(([k, v]) => el('div', {}, el('span', {}, k), el('strong', {}, v)))));

  const de = LANG === 'de';
  const cell = (rec, key) => el('td', {style: 'text-align:center'}, rec ? resultPill((rec.checklist || {})[key]) : '');
  const padded = [...cols, ...Array(PER_SHEET - cols.length).fill(null)];
  const rows = [];
  for (const g of P.checklist) {
    rows.push(el('tr', {}, el('td', {class: 'grp'}, de ? g.de : `${g.de.split('.')[0]}. ${g.en}`), padded.map(r => cell(r, g.key))));
    for (const it of g.items) rows.push(el('tr', {}, el('td', {style: 'padding-left:22px'}, de ? it.de : it.en), padded.map(r => cell(r, it.key))));
  }
  rows.push(el('tr', {}, el('td', {class: 'grp'}, de ? '8. Kontrollergebnis' : '8. Result'),
    padded.map(r => el('td', {style: 'text-align:center'}, r && r.status === 'FLAGGED' ? el('span', {class: 'pill r-DEFECT'}, t('proposal')) : ''))));
  const table = el('section', {class: 'card'},
    el('div', {class: 'card-head'}, el('h2', {}, 'Kontrollblatt Leitern und Tritte'), el('span', {class: 'sub'}, t('release'))),
    el('div', {class: 'table-box'}, el('table', {class: 'checklist', style: 'min-width:720px'},
      el('thead', {}, el('tr', {}, el('th', {}, t('item')),
        padded.map((r, i) => el('th', {style: 'text-align:center'}, `${sheet.first + i}${r ? ' · ' + r.mission_id : ''}`)))),
      el('tbody', {}, rows))));

  const photos = el('section', {class: 'card'}, el('h2', {}, `${t('photos.title')} · ${latest.mission_id}`),
    el('div', {class: 'photos'}, (latest.photos || []).map((p, i) => el('figure', {},
      el('img', {src: `/media/${latest.mission_id}/${p.id}.jpg`, alt: `F${i + 1} ${d.asset_id}`, loading: 'lazy'}),
      el('figcaption', {}, `F${i + 1}`)))));
  fill($('report'), head, table, photos);
}

initChrome(render);
boot();
