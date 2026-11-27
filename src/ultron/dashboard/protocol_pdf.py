"""Fill nesseler's ladder form (kontrollblatt_checkliste_leitern_tritte.pdf) from inspection records.

The form ships as a PDF with empty, unnamed widgets and no working form fields, so this
module draws onto the original pages at the measured widget positions and removes the
widgets. One ladder = the 3 form pages + one photo/evidence page. The robot never ticks
"Leiter i. O. und verwendungsfähig": only the qualified person (Sachkundiger) does.
"""
import io
import os
from datetime import datetime
from pathlib import Path

from pypdf import PdfReader, PdfWriter
from pypdf.generic import NameObject
from reportlab.lib.colors import HexColor
from reportlab.lib.pagesizes import A4
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas

from .checklist import GROUPS

DEFAULT_FORM = Path('data/nesseler/kontrollblatt_checkliste_leitern_tritte.pdf')
FORM_URL = ('https://raw.githubusercontent.com/ctrl-rwth/ctrl-hackathon-2026/main/site-safety-patrol/'
            'nesseler-data/kontrollblatt_checkliste_leitern_tritte.pdf')


def form_path():
    return Path(os.environ.get('ULTRON_NESSELER_FORM', DEFAULT_FORM))


# Measured from the form's widget rectangles (PDF points, origin bottom-left).
P1_TEXT = {'inventory': (267, 460, 199), 'location': (267, 445, 200), 'rungs': (267, 315, 200),
           'length': (268, 300, 200), 'manufacturer': (268, 285, 199), 'article': (267, 271, 201),
           'bought': (267, 256, 200), 'retired': (267, 242, 200), 'inspector': (267, 220, 200),
           'next': (181, 198, 86), 'sticker': (352, 198, 115)}
P1_CHOICES = {'Anlegeleiter': (345, 430), 'Mehrzweckleiter': (452, 430), 'Schiebeleiter': (345, 416),
              'Podestleitern': (452, 416), 'Seilzugleiter': (345, 402), 'Steckleiter': (452, 402),
              'Stehleiter': (345, 387), 'Tritt': (452, 387), 'Sonstige': (345, 372),
              'Aluminium': (345, 358), 'Stahl': (452, 358), 'Kunststoff': (345, 344), 'Edelstahl': (452, 344),
              'Holz': (345, 329)}
P2_COLS = [390, 465, 538, 611, 687]
P2_ROWS = {'holme': 471, 'holme.verformung': 457, 'holme.beschaedigung': 443, 'holme.kanten': 428,
           'holme.abnutzung': 414, 'holme.schutz': 399, 'sprossen': 384, 'sprossen.verformung': 369,
           'sprossen.beschaedigung': 356, 'sprossen.kanten': 339, 'sprossen.verbindung': 320,
           'sprossen.abnutzung': 297, 'spreiz': 282, 'spreiz.vollstaendigkeit': 269, 'spreiz.funktion': 254,
           'spreiz.beschaedigung': 239, 'beschlag': 224, 'beschlag.beschaedigung': 210,
           'beschlag.vollstaendigkeit': 195, 'beschlag.funktion': 181, 'beschlag.abnutzung': 167,
           'beschlag.schmierung': 152, 'fuesse': 138, 'fuesse.vollstaendigkeit': 122, 'fuesse.abnutzung': 107,
           'fuesse.funktion': 94}
P3_COLS = [393, 464, 540, 617, 685]
P3_ROWS = {'zubehoer': 421, 'zubehoer.vollstaendigkeit': 399, 'kennzeichnung': 386,
           'kennzeichnung.betriebsanleitung': 370, 'result': 356, 'io': 342, 'repair': 327, 'remove': 312,
           'remarks': 298}
P3_TEXT_X = [365, 438, 512, 585, 658]
P3_NEXT_Y, P3_DATE_Y = 284, 269
BOX = 15

INK = HexColor('#17191c')
GREEN, RED, AMBER, SLATE, GREY = (HexColor(c) for c in ('#1b5e20', '#b71c1c', '#8a5300', '#37474f', '#8a9099'))

RESULT_DE = {'OK': 'Sichtprüfung ohne Befund', 'DEFECT': 'Mangel festgestellt', 'UNKNOWN': 'Nicht sichtbar / nicht eingeordnet',
             'NOT_CHECKED': 'Prüfung durch Sachkundigen', 'N_A': 'Entfällt'}
VERDICT_DE = {'FLAGGED': 'MANGEL – Vorschlag: Leiter sofort aussondern',
              'MANUAL_REVIEW': 'PRÜFUNG VOR ORT ERFORDERLICH – Pflichtpunkte nicht sichtbar',
              'AUTOMATED_PASS': 'Sichtvorprüfung ohne Befund – Funktions- und Tastprüfung ausstehend'}
STICKER_DE = {'VALID': 'gültig', 'OVERDUE': 'ABGELAUFEN', 'UNREADABLE': 'nicht lesbar'}


class FormMissing(FileNotFoundError):
    pass


def de_date(iso):
    try:
        return datetime.fromisoformat(iso).strftime('%d.%m.%Y')
    except (TypeError, ValueError):
        return ''


def month_year(value):
    if not value:
        return ''
    parts = value.split('-')
    return f'{parts[1]}/{parts[0]}' if len(parts) >= 2 else value


def fit(c, text, x, y, width, size=8, font='Helvetica'):
    text = text or ''
    while size > 5 and c.stringWidth(text, font, size) > width - 4:
        size -= 0.5
    while text and c.stringWidth(text, font, size) > width - 4:
        text = text[:-2] + '…'
    c.setFont(font, size)
    c.drawString(x + 2, y, text)


def mark(c, result, x, cy, photo=None):
    """Draw one result symbol inside a form box whose lower-left corner is (x, cy - BOX/2)."""
    y0 = cy - BOX / 2
    c.setLineWidth(0.6)
    c.setStrokeColor(GREY)
    c.rect(x, y0, BOX, BOX)
    c.setLineWidth(1.4)
    if result == 'OK':
        c.setStrokeColor(GREEN)
        path = c.beginPath()
        path.moveTo(x + 3, cy)
        path.lineTo(x + 6.5, y0 + 3.5)
        path.lineTo(x + 12.5, y0 + 12.5)
        c.drawPath(path)
    elif result == 'DEFECT':
        c.setStrokeColor(RED)
        c.setLineWidth(1.8)
        c.line(x + 3, y0 + 3, x + 12, y0 + 12)
        c.line(x + 3, y0 + 12, x + 12, y0 + 3)
    elif result in ('UNKNOWN', 'NOT_CHECKED'):
        c.setFillColor(AMBER if result == 'UNKNOWN' else SLATE)
        c.setFont('Helvetica-Bold', 10 if result == 'UNKNOWN' else 9)
        c.drawCentredString(x + BOX / 2, y0 + 4, '?' if result == 'UNKNOWN' else 'P')
    elif result == 'N_A':
        c.setStrokeColor(SLATE)
        c.line(x + 4, cy, x + 11, cy)
    if photo:
        c.setFillColor(INK)
        c.setFont('Helvetica', 5.5)
        c.drawString(x + BOX + 2, cy - 2, photo)
    c.setFillColor(INK)
    c.setStrokeColor(INK)


def legend(c, x, y):
    c.setFont('Helvetica-Bold', 7)
    c.setFillColor(INK)
    c.drawString(x, y + 14, 'Legende (Roboter-Sichtvorprüfung, VP):')
    entries = [('OK', 'ohne Befund im Foto'), ('DEFECT', 'Mangel'), ('UNKNOWN', 'nicht sichtbar'),
               ('NOT_CHECKED', 'durch Sachkundigen (Funktion/Tasten)'), ('N_A', 'entfällt')]
    cx = x
    for result, text in entries:
        mark(c, result, cx, y + 3)
        c.setFont('Helvetica', 7)
        c.drawString(cx + BOX + 3, y + 1, text)
        cx += BOX + 10 + c.stringWidth(text, 'Helvetica', 7)
    c.setFont('Helvetica', 7)
    c.drawString(x, y - 12, 'F1, F2 … = Fotonachweis im Anhang. "Leiter i. O. und verwendungsfähig" setzt nur der Sachkundige.')


def overlay_pages(mission, history, photo_refs):
    current = history[-1]
    columns = history[-5:]
    ladder = current.get('ladder') or {}
    buffers = []

    # Page 1: ladder data
    buf = io.BytesIO()
    c = canvas.Canvas(buf, pagesize=(595, 842))
    location = f"{mission['site_id']} · Karte ({current['position'][0]:.1f} / {current['position'][1]:.1f}) m"
    inventory = ladder.get('inventory_no') or f"nicht gelesen – Roboter-ID {current['asset_id']}"
    sticker = ladder.get('sticker_status') or current.get('label_status')
    fields = {'inventory': inventory, 'location': location,
              'rungs': str(ladder['rungs']) if ladder.get('rungs') else '',
              'length': f"{ladder['length_m']:.2f} m" if ladder.get('length_m') else '',
              'manufacturer': ladder.get('manufacturer') or '', 'article': ladder.get('article_no') or '',
              'next': month_year(ladder.get('sticker_due')),
              'sticker': STICKER_DE.get(sticker, '') if sticker else ''}
    for key, text in fields.items():
        x, cy, w = P1_TEXT[key]
        fit(c, text, x, cy - 3, w)
    for choice in (ladder.get('ascent_type'), ladder.get('material')):
        if choice in P1_CHOICES:
            x, cy = P1_CHOICES[choice]
            c.setFillColor(INK)
            c.circle(x + BOX / 2, cy, 4.2, stroke=0, fill=1)
    for x, cy in P1_CHOICES.values():
        c.setStrokeColor(GREY)
        c.setLineWidth(0.6)
        c.circle(x + BOX / 2, cy, 6.5)
    c.setFont('Helvetica', 7)
    c.setFillColor(SLATE)
    c.drawString(104, 140, 'Felder ohne Eintrag wurden vom Roboter nicht gelesen und sind vom Sachkundigen zu ergänzen.')
    if mission.get('synthetic'):
        c.drawString(104, 130, 'Datenquelle: Simulation (synthetische Daten), keine reale Begehung.')
    c.save()
    buffers.append(buf)

    def columns_page(size, cols, rows, text_rows, all_rows):
        b = io.BytesIO()
        cv = canvas.Canvas(b, pagesize=size)
        for x in cols:  # the removed widgets drew these boxes; redraw every one empty first
            for cy in all_rows.values():
                mark(cv, None, x, cy)
        for col, record in enumerate(columns):
            items = record.get('checklist') or {}
            is_current = record is current
            for key, cy in rows.items():
                if key in items:
                    ref = photo_refs.get(key) if is_current else None
                    mark(cv, items[key].value if hasattr(items[key], 'value') else items[key], cols[col], cy, ref)
            text_rows(cv, col, record)
        return cv, b

    # Page 2: sections 1-5
    c2, b2 = columns_page((842, 595), P2_COLS, P2_ROWS, lambda *a: None, P2_ROWS)
    legend(c2, 160, 45)
    c2.save()
    buffers.append(b2)

    # Page 3: sections 6-8, result, remarks, dates
    def page3_text(cv, col, record):
        x = P3_COLS[col]
        if record['status'] == 'FLAGGED':
            mark(cv, 'DEFECT', x, P3_ROWS['remove'])
            cv.setFont('Helvetica', 5.5)
            cv.drawString(x + BOX + 2, P3_ROWS['remove'] - 2, 'Vorschlag')
        mark(cv, 'NOT_CHECKED', x, P3_ROWS['remarks'], 's. Anhang' if record is current else None)
        lad = record.get('ladder') or {}
        fit(cv, f"{month_year(lad.get('sticker_due'))} Plak." if lad.get('sticker_due') else '',
            P3_TEXT_X[col], P3_NEXT_Y - 3, 72, size=7)
        fit(cv, f"{de_date(record.get('mission_started') or mission['started'])} VP", P3_TEXT_X[col], P3_DATE_Y - 3, 72, size=7)

    c3, b3 = columns_page((842, 595), P3_COLS, {k: v for k, v in P3_ROWS.items() if '.' in k or k in ('zubehoer', 'kennzeichnung')},
                          page3_text, P3_ROWS)
    legend(c3, 160, 225)
    c3.setFont('Helvetica-Bold', 8)
    c3.drawString(160, 185, f"Aktuelle Vorprüfung ({current['mission_id']}, Spalte {len(columns)}): {VERDICT_DE[current['status']]}")
    c3.setFont('Helvetica', 8)
    c3.drawString(160, 172, 'Kontrollergebnis und Freigabe: ausschließlich durch den Sachkundigen.')
    c3.drawString(160, 140, 'Sachkundiger: ______________________________    Datum: ____________    Unterschrift: ______________________')
    c3.save()
    buffers.append(b3)
    return buffers


def evidence_page(c, mission, record, photo_refs, media_dir):
    width, height = A4
    c.setFillColor(INK)
    c.setFont('Helvetica-Bold', 14)
    c.drawString(40, height - 50, f"Anhang: Befund und Fotonachweis – {record['asset_id']}")
    c.setFont('Helvetica', 9)
    ladder = record.get('ladder') or {}
    lines = [f"Baustelle {mission['site_id']} · Mission {record['mission_id']} · {de_date(mission['started'])}",
             f"Aufstiegsart: {ladder.get('ascent_type') or record['ladder_type']} · Position ({record['position'][0]:.1f} / {record['position'][1]:.1f}) m",
             f"Ergebnis der Vorprüfung: {VERDICT_DE[record['status']]}",
             f"Begründung (System): {record['reason']}"]
    for i, line in enumerate(lines):
        fit(c, line, 38, height - 72 - i * 13, width - 76, size=9)
    y = height - 135
    c.setFont('Helvetica-Bold', 8)
    c.drawString(40, y, 'Prüfpunkt')
    c.drawString(300, y, 'Ergebnis')
    c.drawString(470, y, 'Foto')
    items = record.get('checklist') or {}
    for group, title, _, _, _, entries in GROUPS:
        y -= 12
        c.setFont('Helvetica-Bold', 7.5)
        c.drawString(40, y, title)
        c.setFont('Helvetica', 7.5)
        c.drawString(300, y, RESULT_DE.get(items.get(group, ''), '–'))
        for key, label, *_ in entries:
            full = f'{group}.{key}'
            y -= 10
            c.drawString(52, y, label[:60])
            result = items.get(full, '')
            c.setFillColor(RED if result == 'DEFECT' else AMBER if result == 'UNKNOWN' else INK)
            c.drawString(300, y, RESULT_DE.get(result, '–'))
            c.setFillColor(INK)
            c.drawString(470, y, photo_refs.get(full, ''))
    photos = record.get('photos') or []
    slot_w, slot_h, top = 250, 141, y - 20
    for index, photo in enumerate(photos[:4]):
        px = 40 + (index % 2) * (slot_w + 15)
        py = top - (index // 2 + 1) * (slot_h + 22)
        path = Path(media_dir) / record['mission_id'] / f"{photo['id']}.jpg"
        try:
            c.drawImage(ImageReader(str(path)), px, py, slot_w, slot_h, preserveAspectRatio=True, anchor='sw')
        except Exception:  # missing or unreadable file: say so on the page, never drop the record
            c.rect(px, py, slot_w, slot_h)
            c.drawString(px + 6, py + slot_h / 2, 'Foto nicht verfügbar oder nicht lesbar')
        c.setFont('Helvetica', 7.5)
        c.drawString(px, py - 10, f"F{index + 1}: {photo.get('caption', '')}"[:70])
    c.setFont('Helvetica', 7)
    c.setFillColor(SLATE)
    c.drawString(40, 40, 'Automatische Sichtvorprüfung, keine Freigabe. Funktions- und Tastprüfung sowie Unterschrift durch den Sachkundigen.')
    if mission.get('synthetic'):
        c.drawString(40, 30, 'Datenquelle: Simulation (synthetische Bilder und Befunde).')
    c.setFillColor(INK)
    c.showPage()


def photo_refs_for(record):
    index = {p['id']: f'F{i + 1}' for i, p in enumerate(record.get('photos') or [])}
    return {key: ','.join(index[r] for r in refs if r in index) for key, refs in (record.get('item_photos') or {}).items()}


def append_ladder(writer, template, mission, history, media_dir):
    reader = PdfReader(str(template))
    current = history[-1]
    refs = photo_refs_for(current)
    overlays = overlay_pages(mission, history, refs)
    for page, overlay in zip(reader.pages, overlays):
        target = writer.add_page(page)  # pypdf 7: pages must belong to the writer before merging
        if '/Annots' in target:
            del target[NameObject('/Annots')]
        target.merge_page(PdfReader(io.BytesIO(overlay.getvalue())).pages[0])
    buf = io.BytesIO()
    c = canvas.Canvas(buf, pagesize=A4)
    evidence_page(c, mission, current, refs, media_dir)
    c.save()
    writer.add_page(PdfReader(io.BytesIO(buf.getvalue())).pages[0])


def cover_page(mission, inspections, alerts):
    buf = io.BytesIO()
    c = canvas.Canvas(buf, pagesize=A4)
    width, height = A4
    c.setFont('Helvetica-Bold', 16)
    c.drawString(40, height - 55, 'Prüfprotokoll Leitern und Tritte – Roboter-Sichtvorprüfung')
    c.setFont('Helvetica', 9)
    counts = {s: sum(1 for i in inspections if i['status'] == s) for s in VERDICT_DE}
    info = [f"Baustelle: {mission['site_id']}    Mission: {mission['id']}    Datum: {de_date(mission['started'])}",
            f"Roboter: Unitree B2 (ULTRON)    Missionsstatus: {mission['status']}",
            f"Leitern bewertet: {len(inspections)}  ·  Mangel: {counts['FLAGGED']}  ·  Prüfung vor Ort: {counts['MANUAL_REVIEW']}"
            f"  ·  ohne Befund: {counts['AUTOMATED_PASS']}",
            'Grundlage: Kontrollblatt / Checkliste zur Überprüfung von Leitern und Tritten (BGI 694), je Leiter 3 Seiten + Fotonachweis.']
    if mission.get('summary', {}).get('coverage_note'):
        info.append(f"Abdeckung: {mission['summary']['coverage_note']}")
    for i, line in enumerate(info):
        fit(c, line, 38, height - 80 - i * 13, width - 76, size=9)
    y = height - 160
    c.setFont('Helvetica-Bold', 9)
    for x, title in ((40, 'Leiter'), (110, 'Aufstiegsart'), (200, 'Ergebnis der Vorprüfung'), (420, 'Begründung')):
        c.drawString(x, y, title)
    c.setFont('Helvetica', 8)
    for record in inspections:
        y -= 14
        c.setFillColor(RED if record['status'] == 'FLAGGED' else AMBER if record['status'] == 'MANUAL_REVIEW' else INK)
        c.drawString(40, y, record['asset_id'])
        c.setFillColor(INK)
        c.drawString(110, y, (record.get('ladder') or {}).get('ascent_type') or record['ladder_type'])
        fit(c, VERDICT_DE[record['status']], 198, y, 220, size=7.5)
        fit(c, record['reason'], 418, y, 140, size=7.5)
    y -= 30
    c.setFont('Helvetica-Bold', 9)
    c.drawString(40, y, 'Benachrichtigungen während der Mission')
    c.setFont('Helvetica', 8)
    for alert in alerts[:12]:
        y -= 13
        who = ', '.join(r['title'] for r in alert.get('recipients', []))
        state = f"quittiert von {alert['acked_by']}" if alert.get('acked_at') else 'nicht quittiert'
        fit(c, f"{alert['title']} → {who} ({state})", 38, y, width - 76, size=8)
    c.setFont('Helvetica', 8)
    c.drawString(40, 110, 'Diese Vorprüfung ersetzt nicht die Prüfung durch den Sachkundigen. Kein Eintrag gilt als Freigabe.')
    c.drawString(40, 80, 'Sachkundiger: ______________________________    Datum: ____________    Unterschrift: ______________________')
    if mission.get('synthetic'):
        c.setFillColor(SLATE)
        c.drawString(40, 50, 'Datenquelle: Simulation (synthetische Daten). Nicht als reale Begehung verwenden.')
    c.save()
    return PdfReader(io.BytesIO(buf.getvalue())).pages[0]


def require_form(template=None):
    template = Path(template or form_path())
    if not template.exists():
        raise FormMissing(f'nesseler form not found at {template}. Run: python -m ultron.dashboard fetch-form')
    return template


def ladder_pdf(mission, history, media_dir, template=None):
    writer = PdfWriter()
    append_ladder(writer, require_form(template), mission, history, media_dir)
    out = io.BytesIO()
    writer.write(out)
    return out.getvalue()


def mission_pdf(mission, inspections, histories, alerts, media_dir, template=None):
    template = require_form(template)
    writer = PdfWriter()
    writer.add_page(cover_page(mission, inspections, alerts))
    for record in inspections:
        append_ladder(writer, template, mission, histories[record['asset_id']], media_dir)
    out = io.BytesIO()
    writer.write(out)
    return out.getvalue()
