"""Inspectable local artifacts with synthetic provenance."""
from html import escape
from pathlib import Path


def evidence_svg(event, destination):
    """A synthetic evidence card, NOT a camera image or defect reconstruction."""
    title = escape(event["track_id"])
    checks = escape(str(event["checks"]))
    Path(destination).write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="900" height="180">
<rect width="900" height="180" fill="#102334"/>
<g fill="white" font-family="sans-serif"><text x="24" y="40" font-size="22">SYNTHETIC observation: {title}</text>
<text x="24" y="80">Step {event['step']} — scripted findings, no camera or ML inference</text>
<text x="24" y="120" font-size="12">{checks}</text></g></svg>''', encoding="utf-8")


def write_report(protocol, destination):
    rows = []
    for record in protocol["records"]:
        reasons = "; ".join(record["assessment"]["reasons"]) or "All demo checks passed; human review pending"
        links = " ".join(f'<a href="{escape(path, quote=True)}">Observation {i+1}</a>'
                         for i, path in enumerate(record["evidence"]))
        rows.append(f"<tr><td>{escape(record['track_id'])}</td><td>{escape(record['assessment']['status'])}</td>"
                    f"<td>{escape(reasons)}</td><td>{links}</td></tr>")
    metrics = escape(str(protocol["simulation_metrics"]))
    Path(destination).write_text('''<!doctype html><html lang="en"><meta charset="utf-8">
<title>Site Safety Patrol — simulation report</title>
<style>body{font:16px system-ui;max-width:1200px;margin:40px auto;padding:20px;background:#f3f6fa;color:#14283c}
table{border-collapse:collapse;width:100%;background:white}td,th{padding:16px;border:1px solid #ccd5df;text-align:left}a{color:#1659a8}</style>
<h1>Site Safety Patrol</h1><p><strong>SYNTHETIC DEMO — human sign-off required for every ladder.</strong></p>
<p>No ladder is certified safe. Findings and SVG evidence are scripted. Metrics use simulator ground truth.</p>
''' + f"<p>Run: {escape(protocol['run_id'])} · Inspection date: {escape(protocol['inspection_date'])}</p>"
        + f"<p>{metrics}</p><table><tr><th>Track</th><th>Result</th><th>Reasons</th><th>Synthetic evidence</th></tr>"
        + "".join(rows) + "</table></html>", encoding="utf-8")
