"""Run and stream a patrol, or replay its saved event log."""
import argparse
from datetime import date, datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import time
import uuid

from .policy import assess
from .reporting import evidence_svg, write_report
from .simulation import observe


def run(scenario_path, output, inspection_date, delay=0):
    raw = Path(scenario_path).read_bytes()
    scenario = json.loads(raw)
    ids = [item["id"] for item in scenario["ladders"]]
    if len(ids) != len(set(ids)) or not all(isinstance(i, str) and re.fullmatch(r"[A-Za-z0-9_-]+", i) for i in ids):
        raise ValueError("Ladder IDs must be unique path-safe strings")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "evidence").mkdir()
    run_id = str(uuid.uuid4())
    records = {}
    with (output / "events.jsonl").open("w", encoding="utf-8") as log:
        for event in observe(scenario):
            event.update(run_id=run_id, synthetic=True,
                         timestamp=datetime.now(timezone.utc).isoformat())
            if event["type"] == "observation":
                track = event["track_id"]
                evidence = f"evidence/{track}-{event['step']}.svg"
                evidence_svg(event, output / evidence)
                event["evidence"] = [evidence]
                assessment = assess(event, inspection_date)
                record = records.setdefault(track, {"track_id": track, "position": event["position"],
                    "label": event["label"], "evidence": [], "assessment": assessment})
                record["evidence"].append(evidence)
                # Conservative aggregation: a later good view never clears an earlier defect.
                rank = {"candidate_for_review": 0, "needs_review": 1, "unsafe": 2}
                all_reasons = sorted(set(record["assessment"]["reasons"] + assessment["reasons"]))
                if rank[assessment["status"]] > rank[record["assessment"]["status"]]:
                    record["assessment"] = assessment
                record["assessment"]["reasons"] = all_reasons
                event["assessment"] = assessment
            line = json.dumps(event, allow_nan=False)
            log.write(line + "\n")
            log.flush()
            print(line, flush=True)
            if delay:
                time.sleep(delay)
    missing = sorted(set(ids) - records.keys())
    protocol = {"schema_version": "0.1", "run_id": run_id, "synthetic": True,
                "inspection_date": inspection_date.isoformat(),
                "protocol_basis": "DEMO ONLY; official nesseler fields unknown",
                "scenario_sha256": hashlib.sha256(raw).hexdigest(),
                "records": list(records.values()),
                "simulation_metrics": {"expected_ladders": len(ids), "found_ladders": len(records),
                    "recall": len(records) / len(ids) if ids else None,
                    "missed_ground_truth_ids": missing, "safe_declarations": 0}}
    (output / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n", encoding="utf-8")
    (output / "scenario.json").write_bytes(raw)
    write_report(protocol, output / "report.html")
    return protocol


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    demo = sub.add_parser("simulate")
    demo.add_argument("--scenario", type=Path, default=Path("scenarios/mock-site.json"))
    demo.add_argument("--out", type=Path, default=None)
    demo.add_argument("--inspection-date", type=date.fromisoformat, default=date.today())
    demo.add_argument("--delay", type=float, default=0, help="Seconds between streamed events")
    replay = sub.add_parser("replay")
    replay.add_argument("events", type=Path)
    args = parser.parse_args()
    if args.command == "replay":
        with args.events.open(encoding="utf-8") as events:
            for line in events:
                print(json.dumps(json.loads(line)), flush=True)
    else:
        if args.delay < 0:
            parser.error("--delay must be nonnegative")
        output = args.out or Path("runs") / (datetime.now().strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:8])
        run(args.scenario, output, args.inspection_date, args.delay)
        print(f"Report: {output / 'report.html'}")
