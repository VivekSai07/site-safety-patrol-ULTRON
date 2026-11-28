#!/usr/bin/env python3
"""Extract a zipped rosbag2 (sqlite3) and salvage what survives if the zip or .db3 is damaged.

Procedure (the original zip is only read, never modified):
1. Scan the zip for local-file (PK\\x03\\x04), central-directory and end-of-central-directory
   signatures. If the central directory is readable, extract normally (CRC checked).
2. Otherwise walk the local file headers and stream-inflate each entry until its data
   ends; a truncated last entry is written as a partial file.
3. For each .db3: run `PRAGMA integrity_check`. If it fails, rebuild a new
   `<name>_recovered.db3` with the sqlite3 CLI `.recover` (if found) or, failing that, by
   scanning every readable SQLite leaf page for topic and message records.
4. Count messages per topic and compare with the recording's metadata.yaml
   (from the zip, or --metadata).

Standard library only. Writes `<out>/recovery_report.json`.

    python3 ros2_ws/tools/recover_bag_zip.py bag.zip --out /data/bags [--metadata metadata.yaml]
"""
from __future__ import annotations

import argparse
import json
import shutil
import sqlite3
import struct
import subprocess
import sys
import zipfile
import zlib
from pathlib import Path

SIGNATURES = {"local_file": b"PK\x03\x04", "central_dir": b"PK\x01\x02", "eocd": b"PK\x05\x06",
              "zip64_eocd": b"PK\x06\x06", "data_descriptor": b"PK\x07\x08"}
CHUNK = 1 << 22


def scan_signatures(path, limit=50):
    """Offsets of zip record signatures (first `limit` each) and total counts. Chunked, low memory."""
    found = {k: [] for k in SIGNATURES}
    counts = {k: 0 for k in SIGNATURES}
    with open(path, "rb") as f:
        pos, tail = 0, b""
        while True:
            block = f.read(CHUNK)
            if not block:
                break
            buf = tail + block
            base = pos - len(tail)
            for name, sig in SIGNATURES.items():
                i = buf.find(sig)
                while i >= 0:
                    if i + 4 > len(tail):  # skip matches already counted in the previous chunk overlap
                        counts[name] += 1
                        if len(found[name]) < limit:
                            found[name].append(base + i)
                    i = buf.find(sig, i + 1)
            tail = buf[-3:]
            pos += len(block)
    return {"size": pos, "offsets": found, "counts": counts}


def _local_header(f, offset):
    f.seek(offset)
    head = f.read(30)
    if len(head) < 30 or head[:4] != SIGNATURES["local_file"]:
        return None
    _, flag, method, _, _, crc, csize, usize, nlen, elen = struct.unpack("<HHHHHIIIHH", head[4:30])
    name = f.read(nlen).decode("utf-8", errors="replace")
    extra = f.read(elen)
    # zip64 extra field (0x0001) may hold the real sizes
    i = 0
    while i + 4 <= len(extra):
        tag, size = struct.unpack_from("<HH", extra, i)
        if tag == 1:
            vals = [v for v in struct.unpack_from("<" + "Q" * (size // 8), extra, i + 4)]
            if usize == 0xFFFFFFFF and vals:
                usize = vals.pop(0)
            if csize == 0xFFFFFFFF and vals:
                csize = vals.pop(0)
        i += 4 + size
    return {"offset": offset, "name": name, "flag": flag, "method": method, "crc": crc,
            "compressed_size": csize, "uncompressed_size": usize, "data_offset": offset + 30 + nlen + elen}


def _safe_target(out, name):
    target = (out / name).resolve()
    if not str(target).startswith(str(out.resolve())):
        raise ValueError(f"unsafe path in zip: {name}")
    return target


def extract_with_central_directory(zip_path, out):
    entries = []
    with zipfile.ZipFile(zip_path) as z:
        for info in z.infolist():
            target = _safe_target(out, info.filename)
            if info.is_dir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            with z.open(info) as src, open(target, "wb") as dst:  # ZipExtFile verifies the CRC at EOF
                shutil.copyfileobj(src, dst, CHUNK)
            entries.append({"name": info.filename, "bytes": target.stat().st_size, "expected_bytes": info.file_size,
                            "complete": target.stat().st_size == info.file_size, "crc_checked": True})
    return entries


def extract_streaming(zip_path, out, local_offsets):
    """Inflate entries from their local headers; tolerate a truncated final entry."""
    entries = []
    with open(zip_path, "rb") as f:
        for off in local_offsets:
            h = _local_header(f, off)
            if not h or h["name"].endswith("/"):
                if h:
                    _safe_target(out, h["name"]).mkdir(parents=True, exist_ok=True)
                continue
            target = _safe_target(out, h["name"])
            target.parent.mkdir(parents=True, exist_ok=True)
            known = h["compressed_size"] not in (0, 0xFFFFFFFF) or not h["flag"] & 8
            f.seek(h["data_offset"])
            crc, written, ended = 0, 0, False
            inflater = zlib.decompressobj(-15) if h["method"] == 8 else None
            if h["method"] not in (0, 8) or (inflater is None and not known):
                entries.append({"name": h["name"], "skipped": f"method {h['method']} unsupported or size unknown"})
                continue
            remaining = h["compressed_size"] if known else None
            with open(target, "wb") as dst:
                while True:
                    n = CHUNK if remaining is None else min(CHUNK, remaining)
                    if n == 0:
                        ended = True
                        break
                    block = f.read(n)
                    if not block:
                        break
                    if remaining is not None:
                        remaining -= len(block)
                    try:
                        data = inflater.decompress(block) if inflater else block
                    except zlib.error:
                        break
                    dst.write(data)
                    crc, written = zlib.crc32(data, crc), written + len(data)
                    if inflater and inflater.eof:
                        ended = True
                        break
                if inflater and not inflater.eof:
                    tail = inflater.flush()
                    dst.write(tail)
                    crc, written = zlib.crc32(tail, crc), written + len(tail)
            expected = h["uncompressed_size"] if h["uncompressed_size"] not in (0, 0xFFFFFFFF) else None
            entries.append({"name": h["name"], "bytes": written, "expected_bytes": expected,
                            "stream_ended": ended, "complete": ended and (expected is None or expected == written),
                            "crc32": f"{crc:08x}", "crc_checked": bool(known and h["crc"] and h["crc"] == crc)})
    return entries


def sqlite_integrity(db):
    try:
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        res = con.execute("PRAGMA integrity_check").fetchone()[0]
        con.close()
        return res
    except sqlite3.DatabaseError as exc:
        return f"error: {exc}"


def topic_counts(db):
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    try:
        rows = con.execute("SELECT t.name, COUNT(m.id) FROM topics t LEFT JOIN messages m ON m.topic_id = t.id "
                           "GROUP BY t.id").fetchall()
    finally:
        con.close()
    return dict(rows)


def recover_with_cli(src, dst, sqlite3_cli):
    dump = subprocess.run([sqlite3_cli, str(src), ".recover"], capture_output=True, check=False)
    if dump.returncode != 0 and not dump.stdout:
        raise RuntimeError(dump.stderr.decode(errors="replace")[:500])
    load = subprocess.run([sqlite3_cli, str(dst)], input=dump.stdout, capture_output=True, check=False)
    if load.returncode != 0:
        raise RuntimeError(load.stderr.decode(errors="replace")[:500])


HUMBLE_SCHEMA = [
    "CREATE TABLE schema(schema_version INTEGER PRIMARY KEY,ros_distro TEXT NOT NULL)",
    "CREATE TABLE metadata(id INTEGER PRIMARY KEY,metadata_version INTEGER NOT NULL,metadata TEXT NOT NULL)",
    "CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT NOT NULL,type TEXT NOT NULL,"
    "serialization_format TEXT NOT NULL,offered_qos_profiles TEXT NOT NULL)",
    "CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER NOT NULL,timestamp INTEGER NOT NULL, data BLOB NOT NULL)",
]


def _varint(buf, i):
    v = 0
    for k in range(9):
        b = buf[i + k]
        if k == 8:
            return (v << 8) | b, i + 9
        v = (v << 7) | (b & 0x7F)
        if b < 0x80:
            return v, i + k + 1
    return v, i + 9


def _record(payload):
    """Decode an SQLite record into a list of Python values (None on malformed input)."""
    try:
        hlen, i = _varint(payload, 0)
        types = []
        while i < hlen:
            t, i = _varint(payload, i)
            types.append(t)
        pos, out = hlen, []
        sizes = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 6, 6: 8, 7: 8, 8: 0, 9: 0}
        for t in types:
            if t in sizes:
                n = sizes[t]
                raw = payload[pos:pos + n]
                if len(raw) != n:
                    return None
                if t == 0:
                    out.append(None)
                elif t == 7:
                    out.append(struct.unpack(">d", raw)[0])
                elif t in (8, 9):
                    out.append(t - 8)
                else:
                    out.append(int.from_bytes(raw, "big", signed=True))
            elif t >= 12:
                n = (t - 12) // 2 if t % 2 == 0 else (t - 13) // 2
                raw = payload[pos:pos + n]
                if len(raw) != n:
                    return None
                out.append(bytes(raw) if t % 2 == 0 else bytes(raw).decode("utf-8", errors="replace"))
            else:
                return None
            pos += n
        return out
    except IndexError:
        return None


def iter_leaf_records(db):
    """(rowid, values) for every table-leaf cell found in the readable pages of an SQLite file.

    Works without a valid sqlite_master or interior pages, like `.recover`; cells whose overflow
    chain points past the end of a truncated file are skipped. Freed pages may yield stale rows.
    """
    import mmap
    with open(db, "rb") as f:
        size = f.seek(0, 2)
        if size < 100:
            return
        m = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        try:
            page_size = struct.unpack(">H", m[16:18])[0]
            page_size = 65536 if page_size == 1 else page_size
            usable = page_size - m[20]
            n_pages = size // page_size
            max_local = usable - 35
            min_local = ((usable - 12) * 32 // 255) - 23
            for pno in range(1, n_pages + 1):
                base = (pno - 1) * page_size
                hdr = base + (100 if pno == 1 else 0)
                if m[hdr] != 0x0D:
                    continue
                ncells = struct.unpack(">H", m[hdr + 3:hdr + 5])[0]
                for c in range(ncells):
                    try:
                        ptr = struct.unpack(">H", m[hdr + 8 + 2 * c: hdr + 10 + 2 * c])[0]
                        if not 0 < ptr < page_size:
                            continue
                        plen, i = _varint(m, base + ptr)
                        rowid, i = _varint(m, i)
                        if plen <= max_local:
                            payload = m[i:i + plen]
                        else:
                            k = min_local + (plen - min_local) % (usable - 4)
                            local = k if k <= max_local else min_local
                            parts, got = [m[i:i + local]], local
                            nxt = struct.unpack(">I", m[i + local:i + local + 4])[0]
                            while got < plen and 0 < nxt <= n_pages:
                                ob = (nxt - 1) * page_size
                                take = min(usable - 4, plen - got)
                                parts.append(m[ob + 4: ob + 4 + take])
                                got += take
                                nxt = struct.unpack(">I", m[ob:ob + 4])[0]
                            if got < plen:
                                continue
                            payload = b"".join(parts)
                    except (struct.error, IndexError):
                        continue
                    values = _record(payload)
                    if values is not None:
                        yield rowid, values
        finally:
            m.close()


def recover_by_page_scan(src, dst):
    """Rebuild a rosbag2 sqlite3 database from leaf-page records (no sqlite3 CLI needed)."""
    tables = {"topics": {}, "messages": {}, "schema": {}, "metadata": {}}
    schema_sql = {}
    for rowid, v in iter_leaf_records(src):
        if len(v) == 5 and v[0] in ("table", "index") and isinstance(v[4], str):
            schema_sql[v[1]] = v[4]
        elif len(v) == 4 and v[0] is None and isinstance(v[1], int) and isinstance(v[2], int) \
                and isinstance(v[3], bytes):
            tables["messages"].setdefault(rowid, (rowid, v[1], v[2], v[3]))
        elif len(v) in (5, 6) and v[0] is None and all(isinstance(x, str) for x in v[1:]) and "/" in v[2]:
            tables["topics"].setdefault(rowid, (rowid, *v[1:5]))
        elif len(v) == 3 and v[0] is None and isinstance(v[1], int) and isinstance(v[2], str):
            key = "schema" if v[2] in ("humble", "iron", "jazzy", "rolling", "galactic", "foxy") else "metadata"
            tables[key].setdefault(rowid, (rowid, v[1], v[2]))
    d = sqlite3.connect(dst)
    for sql in HUMBLE_SCHEMA:
        d.execute(sql)
    d.executemany("INSERT INTO schema VALUES (?,?)", [r[1:] for r in tables["schema"].values()])
    d.executemany("INSERT INTO metadata VALUES (?,?,?)", tables["metadata"].values())
    d.executemany("INSERT INTO topics VALUES (?,?,?,?,?)", tables["topics"].values())
    d.executemany("INSERT INTO messages VALUES (?,?,?,?)", tables["messages"].values())
    d.execute("CREATE INDEX timestamp_idx ON messages (timestamp ASC)")
    d.commit()
    d.close()
    return {"topics_found": len(tables["topics"]), "messages_found": len(tables["messages"]),
            "original_schema_found": sorted(schema_sql)}


def parse_metadata_counts(path):
    """Topic -> message_count from a rosbag2 metadata.yaml (PyYAML if present, else a line parser)."""
    text = Path(path).read_text()
    try:
        import yaml
    except ImportError:
        yaml = None
    if yaml is not None:
        info = yaml.safe_load(text)["rosbag2_bagfile_information"]
        return {t["topic_metadata"]["name"]: t["message_count"] for t in info["topics_with_message_count"]}
    counts, item, inside = {}, None, False
    for line in text.splitlines() + ["end:"]:
        stripped = line.strip()
        key, _, val = stripped.lstrip("- ").partition(":")
        if key == "topics_with_message_count":
            inside = True
            continue
        if not inside:
            continue
        if stripped.startswith("- ") or not line.startswith("  "):  # next list item or end of section
            if item and "name" in item and "count" in item:
                counts[item["name"]] = item["count"]
            item = {}
            if not line.startswith("  "):
                break
        if key == "name":
            item["name"] = val.strip().strip("\"'")
        elif key == "message_count":
            item["count"] = int(val)
    return counts


def recover(zip_path, out, metadata=None, sqlite3_cli=None):
    zip_path, out = Path(zip_path), Path(out)
    out.mkdir(parents=True, exist_ok=True)
    report = {"zip": str(zip_path), "zip_bytes": zip_path.stat().st_size}
    scan = scan_signatures(zip_path)
    report["signatures"] = {"counts": scan["counts"], "local_file_offsets": scan["offsets"]["local_file"][:20]}
    try:
        report["entries"] = extract_with_central_directory(zip_path, out)
        report["mode"] = "central_directory"
    except (zipfile.BadZipFile, zlib.error, EOFError, OSError) as exc:
        report["central_directory_error"] = f"{type(exc).__name__}: {exc}"
        report["entries"] = extract_streaming(zip_path, out, scan["offsets"]["local_file"])
        report["mode"] = "local_headers_streaming"
    dbs = []
    for e in report["entries"]:
        if e.get("name", "").endswith(".db3"):
            db = _safe_target(out, e["name"])
            info = {"path": str(db), "integrity": sqlite_integrity(db)}
            if info["integrity"] != "ok":
                fixed = db.with_name(db.stem + "_recovered.db3")
                if fixed.exists():
                    fixed.unlink()
                cli = sqlite3_cli or shutil.which("sqlite3")
                try:
                    if not cli:
                        raise FileNotFoundError("sqlite3 CLI not found")
                    recover_with_cli(db, fixed, cli)
                    info["recovery"] = {"method": "sqlite3 .recover", "cli": cli}
                except (OSError, RuntimeError) as exc:
                    if fixed.exists():
                        fixed.unlink()
                    info["recovery"] = {"method": "leaf page scan", "cli_note": str(exc)[:300],
                                        **recover_by_page_scan(db, fixed)}
                info["recovered_path"] = str(fixed)
                info["recovered_integrity"] = sqlite_integrity(fixed)
                db = fixed
            try:
                info["topic_counts"] = topic_counts(db)
            except sqlite3.DatabaseError as exc:
                info["topic_counts_error"] = str(exc)
            dbs.append(info)
    report["databases"] = dbs
    meta = Path(metadata) if metadata else next((_safe_target(out, e["name"]) for e in report["entries"]
                                                 if e.get("name", "").endswith("metadata.yaml") and e.get("complete")), None)
    if meta and meta.exists():
        expected = parse_metadata_counts(meta)
        report["metadata_source"] = str(meta)
        for info in dbs:
            got = info.get("topic_counts", {})
            info["survival"] = {t: {"expected": n, "recovered": got.get(t, 0),
                                    "fraction": (got.get(t, 0) / n) if n else None} for t, n in sorted(expected.items())}
            total = sum(expected.values())
            info["survival_total_fraction"] = sum(got.get(t, 0) for t in expected) / total if total else None
    (out / "recovery_report.json").write_text(json.dumps(report, indent=1))
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("zip", type=Path)
    ap.add_argument("--out", type=Path, required=True, help="extraction folder (keep outside any git repository)")
    ap.add_argument("--metadata", type=Path, default=None, help="metadata.yaml if the zip's copy is missing")
    ap.add_argument("--sqlite3-cli", default=None, help="sqlite3 binary for .recover (default: PATH lookup)")
    args = ap.parse_args(argv)
    rep = recover(args.zip, args.out, args.metadata, args.sqlite3_cli)
    summary = {"mode": rep["mode"], "entries": [(e.get("name"), e.get("bytes"), e.get("complete")) for e in rep["entries"]],
               "databases": [{k: v for k, v in d.items() if k in ("path", "integrity", "recovered_path",
                                                                    "survival_total_fraction")} for d in rep["databases"]]}
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
