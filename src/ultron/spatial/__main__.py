"""Spatial memory from the command line: build it from a patrol workspace, ask it questions.

    python -m ultron.spatial --store runs/spatial ingest --data runs/pipeline
    python -m ultron.spatial --store runs/spatial query "wooden step ladder" --limit 3
    python -m ultron.spatial --store runs/spatial audit    # one object stored twice?
    python -m ultron.spatial --store runs/spatial status
"""
import argparse
import json
import math
from pathlib import Path

from .embedder import load_embedder
from .memory import SpatialMemory


def _memory(args, need_embedder):
    embedder = load_embedder()
    if need_embedder and embedder is None:
        raise SystemExit("appearance model unavailable; pip install -e '.[spatial]' and retry")
    return SpatialMemory(args.store, embedder=embedder)


def ingest(args):
    """Feed every keyframe of an existing pipeline workspace into the memory."""
    from PIL import Image

    memory = _memory(args, need_embedder=True)
    workspace = Path(args.data)
    if not (workspace / "objects.json").is_file() or not (workspace / "keyframes.json").is_file():
        raise SystemExit(
            f"no pipeline workspace at {workspace}: expected objects.json and keyframes.json.\n"
            "Point --data at a 'python -m ultron.pipeline' workspace (on the machine that ran the "
            "patrol), or build a demo workspace first; see docs/SPATIAL_MEMORY.md.")
    objects = {record["id"]: record for record in json.loads((workspace / "objects.json").read_text())}
    for keyframe in json.loads((workspace / "keyframes.json").read_text()):
        record = objects.get(keyframe["object_id"])
        if record is None:
            continue
        image = Image.open(workspace / keyframe["file"]).convert("RGB")
        width, height = image.size
        x0, y0, x1, y1 = keyframe["box"]
        crop = image.crop((x0 * width // 1000, y0 * height // 1000,
                           x1 * width // 1000, y1 * height // 1000))
        memory.register_sighting(record["id"], crop, record.get("position"), record["kind"])
        if keyframe.get("pose"):
            memory.remember_place(image, keyframe["pose"], stamp=keyframe.get("stamp"),
                                  mission_id=args.mission)
    print(f"{len(memory.objects)} objects, {len(memory.places)} places remembered in {memory.root}")


def query(args):
    memory = _memory(args, need_embedder=True)
    hits = memory.query_text(args.text, limit=args.limit)
    if not hits:
        print("nothing remembered matches")
    for hit in hits:
        where = ", ".join(f"{value:.1f}" for value in (hit.get("position") or []))
        label = hit.get("kind") or hit.get("thumbnail") or ""
        print(f"{hit['score']:.3f}  {hit['type']:6}  {hit['id']}  {label}  ({where})")


def audit(args):
    """Stored object pairs that look like one object recorded twice."""
    memory = _memory(args, need_embedder=False)
    identifiers = sorted(memory.objects)
    suspects = 0
    for index, id_a in enumerate(identifiers):
        for id_b in identifiers[index + 1:]:
            record_a, record_b = memory.objects[id_a], memory.objects[id_b]
            if record_a["kind"] != record_b["kind"]:
                continue
            if not record_a["position"] or not record_b["position"]:
                continue
            if math.dist(record_a["position"], record_b["position"]) > memory.config.gate_radius:
                continue
            score = memory.cross_score(id_a, id_b)
            if score is not None and score >= memory.config.same_threshold:
                print(f"likely one object stored twice: {id_a} and {id_b} (cosine {score:.3f})")
                suspects += 1
    if not suspects:
        print("no likely duplicates")


def status(args):
    memory = _memory(args, need_embedder=False)
    inspected = sum(1 for record in memory.objects.values() if record["inspected"])
    print(f"{len(memory.objects)} objects ({inspected} inspected), {len(memory.places)} places")
    for object_id, record in sorted(memory.objects.items()):
        where = ", ".join(f"{value:.1f}" for value in (record["position"] or []))
        done = record["inspected"]["mission"] if record["inspected"] else "-"
        print(f"  {object_id}  {record['kind']}  ({where})  samples={len(record['embedding_ids'])}  inspected={done}")


def demo(args):
    """Build a tiny workspace from the repo's ladder renders, then ingest and audit it.

    Two views of one rendered ladder are stored as two objects a metre apart —
    the double-detection failure — so audit should flag them as one.
    """
    from PIL import Image

    renders = sorted(Path("datasets/nesseler_photoreal_v2/images").glob("S01-01-*-a000-*.png"))[:2]
    if len(renders) < 2:
        raise SystemExit("demo needs the datasets/nesseler_photoreal_v2 renders; run from the repo root")
    workspace = Path(str(args.store).rstrip("/") + "-demo-workspace")
    (workspace / "keyframes").mkdir(parents=True, exist_ok=True)
    objects, keyframes = [], []
    for index, source in enumerate(renders):
        name = f"ladder-{index + 1}"
        file = f"keyframes/{name}.jpg"
        Image.open(source).convert("RGB").save(workspace / file)
        objects.append({"id": name, "kind": "ladder", "position": [float(index), 0.0]})
        keyframes.append({"object_id": name, "file": file, "box": [0, 0, 1000, 1000],
                          "pose": [float(index), 0.0, 0.0], "stamp": 100.0 + index})
    (workspace / "objects.json").write_text(json.dumps(objects))
    (workspace / "keyframes.json").write_text(json.dumps(keyframes))
    args.data, args.mission = str(workspace), "DEMO"
    ingest(args)
    print("-- audit: two views of one ladder were stored as two objects --")
    audit(args)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m ultron.spatial",
                                     description="Build and query the patrol's spatial memory.")
    parser.add_argument("--store", default="runs/spatial", help="memory folder (default runs/spatial)")
    commands = parser.add_subparsers(dest="command", required=True)
    command = commands.add_parser("ingest", help="feed a pipeline workspace into the memory")
    command.add_argument("--data", default="runs/pipeline", help="pipeline workspace folder")
    command.add_argument("--mission", default="", help="mission id recorded with the places")
    command.set_defaults(run=ingest)
    command = commands.add_parser("query", help="ask where something was seen")
    command.add_argument("text")
    command.add_argument("--limit", type=int, default=5)
    command.set_defaults(run=query)
    command = commands.add_parser("audit", help="find objects that are likely one object stored twice")
    command.set_defaults(run=audit)
    command = commands.add_parser("status", help="what the memory holds")
    command.set_defaults(run=status)
    command = commands.add_parser("demo", help="self-contained double-detection demo from repo renders")
    command.set_defaults(run=demo)
    arguments = parser.parse_args(argv)
    arguments.run(arguments)


if __name__ == "__main__":
    main()
