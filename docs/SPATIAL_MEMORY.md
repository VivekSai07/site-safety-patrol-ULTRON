# Spatial memory (`ultron.spatial`)

Appearance re-identification and place memory for the patrol, inspired by
DimOS spatial memory (Apache-2.0, github.com/dimensionalOS/dimos; concept
only, no code vendored). It exists to stop one ladder being stored — and
inspected — twice, and to answer "where did you see X?".

## Why appearance

The association pipeline matches sightings to objects by geometry. That
fails two ways: pose drift turns one ladder into two objects, and two
similar ladders close together collapse into one. The memory adds the
missing signal: CLIP image embeddings per object (up to 8 viewpoint-diverse
samples), compared by cosine.

**Decision rule** (`SpatialMemory.advise` + `classify`): geometry gates the
candidates (same kind within 3 m); appearance decides — cosine >= 0.80 is the
same object, <= 0.60 is a different one, in between is undecided and the
caller keeps its geometric behaviour. Thresholds are first guesses in
`SpatialConfig`; calibrate against the hall recording.

## Model

`clip_b32_datacomp.ts` (CLIP B/32, ultralytics asset, one-time 289 MB
download, cached in the repo root). The repo's `mobileclip_blt.ts` only
scripts the *text* encoder and cannot embed images. Without torch or the
weights everything degrades to geometry-only: `load_embedder()` returns
None, `advise()` returns [], places keep pose + thumbnail.

## For the association branch (Kaan)

`from ultron.spatial import AssociationAdvisor, SpatialMemory, load_embedder`

- Construct per site map: `SpatialMemory(sites_dir / map_id / "spatial", embedder=load_embedder())`.
- In `objects.py`, where a sighting is matched: pass your geometric
  candidates to `advise(crop, position, kind, candidate_ids=[...])`; a
  `classify(score) == "same"` top hit justifies matching across drift, a
  "different" verdict on all candidates justifies a new object, undecided
  changes nothing.
- After the match: `register_sighting(object_id, crop, position, kind)`.
- Before re-inspecting: `was_inspected(object_id)`; after a protocol:
  `mark_inspected(object_id, mission_id)`.
- The crop is the detection box cut from the frame (thousandths -> pixels).

## CLI

    pip install -e '.[spatial]'
    python -m ultron.spatial --store runs/spatial ingest --data runs/pipeline
    python -m ultron.spatial --store runs/spatial status
    python -m ultron.spatial --store runs/spatial audit          # one object stored twice?
    python -m ultron.spatial --store runs/spatial query "wooden step ladder"

`ingest` replays an existing pipeline workspace (objects.json + keyframes)
into the memory, so the morning's recording can be audited for the
double-detection immediately. `audit` works offline from stored vectors.

## State

`<store>/objects.json`, `embeddings.npz`, `places.json`,
`place_embeddings.npz`, `places/*.jpg` — JSON + npz, written atomically
after every change. Corrupt files are moved aside with a warning, never
fatal. Design: `docs/superpowers/specs/2026-11-28-spatial-memory-design.md`.
