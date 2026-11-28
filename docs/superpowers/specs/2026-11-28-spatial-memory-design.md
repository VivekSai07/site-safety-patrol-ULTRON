# Spatial memory with appearance re-identification — design

Date: 2026-11-28
Branch: `feature/spatial-memory` (off `main`; deliberately independent of `pipeline/association`)

## Problem

During this morning's patrol the same ladder was detected and inspected twice as two
separate objects. The `pipeline/association` branch addresses this geometrically
(footprint association, anchoring, moved detection), but geometry alone fails in two
ways: pose drift turns one ladder into two objects, and two similar ladders standing
close together collapse into one. The missing signal is appearance.

## Goal

A standalone `ultron.spatial` package, inspired by DimOS's spatial memory
(`dimos/perception/experimental/spatial_perception.py`, `spatial_vector_db.py`,
Apache-2.0), that gives every object the robot sees a visual identity and gives the
patrol a queryable memory of places. Two capabilities:

1. **Object re-ID memory** — per-object appearance embeddings so a new sighting can be
   confidently matched to ("same ladder, already inspected") or separated from
   ("different ladder, inspect it") stored objects. This is the fix for the
   double-inspection bug.
2. **Place memory** — frame + pose embeddings sampled along the patrol route,
   queryable by natural-language text ("where did you see a fire extinguisher?") with
   map coordinates and evidence thumbnails as answers.

Non-goals: owning the object map (that stays in `pipeline/association`'s
`objects.py`), any ChromaDB or DimOS runtime dependency, dashboards (CLI only for
now), real-robot validation.

## Approach (chosen from three)

**Advisory sidecar.** The spatial memory never creates or merges objects itself; it
advises the association logic at its decision points and only overrides geometry when
appearance is confident. Alternatives rejected: a gatekeeper store owning the object
map (collides with the overnight work on `pipeline/association`) and post-survey
embedding dedup (does not prevent double inspection during a mission).

## Embedding model

Reuse the ultralytics CLIP TorchScript assets. The repo's `mobileclip_blt.ts`
(fetched for YOLOE text prompts) only scripts the text encoder — verified by
loading it — so the embedder uses `clip_b32_datacomp.ts` (CLIP B/32, 289 MB,
one-time download from ultralytics' asset store, cached in the repo root),
which exports image and text encoders in a shared 512-d space. No new pip
dependencies: the `spatial` extra is
`["ultralytics>=8.3.0", "torch>=2.2", "numpy>=1.24", "Pillow>=9"]`,
all already installed on the GPU machine.

## Architecture

```
src/ultron/spatial/
  embedder.py   MobileCLIP wrapper: embed_image(np_img) -> f32[512]
                embed_text(str) -> f32[512]; lazy torch import; GPU else CPU
  memory.py     SpatialMemory(site_dir, config): object re-ID store + place store,
                saved after every change
  hook.py       AssociationAdvisor protocol — the only import surface for
                pipeline/association
  __main__.py   CLI: query, status
```

`SpatialMemory` knows nothing of pipeline types: it takes numpy images, `(x, y)`
positions and plain strings.

## Data model

Stored in `sites/<site>/spatial/` beside the pipeline's `objects.json`:

- `objects.json` — per object: `id`, `kind`, `position [x, y]`, `embedding_ids`,
  `inspected` (mission id + timestamp, or null), `last_seen`.
- `embeddings.npz` — one f32[512] row per sample; at most **8 samples per object**.
  A new sample is kept only if its max cosine to existing samples is < 0.95
  (viewpoint diversity); otherwise it replaces the oldest sample.
- `places.json` + `place_embeddings.npz` — per place: pose `(x, y, yaw)`, timestamp,
  mission id, thumbnail path (`places/<n>.jpg`).

## Re-ID decision rule

For a sighting with crop `c` at map position `p`, kind `k`:

1. **Geometry gates:** candidates are stored objects of kind `k` within `r_gate`
   (default 3 m — generous to survive the pose drift behind the original bug).
2. **Appearance decides:** each candidate's score is the max cosine between
   `embed(c)` and its stored samples.
   - best ≥ **0.80** → *same object*, even far within the gate; on the following
     `register_sighting` the memory updates its stored position with an EMA
     (weight 0.3 to the new sighting). The caller corrects its own map as it
     sees fit.
   - best ≤ **0.60** → *different object*, even if adjacent; two similar ladders
     side by side stay distinct.
   - otherwise → *undecided*; the caller keeps its purely geometric behaviour.
3. **Inspection guard:** `was_inspected(id)` lets the inspection stage skip objects
   already inspected this mission regardless of sighting history.

The advisor is conservative by design: a wrong "same" (missed inspection) is worse
than a wrong "different" (duplicate report), so the "same" threshold is the strict
one. All thresholds live in a `SpatialConfig` dataclass and are first guesses to be
calibrated against the hall recording.

## Hook interface (for `pipeline/association`)

```python
class AssociationAdvisor(Protocol):
    def advise(self, crop: np.ndarray, position: tuple[float, float], kind: str,
               candidate_ids: list[str]) -> list[tuple[str, float]]:
        """Appearance scores for the caller's candidates, best first.
        Empty list means undecided / no appearance signal."""
    def register_sighting(self, object_id: str, crop: np.ndarray,
                          position: tuple[float, float], kind: str) -> None: ...
    def mark_inspected(self, object_id: str, mission_id: str) -> None: ...
    def was_inspected(self, object_id: str) -> str | None: ...
```

`objects.py` on the association branch calls `advise()` at its two decision points
("is this sighting a known object?", "are these two the same?") whenever it merges.
Until then, a small adapter on `main`'s survey path feeds detections (crop + lifted
map position) into the memory so the module is demoable standalone.

## Place memory

A frame is remembered when the robot moved ≥ **1 m** or turned ≥ **45°** since the
last stored place, with ≥ **1 s** spacing (DimOS's distance/time thresholds, tuned
for a patrol). `query_text(text, limit)` embeds the text and cosine-ranks both place
frames and object samples, returning map coordinates, thumbnail path and timestamp.

CLI: `python -m ultron.spatial --store runs/sites/ctrl-hall/spatial query "fire extinguisher"`
prints ranked hits with `(x, y)`.

## Error handling

- Embedder unavailable (no torch, missing `.ts` file) → degrade to geometry-only:
  `advise()` returns empty, places store pose + thumbnail without embeddings; the
  pipeline never breaks because the model is absent.
- Corrupt or partial store files on load → renamed aside, fresh start, warning
  logged.
- Per-frame embedding failures are logged and skipped, never raised into the survey
  loop.

## Testing

- Unit tests with a **fake deterministic embedder**: re-ID across drifted positions,
  adjacent-but-different objects kept distinct, undecided band, sample-diversity cap,
  inspection guard, persistence round-trip, corrupt-file recovery, place sampling
  thresholds, text query ranking.
- One integration test loading the real MobileCLIP (skipped when torch or the model
  file is absent): two crops of the same ladder score above the different-object pair
  from existing fixture images.
- No tests require GPU, ROS or the robot.

## Attribution

Concept and thresholds adapted from DimOS (Apache-2.0,
github.com/dimensionalOS/dimos); no DimOS code is vendored.
