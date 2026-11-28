# Seven-section assessment, dashboard adapter and photoreal prompt ZIP

Owner: Vivek. Om owns detection, Kaan (Kann) spatial memory, Sahil simulation SLAM/mapping/navigation, Aravind the dashboard. The image-generation prompts request **real-world-style photoreal photographs**. They do not request MuJoCo renders. Generated images remain synthetic and require actual image review.

**Current scope:** operator-positioned simulation capture and an
evidence-linked draft come first. The live commands below send images and
prompt text to an external API; do not run them, upload the prompt ZIP or
generate external images without separate authorization and rights/privacy
review. The ladder and site-safety packs contain source-derived material
whose reuse rights remain unverified. No live call was made in this
reconciliation.

## First live assessment milestone

`scripts/first-milestone.sh` is a later, separately authorized live path. It
prepares dependencies, starts the dashboard on a loopback port, discovers a
vision model and sends one assessment for dashboard readback. The dashboard
has no user authentication, even on localhost. A prior session reported
network restrictions; that does not describe the current host or establish a
live result. See [dashboard source/setup provenance](DASHBOARD_IMPORT.md).

The runnable bridge is now `ultron.assessment_live`. It uses standard-library HTTP, so the OpenAI Python SDK is not required. It loads `OPENAI_API_KEY` or the team's existing `OpenAI_API` variable from the ignored `.env` as data, without executing it, echoing the key or copying it into artifacts. Environment values take precedence. Optional settings: `ULTRON_OPENAI_MODEL` and `ULTRON_DASHBOARD_URL`; CLI flags override those settings. Default model is `gpt-6.1-terra` with low reasoning effort, configurable and not account-validated here. Official documentation lists its [image input, Responses API and structured-output support](https://developers.openai.com/api/docs/models/gpt-6.1-terra). No finding-accuracy claim follows from those capabilities. Default dashboard origin is `http://127.0.0.1:8000`.

Only after the separate approval and rights/privacy review above, with a
dedicated dashboard already running, the upstream live examples are:

```bash
PYTHONPATH=src python3 -m ultron.assessment_live runs/inspection-360-v1/keyframes.json
# Alternate dashboard address or model:
PYTHONPATH=src python3 -m ultron.assessment_live runs/inspection-360-v1/keyframes.json --dashboard-url http://127.0.0.1:8000 --model gpt-6.1-terra
```

Use a dedicated dashboard with external notification channels disabled; this milestone does not authorize email/chat delivery. The bridge validates `/api/live` and `/api/notify/config` before a billable request. It requires all 24 sampling cells, snapshots hash-verified evidence into a new ignored run folder, creates a fresh mission, posts progress/coverage events, uploads photos and a live preview, then makes exactly one Responses API call. It retains the raw response/usage and validates the bundle-bound 23-criterion proposal before publishing one dashboard inspection. The final mission status records review-pending evidence, not completed site navigation or formal ladder certification.

On API failure/refusal/malformed output it preserves an explicit MANUAL_REVIEW record. Dashboard failures stop publication and retain the local job. There are no automatic billable retries, no image generation and no robot connection. An API attempt marker is written before POST: an interruption/timeout can leave billing uncertain, so inspect the previous run before trying again. A newly executed command deliberately creates a new job/mission; it is not an automatic resume or deduplicated second API call.

Each run reports its output folder and writes `job-status.json`. A completed live milestone requires `transport_mode=LIVE_HTTP`, `api_validated=true`, `dashboard_published=true`, the actual usage/response artifact, and visual confirmation in the dashboard. Tests with injected transports are labeled `INJECTED_TEST` and cannot satisfy this gate. This bridge does not start or merge the dashboard branch; run Aravind's service separately. Its current inspected branch is `feature/dashboard` at `d68e8441552dc666269548d3f20db08a3f700e8b`; the inspection contract is unchanged from the earlier snapshot.

Status on 28 November: `.env` credential loading succeeded, but the authenticated OpenAI probe failed DNS resolution and the local dashboard probe failed. No inference response was obtained. The implementation/test artifacts do not establish a live milestone pass or real-data evaluation. Image generation remains parked.

## Image-generation prompt archive

The existing [ULTRON_inspection_prompt_pack_v1.zip](../deliverables/ULTRON_inspection_prompt_pack_v1.zip)
contains scenario recipes, prompts, review guidance and API prompt/schema
assets; it is not a finished dataset. It includes source-derived rubric
material with unverified reuse rights. Do not upload it or its contents to
ChatGPT or another service without separate authorization and rights/privacy
review.

If image generation is separately approved, inspect actual pixels for
realism, identity and defect visibility before labeling. Keep all views and
condition variants of one `base_asset_group` in one split. Intended conditions
are not labels. Real held-out ladder/session data remains necessary for
real-world evaluation; generated label text requires separate verification.

To rebuild the ZIP after deliberately changing its source recipe:

```bash
python3 scripts/build_inspection_pack.py
```

The script refuses to overwrite edited/extra files in its generated pack directory. It writes prompt assets under `src/ultron/assets/inspection_v1/` for installation with the package. Keep generated images in `datasets/`, outside that prompt directory. Neither API keys nor partner PDFs/images are included in the ZIP.

## Actual 360 simulation capture

```bash
PYTHONPATH=src MUJOCO_GL=egl LP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 python3 -m ultron.inspect360 --out runs/inspection-360-demo
```

This separate runner plans an obstacle-inflated path around an open A-frame and captures eight target-relative azimuths with lower/middle/upper views. It returns to the simulated base and writes `keyframes.json`, RGB/depth, camera matrices, telemetry and `preview.gif`. Existing output folders are refused. The runner uses kinematic base movement, segmentation-oracle identity/visible-band counts and an explicit **virtual pan/tilt camera**. It does not command B2 posture, implement physical walking, read labels or run a learned detector. The normal patrol's conservative orbit policy is preserved. The 24-cell grid is specific to this experiment and the upstream live milestone, not a proven minimum image count or a prerequisite for the label-first draft.

The proposed 24-cell gate requires distinct images, valid caller-attested poses, confirmed target identity, accepted quality and observed region flags, plus checksums. Its angle/band metadata must come from the measured capture stack, not VLM guesses. The simulation uses world-truth orientation and proposed sharpness/pixel thresholds; those do not establish real calibration. Sampling completion is separate from actual surface visibility and seven-section evidence completion. Missing cells stay incomplete. No unsafe path or unverified robot height/tilt command is acceptable to satisfy 360 coverage.

Executed locally: `runs/inspection-360-v1` captured 24/24 accepted sampling cells and returned to base. Each captured band was checked against rendered target depth/segmentation pixels. This is a bounded synthetic capture result, not physical full-surface inspection. See its `keyframes.json` for source and measured values. Hardware completion remains untested.

## Build an API request without making a call

```bash
PYTHONPATH=src python3 -m ultron.assessment request runs/inspection-360-demo/keyframes.json --model YOUR_AVAILABLE_VISION_MODEL --out runs/inspection-360-demo/request.json
```

Replace the model placeholder with a vision/structured-output model actually available to the credited project. The builder reads same-asset JPEGs, validates hashes/root confinement and includes common instructions, all seven detailed prompts and all 23 exact dashboard criterion keys. Human-only fields cannot become image-only OK. Use at most 32 selected images per request, including the 24 accepted grid views and any needed label/detail crops. The API sends images, not directory paths. Treat request JSON as evidence-bearing data and keep it local.

Every proposal must echo a deterministic `bundle_id` binding mission/asset/map/date, view metadata/image hashes and the exact prompt/schema fingerprints. Validation rejects old results for changed pixels, a different mission or changed prompts, even if local image names are reused. Preserve the exact request manifest alongside the raw response; do not rebuild it with different poses or applicability while a job is in flight.

The request uses the official [Responses image input](https://developers.openai.com/api/docs/guides/images-vision?api-mode=responses) and [Structured Outputs](https://developers.openai.com/api/docs/guides/structured-outputs) format. Schema validity does not establish finding accuracy. No live model call was executed here, and compatibility with the project's chosen model must be smoke-tested. Each call costs credits; set backend budget/timeouts before sending.

Only after separate authorization and rights/privacy review, a future caller
could send the prepared body with an environment-owned API key. Save the raw
response and usage, then pass the envelope to
`parse_api_response(response.model_dump(), manifest)`. Incomplete/refused/
malformed outputs produce explicit review proposals. On network exceptions
use `review_proposal(manifest, reason)`. Neither helper makes network calls
or grants motion authority. Keep API keys out of Git/browser and do not log
request authorization headers.

Sticker output preserves date precision (`YYYY-MM` or `YYYY-MM-DD`) and literal text. Month-only due dates are compared by month, without inventing a printed day; the interpretation must match the verified label semantics. Year-only or ambiguous dates stay unresolved. Model-readable dates remain proposals; never equate a sticker with current condition or inspector sign-off.

## Wire to Aravind's current dashboard

Contract inspected: `feature/dashboard` at `42330758a62197122b56d56098a49c8c442350bc`. `feature/protocol-reader` at `698a571c4b3631cf3ed535212a80b660ee071fe2` already has a different VLM/publisher path. These branches are not merged by this work. Reuse their app rather than duplicating a dashboard; reconcile the reader's assessment policy before integrating its model-only pass behavior.

Produce a payload after extracting/validating the proposal JSON:

```bash
PYTHONPATH=src python3 -m ultron.assessment dashboard runs/inspection-360-demo/keyframes.json --response /path/to/validated-proposal.json --out runs/inspection-360-demo/dashboard.json
PYTHONPATH=src python3 -m ultron.assessment uploads runs/inspection-360-demo/keyframes.json --out runs/inspection-360-demo/upload-plan.json
```

The generated payload matches `dashboard.models.Inspection`: exact group/item keys, `photos`, `item_photos`, `position`, status and pending human fields. Dashboard photo IDs are derived from the full asset ID plus image hash, preventing a second ladder's generic local frame IDs from overwriting the first ladder's mission-wide media. `upload-plan.json` maps original image IDs/paths/hashes to these photo IDs; use it for uploads and preserve it with the record. Model-only clean results remain `MANUAL_REVIEW`; visible defects/overdue proposals are `FLAGGED`. An unknown map position is rejected rather than fabricated at the origin. The coverage ledger and raw proposal remain sidecars; add them to event data because the dashboard drops unknown Inspection fields.

Publication sequence to the running dashboard (not executed in this environment):

1. `POST /api/missions`: supply `mission_id`, `site_id`, `map_version`, `synthetic` and optional site map. Use the same IDs as the evidence bundle. Check returned state on restart; do not blindly reuse an ended mission.
2. `POST /api/events`: event type such as `INSPECTION_STARTED` / `COVERAGE_360`, with mission-relative time, asset ID and the coverage/progress data. These drive pending states without duplicate inspection records.
3. For each `photos[].id`, `POST /api/missions/{mission_id}/photos/{photo_id}` with the corresponding JPEG body and `Content-Type: image/jpeg`; each must be under 5 MB. Verify every upload succeeds before posting the verdict. Keep image IDs stable and never overwrite a prior inspection's evidence with new pixels.
4. `POST /api/inspections` with `dashboard.json`. The dashboard validates photo presence and one inspection per asset/mission; a second post returns 409. Do not silently treat that as a successful update. Gather API proposals and required capture progress before finalizing; use events for progress. A later inspection needs a new mission or an explicitly designed dashboard update API.
5. The dashboard emits its existing `inspection` SSE event on `/api/stream`, exposes `/api/live`, and generates protocol exports. Pending tactile checks and human sign-off remain visible. No extra notification channel is configured or exercised by this pack.

Contract check executed: all 23 rubric item keys equal Aravind's `ITEM_KEYS`, and an exported MANUAL_REVIEW fixture was accepted by the exact snapshot's Pydantic `Inspection` model. FastAPI/server runtime, HTTP/SSE publication and live OpenAI calls were **not** tested here. The offline files are runnable artifacts, not an assertion the deployed UI already displays the new coverage grid.

## Assessment evaluation

Vivek owns the frozen prompt/model/rubric evaluation; Om owns detector recall and Kaan identity errors. Review at least one real held-out case per section plus unreadable label, partial/blocked ring, look-alike ladders, clean-looking human-only mechanisms and API refusal/outage. Preserve ground-truth provenance and human adjudication. Do not reuse prompt-tuning scenes as the held-out test set.

Record per-criterion defect recall/false alarms, visual-unknown and human-only rates, evidence-ID correctness, false OK on known defects, sticker transcription/date errors, 24-cell sampling completion, actual visible surface gaps, latency and spend. Report synthetic and real metrics separately. A policy that reviews everything has zero automated safe passes by design; report its abstention rate so that this is not mistaken for model accuracy. No quantitative model result is claimed by the contract fixtures or this prompt ZIP.
