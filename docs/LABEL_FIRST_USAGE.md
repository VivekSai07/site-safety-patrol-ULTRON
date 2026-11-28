# Local label-first capture and draft usage

The code below is simulation-only. A rendered synthetic capture-to-draft
probe used HTTP inputs and manual fixture transcription; bounded browser
controls and focus behavior were checked separately. See [validation](VALIDATION.md). The teleop scene
uses synthetic ladder fixtures and label/sticker text; this is not real OCR,
validated perception, walking, or inspection. No hardware commands or
connections are authorized.

The recorded local draft probe used a source-specific catalog that is
excluded from distribution and may be used locally only after applicable
local-use rights are verified. An independently authored, minimal synthetic
public example and an `ULTRON_FORM_SCHEMA` override for a compatible,
locally supplied schema are available. The public example is not the
official form or a complete mapping; the recorded probe does not validate
the new example or override.

## 1. Start the simulated capture UI

The host-accessible derived teleop launcher requires Docker, the organizer kit
image already built locally, an unused loopback port and an existing writable
directory owned by the non-root user. From the team repository root, build
the image first if it does not exist (use an unused port and fresh build ID;
the launcher refuses an existing image tag):

```bash
python3 -B scripts/organizer_b2.py --kit ../ctrl-hackathon-2026-kit/unitree-mujoco --run-id 1127b2001 --port 6081 build
```

The recorded environment already has `ultron-organizer-b2:1127b2001`, so
reuse it rather than rebuilding that tag. The following exact launch was
tested; use a fresh run ID and unused port if they conflict:

```bash
mkdir -p runs/teleop-host-demo
PIN=sha256:1fc95a07c9e7b7a4589f04dd758440d9c6cb4ff0a0a8e5c7fba00ea9fd1c5121
OUT=runs/teleop-host-demo
python3 -B scripts/teleop_launch.py up --run-id hostuifinal3 --image-id "$PIN" --port 18768 --output "$OUT"
python3 -B scripts/teleop_launch.py status --run-id hostuifinal3 --image-id "$PIN" --port 18768 --output "$OUT"
```

Open <http://127.0.0.1:18768/> while it is running. The SHA is the **local
image ID of this build**, not a universal or reproducible constant. For a
different build, obtain its full ID with
`docker image inspect ultron-organizer-b2:<build-id> --format '{{.Id}}'`,
set `PIN` accordingly and provide
`--image ultron-organizer-b2:<build-id>` to all three lifecycle commands.
The launcher's default image tag is `ultron-organizer-b2:1127b2001` and it
checks the tag against the supplied full SHA. Run IDs are 6–24 lowercase
alphanumeric characters; use identical run ID, image, SHA, port and output
for `up`, `status` and `down`.

The launcher uses a task-owned private network, read-only source mount and
explicit output mount, with a loopback-only published port. In the recorded
environment Docker did not publish the port, so the launcher used its
task-owned loopback relay. The host-local UI has no user authentication:
localhost does not protect it from other local users. Use a trusted host and
non-sensitive data. **Cleanup is mandatory** when finished:

```bash
python3 -B scripts/teleop_launch.py down --run-id hostuifinal3 --image-id "$PIN" --port 18768 --output "$OUT"
```

`down` removes only the validated task container, network and any relay; the
output files remain. The host HTTP page, PNG frame and capture endpoints were
tested, and the task was removed. Subsequent browser-control checks are scoped
in [validation](VALIDATION.md). This derived 1280×720 kinematic mode does not run the kit's DDS/stand
writer and is separate from the isolated organizer viewer/stand probe; that
probe did not test browser GUI or host access.

The underlying module also exposes `--kit-root`, `--output`, `--width`,
`--height`, `--host` and `--port` via `python -m ultron teleop --help`.
Its kit root must contain `unitree_robots/b2/b2.xml` and assets; it does not
fall back to the legacy model. Direct container binding to `0.0.0.0` requires
`ULTRON_TELEOP_ISOLATED=1` **and** a private container network with a
loopback-only published port; the flag alone does not provide isolation.

In the UI:

1. Select the ladder ID, optionally supply an operator and session ID, and
   create/select the session. A blank session ID generates a unique one.
   Movement starts disarmed.
2. Enable movement and use buttons or W/S/A/D for small simulated increments.
   STOP, Escape, focus loss and hiding the page disarm movement; re-enable
   explicitly. Collision/boundary rejection is simulation behavior only.
3. Use overview and simulated inspection camera views. Capture an overview,
   then drag a rectangle over the label and choose **Capture label ROI**.
   The full label image is retained alongside its derived crop. The scene
   label text and dates are fixtures, not real equipment information.

The UI shows the container session path and `adaptive_manifest.template.json`
path. For the launcher example, host files are under
`runs/teleop-host-demo/captures/<session-id>/`; `/sessions` in the UI is the
mounted host output directory. Each capture refreshes that generated template; it starts with empty
`visual_fields`, `observations` and `date_observations`. Suggested targets in
provenance are initially empty; select IDs from the active schema only after
reviewing the evidence. No field is implied to be observed.

## 2. Review and edit a copy of the manifest

After capture, copy the template **in the same session directory**, because
its image `path` entries are relative to the manifest's directory. Substitute
the host session path for `runs/teleop-host-demo/captures/session-example`:

```bash
cp runs/teleop-host-demo/captures/session-example/adaptive_manifest.template.json runs/teleop-host-demo/captures/session-example/adaptive_manifest.reviewed.json
```

Review the original images and linked ROI, verify they belong to the selected
ladder, then edit only the copied `adaptive_manifest.reviewed.json`. If
another capture is made, the generated template is refreshed; update the
reviewed copy deliberately rather than editing the generated file.

The input manifest has `schema_version: "1.0.0"` and a non-empty `views`
array with at least one original `OVERVIEW` and one original `LABEL`. Each
view carries `evidence_id`, matching `ladder_id`, `kind`, `path`,
`visual_fields`, `provenance`, `crops`, `observations`,
`date_observations`, `pose` and `camera`. Keep those generated keys and
unique evidence IDs; do not relabel a crop as an independent view. The
separate output draft uses the active schema's field catalog, not the input
manifest schema. The bundled public catalog is a minimal synthetic example,
not an official or complete checklist. Its visual field IDs are
`demo_label_text`, `demo_overview_note`, `demo_last_inspection_date` and
`demo_next_inspection_date`; `demo_physical_review` is human-only. Page 1
is a contract placeholder, not a source-form page mapping.

To select a separately supplied compatible v1 schema, set
`ULTRON_FORM_SCHEMA` to its local path before the draft command in section 3.
An empty, missing, unreadable or invalid path fails rather than falling back.
Unset the variable to use the bundled example. The local schema is not
embedded or hashed into the draft: preserve its exact version with the
session. Source-specific catalogs require verified local-use rights and
review against their source, and are excluded from distribution. The optional
OpenAI adapter accepts local catalogs only in fixture mode; its live path
refuses any schema override.

```bash
export ULTRON_FORM_SCHEMA=/absolute/path/to/authorized-local-schema.json
# Run the draft command in section 3 in this same shell.
```

For each reviewed view, manually assign only relevant visual field IDs from
the active catalog to `visual_fields`; these are selection *targets*, not
evidence that the field is visible or passes. A field observation must use an ID
declared in that view's `visual_fields`; it must refer to that original or
one of its linked crops. After reviewing a label, an unreadable field may be
recorded as the following illustrative fragment. If using a different catalog,
replace `demo_label_text` with an appropriate ID from that catalog:

```json
"visual_fields": ["demo_label_text"],
"observations": [{
  "field_id": "demo_label_text",
  "value": null,
  "coverage": "UNREADABLE",
  "reason": "Not readable in this image.",
  "evidence_ids": [],
  "provenance": {"method": "MANUAL_REVIEW", "producer": "operator", "untrusted": true}
}]
```

This is a fragment to replace the corresponding arrays of the generated
view, not a standalone manifest. An empty `evidence_ids` list defaults to
that view; use the generated crop ID explicitly if the reading comes from
the crop. An `OBSERVED` value must be provided only when actually visible and
manually transcribed. `NOT_APPLICABLE` requires a reason. Human-only fields
cannot be assigned as visual targets.

Date readings go in `date_observations` with `date_role`, `value`, `coverage`,
`evidence_ids` and `provenance`. The distinct roles are
`MANUFACTURER_FINAL_CONTROL`, `ACQUISITION`, `RECURRING_INSPECTION` and
`NEXT_INSPECTION`; date coverage is `OBSERVED`, `UNREADABLE` or `CONFLICTING`.
Check which roles the active schema and implementation support; the public
example is intentionally minimal.
Do not invent values or conflate manufacturer and recurring inspection dates.
Preserve uncertainty and conflicts for human review.

## 3. Generate a separate local draft session

On a local Python 3.10+ environment with Pillow installed, from the team
repository root, pass the **reviewed copy** and the same ladder ID. The
capture session files must be accessible at the paths in the copied manifest:

```bash
PYTHONPATH=src python3 -B -m ultron adaptive-draft runs/teleop-host-demo/captures/session-example/adaptive_manifest.reviewed.json --ladder-id LAD-001 --operator-id operator --sessions-root runs/draft-sessions --session-id draft-example
```

After finishing with a local override, run `unset ULTRON_FORM_SCHEMA` to return
to the bundled example.

Use the actual ladder ID and a fresh `--session-id`; omit it to generate a
unique one. `--operator-id` is optional and `--sessions-root` defaults to
`runs/label-first-sessions`. The command creates a **new**, separate draft
session, refusing an existing session directory; it copies all original
inputs, retains unused images and regenerates linked crops. The command prints
paths to `draft.json`, `selection.json`, and `review.html`; open the latter
locally to review evidence and unresolved fields. It initially selects the
first overview and label and may select additional originals based on
operator-declared targets and remaining gaps. It does not evaluate visibility
or establish a fixed sufficient image count.

No automatic OCR happens in this CLI. Manual or fixture values are untrusted
proposals, not verified findings; an empty template produces unresolved
fields. The separate OpenAI adapter defaults to an offline empty fixture
response and is not invoked by this command. Live use is opt-in and requires
an approved-image manifest and separate authorization and rights/privacy
review for any images and text; it is unverified. Do not send images or
reference-derived text or use a live API as part of this guide.

The draft is not a completed inspection or safe-to-use result.
`AUTOMATED_PASS`, where present elsewhere, is only scoped pre-screening.
Confirmed hard failures must not be erased, coverage gaps remain separate
from an individual ladder's assessment, and qualified human review and
tactile/functional checks remain mandatory.

See the [source overview](LABEL_FIRST_BASELINE.md) and separate
[organizer DDS smoke test](ORGANIZER_B2_SIM.md).
