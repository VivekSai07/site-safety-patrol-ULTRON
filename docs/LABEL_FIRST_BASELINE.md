# Label-first inspection baseline and source overview

**Status: synthetic capture-to-draft HTTP probe observed.** This document
records the current user-directed baseline. Code for a simulated kinematic
capture UI, a Nesseller draft contract and local adaptive drafting exists.
A rendered 1280×720 fixture capture and manually transcribed draft were
exercised through HTTP. Bounded browser-control and focus checks were
subsequently exercised in specified simulated setups; real OCR and real
inspection are not established. See [validation](VALIDATION.md).
The existing autonomous MuJoCo demo remains a separate simulation-only
reference workflow. “Baseline” means the prioritized design target, not a
completed inspection or permission to operate hardware.

## Source authority and preservation

The source is `nesseler-data/kontrollblatt_checkliste_leitern_tritte.pdf`
(German, three pages), from the locally supplied
[`ctrl-rwth/ctrl-hackathon-2026` source at
`bd3247546f0c753a7535da1554bbbb1315e0bfa8`](https://github.com/ctrl-rwth/ctrl-hackathon-2026/tree/bd3247546f0c753a7535da1554bbbb1315e0bfa8).
The original PDF remains in the source checkout and is not copied into this
team repository. Keep the original, the team jury/requirement sources, Pass 6
and prior comparison records unchanged.

The source identifies itself as a sample based on BGI 694. The local
42-field source-specific catalog is an approximate internal aid requiring
review against the source, not the official form or a complete requirements
list. Its rights remain unverified: it is excluded from distribution and may
be used locally only after applicable local-use rights are verified.

An independently authored, minimal synthetic example schema and an
`ULTRON_FORM_SCHEMA` override for a compatible, locally supplied schema are
available. The public example is not the official form or a complete
mapping, and the earlier probe does not validate the example or override.
The source's inspection-column heading anomaly remains unresolved; do not
silently infer a correction. This overview is not legal advice or a standards
interpretation.

## Label-first B2 simulation workflow (partially implemented)

The team code includes `ultron.teleop` and `kit_sim` for kinematic simulation
capture, `capture_session` and `checklist` for evidence/draft contracts,
and `adaptive-draft` for local manifest-based drafting. An off-by-default
OpenAI adapter is also present; live calls require separate rights/privacy
approval and remain unverified. The rendered fixture capture-to-draft probe
used manual synthetic transcription, not real OCR or real inspection.
The target sequence is:

For CLI options, generated-manifest review and separate draft-session usage,
see [local label-first usage](LABEL_FIRST_USAGE.md).

The host-local teleop UI has no user authentication. Localhost does not
protect it from other local users; use a trusted host and non-sensitive data.

1. An operator selects and associates one target ladder in the simulation.
   The proposed keyboard/button interface is simulated kinematic positioning,
   not a walking controller or real-robot teleoperation.
2. Capture a ladder overview and a readable manufacturer/identity label and
   inspection sticker view. Retain the original image; any ROI/crop is a
   linked derivative, not an independent view. Select additional views only
   when a required visual field is missing, occluded or unreadable. There is
   no established minimum photo count.
3. Keep distinct the product/manufacturer label, inventory identity, and
   recurring inspection sticker. Preserve competing readings and dates;
   never treat a manufacturer final-control marking as the most recent
   recurring inspection. Do not infer a universal annual due date.
4. Link each proposed field value to an active-schema field and evidence image
   or crop, ladder/session association, reader method and provenance. The
   public example does not supply an official field-to-source mapping.
   Field coverage should distinguish observed, occluded, unreadable,
   conflicting, not applicable (with reason), and human-only. These are
   draft semantics; the earlier local schema and synthetic fixture path
   exist, but this does not establish a tested real reader or completed
   inspection.
5. Produce an incomplete, evidence-linked checklist draft for qualified
   human review. Photos cannot establish tactile, functional or structural
   checks, inspector qualification, authorization, a signature, or “safe to
   use.” A future draft must not silently fill or sign those fields.

The supplied fourteen ladder photos are unannotated examples, not fourteen
independent physical ladders and not a validated training or performance set.
Manual crops or fixtures do not establish OCR, barcode, VLM, visibility or
camera performance.

## Nesseller ladder checklist: high-level overview

The first page records equipment and inspection context; the following pages
address condition checks and human inspection outcomes. This is a broad
summary, not a field-by-field reproduction or a claim that the draft covers
the official form. Preserve evidence and inspection history rather than
overwriting earlier observations. Hidden condition and functional checks
remain unresolved until appropriately assessed by a qualified human.

## Other five supplied forms: inventory only

These are source-mapped for future scoping, not included in the ladder
baseline. No field population, form automation, completeness claim or
compliance interpretation is implemented by this documentation change.

| Supplied source | What it covers | Why it is later / human-context dependent |
|---|---|---|
| `checklists/geruest_pruefprotokoll.pdf` (2 pp.) | Scaffold pre-use review. | Requires site context and accountable physical inspection. |
| `checklists/staubarme_baustelle.pdf` (1 p.) | Dust-control practices. | Work practices cannot be inferred from a ladder image. |
| `checklists/erste_hilfe_brandschutz.pdf` (5 pp.) | First-aid and fire-protection readiness. | Requires site records and responsible people. |
| `checklists/einweisung_nachunternehmen.pdf` (2 pp.) | Subcontractor safety briefing. | Briefing and acknowledgement are human events. |
| `checklists/wirksamkeitskontrolle_gefaehrdungsbeurteilung.pdf` (1 p.) | Review of risk-assessment measures. | Requires site context and responsible-person judgment. |

## Safety and implementation boundary

The planned baseline is B2-only and simulation-first. Do not connect to
hardware or issue robot commands. Kinematic simulated positioning is not
validated walking. Oracle detections, fixture values and image examples are
not real perception or inspection results. Preserve hard failures, unknowns
and conflicting evidence; a confirmed hard failure dominates, while a
coverage gap remains separate from an individual ladder's draft.

Any later `AUTOMATED_PASS` remains scoped pre-screening, never safe-to-use
certification. Qualified human review and tactile/functional checks remain
required. The full-system simulation gate and independent real-world
readiness checks remain prerequisites for integrated hardware deployment;
no part of this overview authorizes that deployment.

See [source review](SOURCE_REVIEW.md), [design](DESIGN.md), [data collection](DATA_COLLECTION.md),
[Pass 7 reconciliation](RECONCILIATION_PASS7.md) and the [Pass 7 Build Bible](bible/BUILD_BIBLE_PASS7.md).
