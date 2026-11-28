# Attachments for the next Pass 7 review

Use files from one recorded team commit. The team and personal workspaces have
separate histories; the personal repository records a team submodule pointer.
Record `git rev-parse HEAD` in each repository and any uncommitted changes
separately. Do not assume a branch name identifies a fixed snapshot.

## Essential team documents

Paths are relative to the team repository:

1. `README.md`, `AGENTS.md` and `CLAUDE.md`
2. `docs/bible/BUILD_BIBLE_PASS7.md` and
   `docs/bible/BUILD_BIBLE_PASS7_REVIEW_PROMPT.md`
3. `docs/RECONCILIATION_PASS7.md`, `docs/EVENT_HANDOFF.md`,
   `docs/DESIGN.md` and `docs/INTEGRATION.md`
4. `docs/LABEL_FIRST_BASELINE.md` and `docs/LABEL_FIRST_USAGE.md`
5. `docs/DETECTION_WORLD_USAGE.md`, `docs/DETECTION_WORLD_SOURCES.md`,
   `docs/DETECTION_EVALUATION.md` and `docs/DETECTION_LABEL_BASELINE.md`
6. `docs/API_PROMPTS.md`, `docs/DASHBOARD.md` and
   `docs/dashboard_nav_goal.md`
7. `docs/SOURCE_REVIEW.md`, `docs/ORGANIZER_ALIGNMENT.md`,
   `docs/VALIDATION.md`, `docs/SIM_TO_HARDWARE.md` and
   `docs/DATA_COLLECTION.md`

If attachment limits are tight, provide a rights-reviewed text pack with
paths and headings intact. Do not omit validation and safety limitations
while retaining architecture claims.

## Implementation evidence

Prefer read access to the recorded repository commit over a workspace upload.
Include relevant code, tests, schemas, exact test commands, summarized results,
dependency versions and a branch inventory. Identify whether each result is a
test, a limited observation, an unverified integration or a proposal. If a
report or source is unavailable, state that explicitly.

The public schema is a minimal synthetic example. The local source-specific
catalog is excluded from distribution and is not the official form or proof
that all source requirements have been implemented. Supply either the
catalog or an approved source form separately only when sharing rights have
been confirmed.

## Optional comparison sources

After confirming sharing rights, include the preserved Pass 6 Bible, organizer
jury and requirements, the relevant Nesseler form, and manufacturer or SDK
documentation where exact source verification is needed. Otherwise provide
permitted links and page references. Historical research is comparison
material, not evidence that an interface or hardware result exists.

## Do not attach by default

Do not upload raw photos, reference PDFs, private research, extracted
figures, model weights, databases, generated runs, browser profiles,
credentials or identifiable personal data. Share only separately approved
summaries and provenance. Local availability or code-push permission does not
grant rights to upload these materials to a model provider.

For each supplied attachment record its path or title, source, revision,
role, and sharing status; record missing items and unresolved rights.
