# Candidate inspection_v2

The observed error was a false negative: an absent rung was not flagged without a design reference, even with an enlarged image. The matched intact fixture was not falsely flagged. Adding a verified intact reference produced one successful synthetic comparison. That does not show that prompt wording alone improves accuracy.

## Changes and activation

All seven sections receive revised common instructions: keep a confirmed visible defect despite unrelated missing views; distinguish a local normal-looking observation from completeness; lead summaries with confirmed defects or suspicious anomalies and the next evidence needed. Section two explicitly addresses rung/interface damage and unusual spacing. Ambiguous spacing stays UNKNOWN unless adequate current evidence and a verified matching design establish an absent component. No human-only restriction or safe-use constraint is relaxed.

V1 assets and default selection remain unchanged. V2 uses the same seven categories, 23 keys and response shape, with a distinct pack/schema identity and bundle digest. It is packaged with the Python distribution. Opt in with `--assessment-pack inspection_v2` on a new patrol, or `"pack": "inspection_v2"` in a new evidence manifest. The live tailer consumes the selected pack from ASSESSMENT_READY jobs; cloud calls still require explicit `--assess-openai`.

## Before changing the default

Use new, independently labelled ladder/session groups, including damaged and intact examples, ambiguous spacing, occlusion, unreadable labels and contradictory views. Keep all frames from one physical ladder/session together; reused frames and the existing synthetic pair are regression diagnostics, not held-out performance evidence. Have the human reviewer label visible defects, applicability and unobservable/functional checks without model output influencing the labels.

For each case, compare v1 and v2 with identical current images, verified references (when available), model/version/settings and schema. Evaluate reference-present and reference-absent cases separately. Preserve requests, raw responses/usage, prompt fingerprints, model IDs and actual current evidence hashes in ignored run folders. A single call per condition is exploratory evidence; measure consistency over predeclared repeats before making comparative claims. Never retry selectively until a desired answer appears.

Report per-criterion missed visible defects, false defect flags, review rate, correct evidence IDs, actionable anomaly summaries, and preservation of personal checks. Review-all behavior is not defect-detection success. Require no unsafe automated passes, no reference-only image used as proof of current condition, no prior defect erased by a later clean view, and no confirmed local defect downgraded merely because another criterion is incomplete. Retain hard failures and human review for uncertain cases.

Record the number of independent ladders/sessions and raw counts; do not publish accuracy percentages from the existing two synthetic cases. Adoption requires reviewer assessment against these predeclared criteria and documented regressions, not just successful JSON validation. No new live v2 calls or hardware tests were performed during this change.

## Local validation

The supported suite completed 218 cases: 203 passed and 15 skipped. The 23 previously identified socket/TestClient cases remain excluded by this execution environment, not passed. Targeted v2/reference/hard-fail tests passed; the built wheel includes all ten v2 asset files, and v1 asset bytes match the published baseline. Code review found no important issues. These checks do not establish model accuracy; no live v2 inference was performed.
