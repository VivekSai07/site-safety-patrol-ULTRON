# Simulation-first to hardware

Do most development and validation in simulation. **Complete integrated hardware deployment requires documented full-system simulation acceptance**, followed by a separate real-world readiness decision, explicit user/operator authorization and human supervision. Simulation acceptance is scoped to what was tested; it is not hardware authorization or a safety certification. Targeted component hardware tests may occur earlier under the conditions below. This documentation change authorizes no robot connection or hardware-settings change.

## Simulation evidence and acceptance

1. **Unit and component tests:** Exercise the relevant interfaces, normal behavior and failure cases, including missing or conflicting evidence and hard-fail precedence. Record actual test results, not just intended tests.
2. **Existing MuJoCo workflow:** Run representative patrol, inspection, memory and failure/recovery cases, including the applicable standard and rendered tests in [README](../README.md#verify). Motion is kinematically prescribed; detection/type use simulation oracles and findings are scripted. This tests the simulated workflow, not walking, real perception or SDK operation.
3. **Real-recording replay where applicable:** With suitable authorized recordings and a supported replay path, exercise relevant processing on real data and record its provenance, calibration, timing and results. If recordings or replay support are unavailable, record that gap and its owner; do not claim a replay pass. Replay of captured data does not validate live sensors or physical motion.
4. **Full-system simulation:** Define acceptance cases and criteria before testing the complete integrated software baseline, then exercise the end-to-end mission and relevant failure paths. Document passing and failing cases, interfaces covered, stubs/oracles, unmodeled parts and unresolved assumptions. Required failed or unrun cases prevent acceptance. A responsible reviewer records the scoped acceptance and dispositions for remaining assumptions; safety-relevant gaps must be resolved at the separate hardware gate or deployment does not proceed.

For each gate retain an evidence record: software revision and configuration, scope, test/data provenance, results and artifacts, limitations, open risks/owners, and reviewer decision/date. Acceptance applies to the recorded baseline; re-evaluate affected gates after changes. See the historical, environment-specific [validation record](VALIDATION.md); it is not acceptance of a later integrated system.

## Early, targeted hardware component tests

These tests do **not** require full-system simulation acceptance, and must not expand into integrated deployment. Before any connection, including read-only acquisition, obtain explicit user/operator authority and independent safety validation by a responsible reviewer. This reconciliation authorizes no robot connection, hardware-setting change or hardware test.

Before each test, pass component-appropriate unit, simulation, replay or bench tests and record their results along with component/interface-specific risks and mitigations. Failed prerequisites or unmitigated risks block the test. Approve a bounded plan naming the component, objective, isolated interfaces, environment, operator, permitted actions, success/abort criteria, stop and recovery procedures, and evidence to retain. Non-motion tests must remain non-actuating, with command paths isolated. Any test that could cause motion must also satisfy **all** motion prerequisites below.

## Separate real-world readiness

Before **any hardware motion**, including an early component test, independently verify on the actual equipment using safe procedures:

- RC priority reliably overrides commanded autonomy; an independent watchdog/stop, separate from the application command path, stops on loss or stale commands.
- A functional, accessible E-stop is verified; the measured robot footprint, route, clearances and surroundings define a controlled workspace.
- A named, authorized human operator remains in control and can stop the test; access, abort conditions and recovery are controlled.

For complete integrated deployment, first retain the full-system simulation acceptance above, then separately verify the actual B2 control interface, localization, sensor calibration/timing, communications and applicable safety functions for the intended environment. Record the real-world go/no-go decision and supervise operation. A failed or unverified prerequisite means no motion; do not blindly replay simulation command traces.

Numeric thresholds and launch, network and device details are proposals until measured or verified for the actual setup. A simulated motion profile or an injected/mock SDK test is not a physical limit or integration result.

## Intelligence and inspection verdicts

Optional advanced intelligence must not gain control authority. If introduced, System One is **off by default**, offboard and read-only/audit-only, and runs only after the deterministic hard-fail scan. It has no direct or indirect route to motion, verdict or veto; audit output is for human review, not automatic feedback into protected decisions.

Verdicts are scoped prescreening, never safe-to-use certification. Hard failures dominate; missing or conflicting evidence requires review. Human sign-off and tactile/functional checks remain necessary.

The [Pass 7 Build Bible](bible/BUILD_BIBLE_PASS7.md) and [reconciliation record](RECONCILIATION_PASS7.md) are planning artifacts, not evidence of acceptance or permission to operate hardware.
