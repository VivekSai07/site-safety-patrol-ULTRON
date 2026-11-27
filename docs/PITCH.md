# ULTRON five-minute pitch and two-minute video

0:00–0:35: Site managers need to know which ladder needs attention and why. Our robot performs visual pre-screening and creates evidence-backed records; qualified human inspection remains separate.

0:35–1:15: Explain the actual implemented loop: patrol, discover, stable asset ID, stationary evidence, deterministic result, resume. Show the real hardware, not just a diagram.

1:15–2:45: Play an uncut hardware segment: inspect an intact ladder, flag a defect/overdue label, and show an evidence gap leading to one second look or an honest review. Keep operator-intervention count visible if measured.

2:45–3:35: Show the same ladder's history across two patrols: changed condition, source images and location. Show an expected missing ladder and explicit coverage gap.

3:35–4:20: Present measured counts: recall, hard-fail-to-pass errors, review rate, retries and interventions. Separate simulation, recorded replay and live hardware. Say unmeasured wherever needed.

4:20–5:00: Explain the operational value and next validation with nesseler: official template import, reliable asset history, approved dataset and qualified human completion.

## Required ≤120-second hardware video

0–15 s: Robot, bounded site, start mission and operator status.
15–45 s: Actual navigation/discovery, stable ID and stationary capture.
45–85 s: Flagged condition or evidence recovery with source image and reason.
85–120 s: Report/history, coverage, measured results and intervention disclosure.

A live demo during the pitch is optional according to the kickoff recap; the video is mandatory. `preview.gif` is a simulation discussion artifact and cannot replace it.

## One-pager seed

ULTRON uses a Unitree B2 as a mobile ladder evidence collector. The approach combines visual discovery, depth-backed map locations, persistent identity, conservative pre-screening and structured records. A second look is limited to one approved pose when evidence is incomplete. Each inspection preserves raw evidence and history so a site manager can locate changed, flagged or missing assets and route remaining work to a qualified human. Current local implementation is a rendered MuJoCo workflow with oracle perception; hardware operation and real detector performance remain separate event integration tasks.
