# Dashboard source for the first milestone

The 19 files under `src/ultron/dashboard/` are an unchanged local import of Aravind's `feature/dashboard` implementation at commit `d68e8441552dc666269548d3f20db08a3f700e8b` in the team's repository. This integrates the actual application, not a replacement UI or a fake HTTP receiver. It does not merge other changes on that branch or publish anything remotely. Reconcile subsequent teammate changes normally rather than overwrite them.

`scripts/first-milestone.sh` installs missing dashboard dependencies into `runs/milestone-runtime/.venv`, starts the app on a free loopback port and opens it, checks models available to the configured key, captures 24 simulated views if no recording is available, performs one live assessment, and verifies the persisted dashboard record by GET. The server runs until Ctrl+C; its database/media and assessment artifacts remain in the unique run directory.

The bootstrap factory supplies an empty external-channel list so no email/chat notifications are delivered. The API key stays in the parent process; it is not passed as an argument or intentionally exported to the child server. A fresh mission and a dedicated local database prevent collisions with shared team sessions.

An end-to-end pass requires a genuine OpenAI response, successful bundle validation, successful dashboard publication and readback. Network/DNS/credential failures produce explicit unsuccessful artifacts or an early preflight failure, never a success banner. Injected test transports cannot establish a live pass. Existing prompts, capture metadata, JSON schema, adapter tests and virtual-camera simulation are separate checks. Real ladder annotation/evaluation and hardware operation remain separate tasks.

This agent session denies AF_INET sockets and cannot resolve api.openai.com. Consequently it cannot launch this TCP service or obtain live inference. The command is intended for the normal desktop terminal with network access; no sandbox controls are changed or bypassed. Dataset generation remains parked.
