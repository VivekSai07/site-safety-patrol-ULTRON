# Dashboard source for the first milestone

The dashboard derives from Aravind's `feature/dashboard` implementation,
including the cited `d68e8441552dc666269548d3f20db08a3f700e8b` snapshot.
It is the actual application, not a fake HTTP receiver; subsequent
reconciliation changed some documentation and CLI warnings. Use Git history
to determine the precise relationship rather than assuming it is unchanged.

`scripts/first-milestone.sh` installs missing dashboard dependencies into `runs/milestone-runtime/.venv`, starts the app on a free loopback port and opens it, checks models available to the configured key, captures 24 simulated views if no recording is available, performs one live assessment, and verifies the persisted dashboard record by GET. It sends images and prompt text externally: do not run it without separate authorization and rights/privacy review. The dashboard has no user authentication, including on localhost.

The bootstrap factory supplies an empty external-channel list so no email/chat notifications are delivered. The API key stays in the parent process; it is not passed as an argument or intentionally exported to the child server. A fresh mission and a dedicated local database prevent collisions with shared team sessions.

An end-to-end pass requires a genuine OpenAI response, successful bundle validation, successful dashboard publication and readback. Network/DNS/credential failures produce explicit unsuccessful artifacts or an early preflight failure, never a success banner. Injected test transports cannot establish a live pass. Existing prompts, capture metadata, JSON schema, adapter tests and virtual-camera simulation are separate checks. Real ladder annotation/evaluation and hardware operation remain separate tasks.

A previous session reported socket and DNS restrictions; these do not
establish the current environment or a live result. Dataset generation
remains parked.
