# Security Policy

## Reporting a vulnerability

If you find a security issue in this fork, please **don't** file a public GitHub issue. Instead use GitHub's [private vulnerability reporting](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/security/advisories/new) form.

I'll acknowledge within a week and aim to release a fix or workaround within 30 days. If the issue has user-impacting consequences (e.g. a malicious peer can crash SteamVR via the IPC pipe), I'll coordinate disclosure with you.

## Threat model summary

OpenVR-WKSpaceCalibrator runs as:

- **An overlay app** (`SpaceCalibrator.exe`) — runs at user privilege, talks only to the local SteamVR session.
- **A SteamVR driver DLL** — loaded into `vrserver.exe`, runs at user privilege.

The two communicate over a local Windows named pipe (`\\.\pipe\OpenVRSpaceCalibratorDriver`) and a local shared-memory region. Both are user-scoped — neither crosses a session boundary or accepts network connections.

In scope:
- Memory-safety issues in the driver DLL (anything that crashes vrserver counts).
- IPC parsing bugs that let a malicious overlay corrupt driver state.
- Parsing bugs in the registry-stored profile (Configuration.cpp).

Out of scope (won't be treated as security issues):
- Bugs that require an attacker to already have your user account.
- Issues in upstream dependencies (OpenVR SDK, MinHook, GLFW, ImGui, ImPlot, Eigen). Report those upstream.
- "The calibration is wrong" — file a normal issue.
