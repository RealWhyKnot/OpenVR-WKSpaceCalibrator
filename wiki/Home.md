# OpenVR-SpaceCalibrator Wiki

OpenVR-SpaceCalibrator aligns the coordinate frames of two VR tracking systems — for example a Lighthouse setup with a Quest headset, or a SteamVR HMD with Slime IMU trackers — so devices from one system show up in the right place in the other system's playspace.

This wiki is the long-form documentation. The [README](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/blob/main/README.md) is the quick-start; the pages below go deeper.

## How it works (60 seconds)

The system has two halves:

1. **An overlay app** (`SpaceCalibrator.exe`). Runs as a SteamVR overlay with a window. Reads tracked-device poses from OpenVR, asks you to wave a target tracker around for a few seconds, and solves for the rigid transform that maps target-system coordinates → reference-system coordinates.
2. **A SteamVR driver** (`driver_01spacecalibrator.dll`). Hooks `IVRServerDriverHost::TrackedDevicePoseUpdated` and applies the calibrated offset to every pose update from the target system before SteamVR sees it.

The two halves talk over a named pipe. The math lives in the overlay; the driver is just a transform applier with smoothing. See [[Architecture]] for the full picture and [[Continuous Calibration]] for what happens when continuous mode is on.

## What this fork adds vs. upstream

- **First-run setup wizard.** New installs auto-launch a wizard that detects your tracking systems and walks you through aligning each non-HMD system to your headset. See [[Setup Wizard]].
- **Multi-ecosystem calibration.** Three or more tracking systems are aligned in parallel, each with its own continuous-calibration loop running against the HMD as the shared reference. See [[Multi-Ecosystem]].
- **Auto-detect rigid attachment.** "Lock relative position" is a tristate (Off / On / Auto). Auto observes the relative pose between reference and target devices and locks automatically when they're rigidly attached (tracker glued to HMD, taped to a controller, etc.). See [[Settings Reference]] § Lock relative position.
- **Auto-adopt for newly connected trackers.** A tracker powered on after calibration completes inherits the offset on its very first pose update. See [[Continuous Calibration]] § Auto-adopt.
- **Stuck-state watchdogs.** Three independent watchdogs catch the cases where continuous calibration would otherwise lock into a bad fixpoint. See [[Continuous Calibration]] § Stuck-state escapes.
- **Recalibrate on movement.** The driver-side blend toward a new offset only advances while the device is actually moving. Stationary users (e.g. lying down) don't see "phantom body shifts" when the math updates. See [[Settings Reference]] § Recalibrate on movement.
- **Per-tracker prediction smoothness slider.** 0-100 strength per tracker, replacing the old binary on/off. The HMD and active calibration reference / target are hard-blocked at 0 (suppressing them would corrupt your view or the math). External smoothing tools like OVR-SmoothTracking are detected and the user is warned to stop them — we don't try to interop. See [[Prediction Suppression]].
- **Cross-correlation latency auto-detect.** Estimates the inter-system end-to-end latency from motion correlation; manual offset slider also still available. See [[Continuous Calibration]] § Inter-system latency offset.
- **Math improvements.** SO(3) Kabsch + Rodrigues yaw projection, IRLS with Cauchy weighting on translation, condition-ratio guards, rejection-floor at 5 mm so sub-mm convergence doesn't trip the watchdog, single-step EMA on the published transform.
- **AUTO calibration speed.** Picks Fast / Slow / Very Slow from observed jitter; the user no longer has to know what speed to pick. See [[Settings Reference]] § Calibration speed.
- **One-shot auto-completion.** Calibration finishes when the motion-coverage bars both hit 70% AND the buffer has the AUTO-resolved sample count -- no fixed timer. The popup shows live progress so you know what variety the math is still missing. See [Continuous Calibration § One-shot auto-completion](Continuous-Calibration#one-shot-auto-completion-rotation-phase).
- **Non-continuous UI parity.** One-shot users get the same tabbed depth (Settings / Advanced / Prediction / Logs) as continuous-mode users -- previous versions hid all of those behind clicking "Continuous Calibration" first. Debug logs, jitter threshold, lock mode, etc. are all reachable without entering continuous mode.
- **In-app updater.** Release builds notice new GitHub releases on launch and offer a one-click upgrade with SHA-256 verification before launching the installer. See [[In-App Updater]].

## Read these first

- **[[Setup Wizard]]** — the first-run guided setup; what each step does and when to re-run
- **[[Settings Reference]]** — plain-language explanation of every user-facing toggle
- **[[Architecture]]** — how the overlay and driver fit together
- **[[Continuous Calibration]]** — the math, the watchdogs, the auto-adopt path for new trackers, recalibrate-on-movement, latency

## Reference

- **[[Multi-Ecosystem]]** — calibrating 3+ tracking systems in parallel
- **[[Prediction Suppression]]** — per-tracker smoothness slider, hard-blocked devices, external-tool detection
- **[[Driver Protocol]]** — the IPC protocol versions and message types
- **[[In-App Updater]]** — the auto-update flow, threat model, file layout
- **[[Building]]** — submodules, `build.ps1`, version stamping
- **[[Troubleshooting]]** — common failure modes and what to check

## For new contributors

Read [[Architecture]] then [[Continuous Calibration]]. The README is fine for orientation; the wiki has the load-bearing detail.
