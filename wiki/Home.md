# OpenVR-WKSpaceCalibrator Wiki

Long-form documentation for Space Calibrator. The
[README](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/blob/main/README.md)
is the quick-start; pages here go deeper.

## What it does in 60 seconds

Two halves talking over a named pipe:

1. **Overlay app** (`SpaceCalibrator.exe`). SteamVR overlay with a
   visible window. Reads tracked-device poses from OpenVR. Asks you
   to wave a tracker around for ~30 seconds. Solves for the rigid
   transform that maps target-system coordinates into reference-system
   coordinates.
2. **SteamVR driver** (`driver_01spacecalibrator.dll`). Hooks
   `IVRServerDriverHost::TrackedDevicePoseUpdated`. Applies the
   calibrated offset to every pose update from the target system
   before SteamVR sees it.

The math lives in the overlay. The driver is a transform applier
with smoothing. See [[Architecture]] for the full picture.

## What this fork adds vs. upstream

- **Finger smoothing for Index Knuckles** on Quest setups. Hooks the
  public `IVRDriverInput` vtable. Default OFF, opt-in via the Fingers
  tab. See [[Finger Smoothing]].
- **Auto-recovery when the HMD drops tracking.** Detects 30+ cm HMD
  jumps (Quest re-localization, sleep/wake, USB reset) and silently
  restarts continuous-cal cold. See [[Auto Recovery]].
- **Smart motion-gate floor.** Calibration drifts toward truth at
  10/50/90 percent based on correction size, even when you're standing
  still. No more waving a controller to unfreeze convergence. See
  [[Continuous Calibration]] section "Motion-gate floor".
- **First-run setup wizard.** Detects your tracking systems, walks
  you through aligning each one. ~30 seconds per system. See
  [[Setup Wizard]].
- **Multi-ecosystem.** Three or more tracking systems in parallel,
  each with its own continuous-cal loop against the HMD as shared
  reference. See [[Multi-Ecosystem]].
- **AUTO calibration speed.** Picks Fast / Slow / Very Slow from
  observed jitter. See [[Settings Reference]] section "Calibration
  speed".
- **Auto-detect rigid attachment.** Lock-relative-position is a
  tristate (Off / On / Auto). Auto detects "tracker glued to HMD"
  patterns and locks automatically. See [[Settings Reference]]
  section "Lock relative position".
- **Per-tracker prediction smoothness slider** (0..100). HMD and
  active calibration devices hard-blocked at 0 to keep the math
  honest. See [[Prediction Suppression]].
- **In-app updater.** Release builds notice new GitHub releases
  on launch and offer one-click upgrade with SHA-256 verification.
  See [[In-App Updater]].
- **92 unit tests + compile-time pins** on the math, profile schema,
  state-machine transitions, and recovery decisions.

## Read these first

- **[[Setup Wizard]]** -- first-run guided setup
- **[[Settings Reference]]** -- every user-facing toggle, plain language
- **[[Continuous Calibration]]** -- the math, the watchdogs, motion-gate
- **[[Auto Recovery]]** -- what triggers it, what to expect

## Reference

- **[[Architecture]]** -- driver / overlay split
- **[[Multi-Ecosystem]]** -- 3+ tracking systems
- **[[Prediction Suppression]]** -- per-tracker smoothness slider
- **[[Finger Smoothing]]** -- per-bone slerp for Knuckles on Quest
- **[[Driver Protocol]]** -- IPC versions and message types
- **[[In-App Updater]]** -- auto-update flow
- **[[Building]]** -- toolchain, build.ps1, version stamping
- **[[Troubleshooting]]** -- actual failure modes, what to do

## For contributors

Read [[Architecture]] then [[Continuous Calibration]]. The README is
fine for orientation; the wiki has the load-bearing detail.
