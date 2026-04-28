# OpenVR-SpaceCalibrator Wiki

OpenVR-SpaceCalibrator aligns the coordinate frames of two VR tracking systems — for example a Lighthouse setup with a Quest headset, or a SteamVR HMD with Slime IMU trackers — so devices from one system show up in the right place in the other system's playspace.

This wiki is the long-form documentation. The [README](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/blob/main/README.md) is the quick-start; the pages below go deeper.

## How it works (60 seconds)

The system has two halves:

1. **An overlay app** (`SpaceCalibrator.exe`). Runs as a SteamVR overlay with a window. Reads tracked-device poses from OpenVR, asks you to wave a target tracker around for a few seconds, and solves for the rigid transform that maps target-system coordinates → reference-system coordinates.
2. **A SteamVR driver** (`driver_01spacecalibrator.dll`). Hooks `IVRServerDriverHost::TrackedDevicePoseUpdated` and applies the calibrated offset to every pose update from the target system before SteamVR sees it.

The two halves talk over a named pipe. The math lives in the overlay; the driver is just a transform applier with smoothing. See [[Architecture]] for the full picture and [[Continuous Calibration]] for what happens when continuous mode is on.

## What this fork adds vs. upstream

- **Auto-adopt for newly connected trackers.** A tracker powered on after calibration completes inherits the offset on its very first pose update. See [[Continuous Calibration]] § Auto-adopt.
- **Stuck-state watchdogs.** Three independent watchdogs catch the cases where continuous calibration would otherwise lock into a bad fixpoint. See [[Continuous Calibration]] § Stuck-state escapes.
- **Recalibrate on movement.** The driver-side blend toward a new offset only advances while the device is actually moving. Stationary users (e.g. lying down) don't see "phantom body shifts" when the math updates. See [[Continuous Calibration]] § Recalibrate on movement.
- **Built-in prediction suppression.** Native equivalent of OVR-SmoothTracking, with auto-detection of the external tool and an in-app warning. See [[Prediction Suppression]].
- **Cross-correlation latency auto-detect.** Estimates the inter-system end-to-end latency from motion correlation; manual offset slider also still available. See [[Continuous Calibration]] § Inter-system latency offset.
- **Math improvements.** SO(3) Kabsch + Rodrigues yaw projection, IRLS with Cauchy weighting on translation, condition-ratio guards, dynamic RMS gate, single-step EMA on the published transform.

## Read these first

- **[[Architecture]]** — how the overlay and driver fit together
- **[[Continuous Calibration]]** — the math, the watchdogs, the auto-adopt path for new trackers, recalibrate-on-movement, latency
- **[[Driver Protocol]]** — the IPC protocol versions and message types

## Reference

- **[[Prediction Suppression]]** — built-in OVR-SmoothTracking equivalent; per-device velocity-zeroing for clean math
- **[[Building]]** — submodules, `build.ps1`, version stamping
- **[[Troubleshooting]]** — common failure modes and what to check

## For new contributors

Read [[Architecture]] then [[Continuous Calibration]]. The README is fine for orientation; the wiki has the load-bearing detail.
