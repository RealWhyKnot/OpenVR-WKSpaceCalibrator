# OpenVR-SpaceCalibrator (WhyKnot fork)

This is the **WhyKnot fork** of [hyblocker's OpenVR-SpaceCalibrator](https://github.com/hyblocker/OpenVR-SpaceCalibrator), which itself forks [pushrax's original](https://github.com/pushrax/OpenVR-SpaceCalibrator). Space Calibrator aligns the coordinate frames of two SteamVR tracking systems — for example a Lighthouse setup with a Quest headset, or a SteamVR HMD with Slime IMU trackers — so devices from one system show up in the right place in the other system's playspace.

This fork focuses on **continuous-calibration robustness**: auto-adopting trackers connected mid-session, watchdog escapes for stuck calibrations, and tighter math on the solver itself.

## What's different in this fork

- **Auto-adopt for newly connected trackers.** Driver-side per-tracking-system fallback transform (protocol v5) means a tracker powered on after calibration completes inherits the offset on its very first pose update, instead of floating in the wrong space until the next manual recalibration.
- **Stuck-state watchdogs.** A 50-rejection consecutive watchdog forces a sample re-collection when the 1.5x rejection gate locks in a bad fixpoint. An HMD-stall watchdog purges the buffer if the headset position freezes for ~1.5 s. Driver-side `lastPoll` resets on every transform update so a tracker returning from offline doesn't visibly jump.
- **Recalibrate on movement.** The driver-side blend toward a new offset only advances when the device is actually moving. Stationary users (e.g. lying down) don't see "phantom body shifts" when the math updates; the catch-up happens during their next natural motion, masked by the movement. Toggleable, default on. See the [Continuous Calibration](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Continuous-Calibration#recalibrate-on-movement) wiki page.
- **Math improvements.** Iterative outlier rejection on samples, weighted least-squares for the translation solve (per-pair `√min(refRotMag, targetRotMag)` weighting), 2D Kabsch SVD on yaw only, and a single-step EMA on the published transform to soften per-tick wobble that survives the gate.
- **Native prediction suppression.** Built-in equivalent of [OVR-SmoothTracking](https://yuumu.booth.pm/items/4018006) — per-device toggle that zeros velocity/acceleration at the driver level. The overlay also detects the external tool running and auto-applies the fix to calibration trackers with a visible warning. See the [Prediction Suppression](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Prediction-Suppression) wiki page.
- **Build pipeline.** A reproducible `build.ps1`, `.githooks/` for version stamping, fork CI workflows, and release zip generation. Driver and overlay are version-stamped so a mismatched pair fails fast at handshake.
- **Source-controlled wiki.** The wiki lives in `wiki/` in this repo and is mirrored to the GitHub wiki by CI, so documentation changes flow through code review like everything else.

## Quick start (end users)

1. Grab the latest release zip from the [Releases page](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/releases).
2. Make sure SteamVR is **closed**, then drop the `01spacecalibrator/` driver folder into Steam's `steamapps/common/SteamVR/drivers/`.
3. Install the [Visual C++ Redistributable](https://aka.ms/vs/17/release/vc_redist.x64.exe) if you don't already have it.
4. Run `SpaceCalibrator.exe` from the unzipped folder (or let it auto-register itself as a SteamVR overlay on first launch).
5. Start SteamVR. Open the Space Calibrator overlay from the dashboard and follow the on-screen flow. See the [wiki](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki) for the calibration walkthrough and [Continuous Calibration](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Continuous-Calibration) for the auto-aligning mode.

## Build from source

See the [Building wiki page](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Building) for the full walkthrough (toolchain, MSVC version, signing). The short version, after cloning:

```
git submodule update --init --recursive && powershell -ExecutionPolicy Bypass -File build.ps1
```

The script handles version stamping and produces both the driver and overlay binaries plus a release zip.

## Documentation

The wiki is the long-form reference; the README is just the front door.

- [Wiki home](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki) — landing page and orientation
- [Architecture](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Architecture) — how the overlay and driver fit together
- [Continuous Calibration](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Continuous-Calibration) — the math, the watchdogs, the auto-adopt path
- [Driver Protocol](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Driver-Protocol) — IPC protocol versions and message types (v5 is fork-specific)
- [Building](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Building) — submodules, `build.ps1`, version stamping
- [Troubleshooting](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Troubleshooting) — common failure modes and what to check

## Credits

- **[pushrax](https://github.com/pushrax/OpenVR-SpaceCalibrator)** — original author of OpenVR-SpaceCalibrator, including the static calibration solver and SteamVR driver architecture this fork still rests on.
- **[hyblocker](https://github.com/hyblocker/OpenVR-SpaceCalibrator)** — the fork this branch is based on; introduced continuous calibration and the modern UI.
- **All upstream contributors** — see the commit history of both upstream repositories for the long list of people who shaped this codebase.

This fork (WhyKnot) layers continuous-calibration robustness work, the v5 driver protocol, and the build/CI pipeline on top of that foundation.

## License

Same as upstream. See [LICENSE](LICENSE) for the full terms.
