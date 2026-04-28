# OpenVR-SpaceCalibrator (WhyKnot fork)

This is the **WhyKnot fork** of [hyblocker's OpenVR-SpaceCalibrator](https://github.com/hyblocker/OpenVR-SpaceCalibrator), which itself forks [pushrax's original](https://github.com/pushrax/OpenVR-SpaceCalibrator). Space Calibrator aligns the coordinate frames of two SteamVR tracking systems — for example a Quest headset with SlimeVR body trackers, or a Lighthouse HMD with Quest controllers — so devices from one system show up in the right place in the other system's playspace.

This fork focuses on **getting users running quickly** (a first-run wizard does the setup for you) and on **calibration robustness** — auto-detecting rigid attachments, surviving stuck-state edge cases, and supporting setups with three or more tracking systems running in parallel.

## What's different in this fork

- **First-run setup wizard.** New installs auto-launch a wizard that detects your tracking systems and walks you through aligning each non-HMD system to your headset. No more "what do I click first" — the wizard does the device picking, kicks off continuous calibration, and saves a working profile. See the [Setup Wizard](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Setup-Wizard) wiki page.
- **Multi-ecosystem calibration.** Three or more tracking systems are aligned in parallel — each non-HMD system runs its own continuous calibration loop against the HMD as the shared reference. Useful when you have, e.g., a Quest HMD + SlimeVR body trackers + a Vive lighthouse tracker glued to your headset. See [Multi-Ecosystem](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Multi-Ecosystem).
- **Auto-detect rigid attachment.** "Lock relative position" is a tristate (Off / On / **Auto**). Auto observes the relative pose between reference and target devices and locks automatically when it sees them moving together (tracker glued to HMD, taped to a controller, etc.). Most users don't know whether their target is rigidly attached; the detector picks the right answer from observation.
- **Auto-adopt for newly connected trackers.** A tracker powered on after calibration completes inherits the offset on its very first pose update via per-tracking-system fallback transforms.
- **Stuck-state watchdogs with healthy-skip.** A 50-rejection consecutive watchdog forces sample re-collection when the math is genuinely stuck — but it now skips firing when the prior calibration is already healthy (sub-10 mm error). Symptom this fixes: previously, sub-mm calibrations would hit the rejection counter every ~25 s and the watchdog would clear a perfectly good calibration.
- **Recalibrate on movement.** The driver-side blend toward a new offset only advances while you're physically moving. Stationary users (lying down, sitting still) don't see "phantom body shifts" when the math updates; the catch-up happens during natural motion. See [Settings Reference](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Settings-Reference#recalibrate-on-movement).
- **Per-tracker prediction smoothness slider (0-100).** Replaces the old binary on/off. Trade response for jitter on a per-device basis; HMD and active calibration ref/target are hard-blocked at 0 (suppressing them would corrupt either your view or the math). External smoothing tools like OVR-SmoothTracking are detected and the user is warned to stop them — we don't try to interop. See [Prediction Suppression](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Prediction-Suppression).
- **AUTO calibration speed.** Picks Fast / Slow / Very Slow from observed jitter; the user no longer has to know what speed to pick.
- **In-app updater.** Release builds notice new GitHub releases on launch and offer a one-click upgrade with SHA-256 verification before launching the installer. Dev builds skip the check. See [In-App Updater](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/In-App-Updater).
- **Math improvements.** SO(3) Kabsch + Rodrigues yaw projection, IRLS with Cauchy weighting on translation, condition-ratio guards, rejection-floor at 5 mm so sub-mm convergence doesn't trip the watchdog, single-step EMA on the published transform.
- **Settings persistence redesigned.** Profile JSON v3: skip-if-default save (settings that match the in-code defaults aren't written) means adding a new option no longer needs migration code. Schema versioning is reserved for the rare breaking change.
- **Build pipeline.** A reproducible `build.ps1`, `quick.ps1` for fast inner-loop iteration, `deploy-test.ps1` for hot-swapping into the install directory, `.githooks/` for version stamping, fork CI workflows, and release-zip generation. Driver and overlay are version-stamped so a mismatched pair fails fast at handshake (current protocol: v8).
- **Source-controlled wiki.** The wiki lives in `wiki/` in this repo and is mirrored to the GitHub wiki by CI, so documentation changes flow through code review like everything else.

## Quick start (end users)

1. Grab the latest installer from the [Releases page](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/releases) (`OpenVR-SpaceCalibrator-<version>-Setup.exe`). Or grab the drop-in zip if you'd rather install manually.
2. Run the installer. It registers the SteamVR overlay and drops the driver into your SteamVR runtime's `drivers/` folder automatically.
3. Start SteamVR. Open the Space Calibrator overlay from the dashboard.
4. **The setup wizard launches the first time.** It detects your tracking systems and walks you through calibrating each one. ~30 seconds per non-HMD system; you click Start, move around naturally, click Done. That's it.

If you'd rather configure things yourself, hit Skip on the wizard and the existing tabs are right there. The [Settings Reference](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Settings-Reference) wiki page explains every toggle in plain language.

## Build from source

See the [Building wiki page](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Building) for the full walkthrough (toolchain, MSVC version, signing). The short version, after cloning:

```
git submodule update --init --recursive
powershell -ExecutionPolicy Bypass -File build.ps1
```

The script handles version stamping and produces both the driver and overlay binaries plus a release zip.

For inner-loop iteration:
- `quick.ps1` — wraps `build.ps1 -SkipConfigure -SkipZip` for fast rebuilds.
- `deploy-test.ps1` — builds, hot-swaps the EXE into your install dir, relaunches. Pass `-KillSteamVR` if a driver-DLL swap is needed (when the IPC protocol bumps).

## Documentation

The wiki is the long-form reference; the README is just the front door.

- [Wiki home](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki) — landing page and orientation
- [Setup Wizard](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Setup-Wizard) — what the first-run wizard does, when to re-run
- [Settings Reference](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Settings-Reference) — plain-language explanation of every UI toggle
- [Architecture](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Architecture) — how the overlay and driver fit together
- [Continuous Calibration](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Continuous-Calibration) — the math, the watchdogs, the auto-adopt path
- [Multi-Ecosystem](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Multi-Ecosystem) — calibrating 3+ tracking systems in parallel
- [Prediction Suppression](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Prediction-Suppression) — per-tracker smoothness slider, hard-blocked devices, external-tool detection
- [Driver Protocol](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Driver-Protocol) — IPC protocol versions and message types
- [In-App Updater](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/In-App-Updater) — auto-update flow, threat model
- [Building](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Building) — submodules, `build.ps1`, version stamping
- [Troubleshooting](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Troubleshooting) — common failure modes and what to check

## Credits

- **[pushrax](https://github.com/pushrax/OpenVR-SpaceCalibrator)** — original author of OpenVR-SpaceCalibrator, including the static calibration solver and SteamVR driver architecture this fork still rests on.
- **[hyblocker](https://github.com/hyblocker/OpenVR-SpaceCalibrator)** — the fork this branch is based on; introduced continuous calibration and the modern UI.
- **All upstream contributors** — see the commit history of both upstream repositories for the long list of people who shaped this codebase.

This fork (WhyKnot) layers the first-run wizard, multi-ecosystem support, the v8 driver protocol, the math-robustness fixes, and the build/CI pipeline on top of that foundation.

## License

Same as upstream. See [LICENSE](LICENSE) for the full terms.
