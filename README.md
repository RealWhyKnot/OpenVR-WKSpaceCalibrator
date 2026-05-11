# OpenVR-WKSpaceCalibrator

Run two VR tracking systems together, get their coordinate frames aligned.
Quest HMD + Vive trackers. Pico HMD + Lighthouse base stations. Knuckles
controllers on a Quest setup. Mix the gear, calibrate once, all the
trackers show up where they actually are.

This is the WhyKnot fork. Built on
[hyblocker/OpenVR-WKSpaceCalibrator](https://github.com/hyblocker/OpenVR-WKSpaceCalibrator)
which is built on [pushrax's original](https://github.com/pushrax/OpenVR-WKSpaceCalibrator).

## What it does

- Reads pose data from every connected SteamVR device.
- Solves for the 6-DoF rigid transform between any two tracking systems.
- Applies the offset inside SteamVR's driver layer, so trackers from
  the secondary system show up in the right place to every VR app.
- Re-solves continuously while you play, so small drift between the
  systems doesn't accumulate over a long session.

Typical use: HMD on one system (Quest, Pico, Index), body trackers
on another (Vive, SlimeVR via Lighthouse), and you want all of it
in one playspace for VRChat / Resonite / iRacing / etc.

## Get it

[Latest release](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/releases/latest)
ships an installer (`OpenVR-WKSpaceCalibrator-<version>-Setup.exe`) and
a portable zip. Installer registers the SteamVR overlay and drops the
driver into the SteamVR `drivers/` folder. SteamVR must restart once
after install for the driver to load.

## What this fork adds on top of upstream

- **Finger smoothing for Index Knuckles** on Quest setups. Hooks the
  public `IVRDriverInput` vtable (slots 5/6) to slerp per-bone rotations
  with 0..100 strength. Default OFF, opt-in via the Fingers tab.
- **Auto-recovery when the HMD drops tracking.** Detects a 30+ cm jump
  in HMD pose between consecutive ticks (Quest re-localization, sleep/wake,
  USB reset), wipes the calibration, restarts continuous-cal cold so
  trackers walk back to the right place over the next ~30 seconds. No
  manual reset button.
- **Smart motion-gate floor.** When you stand still, calibration drifts
  toward truth at 10% rate for sub-mm noise corrections, 50% for normal
  corrections, 90% (effectively snap) for cm-scale corrections from
  recovery events. Old behavior froze the lerp at zero when you weren't
  moving, forcing you to wave a controller before convergence resumed.
- **First-run setup wizard.** Detects your tracking systems, walks you
  through aligning each non-HMD system to your headset. Roughly 30 seconds
  per system. Skip the wizard if you'd rather configure manually.
- **Multi-ecosystem.** Three or more tracking systems in parallel, each
  with its own continuous-calibration loop against the HMD as the shared
  reference. Useful for Quest + SlimeVR + a Vive tracker on a prop.
- **AUTO calibration speed.** Picks Fast / Slow / Very Slow from observed
  jitter. You don't need to know what to set.
- **Auto-detect rigid attachment.** When a tracker is glued/taped to the
  HMD, locks the relative pose automatically; otherwise leaves it free.
- **Per-tracker prediction smoothness slider** (0..100). HMD and active
  cal devices are hard-blocked at 0 to keep the math honest.
- **In-app updater.** Release builds notice new GitHub releases on launch
  and offer a one-click upgrade with SHA-256 verification.
- **92 unit tests + compile-time pins** on the math, profile schema,
  state-machine transitions, and the recovery decision functions.

## Quick start

1. Grab the installer from
   [Releases](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/releases),
   run it. Restart SteamVR.
2. Open Space Calibrator from the SteamVR dashboard. The setup wizard
   pops up on first launch.
3. Pick your reference tracking system (usually your HMD's). Pick the
   target system (the one whose trackers are showing up wrong).
4. Hit Start. Move around naturally for ~30 seconds. The wizard tells
   you when it's done.
5. Body trackers should now show up in the right place. If they drift
   over time, leave continuous calibration on (the default) and they
   correct themselves while you move.

The Wiki has step-by-step screenshots:
[Setup Wizard](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/wiki/Setup-Wizard).

## What it doesn't do

- Doesn't fix bad room setup. If your Lighthouse base stations have line-of-sight
  problems or your Quest guardian is drifting, calibration can only do so much.
  It aligns two systems against each other; it doesn't fix the underlying tracking.
- Doesn't pre-detect every wedge case. The math can converge to a self-consistent
  fit that's physically wrong (rare on healthy hardware). If your trackers look
  off after a long session, leave continuous calibration running -- it walks
  back to truth on its own as you move. Sudden tracking jumps (HMD reset, big
  controller bump) are caught by the auto-recovery detector without you doing
  anything. There's no one-click rescue button mid-session by design: when
  calibration is broken, your controllers are exactly the tool that's broken,
  so any UI that requires aiming them is a trap.
- Quest+Lighthouse is the most-tested combination. Pico, Varjo, and
  WMR setups have been reported working but receive less testing.
- Driver swaps need Steam fully closed (not just SteamVR). The installer
  handles this for you; manual swaps via `quick.ps1 -DeployDriver` close
  Steam automatically.

## Build from source

```
git submodule update --init --recursive
powershell -ExecutionPolicy Bypass -File build.ps1
```

Full toolchain notes (MSVC, signing, version stamping) are in the
[Building](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/wiki/Building)
wiki page.

For inner-loop iteration: `quick.ps1 -Install` rebuilds and hot-swaps
the overlay. `quick.ps1 -DeployDriver` does the full driver+overlay
swap (closes Steam, copies the DLL, relaunches Steam).

## Documentation

Full docs in the [Wiki](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/wiki).
Notable pages:

- [Setup Wizard](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/wiki/Setup-Wizard)
- [Continuous Calibration](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/wiki/Continuous-Calibration)
- [Settings Reference](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/wiki/Settings-Reference)
- [Troubleshooting](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/wiki/Troubleshooting)
- [Architecture](https://github.com/RealWhyKnot/OpenVR-WKSpaceCalibrator/wiki/Architecture) (for contributors)

## Credits

[pushrax](https://github.com/pushrax/OpenVR-WKSpaceCalibrator) wrote the
original calibration solver and SteamVR driver. [hyblocker](https://github.com/hyblocker/OpenVR-WKSpaceCalibrator)
added continuous calibration and the modern UI. This fork adds the
items in the list above.

## License

GNU General Public License v3.0, see [LICENSE](LICENSE). Project copyright lines and third-party attributions in [NOTICE](NOTICE). Earlier upstream contributions from Justin Li and Hyblocker were originally MIT-licensed and remain available under MIT terms from their origin repos; this fork's combined work is GPL-3.0 going forward.
