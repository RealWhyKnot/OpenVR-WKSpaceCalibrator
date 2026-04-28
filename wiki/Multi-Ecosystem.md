# Multi-ecosystem calibration

Most users have two tracking systems: their HMD's native tracking, plus one set of body trackers in another system (Slime, Tundra, lighthouse, etc.). The classic Space Calibrator flow handles that case directly — calibrate one system against the other, done.

This fork adds support for **three or more** tracking systems. If you have a Quest HMD plus SlimeVR body trackers plus a Vive tracker glued to the headset, that's three coordinate spaces, and the wizard will walk you through aligning each non-HMD system to the HMD individually. All the resulting alignments run continuously in parallel, and the driver applies each one to the matching system's devices.

## When you need this

- Quest/Pico HMD + SlimeVR full-body + a single lighthouse tracker on the headset (the "glued tracker" pattern).
- Lighthouse HMD + Quest controllers + SlimeVR (rare, but exists).
- Any combination where three or more distinct OpenVR `TrackingSystemName` values are reporting devices.

If you only have two tracking systems, none of this matters — the primary calibration handles you.

If you have *one* tracking system, the wizard tells you to close the app. See [[Setup Wizard]] § "One tracking system only".

## Mental model

The HMD is always the implicit reference. Every other tracking system has at most one calibration that aligns its space to the HMD's space. So if you have N tracking systems, you end up with N − 1 calibrations:

```
HMD system (reference)  --calibration A-->  System B
                        --calibration B-->  System C
                        --calibration C-->  System D
                        ...
```

Each calibration is independent. The math doesn't try to triangulate — it doesn't care that system C is "near" system B; it just aligns C to the HMD directly. This is correct because the HMD is the only common reference.

## How it's stored

`CalibrationContext` tracks two slots:

- The **primary calibration**, in singular fields like `calibratedRotation`, `calibratedTranslation`, `targetTrackingSystem`, `referenceID`, `targetID`, etc. This carries the first non-HMD system you calibrated.
- An array of **additional calibrations** (`additionalCalibrations`), one entry per *further* non-HMD system. Each entry holds its own target tracking system, target standby device record, calibrated transform, lock mode, and — critically — its own `CalibrationCalc` instance with its own sample buffer.

The split between "primary" and "additional" is purely an implementation detail of how the existing math state machine got extended without rewriting the world. From a user perspective there's no difference: the wizard creates them in order, and continuous calibration runs them all in parallel.

## How continuous mode runs them

Each tick of the calibration loop:

1. Process the primary as before — collect a sample for ref + primary.target, run `ComputeIncremental` on the primary's `CalibrationCalc`.
2. For each additional calibration, do the same with its own ref + target IDs, against its own `CalibrationCalc`.

The math is identical in both paths. The sample buffers are independent so a noisy SlimeVR sample doesn't taint the lighthouse calibration's stats and vice versa.

After all calibrations have run, `ScanAndApplyProfile` sends one per-tracking-system fallback transform to the driver per active calibration. The driver's per-system fallback table has 8 slots, so up to 8 distinct tracking systems can be aligned simultaneously (which is far more than any real-world setup).

## How the driver applies them

Driver-side, every tracked device's pose update goes through `HandleDevicePoseUpdated`. The hook checks:

1. Does this device have an active **per-ID transform**? → apply it.
2. Otherwise, does this device's tracking system have a **per-system fallback** with `enabled=true`? → apply it.

So a SlimeVR hip tracker that doesn't have a per-ID transform applied will pick up the SlimeVR-system fallback transform automatically. Same for any lighthouse tracker that just got plugged in — it inherits the lighthouse-system fallback the moment its first pose update arrives.

This auto-adoption is the same mechanism that makes a single newly-connected tracker work in the 2-system case. The fork just multiplies it across N systems.

## Wizard flow for multi-ecosystem

See [[Setup Wizard]] for the full walkthrough. The short version:

1. Wizard detects 3+ tracking systems on launch.
2. Pops up the per-system pick screen for the first non-HMD system.
3. You pick a tracker, hit Start, calibrate, hit Done.
4. Wizard pops the next pending system, repeats.
5. After all are done (or you've skipped the rest), wizard closes.

The first system you calibrate goes into the primary slot. Each subsequent one becomes an entry in `additionalCalibrations`. They all run continuously after the wizard exits.

## Editing / removing extras

There's no UI yet for managing the `additionalCalibrations` list directly. To remove one, run the wizard again — but the cleanest path right now is to hit **Reset settings** in Advanced and re-run the wizard from scratch with only the systems you want.

A future version may add a list-of-calibrations panel in Advanced for fine-grained editing. The data model supports it; the UI just isn't built out yet.

## Profile JSON shape

```jsonc
{
  "schema_version": 3,
  // ... primary calibration fields (target_tracking_system, roll/yaw/pitch, x/y/z, ...) ...
  "additional_calibrations": [
    {
      "target_tracking_system": "lighthouse",
      "target_device": { "tracking_system": "lighthouse", "model": "...", "serial": "..." },
      "roll": 0.0, "yaw": 12.34, "pitch": 0.0,
      "x": 0.05, "y": 1.61, "z": -0.3,
      "scale": 1.0,
      "lock_mode": 2,        // 0=OFF, 1=ON, 2=AUTO
      "valid": true,
      "enabled": true
    }
  ],
  "wizard_completed": true
}
```

Old v2 profiles (no `additional_calibrations` key) load with an empty extras array — backward-compatible.

## Limitations

- **Reference is always the HMD.** You can't calibrate, say, "system B against system C" directly. The wizard always pairs each non-HMD system against the HMD.
- **One target tracker per system at calibration time.** Once continuous calibration is running, all devices in a calibrated tracking system inherit the per-system fallback automatically — but during the initial calibration you only pick one tracker as the "lead". For most users this is fine; if you have unusual rigs (e.g. one SlimeVR cluster on the body + a totally separate SlimeVR cluster on a prop), you may want to talk to us about it before assuming this works.
- **Tested on 2-system rigs.** The 3+ system code paths are exercised by the test suite and the build but haven't been live-tested against three real tracking systems. The math is per-pair so any bugs would be in orchestration (sample dispatch, fallback timing) rather than the alignment math itself.

## Source

- [src/overlay/Calibration.h](../src/overlay/Calibration.h) — `AdditionalCalibration` struct
- [src/overlay/Calibration.cpp](../src/overlay/Calibration.cpp) — per-extra tick processing in `CalibrationTick`, fallback dispatch in `ScanAndApplyProfile`
- [src/overlay/Configuration.cpp](../src/overlay/Configuration.cpp) — `additional_calibrations` JSON load/save
- [src/driver/ServerTrackedDeviceProvider.cpp](../src/driver/ServerTrackedDeviceProvider.cpp) — per-system fallback dispatch (`systemFallbacks[8]`)
