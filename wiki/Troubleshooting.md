# Troubleshooting

## Calibration won't complete

**Symptom:** the progress bar fills but the result is rejected ("Low-quality calibration result" or similar).

Likely causes:

- **Insufficient motion variety.** The math needs rotation in at least two axes — pure-translation or single-axis rotation gives a degenerate solve. Wave the target tracker so it sees yaw and pitch (or yaw and roll) movement, not just side-to-side. The Debug tab's Axis Variance and rotation-condition-ratio plots will tell you which dimension you're short on.
- **Bad tracking on either device.** Check the per-device pose-validity panel in the overlay. If `result != Running_OK` (i.e. the device is in extrapolation mode because of occlusion or HMD-lifted), the math rejects those samples but if a majority are bad, the buffer doesn't fill. Make sure both devices have a clean line of sight to their tracking volume.
- **Jitter too high.** The Reference Jitter and Target Jitter plots show per-axis std-dev. Above `jitterThreshold` (default 3 mm), calibration won't even start. Common cause: a base station that's loose, a Slime IMU that hasn't finished settling, or a tracker that's too close to a base station and getting interference.

## Continuous calibration is "stuck"

**Symptom:** the offset is wrong but the overlay won't accept any new estimate. Restarting the program fixes it.

This is exactly what the watchdogs are designed to recover from automatically. As of the WhyKnot fork:

- After ~25 seconds of continuous rejections, the stuck-loop watchdog clears the sample buffer and demotes to `ContinuousStandby`. Look for `"Continuous calibration appears stuck — recollecting samples"` in the overlay log.
- After ~1.5 seconds of stalled HMD tracking, the HMD-stall watchdog purges the buffer.

If you're seeing genuinely stuck behavior (waited > 30 seconds, the watchdog never fired), enable CSV logging via the Debug tab and capture a few minutes of `consecutiveRejections`, `error_currentCal`, `error_rawComputed`. Open an issue with the log file attached.

## I'm not using continuous mode — does anything correct drift for me?

**Symptom:** you ran a one-shot calibration once and don't want continuous mode running, but tracking has drifted over the session.

There's an experimental passive silent-recal subsystem (Phase 1+2) that watches for natural moments to silently re-fit your calibration: T-poses, idle stillness, hand-on-HMD adjustment, HMD wake events, residual-EMA drift, and floor-touch Y-anchor. It is **off by default** because in current testing it produces worse tracking than no correction in some setups. Opt in via the **Silent drift correction** checkbox in the Settings panel (one-shot Settings tab or continuous Basic). See [Continuous Calibration § Silent drift correction](Continuous-Calibration#silent-drift-correction-one-shot-users-only) for the trigger list.

If you've enabled it and it isn't firing when you expect:

1. **Confirm the toggle is on AND a profile is loaded and enabled.** The mode pill at the top should show `[FIXED OFFSET ACTIVE]`. `[NO PROFILE]` means there's nothing to refine. The silent-recal driver also no-ops while continuous mode (`[LIVE]`) is running -- continuous already updates every tick.
2. **Confirm you're actually wearing the headset.** The whole subsystem suppresses itself when SteamVR reports the HMD as Idle / Standby / Idle_Timeout — i.e. the user has taken the headset off. Put it back on and give the activity-level a few seconds to flip to UserInteraction.
3. **Throttle.** At most one accepted recal fires per ~30 seconds across all triggers. If you just had a recal accept, the next one won't fire for 30 s.
4. **Acceptance gate.** A candidate must beat the current calibration's RMS by ≥10% to be applied. If the user has been moving very little or in a degenerate pattern (only side-to-side, etc.), there may not be enough motion variety in the buffer to find a meaningfully-better fit. Move around for ~30 s before expecting a fire.
5. **Tracking-quality gate.** The triggers are suppressed when the most recent reference or target pose is anything other than `Running_OK` — wireless dropouts, out-of-bounds frames, etc. defer the trigger.

If you'd rather see calibration corrections in real-time as you move, switch to continuous mode. The silent subsystem is specifically for users who don't want continuous mode running but want some passive maintenance.

## My body visibly drifts while I'm lying still

**Symptom:** while stationary (lying down, sitting still in a meditation app, etc.) the calibrated trackers slowly walk to a new position, looking like phantom body movement. Most often noticed shortly after the calibration math has updated or after a watchdog fired.

This is the failure mode the **Recalibrate on movement** option (default on) was added to fix. With it on, the driver only advances the lerp toward a new offset proportional to detected per-frame motion — a stationary device barely moves at all. The catch-up happens during your next natural motion, hidden by the movement.

If you're still seeing drift while still:

1. Open the Continuous Calibration → Settings panel and confirm **Recalibrate on movement** is checked. (Toggling it off then on re-establishes a clean baseline pose for the gate.)
2. Confirm both your driver and overlay are protocol v7 or newer — the option is a no-op against an older driver. Reinstall the latest release if not.
3. Check the overlay log for repeated stuck-loop watchdog firings (`"Continuous calibration appears stuck"`) — if the math is constantly resetting, the gate works but every reset re-introduces a target the gate has to catch up to. Underlying instability is causing the resets; investigate that first via `error_currentCal` and `consecutiveRejections` in the Debug tab.

To intentionally restore the pre-feature instant-blend behavior (e.g. you specifically want stationary corrections), uncheck the option.

## Body trackers fly across the room after a recenter

**Symptom:** you press the Quest Home button to recenter your view (or your headset's tracking otherwise re-origins) and your body trackers visibly teleport to the wrong place.

The HMD recenter compensation path can detect this automatically -- but it's **part of the experimental silent-recal subsystem and is off by default**. Toggle on via the **Silent drift correction** checkbox in Settings to enable it. When on, an HMD pose jump >30 cm or >30° in a single tick (faster than legitimate motion) is interpreted as a driver-side re-origin, and the same delta is applied to every stored calibrated transform so body trackers stay aligned with the user's body.

Compensation is suppressed during active calibration (Begin/Rotation/Translation/Continuous) and outside `state == None`. It also requires the previous HMD frame to have been valid AND recent (< 0.5 s).

If recenter compensation isn't doing what you expect, capture a debug log (the Logs tab is the easiest path) and attach it to a bug report -- this is exactly the kind of failure mode the team is debugging the silent-recal subsystem against.

## Newly connected tracker doesn't pick up the offset

**Symptom:** a tracker powered on after calibration shows up in the wrong position.

As of the WhyKnot fork, this should resolve within ~1 second of the device sending its first pose update — the driver-side per-tracking-system fallback applies immediately, and the overlay's 1 Hz scan promotes it to a per-ID transform on the next tick.

If it doesn't:

1. Confirm the new tracker's tracking-system name matches the calibrated `targetTrackingSystem`. Slime trackers report `"slime"`, Lighthouse devices report `"lighthouse"`, etc. A Pimax Crystal HMD masquerading as `"aapvr"` has special handling in `ScanAndApplyProfile`; see [[Architecture]] for the disambiguation logic.
2. Check the overlay log for `Reset of stale serial` messages — if the new tracker reused an OpenVR ID that previously belonged to a different physical device, the overlay should detect the serial change and force a clean disable before applying any new transform.
3. Verify the IPC pipe is alive. Look for `"IPC client connected"` in the SteamVR log file (`vrserver.txt`) right after starting the overlay.

## Driver doesn't load

**Symptom:** SteamVR starts, the overlay starts, but no offsets are ever applied.

- Open SteamVR's web console (`http://localhost:8998` while SteamVR is running) and look for `driver_01spacecalibrator` in the loaded-drivers list. If it's missing, SteamVR didn't find or didn't accept the DLL.
- Verify the layout: `<drivers>/driver_01spacecalibrator/bin/win64/driver_01spacecalibrator.dll` and `<drivers>/driver_01spacecalibrator/driver.vrdrivermanifest`.
- Check `vrserver.txt` for load errors. A missing dependency (e.g. wrong VS runtime) shows up here.

## "Incorrect driver version installed"

**Symptom:** overlay startup throws this error.

The overlay and driver were built from different commits (different `protocol::Version`). Reinstall both — the release zip ships them together, so just extract the latest zip into your SteamVR `drivers/` directory and re-run the overlay from the same zip.

## Unicode in profile path

**Symptom:** profile won't load if your Windows username contains non-ASCII characters.

Fixed in commit `e752ea5`. If you're seeing this on a build before that, update.

## Where to look for logs

- **Overlay in-app log.** Bottom of the overlay window. Cleared when calibration starts; shows recent state-machine transitions and watchdog firings.
- **CSV metric log.** `%LOCALAPPDATA%\Low\SpaceCalibrator\Logs\spacecal_log.<timestamp>.txt`. Enable via the Debug tab toggle. One row per tick; rotates to a new file per overlay session. Old logs > 24h are auto-deleted.
- **SteamVR vrserver log.** `<Steam>\logs\vrserver.txt`. Driver-side errors and our `printf` debug lines (Pimax universe-switch tracing, etc.).
