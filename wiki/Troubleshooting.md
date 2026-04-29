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

Currently no. One-shot calibration is fixed at the moment you compute it; if drift accumulates, you need to recalibrate manually (open the menu and click **Recalibrate**). An earlier "passive silent-recal" experiment was removed -- detection of "good moments" doesn't actually constrain the rigid transform without varied motion, so the math fell back to overfitting whatever happened to be in the recent sample buffer. If you want live drift correction, switch to continuous mode.

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

There's currently no automatic recenter compensation. The HMD's reference frame just got re-origined and the body trackers' frame didn't, so the calibrated offset between them is now applying to the wrong relative position. Run a fresh calibration (or, if continuous mode is on, the math will catch up over the next few seconds of motion). An earlier heuristic-based recenter detector was removed because it false-fired on tracking-loss recovery and other big pose jumps, producing more disruption than it prevented.

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
