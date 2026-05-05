# Auto Recovery

Calibration auto-recovery for catastrophic tracking events.

## What triggers it

The HMD's pose jumps 30+ cm between consecutive ticks. Common
causes:

- Quest re-localizes its room origin (sleep/wake, after taking the
  headset off and putting it back on, after a long pause).
- Headset USB reset.
- HMD physically moves while reporting valid tracking (the user
  actually teleported across the room, e.g.).

Plus all of these gates must pass:

- State is Continuous or ContinuousStandby. (Wizard active or
  one-shot mode in progress: skip.)
- Session age >= 30 seconds. (Bootstrap noise filter.)
- 30+ seconds since the last auto-recovery fired. (Throttle.)
- Body trackers in the OTHER tracking system are stable in their
  own frame. (Cross-system corroboration: a real HMD relocalization
  shows up only on the HMD's tracking system, not on the body
  trackers'. If both jumped, it's not a relocalization.)

If any gate fails, the detector logs `auto_recover_skipped` with
the failing gate flagged. No action taken.

## What it does

1. Wipe the calibration. `calibration.Clear()` purges the sample
   buffer; `CalCtx.refToTargetPose` resets to identity;
   `relativePosCalibrated` and `hasAppliedCalibrationResult` go
   false.
2. Restart continuous-cal cold. `StartContinuousCalibration()` puts
   the state machine back into Begin -> Continuous. Sample collection
   restarts from zero.
3. Snap the next per-ID transform send. The next `ScanAndApplyProfile`
   cycle sends `payload.lerp = false` so the driver assigns
   `transform := target` immediately rather than smoothly
   interpolating through the now-stale steady-state.

## What you'll see

- Body trackers briefly snap to their native (un-calibrated)
  positions. A SlimeVR pelvis tracker, e.g., shows up at wherever
  Slime says it is in raw Slime coordinates.
- Over the next ~30 seconds of natural movement, continuous-cal
  re-acquires the alignment and trackers walk back to correct.
- The motion-gate floor (Continuous Calibration) sets the still-floor
  to 90 percent for cm-scale corrections, so even if you don't
  move, recovery converges fast.
- Total user-visible window: 5 to 30 seconds depending on how much
  natural motion you produce in that time.

## What's logged

- `hmd_relocalization_detected: dx=... dy=... dz=... hmdDelta=... bsCount=N`
  -- the underlying detection event, fires whenever the 30 cm
  threshold is crossed regardless of whether recovery actually fires.
- `auto_recover_from_relocalization: hmdDelta=... -> calibration cleared, continuous-cal restarting`
  -- recovery actually fired.
- `auto_recover_skipped: hmdDelta=... magnitudeOK=N stateOK=N startupOK=N throttleOK=N postStallGrace=N`
  -- detection fired but a gate blocked the recovery action.
- `auto_recovery_snap_consumed: post-recovery profile sent with payload.lerp=false`
  -- the snap-flag took effect on the next ScanAndApplyProfile cycle.

Grep these in `%LOCALAPPDATA%\Low\SpaceCalibrator\Logs\spacecal_log.<timestamp>.txt`.

## What it doesn't catch

- HMD jumps under 30 cm. Tighter thresholds false-fire on stall
  recovery. Sub-30 cm drift is left for continuous-cal to walk
  back through normally.
- Quest origin shifts that the SLAM dampens (Quest re-anchors a
  few cm rather than across the room). Same reason.
- Slow drift over a long session. That's the continuous-cal
  motion-gate floor's job, not auto-recovery's.
- Body-tracker-side resets (a Slime IMU recalibrates while the HMD
  is stable). The detector specifically requires the HMD to be the
  one that jumped; body-tracker resets fall through to continuous-cal.

## Source

`TickHmdRelocalizationDetector` in [src/overlay/Calibration.cpp](../src/overlay/Calibration.cpp).
Constants near the top: `kRelocAutoRecoverThresholdM = 0.30`,
`kRelocAutoRecoverStartupSec = 30.0`, `kRelocAutoRecoverThrottleSec = 30.0`,
`kRelocAutoRecoverPostStallSec = 10.0`.
