# Continuous Calibration

Continuous mode is what runs while the calibration toggle is on: every tick, the overlay collects a fresh pose pair, slides the oldest 10% of the sample buffer out, and tries to compute a better calibration. Better is defined two ways depending on which sub-path runs.

## The two sub-paths

Both run inside `CalibrationCalc::ComputeIncremental`.

### 1. Static (relative-pose) calibration

Used when `lockRelativePosition` is on. Assumes the target is rigidly attached to the reference (a tracker zip-tied to the HMD, say). Computes the calibration as the average of `R · S · T⁻¹` over the sample buffer, where `S` is the estimated static target-in-reference pose. This works without any motion: pure geometry.

Accept thresholds:
- `< 0.010 m` RMS error: always accept.
- `< 0.025 m` AND `m_relativePosCalibrated`: accept (looser threshold once we've seen good data).
- `> maxRelativeErrorThreshold` (default 0.005 m): reject.

### 2. Full (motion-based) calibration

Used when motion data is available. Solves rotation and translation separately:

- **Rotation.** Differential rotation axes between sample pairs are extracted, then a 2D Kabsch SVD is run on the (x, z) components — only yaw is recovered, since the assumption is that gravity has been calibrated by the underlying tracking systems and the only freedom is rotation around the up axis.
- **Translation.** Stacked least-squares with `BDCSVD`, weighted by `√min(refRotMag, targetRotMag)` per sample pair. Pairs with tiny rotation contribute mostly noise to the translation solve and so are weighted down.

Accept thresholds:
- RMS error must beat the prior calibration's RMS by `1.5x` (`continuousCalibrationThreshold`).
- Quaternion-PCA axis variance (`ComputeAxisVariance`) must be ≥ `AxisVarianceThreshold = 0.001`. Otherwise the motion didn't span enough rotation axes.
- 2D Kabsch min/max singular value ratio must be ≥ `0.05`. Otherwise the motion was effectively single-axis (e.g. user moved on Y only). Below this, rotation is held — the prior solution stays in effect.

## State machine

`CalibrationContext::state` (see `Calibration.h`):

```
None ── trigger ──> Begin ── samples valid ──> Continuous
                       │                            │
                       │                            ├── tracking lost (1.5s) ──> ContinuousStandby
                       │                            │
                       │                            ├── 50 consecutive rejects ──> watchdog: Clear()
                       │                            │                              ContinuousStandby
                       │                            │
                       │                            └── EndContinuousCalibration ──> None
                       │
                       └── tracking bad ──> None
```

`ContinuousStandby` is the "I have a profile but I'm waiting for valid pose data again" state. Sample collection is paused; only the per-1-second `ScanAndApplyProfile` loop runs (so newly connected trackers still get adopted into the existing offset).

## Auto-adopt for newly connected trackers

When you power on a tracker mid-session, two things make sure it picks up the existing offset within ~1 second:

1. **Overlay-side scan.** `ScanAndApplyProfile` runs every 1 s in any state where a profile is loaded — including active continuous mode. It enumerates every OpenVR device, matches their tracking-system name against the calibrated `targetTrackingSystem`, and sends per-ID `SetDeviceTransform` IPC messages. Per-ID payloads are deduped (by hashing the message) so this isn't a thousand redundant writes per second; it only sends when something changed.

2. **Driver-side fallback.** The overlay also sends a per-tracking-system fallback transform (`SetTrackingSystemFallback`). The driver maintains a `systemFallbacks[name]` map; when a pose update arrives for a device whose per-ID slot is disabled, the driver looks the device's tracking system up in the map and applies the fallback through the same blend state. This covers the gap between "device sends first pose update" and "next overlay scan tick".

Together: a fresh tracker gets the offset on its very first pose update via the fallback path, and then the next scan tick promotes it to a per-ID transform. Both paths use the same blend state in `transforms[id]`, so transitions are smooth.

## Stuck-state escapes

The biggest historical pain point with continuous calibration is getting stuck in a bad fixpoint: the calibration is wrong but the 1.5x threshold gate prevents better estimates from displacing it. Four watchdogs keep this from being terminal:

- **Stuck-loop watchdog.** `ComputeIncremental` counts consecutive rejections. After 50 rejections (~25 s with a full sample buffer), `m_isValid` is forced false, the buffer is cleared, and the overlay drops to `ContinuousStandby` to re-acquire from scratch. Logs `"Continuous calibration appears stuck — recollecting samples"`.
- **Sudden-tracking-shift watchdog.** Lives in the overlay (not `CalibrationCalc`) and reacts much faster than the 50-rejection counter. Watches the recent `error_currentCal` time series: when the latest error sample is more than 5× the 30-tick rolling median for 3 consecutive ticks, the calibration is almost certainly invalid even if `CalibrationCalc` still considers it valid. Forces `Clear()` and demotes to `ContinuousStandby`. This catches catastrophic geometry shifts (a lighthouse gets bumped, a tracker goes through a portal, the user resets a slime cluster) where the 50-rejection window is too slow. Logs `"Tracking geometry shifted — restarting calibration"`.
- **HMD-stall watchdog.** If the HMD's reported position doesn't change for 30 consecutive ticks (~1.5 s), the sample buffer is purged and continuous mode demotes to standby. Without this, samples from before the stall would mix with samples from after, producing a permanently-skewed calibration.
- **Driver-side state hygiene.** Every `SetDeviceTransform` resets the driver's `lastPoll` timestamp and snaps `transform = targetTransform` on enable transitions. Without this, a tracker that went offline for 10 seconds would on its next pose update compute a saturated lerp factor and visibly jump to the new offset instead of smoothly arriving at it. The matching cleanup on disable transitions prevents stale `targetTransform` values from bleeding into the next enable.

## Output smoothing

Accepted continuous updates pass through a single-step EMA on the published transform: `α = 0.3` for the new estimate, `0.7` retained from the prior. This is skipped for first calibration (snap to the only thing we have) and for rapid-correct (the relative-pose path is supposed to snap to a known-better solution, not be smoothed). The 1.5x rejection gate already filters most bad updates; the EMA softens the per-tick wobble that survives the gate.

## Recalibrate on movement

The driver-side `BlendTransform` advances the active offset toward the latest target every tick. By default that progress is gated by per-frame motion magnitude — a stationary device gets ~zero blend progress; a moving one gets the full time-based rate. The result: a user lying still won't see calibration drift even when the math is updating; the catch-up happens during their next natural motion, hidden by the movement instead of looking like phantom body shifts.

Threshold: each pose update computes the position delta and rotation delta from the previous frame. The "fully-moving" thresholds are 5 mm position OR ~1° rotation per frame. Below those, the gate is proportional (small jitter still produces small blend progress so the offset doesn't get permanently stuck); at or above, the gate is 1 (full convergence rate).

Toggleable from the Continuous Calibration → Settings panel ("Recalibrate on movement"). Default **on**. Turning it off restores the pre-feature behaviour: the lerp advances purely on elapsed time, regardless of whether the device is moving — useful if you specifically want instantaneous corrections while stationary, or if the motion gate is stalling convergence on a tracker with unusually low natural motion.

## Inter-system latency offset (manual)

Different tracking systems have different end-to-end latencies — a wireless tracker (Slime IMU, Oculus Quest tracker) typically lags a Lighthouse reference by 10–30 ms. During quick motion this lag manifests as motion-correlated calibration error: every collected sample pair is taken at slightly different effective times, and the difference shows up as a position error proportional to velocity.

The **Target latency offset (ms)** slider in the Continuous Calibration panel exposes a per-target-system offset (range ±100 ms, default 0). When non-zero, `CollectSample` shifts the reference pose along its velocity vector at the time of sample collection so it lines up with the target's effective timestamp. The shift is computed as `(targetSampleTime - referenceSampleTime) - targetLatencyOffsetMs / 1000` seconds (positive means extrapolate the reference forward) and applied via `pose.vecPosition += vecVelocity * dt` plus an axis-angle rotation built from `vecAngularVelocity` — both in the driver-local frame, so `qWorldFromDriverRotation` handles the projection into world space downstream. If the velocity data isn't finite or is implausibly large (NaN, infinite, > 50 m/s, > 50 rad/s — typically a momentary tracking glitch), the un-extrapolated reference pose is used for that tick rather than throwing.

Default 0 produces bit-for-bit identical behaviour to before the feature existed — the conditional that gates the extrapolation never triggers.

## Inter-system latency offset (auto)

The fork also includes an auto-detector. Toggle **Latency auto-detect** in the Continuous Calibration → Settings panel. When on, the overlay maintains rolling 5-second buffers of `||vecVelocity||` for the reference and target devices, computes a discrete cross-correlation once per second when the user has been moving (RMS speed > 0.1 m/s on both signals), takes the lag at the cross-correlation peak, fits a quadratic around the peak for sub-sample resolution, and feeds the result into an EMA. The active offset (`GetActiveLatencyOffsetMs(ctx)`) returns the auto-detected value when the toggle is on, the manual `targetLatencyOffsetMs` slider value when off.

Both the toggle and the EMA value persist in the registry profile (`latency_auto_detect`, `estimated_latency_offset_ms`) so the auto-detected offset is restored on overlay restart.

## Silent drift correction (one-shot users only)

> **Disabled by default.** Toggle on via the **"Silent drift correction"** checkbox in the Settings panel (continuous mode's Basic tab, or the non-continuous Settings tab). The subsystem produced worse tracking than no-correction in real-world testing during the Phase 1+2 rollout, and its internal "Not updating: ..." rejection diagnostics leaked into the user-facing Calibration Progress popup. The flag is persisted in the profile, so a one-time opt-in sticks across launches.

When the toggle is on AND continuous calibration is off AND a profile is loaded (the typical "I calibrated once and now I'm playing" flow), a passive subsystem watches for drift and silently corrects it. None of these triggers run while continuous calibration is active — the math there already updates every tick — and none run while a calibration session is in progress (Begin/Rotation/Translation states). They only fire when `state == None`, the toggle is on, a profile is loaded, the offset is enabled, and the HMD's activity level is `UserInteraction` or `UserInteraction_Timeout` (i.e. the user is currently in VR, not just left the headset on a desk).

A single 30-second throttle is shared across every trigger so the user never sees more than one accepted recal per ~30 s. Every full-recal trigger goes through the same accept-if-better gate: a candidate must beat the current calibration's RMS by ≥10% on the silent-recal sample buffer to be applied, otherwise the existing offset is left alone.

| Trigger | What it catches | Hold | Notes |
|---|---|---|---|
| **Floor-touch Y anchor** | Calibrated target tracker's world-Y is within ±5 cm of zero AND still | 1 s | Y-only correction; rotation and X/Z untouched. Fastest of the lot — runs first, short-circuits the others on a hit. Sub-5 mm corrections suppressed (noise floor). |
| **Residual-EMA drift monitor** | Current calibration's RMS error against the rolling buffer climbs above ~3 cm sustained (τ = 30 s real seconds) | n/a (EMA itself encodes the sustained part) | Always-on background detector. Fires when calibration silently degrades. |
| **HMD wake** | `vr::VRSystem()` reports `VREvent_TrackedDeviceUserInteractionStarted` for the HMD | 5 s of post-wake activity | Catches the case where the user took the headset off, the tracking system re-origined, and the calibration is now stale on put-back-on. |
| **T-pose** | HMD upright (≤15° pitch) + both hands extended laterally (>50 cm) at HMD height (±30 cm) + all-tracker stillness | 1.5 s | The flagship trigger — VRChat users T-pose for IK calibration anyway, so this is free recalibration on every world-join. |
| **Hand-on-HMD** | At least one hand controller within 25 cm of HMD + all relevant trackers still | 1 s | Catches the headset-fit-adjustment moment. Strictly more specific than idle-pose; checked first. |
| **Idle-pose** | All relevant trackers (HMD + target + any controllers) stationary | 5 s | Long hold to distinguish "user paused" from "user is just between movements". Catches "user stopped to think" moments. |

Each trigger also pre-flights through a tracking-quality gate: if the most recent reference or target pose is anything other than `Running_OK`, the trigger is suppressed for that tick. Avoids fitting against samples that included a wireless dropout or out-of-bounds frame.

The silent-recal sample buffer is dedicated (`silentRecalCalc`, separate from the `calibration` global used by user-initiated sessions). It collects passively at ~20 Hz when a profile is loaded and idle, capped at a 30-second rolling window (~600 samples). The buffer is dropped on:

- An HMD recenter compensation event (samples in the old reference frame would corrupt any new fit).
- A change to the calibration's `referenceID` or `targetID` (different physical pair).
- A successful accepted recal (so the next trigger collects fresh post-recal motion rather than re-fitting against samples that already produced this offset).

There's also an HMD recenter compensation path that runs every tick: when the HMD's pose in the reference tracking world jumps by >30 cm or >30° between two consecutive valid frames, the same delta is applied to every stored calibrated transform (primary + every additional ecosystem). This catches Quest Home-button recenters and Oculus Link resets, where the reference-tracking-world re-origins but the body-tracker world doesn't — without this, the user's body trackers would visibly fly across the room.

## One-shot auto-completion (Rotation phase)

The one-shot path (Start Calibration -> Rotation -> ComputeOneshot) used to be a fixed-duration sample collection: wave for X seconds (X = AUTO-resolved buffer size), math runs once, pass-or-fail. With AUTO picking VERY_SLOW (500 samples = ~25 s) for noisy IMU rigs, a user could wave for the full duration and then get rejected for "motion too planar" -- a frustrating outcome the program could have prevented mid-collection.

The current behaviour gates the math on **both** conditions:
1. Buffer is at least the AUTO-resolved size (noise averaging satisfied).
2. `TranslationDiversity` AND `RotationDiversity` are both ≥ 70% (motion variety satisfied).

Below the variety threshold, the buffer rolls forward (oldest sample dropped each tick) and CollectSample keeps going. The motion-coverage progress bars in the Calibration Progress popup show both diversities in real time, so the user can see whether they're missing a translation axis or a rotation direction. Above the variety threshold, ComputeOneshot runs and -- if RMS-valid -- the calibration is saved and the popup advances.

## AUTO calibration speed (jitter staleness fix)

The AUTO calibration speed selector (FAST / SLOW / VERY_SLOW based on observed jitter) reads `Metrics::jitterRef.last()` and `Metrics::jitterTarget.last()` to bucket the right speed. Originally those metrics were pushed exactly once -- in the Begin state, when the sample buffer was still empty -- so jitter always read 0 and AUTO was permanently stuck on FAST. Fixed by pushing jitter from inside `CollectSample` after every accepted sample; the buckets now reflect live tracker quality. If you see the resolved-speed caption show "(jitter ref 0.00 mm, target 0.00 mm)" you're on a build older than this fix.

## Diagnostics

The Debug tab plots every interesting per-tick value as a 30-second rolling time series. The CSV log (enabled via `enableLogs`) writes one row per tick to `%LOCALAPPDATA%\Low\SpaceCalibrator\Logs\spacecal_log.<timestamp>.txt`. Useful columns when something goes wrong:

| Column | What it tells you |
|---|---|
| `error_currentCal` | RMS error of the active calibration. Should stay < 10 mm in good conditions. |
| `error_rawComputed` | RMS error of the most recent candidate. Compare to the active to see how close we are to displacing it. |
| `axisIndependence` | 4D quaternion-PCA variance. Drops near zero when motion is too planar. |
| `rotationConditionRatio` | 2D Kabsch min/max SV ratio. Drops near zero when motion is single-axis. |
| `consecutiveRejections` | Watchdog counter. Climbs when the gate keeps refusing updates. |
| `jitterRef`, `jitterTarget` | Per-device translation std-dev over the buffer. High values indicate tracking instability. |

Watchdog firings are written as `# [timestamp] watchdog: ...` annotation lines in the same log.
