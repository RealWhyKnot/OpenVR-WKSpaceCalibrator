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

The biggest historical pain point with continuous calibration is getting stuck in a bad fixpoint: the calibration is wrong but the 1.5x threshold gate prevents better estimates from displacing it. Three watchdogs keep this from being terminal:

- **Stuck-loop watchdog.** `ComputeIncremental` counts consecutive rejections. After 50 rejections (~25 s with a full sample buffer), `m_isValid` is forced false, the buffer is cleared, and the overlay drops to `ContinuousStandby` to re-acquire from scratch. Logs `"Continuous calibration appears stuck — recollecting samples"`.
- **HMD-stall watchdog.** If the HMD's reported position doesn't change for 30 consecutive ticks (~1.5 s), the sample buffer is purged and continuous mode demotes to standby. Without this, samples from before the stall would mix with samples from after, producing a permanently-skewed calibration.
- **Driver-side state hygiene.** Every `SetDeviceTransform` resets the driver's `lastPoll` timestamp and snaps `transform = targetTransform` on enable transitions. Without this, a tracker that went offline for 10 seconds would on its next pose update compute a saturated lerp factor and visibly jump to the new offset instead of smoothly arriving at it. The matching cleanup on disable transitions prevents stale `targetTransform` values from bleeding into the next enable.

## Output smoothing

Accepted continuous updates pass through a single-step EMA on the published transform: `α = 0.3` for the new estimate, `0.7` retained from the prior. This is skipped for first calibration (snap to the only thing we have) and for rapid-correct (the relative-pose path is supposed to snap to a known-better solution, not be smoothed). The 1.5x rejection gate already filters most bad updates; the EMA softens the per-tick wobble that survives the gate.

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
