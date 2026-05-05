# Driver Protocol

The overlay and driver communicate over a Windows named pipe at `\\.\pipe\OpenVRSpaceCalibratorDriver`. The protocol is fully synchronous: every `Request` from the overlay receives exactly one `Response`. Messages are framed by Windows pipe message mode, not length-prefixed.

The pose feed is a separate channel: a shared-memory ring buffer at `OpenVRSpaceCalibratorPoseMemoryV1` written by the driver on every device pose update and read by the overlay each tick.

## Versioning

`protocol::Version` (see `src/common/Protocol.h`) is exchanged on connect via `RequestHandshake`. A mismatch throws on the overlay side with the message *"Incorrect driver version installed, try reinstalling Space Calibrator."*

| Version | Changes |
|---|---|
| 4 | Pre-fork. Per-ID transforms only. |
| 5 | Added `SetTrackingSystemFallback` request. Added `target_system[32]` field to `SetDeviceTransform`. Driver-side per-system fallback map enables auto-adopt for trackers connected after calibration completes. |
| 6 | Added `freezePrediction` field to `SetDeviceTransform` and `SetTrackingSystemFallback`. Driver zeroes velocity/acceleration on flagged devices, replacing OVR-SmoothTracking. See [[Prediction Suppression]]. |
| 7 | Added `recalibrateOnMovement` field to `SetDeviceTransform` and `SetTrackingSystemFallback`. When set, the driver's `BlendTransform` gates lerp progress on detected per-frame motion magnitude -- stationary devices don't see calibration drift. See [Recalibrate on movement](Continuous-Calibration#recalibrate-on-movement). |
| 8 | Replaced `freezePrediction` (`bool`) with `predictionSmoothness` (`uint8_t`, 0-100). Old field was a binary on/off; new field is a strength knob -- driver scales velocity / acceleration / poseTimeOffset by `(1 - smoothness/100)` instead of zeroing them when a bool was true. `smoothness=100` reproduces the old freeze behaviour; `smoothness=0` leaves the pose untouched. See [[Prediction Suppression]]. |
| 9 | Added `RequestSetFingerSmoothing` request type and `FingerSmoothingConfig` union member. Driver hooks `IVRDriverInputInternal::UpdateSkeletonComponent` (a private Valve interface reachable via `IVRDriverContext::GetGenericInterface`; vtable indices recovered by parsing the public `IVRDriverInput_003` pimpl thunks at install time) and per-bone slerps Index Knuckles bone arrays toward incoming poses by a strength factor. Default-OFF -- users who don't open the new "Fingers" tab see byte-identical wire format and zero behaviour change. The `Request` union sizeof is unchanged (`FingerSmoothingConfig` is much smaller than `SetDeviceTransform`); the version bump is purely to force paired overlay+driver reinstall so a partial upgrade fails the handshake loudly instead of silently ignoring the new request type. |

The overlay and driver MUST be built from the same source tree -- there's no negotiation, just a hard equality check. Mixing builds with different `protocol::Version` results in handshake failure on overlay startup.

## Request types

### `RequestHandshake`

Exchanged once per connection. Driver responds with `ResponseHandshake` carrying its `protocol.version`.

### `RequestSetDeviceTransform`

Per-ID transform application. Payload `protocol::SetDeviceTransform`:

| Field | Type | Notes |
|---|---|---|
| `openVRID` | `uint32_t` | OpenVR device index. |
| `enabled` | `bool` | When false, the per-ID slot is disabled and the driver consults the per-system fallback. |
| `updateTranslation`, `updateRotation`, `updateScale` | `bool` × 3 | If false, that field of the existing transform is preserved. |
| `translation` | `vr::HmdVector3d_t` | World-space, in meters. |
| `rotation` | `vr::HmdQuaternion_t` | World-space. |
| `scale` | `double` | Pose-position scale factor (typically 1.0). |
| `lerp` | `bool` | When false, the driver snaps `transform = targetTransform`. When true, the driver smoothly interpolates. |
| `quash` | `bool` | When true, the device is hidden by being moved 9001 m above the origin. Used for the active calibration target during continuous mode if `quashTargetInContinuous` is enabled. |
| `target_system` | `char[32]` | The device's tracking-system name. Lets the driver associate this slot with the per-system fallback map without querying VR properties. Empty if unknown. |
| `predictionSmoothness` (v8+) | `uint8_t` | 0-100 strength knob for native pose-prediction suppression. The driver scales `vecVelocity` / `vecAcceleration` / `vecAngularVelocity` / `vecAngularAcceleration` / `poseTimeOffset` by `(1 - smoothness/100)` before the pose ships. `0` = pose untouched (off). `100` = all those fields zeroed (defeats SteamVR's extrapolation entirely; equivalent to the v6/v7 `freezePrediction = true` behaviour). Replaced the v6 `freezePrediction` bool -- a binary toggle wasn't expressive enough for users with mildly jittery IMU trackers who want partial suppression. See [[Prediction Suppression]]. |
| `recalibrateOnMovement` (v7+) | `bool` | When true, the driver gates `BlendTransform`'s lerp progress on detected per-frame motion magnitude (5 mm position OR ~1° rotation = full convergence rate; below those, scaled proportional). A stationary user doesn't see calibration drift; the catch-up happens during natural motion. See [Recalibrate on movement](Continuous-Calibration#recalibrate-on-movement). |

Side effects on receipt (every call):
- `lastPoll` is reset to the current time. Without this, a device that went offline for 10 seconds would saturate the lerp on the next pose update.
- `currentRate` is reset to `TINY`.
- On enable transition (`!wasEnabled && enabled`): `transform` is forcibly set to `targetTransform` (snap, no ramp-in).
- On disable transition (`wasEnabled && !enabled`): `targetTransform = transform` (drop pending lerp target).
- `deviceSystem[id]` is updated from `target_system`.
- `fallbackActive` is cleared.
- If `recalibrateOnMovement` transitions from on to off, `blendMotionInitialized` is reset so a future re-enable doesn't see a stale prior pose.

### `RequestSetTrackingSystemFallback` (v5+)

Per-tracking-system fallback transform. Applied to any device whose tracking-system matches `system_name` and whose per-ID slot is disabled. Lets newly connected trackers inherit the calibrated offset on their first pose update.

Payload `protocol::SetTrackingSystemFallback`:

| Field | Type | Notes |
|---|---|---|
| `system_name` | `char[32]` | Tracking-system name (e.g. `"lighthouse"`, `"oculus"`). |
| `enabled` | `bool` | When false, the fallback is removed and any slots currently following it are reset. |
| `translation`, `rotation`, `scale` | as `SetDeviceTransform` | The transform to apply. |
| `predictionSmoothness` (v8+) | `uint8_t` | Same semantics as `SetDeviceTransform::predictionSmoothness`. Applied to every device that picks up this fallback. The overlay always sends `0` here regardless of per-tracker settings, because the fallback applies to ANY device of that system that doesn't have an active per-ID transform -- including a freshly-connected reference or target tracker, which we hard-block from suppression. The per-ID path carries the real per-tracker smoothness value. |
| `recalibrateOnMovement` (v7+) | `bool` | Same semantics as `SetDeviceTransform::recalibrateOnMovement`. Newly-connected matching-system trackers handled by the fallback path get the motion-gated blend automatically. |

When the fallback first activates for a slot, the driver snaps `transform = fb.transform` and sets `fallbackActive = true`. Subsequent pose updates lerp normally (though there's nothing to lerp toward unless the fallback itself is updated).

### `RequestSetAlignmentSpeedParams`

Tunes the per-tick blend rate. Payload `protocol::AlignmentSpeedParams` describes three thresholds (TINY/SMALL/LARGE) for translation and rotation deltas, plus the corresponding lerp speeds. The driver classifies each tick's required correction by magnitude and uses the matching speed. Larger required corrections lerp faster.

### `RequestDebugOffset`

Applies a random ±25 cm + identity-rotation perturbation to all devices for testing. Used only by the developer-mode random-offset button in the UI.

## Overlay-side dedupe

The overlay maintains per-ID and per-system caches of the last payload sent (see `g_lastApplied`, `g_lastFallback` in `Calibration.cpp`). Identical payloads aren't re-sent. This is what makes the 1 Hz `ScanAndApplyProfile` loop cheap even when running during active continuous calibration -- most ticks send zero IPC messages.

The cache is invalidated when:
- The calibrated `targetTrackingSystem` changes (a different profile was loaded).
- `validProfile` toggles (the profile was cleared or freshly committed).
- A device's serial number changes for a given OpenVR ID (someone repaired a tracker, SteamVR reassigned the slot). On serial mismatch the overlay forces a `ResetAndDisableOffsets` before any new transform takes effect, so the driver doesn't apply the old device's offset to the new device.

## Pose feed (shared memory)

`protocol::DriverPoseShmem` lives at `OpenVRSpaceCalibratorPoseMemoryV1`. Layout:

```
struct ShmemData {
    std::atomic<uint64_t> index;     // monotonic write counter
    AugmentedPose poses[64 * 1024];  // ring buffer
};

struct AugmentedPose {
    LARGE_INTEGER sample_time;
    int deviceId;
    vr::DriverPose_t pose;
};
```

The driver writes every pose update from every device. The overlay reads forward from a private cursor. If the driver has lapped the cursor (gap > 32 K samples), the overlay snaps the cursor forward by half the buffer size -- losing samples is preferable to reading garbage.

`DriverPoseShmem::GetPose` filters by `poseIsValid && result == TrackingResult_Running_OK`, so IMU-only and extrapolated poses don't poison the sample buffer. (Added in commit `4db3c77`.)
