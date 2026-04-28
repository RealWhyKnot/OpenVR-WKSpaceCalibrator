# Driver Protocol

The overlay and driver communicate over a Windows named pipe at `\\.\pipe\OpenVRSpaceCalibratorDriver`. The protocol is fully synchronous: every `Request` from the overlay receives exactly one `Response`. Messages are framed by Windows pipe message mode, not length-prefixed.

The pose feed is a separate channel: a shared-memory ring buffer at `OpenVRSpaceCalibratorPoseMemoryV1` written by the driver on every device pose update and read by the overlay each tick.

## Versioning

`protocol::Version` (see `src/common/Protocol.h`) is exchanged on connect via `RequestHandshake`. A mismatch throws on the overlay side with the message *"Incorrect driver version installed, try reinstalling Space Calibrator."*

| Version | Changes |
|---|---|
| 4 | Pre-fork. Per-ID transforms only. |
| 5 | Added `SetTrackingSystemFallback` request. Added `target_system[32]` field to `SetDeviceTransform`. Driver-side per-system fallback map enables auto-adopt for trackers connected after calibration completes. |

The overlay and driver MUST be built from the same source tree — there's no negotiation, just a hard equality check. Mixing a v4 driver with a v5 overlay (or vice versa) results in handshake failure on overlay startup.

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

Side effects on receipt (every call):
- `lastPoll` is reset to the current time. Without this, a device that went offline for 10 seconds would saturate the lerp on the next pose update.
- `currentRate` is reset to `TINY`.
- On enable transition (`!wasEnabled && enabled`): `transform` is forcibly set to `targetTransform` (snap, no ramp-in).
- On disable transition (`wasEnabled && !enabled`): `targetTransform = transform` (drop pending lerp target).
- `deviceSystem[id]` is updated from `target_system`.
- `fallbackActive` is cleared.

### `RequestSetTrackingSystemFallback` (v5+)

Per-tracking-system fallback transform. Applied to any device whose tracking-system matches `system_name` and whose per-ID slot is disabled. Lets newly connected trackers inherit the calibrated offset on their first pose update.

Payload `protocol::SetTrackingSystemFallback`:

| Field | Type | Notes |
|---|---|---|
| `system_name` | `char[32]` | Tracking-system name (e.g. `"lighthouse"`, `"oculus"`). |
| `enabled` | `bool` | When false, the fallback is removed and any slots currently following it are reset. |
| `translation`, `rotation`, `scale` | as `SetDeviceTransform` | The transform to apply. |

When the fallback first activates for a slot, the driver snaps `transform = fb.transform` and sets `fallbackActive = true`. Subsequent pose updates lerp normally (though there's nothing to lerp toward unless the fallback itself is updated).

### `RequestSetAlignmentSpeedParams`

Tunes the per-tick blend rate. Payload `protocol::AlignmentSpeedParams` describes three thresholds (TINY/SMALL/LARGE) for translation and rotation deltas, plus the corresponding lerp speeds. The driver classifies each tick's required correction by magnitude and uses the matching speed. Larger required corrections lerp faster.

### `RequestDebugOffset`

Applies a random ±25 cm + identity-rotation perturbation to all devices for testing. Used only by the developer-mode random-offset button in the UI.

## Overlay-side dedupe

The overlay maintains per-ID and per-system caches of the last payload sent (see `g_lastApplied`, `g_lastFallback` in `Calibration.cpp`). Identical payloads aren't re-sent. This is what makes the 1 Hz `ScanAndApplyProfile` loop cheap even when running during active continuous calibration — most ticks send zero IPC messages.

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

The driver writes every pose update from every device. The overlay reads forward from a private cursor. If the driver has lapped the cursor (gap > 32 K samples), the overlay snaps the cursor forward by half the buffer size — losing samples is preferable to reading garbage.

`DriverPoseShmem::GetPose` filters by `poseIsValid && result == TrackingResult_Running_OK`, so IMU-only and extrapolated poses don't poison the sample buffer. (Added in commit `4db3c77`.)
