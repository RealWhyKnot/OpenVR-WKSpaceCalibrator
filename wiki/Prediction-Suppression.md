# Prediction Suppression

Native equivalent of [OVR-SmoothTracking](https://yuumu.booth.pm/items/4018006). When enabled on a device, the SteamVR driver zeroes that device's `vecVelocity` / `vecAcceleration` / `vecAngularVelocity` / `vecAngularAcceleration` / `poseTimeOffset` fields on every pose update. Two things follow:

1. SteamVR's pose extrapolation has nothing to extrapolate — the device is reported "stationary between samples." For a 1000 Hz Lighthouse-tracked device that's invisible. For lower-rate trackers (Slime IMU, Quest-link wireless) it can introduce micro-stutter, traded for the next bullet.
2. External smoothing tools that scale the same fields (OVR-SmoothTracking is the prevalent one — it's prediction *suppression*, not low-pass filtering) become no-ops on the affected device. They keep working on devices you didn't enable.

## Why this exists

The fork's continuous-calibration math reads pose data through the shared-memory pose feed. When SteamVR's predictor extrapolates a pose forward in time using a non-zero `vecVelocity`, the pose-pair the calibrator solves is sampled with effective per-system timing skew. That skew shows up in the recovered translation as motion-correlated error. Zeroing the velocity field at the driver level removes the skew at its source — no compensation downstream needed.

## How to use it

Open the **Prediction** tab in the overlay's Continuous Calibration window.

- **Per-device list.** Every connected non-HMD device is listed by render model + tracking system + serial. Toggle suppression on the trackers you care about. The setting is keyed by serial, so a tracker that disconnects and reconnects (different OpenVR ID, same physical device) keeps its setting.
- **Auto-apply on calibration trackers when an external smoothing tool is detected.** Default on. When the overlay's process scan detects a known external tool (currently `OVR-SmoothTracking`) running, it automatically applies suppression to the calibration *reference* and *target* trackers and shows a yellow banner explaining what it did. The user can disable the external tool at this point — the fork's suppression covers it.
- **Status banner.** When detection fires the Prediction tab shows the detected tool and the action taken. Disabling auto-apply switches the banner to a red warning that the external tool's smoothing may be disturbing calibration math.

## Persistence

The suppressed-serial list and the auto-apply toggle are saved to the registry profile (`HKEY_CURRENT_USER_LOCAL_SETTINGS\Software\OpenVR-SpaceCalibrator\Config`, schema v1). Profiles from before this feature existed load with an empty suppressed-serial list and auto-apply on (the default).

## What it doesn't do

- It does not replace the smoothing tool's *display* — if you keep OVR-SmoothTracking running, its UI will still show sliders and "smoothing enabled" indicators for suppressed devices. That's harmless; the values arrive at the driver hook with velocity already zeroed by us, so OVR-SmoothTracking has nothing to act on.
- It does not affect the HMD. Zeroing HMD velocity would degrade reprojection. The per-device list explicitly excludes the HMD.
- It does not turn off SteamVR's compositor Motion Smoothing (the HMD frame-fake feature). That's a separate, unrelated thing.

## Implementation pointers

- Protocol field: `protocol::SetDeviceTransform::freezePrediction` and `protocol::SetTrackingSystemFallback::freezePrediction` (Protocol v6).
- Driver gate: `ServerTrackedDeviceProvider::HandleDevicePoseUpdated` in `src/driver/ServerTrackedDeviceProvider.cpp` — zeros the velocity fields before `shmem.SetPose` so the overlay's sample collection sees the same frozen data.
- Detection: `DetectExternalSmoothingTool` in `src/overlay/Calibration.cpp` — process enumeration via `EnumProcesses` + `GetModuleBaseNameW`, matched against a static table of known executable names. Adding a new tool to detect is a one-line table entry.
- Persistence: `suppressed_serials` array + `auto_suppress_on_external_tool` bool in the profile JSON; see `src/overlay/Configuration.cpp`.
- UI: `CCal_DrawPredictionSuppression` in `src/overlay/UserInterface.cpp`.
