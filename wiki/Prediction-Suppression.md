# Prediction Suppression

Native pose-prediction suppression with a per-tracker 0-100 strength slider. When you set a tracker's smoothness above 0, the SteamVR driver scales that device's velocity / acceleration / poseTimeOffset down toward zero on every pose update before SteamVR's predictor sees them. Two things follow:

1. **Smoother apparent motion.** SteamVR extrapolates poses forward in time using `vecVelocity` / `vecAcceleration`; cutting those scales the predictor's lookahead, which removes prediction-induced jitter at the cost of a tiny lag.
2. **Cleaner calibration math.** The fork's continuous-calibration math reads pose data through the shared-memory pose feed. With non-zero velocity, the pose-pair the calibrator solves is sampled with effective per-system timing skew, showing up in the recovered translation as motion-correlated error. Zeroing the velocity removes the skew at its source — no compensation downstream needed.

## How to use it

Open the **Prediction** tab in the overlay's Continuous Calibration window.

For each tracked device, you get a 0-100 slider:

- **0** — pose is untouched. Sharp response, raw motion. Default for everything.
- **50-75** — try this for IMU-based trackers (Slime, etc.) that feel jittery. Halves to a quarter the prediction strength.
- **100** — fully zeros velocity / acceleration / poseTimeOffset. Defeats SteamVR's predictor entirely (matches the old binary "freeze" behaviour). Smoothest, most lag.

Driver-side: `pose.vecVelocity *= (1 - smoothness/100)` and similarly for the other prediction fields. Linear scale; 50% gives you 50% of the predictor's lookahead.

The slider value sticks to a tracker by serial number, so a device that disconnects and reconnects keeps its setting.

## Hard-blocked devices

Three devices are **always** locked to 0 regardless of what's stored:

- **The HMD** — suppressing its prediction causes judder in your view, since reprojection relies on accurate forward-projected pose data.
- **The active calibration reference tracker** — the math reads its velocity to estimate inter-system latency and to drive the motion-gated blend. Zeroing it would corrupt those.
- **The active calibration target tracker** — same reason.

The UI shows these as `[locked]` rows with a tooltip explaining why. The hard-block is enforced both at the slider widget (disabled) and at `ScanAndApplyProfile` (the smoothness sent to the driver is forced to 0 for these devices regardless of map value).

## External smoothing tools

If you're running OVR-SmoothTracking (or a similar process), the Prediction tab shows a red warning banner:

> "OpenVR-SmoothTracking is running. We don't support working alongside it -- our smoothing and its smoothing will fight, and the result is unpredictable. Please close it and use the per-tracker smoothness sliders below instead."

We deliberately do **not** try to interop with external tools any more. The previous fork tried to "auto-suppress on calibration trackers when an external tool was detected" — the result was that our scaling fought the external tool's scaling and the resulting behaviour depended on the order of pose-update hooks. Better to be honest with the user: stop the external tool, use ours.

Detection: process enumeration via `EnumProcesses` + `GetModuleBaseNameW`, matched against a small list of known names plus a substring fallback (any process whose name contains both "smooth" and "track"). Adding a new known tool is a one-line table entry.

## Persistence

Per-tracker smoothness is saved to the profile JSON as `tracker_smoothness: { "<serial>": <0-100>, ... }`. Sliders at 0 aren't written (skip-if-default).

Legacy migration: profiles from before this slider existed had `suppressed_serials: ["<serial>", ...]` (the binary on/off). Each entry in that legacy list migrates to `tracker_smoothness: { serial: 100 }` on first load — preserving the old "fully suppressed" behaviour for anyone upgrading.

## What it doesn't do

- It does not affect the HMD's tracking — see "Hard-blocked devices" above. Zeroing HMD velocity would degrade reprojection.
- It does not turn off SteamVR's compositor Motion Smoothing (the HMD frame-fake feature). That's a separate, unrelated thing.
- It does not work alongside external smoothing tools — see above. Use one or the other, not both.

## Implementation pointers

- Protocol fields: `protocol::SetDeviceTransform::predictionSmoothness` (uint8 0-100) and `protocol::SetTrackingSystemFallback::predictionSmoothness` (Protocol v8).
- Driver scaling: `ServerTrackedDeviceProvider::HandleDevicePoseUpdated` in [src/driver/ServerTrackedDeviceProvider.cpp](../src/driver/ServerTrackedDeviceProvider.cpp) — multiplies the velocity / acceleration / poseTimeOffset fields by `(1 - smoothness/100)` before `shmem.SetPose`.
- Detection: `DetectExternalSmoothingTool` in [src/overlay/Calibration.cpp](../src/overlay/Calibration.cpp) — process enumeration with exact-name + substring matching.
- Hard-block: `ScanAndApplyProfile` in [src/overlay/Calibration.cpp](../src/overlay/Calibration.cpp) forces smoothness to 0 for HMD / ref / target before the IPC payload ships.
- Persistence: `tracker_smoothness` object in the profile JSON; see [src/overlay/Configuration.cpp](../src/overlay/Configuration.cpp).
- UI: `CCal_DrawPredictionSuppression` in [src/overlay/UserInterface.cpp](../src/overlay/UserInterface.cpp).
