# Architecture

Two processes, one named pipe, one shared-memory pose buffer.

```
+-----------------------------+        +------------------------------+
|       SpaceCalibrator       |  IPC   |   driver_01spacecalibrator   |
|         (overlay)           | <----> |   (SteamVR driver DLL)       |
|                             |        |                              |
|  - reads VR poses           |        |  - hooks pose updates        |
|  - solves rigid transform   |        |  - applies per-ID offsets    |
|  - sends transforms via IPC |        |  - per-system fallback map   |
+-----------------------------+        +------------------------------+
            ^                                       |
            |  shared memory ring buffer            |
            +---------------------------------------+
                       (driver pose updates)
```

## The overlay (`src/overlay/`)

The overlay is a windowed app built on GLFW + ImGui + ImPlot. It runs as a SteamVR overlay, which means it appears as a window inside the headset and the user can interact with it via controller laser pointers.

Key files:

- **`Calibration.cpp`** -- the per-tick state machine. Pulls the latest poses, drives the calibration solver, handles the standby/active state transitions, and pushes results to the driver via IPC.
- **`CalibrationCalc.cpp`** -- the math. Sample buffer, Kabsch SVD on differential-rotation axes, BDCSVD weighted least squares for translation, validation against RMS error and axis variance, the continuous-calibration accept/reject loop.
- **`VRState.cpp`** -- enumerates OpenVR devices and classifies them by tracking-system name (with special cases for Pimax Crystal HMDs/controllers that masquerade as something else).
- **`Configuration.cpp`** -- persists the calibrated profile to the Windows registry under `HKEY_CURRENT_USER_LOCAL_SETTINGS\Software\OpenVR-SpaceCalibrator\Config`. Profile is keyed by reference + target tracking-system names (e.g. "lighthouse" + "oculus").
- **`UserInterface.cpp`** -- the ImGui front-end. Device picker, calibration progress, settings.
- **`CalibrationMetrics.cpp`** -- rolling 30-second time series of every interesting per-tick value (RMS error, axis variance, jitter, computation time, condition ratio). Driven into a CSV log file for postmortem analysis when `enableLogs` is on. The graphs you see in the Debug tab are these series.

The overlay never directly modifies device poses. Everything goes through the driver.

## The driver (`src/driver/`)

The driver is a standard SteamVR driver DLL loaded by SteamVR's vrserver process. It uses [MinHook](https://github.com/TsudaKageyu/minhook) to detour `IVRServerDriverHost::TrackedDevicePoseUpdated` so it sees every pose update from every device before SteamVR processes it.

Key files:

- **`ServerTrackedDeviceProvider.cpp`** -- the heart of the driver. Maintains the per-device transform table, handles incoming IPC commands, and rewrites poses inside the hooked update path.
- **`InterfaceHookInjector.cpp`** -- installs the MinHook detour. Hooks both `IVRServerDriverHost_005` and `IVRServerDriverHost_006` since SteamVR can negotiate either.
- **`IPCServer.cpp`** -- named-pipe server. Accepts overlapped I/O so multiple clients (typically just one -- the overlay) can connect concurrently.
- **`IsometryTransform.h`** -- a thin `{Eigen::Quaterniond, Eigen::Vector3d}` pair with an `interpolateAround(lerp, target, localPoint)` helper that smoothly blends two transforms while keeping a chosen point continuous.

## The IPC pipe

Bidirectional named pipe at `\\.\pipe\OpenVRSpaceCalibratorDriver`. Message-mode, synchronous request/response. The overlay sends `Request`s, the driver sends a `Response` per request.

A handshake at connect time exchanges `protocol::Version` to refuse mismatched builds. See [[Driver Protocol]] for the full message catalogue.

## The shared-memory pose buffer

Separate from the IPC pipe. The driver writes every pose update it sees into a shared-memory ring buffer (`OpenVRSpaceCalibratorPoseMemoryV1`), and the overlay reads from it on each tick to drive its sample collection. This avoids the overhead of round-tripping every pose through the named pipe and lets the overlay see poses without depending on its own polling cadence matching SteamVR's.

The ring buffer holds 64 K samples. The overlay tracks a cursor and reads forward; if it's been away long enough that the writer has wrapped past it, it skips the gap.

## Continuous mode vs one-shot mode

- **One-shot.** User clicks "Calibrate", waves the target tracker for ~10-30 seconds (depending on FAST/SLOW/VERY_SLOW). The math solves once, the result is committed. Done.
- **Continuous.** Same math runs on a rolling sample buffer every tick. New estimates replace the prior estimate only when they beat it by a 1.5x error margin. The transform applied to the driver lerps smoothly toward the new value. See [[Continuous Calibration]] for the full state machine.

## Reading order for new contributors

1. `Calibration.cpp::CalibrationTick()` -- entry point, follow the state machine top-down.
2. `CalibrationCalc.cpp::ComputeIncremental()` -- the heart of continuous mode.
3. `ServerTrackedDeviceProvider.cpp::HandleDevicePoseUpdated()` -- what the driver actually does to your poses.
4. `Calibration.cpp::ScanAndApplyProfile()` -- how transforms get to the driver.
