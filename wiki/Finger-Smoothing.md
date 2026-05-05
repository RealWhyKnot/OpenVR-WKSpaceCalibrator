# Finger Smoothing

Slerp Index Knuckles finger bones on Quest setups so they don't
jitter.

## What it solves

Index Knuckles capacitive sensors produce noisy per-bone rotations,
especially at sub-1mm finger movement. On a Lighthouse-only setup
this barely matters because the rest of the tracking is also
Lighthouse-precision. On a Quest+Lighthouse mixed setup, the
controllers' finger jitter sticks out against the Quest hands'
relative smoothness. Finger smoothing per-bone-slerps the rotations
to dampen the jitter without adding visible lag.

## Default-OFF rationale

Knuckles users on a pure-Lighthouse setup don't need this -- their
fingers already look fine, and slerp adds a small phase delay no
matter how light. So the feature ships default OFF and you opt in
via the Fingers tab.

## How to enable

1. Open Space Calibrator overlay.
2. Click the **Fingers** tab (visible in both one-shot and
   continuous-cal flows).
3. Check **Enable finger smoothing**.
4. Adjust the **Strength** slider (0..100). 0 is passthrough; 100
   is maximum slerp factor 0.05 (very heavy smoothing, slight lag).
   Default 50 is a reasonable starting point.
5. Optionally untick individual fingers from the **Active fingers**
   mask. Useful if your trigger finger feels too laggy at 50 but the
   pinky is still jittery -- raise strength and untick the trigger.

Settings persist in the saved profile.

## How to verify it's working

After enabling + restarting SteamVR (driver-side hooks load on
SteamVR startup), grep the driver log:

```
%LOCALAPPDATA%\Low\SpaceCalibrator\Logs\driver_log.<timestamp>.txt
```

Look for these lines, in this order:

```
[skeletal] IVRDriverInput_004 queried via context: iface=...
[skeletal] TryInstallPublicHooks invoked: iface=... createInRegistry=0 updateInRegistry=0
[skeletal] pre-install snapshot: vtable[5] (Create) = ..., vtable[6] (Update) = ..., spread=...
[skeletal]   Create hook installed at vtable[5]: ...
[skeletal]   Update hook installed at vtable[6]: ...
[skeletal] installed PUBLIC IVRDriverInput hooks: vtable[5]=Create, vtable[6]=Update -- waiting for first calls
```

Then once you grip your Knuckles:

```
[skeletal] FIRST CreateSkeleton call: ...
[skeletal] CreateSkeleton MAPPED handle=... -> right (path=/skeleton/hand/right ...)
[skeletal] first UpdateSkeleton on right hand: handle=... cfg{enabled=1 smoothness=50 mask=0x03ff}
```

The `cfg{enabled=1 ...}` line is the one to watch -- it confirms
your IPC config push reached the driver and the hook is using your
chosen strength.

## Implementation notes

- Hooks the **public** `IVRDriverInput` vtable at slots 5
  (`CreateSkeletonComponent`) and 6 (`UpdateSkeletonComponent`).
  An earlier approach hooked `IVRDriverInputInternal_XXX` instead;
  that interface returns a JsonCpp settings pointer rather than
  a vtable on current SteamVR runtimes, so the install silently
  no-op'd. Switched to the public vtable hook in 2026-05-04.
- Slerp factor: `0.05 + 0.95 * (1 - strength/100)`. Strength 100 ->
  factor 0.05 (very smooth). Strength 0 -> factor 1 (passthrough).
- Per-bone: each of the 31 bone rotations slerps independently
  against its previous frame. Translations pass through unchanged.
- Per-finger mask: the active-fingers bitmask (0x03FF default = all
  10 fingers enabled) gates which bones the slerp applies to. Bones
  outside the mask are passed through.

## Source

- [src/driver/SkeletalHookInjector.cpp](../src/driver/SkeletalHookInjector.cpp)
- [src/driver/SkeletalHookInjector.h](../src/driver/SkeletalHookInjector.h)
- IPC: `protocol::FingerSmoothingConfig` in
  [src/common/Protocol.h](../src/common/Protocol.h)
- Overlay UI: search for "fingerSmoothing" in
  [src/overlay/UserInterface.cpp](../src/overlay/UserInterface.cpp)
