#pragma once

#include <openvr_driver.h>

#include "Protocol.h"

class ServerTrackedDeviceProvider;

// Public entry points for the IVRDriverInput hook subsystem.
//
// Architecture (2026-05-04 pivot — see memory/project_finger_smoothing_real_hook_target.md):
//
// SteamVR's lighthouse driver publishes Index Knuckles per-frame bone arrays
// by calling vr::VRDriverInput()->UpdateSkeletonComponent. vr::VRDriverInput()
// is the public OpenVR accessor that resolves to a pointer obtained through
// IVRDriverContext::GetGenericInterface("IVRDriverInput_003" or "_004"). That
// returned pointer is the public IVRDriverInput vtable, and lighthouse calls
// dispatch through its slots [5] (CreateSkeleton) and [6] (UpdateSkeleton).
//
// We intercept GetGenericInterface, match any name containing "IVRDriverInput_"
// (excluding "Internal" — see memo for why), and patch slots 5 and 6 of the
// returned vtable with MinHook. Same pattern HandOfLesser uses.
//
// All other IVRDriverInput methods are NOT hooked — the hook surface is
// intentionally narrow to minimise risk of breaking existing SpaceCalibrator
// behaviour.
//
// Default-OFF: with master_enabled=false the detour is a one-line counter-bump
// + originalFunc forward, so installing the hook has zero behaviour change for
// users who don't opt in.

namespace skeletal {

// Cache the driver pointer for SetFingerSmoothingConfig / GetFingerSmoothingConfig
// access from inside the detour. Called once from InjectHooks() in the existing
// InterfaceHookInjector.cpp during driver Init.
void Init(ServerTrackedDeviceProvider *driver);

// Tear down our skeletal hooks. Called from DisableHooks(). The IHook registry
// drops our entries on its own; this is a hook to reset cached driver-pointer
// state and clear per-hand smoothing memory so a driver-reload cycle starts
// from a clean state.
void Shutdown();

// Called from DetourGetGenericInterface in InterfaceHookInjector.cpp when the
// returned interface name matches "IVRDriverInput_*" (substring; excludes
// "Internal"). Patches the returned vtable's slots 5 (CreateSkeletonComponent)
// and 6 (UpdateSkeletonComponent) with MinHook. Idempotent — IHook::Exists
// guards the second invocation. Includes a defensive sanity check borrowed
// from BattleAxeVR/PSVR2 shim: if the iface's vtable spread between slots 0
// and 6 is implausibly large (>64 KB), refuse to install. Real vrserver
// vtables fit easily within the same .text page; a huge spread means the
// "interface" pointer is garbage (e.g. settings memory, like the Internal
// chase encountered).
void TryInstallPublicHooks(void *iface);

} // namespace skeletal
