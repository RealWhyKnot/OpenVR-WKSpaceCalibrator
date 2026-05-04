#pragma once

#include <openvr_driver.h>

#include "Protocol.h"

class ServerTrackedDeviceProvider;

// Public entry points for the IVRDriverInputInternal hook subsystem.
//
// Architecture (proven 2026-05-02 in VRChat):
//
// SteamVR's lighthouse driver publishes Index Knuckles per-frame bone arrays
// via IVRDriverInputInternal::UpdateSkeletonComponent (a private interface
// distinct from the public IVRDriverInput_003 every other driver sees).
// The Internal interface is reachable via IVRDriverContext::GetGenericInterface
// with name "IVRDriverInputInternal_XXX". The vtable indices for its methods
// are recovered by parsing the public IVRDriverInput's pimpl thunks at install
// time — no header for IVRDriverInputInternal exists, so we recover the layout
// empirically from the public-stub forwarders.
//
// This subsystem hooks vtable[6] (UpdateSkeletonComponent) on whatever Internal
// interface pointer comes through GetGenericInterface, applies per-bone slerp
// smoothing gated by a config block the overlay pushes via IPC, and forwards
// to the original. All other Internal vtable methods are NOT hooked — the hook
// surface is intentionally narrow to minimise risk of breaking existing
// SpaceCalibrator behaviour.
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
// returned interface name matches "IVRDriverInputInternal*". Idempotent — only
// the first non-null pointer actually installs the MinHook patch; subsequent
// calls (e.g. for fresh driver contexts) no-op.
void TryInstallInternalHook(void *iface);

// Called when IVRDriverInput_003 is queried — used to capture the public
// vtable so we can parse its pimpl thunks and recover the Internal vtable
// indices for free, before any IVRDriverInputInternal pointer ever appears.
// No hooks are installed against the public interface (we proved in M1 that
// it never receives lighthouse traffic); this is purely for thunk-parse.
void CapturePublicVTable(void *iface);

// True once CapturePublicVTable has successfully parsed the public vtable
// thunks. InterfaceHookInjector consults this when an IVRDriverInputInternal
// query lands — if the public interface hasn't been queried through SC's
// context yet (the SC driver doesn't call VRDriverInput itself, so OpenVR's
// lazy accessor never fires), we proactively fetch it ourselves so the
// Internal install isn't permanently stuck.
bool IsPublicVTableCaptured();

} // namespace skeletal
