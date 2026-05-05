#include "Logging.h"
#include "Hooking.h"
#include "InterfaceHookInjector.h"
#include "ServerTrackedDeviceProvider.h"
#include "SkeletalHookInjector.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

static ServerTrackedDeviceProvider *Driver = nullptr;

// In-flight detour counter. Every detour body in this file AND in
// SkeletalHookInjector.cpp brackets itself with DetourScope which inc/decs
// this counter. DrainInFlightDetours spins until it hits zero so DisableHooks
// can guarantee no detour is mid-execution when it returns.
static std::atomic<int> g_inFlightDetours{0};

namespace InterfaceHooks {

void EnterDetour() noexcept
{
	g_inFlightDetours.fetch_add(1, std::memory_order_acq_rel);
}

void ExitDetour() noexcept
{
	g_inFlightDetours.fetch_sub(1, std::memory_order_acq_rel);
}

void DrainInFlightDetours() noexcept
{
	// Detours are µs-scale; in practice this loop exits in a single yield or
	// less. The 500ms cap is a watchdog: if a detour is genuinely stuck (e.g.
	// blocked on a SteamVR mutex held by a thread we're racing) we'd rather
	// log and proceed than hang shutdown forever and force-kill the driver.
	using clock = std::chrono::steady_clock;
	const auto deadline = clock::now() + std::chrono::milliseconds(500);
	while (g_inFlightDetours.load(std::memory_order_acquire) > 0) {
		if (clock::now() > deadline) {
			LOG("DrainInFlightDetours: timeout with %d in-flight callers; proceeding with shutdown anyway",
				g_inFlightDetours.load(std::memory_order_acquire));
			return;
		}
		std::this_thread::yield();
	}
}

} // namespace InterfaceHooks

static Hook<void*(*)(vr::IVRDriverContext *, const char *, vr::EVRInitError *)> 
	GetGenericInterfaceHook("IVRDriverContext::GetGenericInterface");

static Hook<void(*)(vr::IVRServerDriverHost *, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook005("IVRServerDriverHost005::TrackedDevicePoseUpdated");

static Hook<void(*)(vr::IVRServerDriverHost *, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook006("IVRServerDriverHost006::TrackedDevicePoseUpdated");

static void DetourTrackedDevicePoseUpdated005(vr::IVRServerDriverHost *_this, uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	InterfaceHooks::DetourScope _scope;
	// Inherited from upstream: only run our HandleDevicePoseUpdated when the
	// pose struct size matches the version we expect. If a future SteamVR
	// runtime extends the struct, fall through to the original without
	// touching the pose ourselves rather than risk reading off the end of
	// a smaller-than-expected struct or writing past the end of a larger
	// one. The cleanup vs upstream: dropped a `pNewPose = &newPose; if
	// (pNewPose ...)` indirection that was a dead null-check — references
	// can't be null in well-formed code, and address-of-reference does
	// nothing the size check doesn't already.
	if (unPoseStructSize == sizeof(vr::DriverPose_t)) {
		auto pose = newPose;
		if (Driver->HandleDevicePoseUpdated(unWhichDevice, pose))
		{
			TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
		}
	} else {
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
	}
}

static void DetourTrackedDevicePoseUpdated006(vr::IVRServerDriverHost *_this, uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	InterfaceHooks::DetourScope _scope;
	// See DetourTrackedDevicePoseUpdated005 above for the rationale —
	// same cleanup applied here.
	if (unPoseStructSize == sizeof(vr::DriverPose_t)) {
		auto pose = newPose;
		if (Driver->HandleDevicePoseUpdated(unWhichDevice, pose))
		{
			TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
		}
	}
	else {
		TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
	}
}

static void *DetourGetGenericInterface(vr::IVRDriverContext *_this, const char *pchInterfaceVersion, vr::EVRInitError *peError)
{
	InterfaceHooks::DetourScope _scope;
	TRACE("ServerTrackedDeviceProvider::DetourGetGenericInterface(%s)", pchInterfaceVersion);
	auto originalInterface = GetGenericInterfaceHook.originalFunc(_this, pchInterfaceVersion, peError);

	std::string iface(pchInterfaceVersion);
	if (iface == "IVRServerDriverHost_005")
	{
		if (!IHook::Exists(TrackedDevicePoseUpdatedHook005.name))
		{
			TrackedDevicePoseUpdatedHook005.CreateHookInObjectVTable(originalInterface, 1, &DetourTrackedDevicePoseUpdated005);
			IHook::Register(&TrackedDevicePoseUpdatedHook005);
		}
	}
	else if (iface == "IVRServerDriverHost_006")
	{
		if (!IHook::Exists(TrackedDevicePoseUpdatedHook006.name))
		{
			TrackedDevicePoseUpdatedHook006.CreateHookInObjectVTable(originalInterface, 1, &DetourTrackedDevicePoseUpdated006);
			IHook::Register(&TrackedDevicePoseUpdatedHook006);
		}
	}
	else if (iface.find("IVRDriverInput_") != std::string::npos
	         && iface.find("Internal") == std::string::npos)
	{
		// Public IVRDriverInput query — substring match catches both
		// IVRDriverInput_003 (SC's bundled SDK; user's runtime currently
		// publishes this) and IVRDriverInput_004 (current OpenVR master;
		// adds Pose+EyeTracking at slots 7-10 but keeps Skeleton creates+
		// updates at slots 5/6). Internal exclusion guards against
		// IVRDriverInputInternal_XXX (a 2-method eye-tracking interface,
		// unrelated to skeletal data — see memory file).
		//
		// The lighthouse driver calls vr::VRDriverInput()->UpdateSkeleton-
		// Component() which is a virtual dispatch through the vtable
		// returned here; patching slot 6 catches the call. Same approach
		// HandOfLesser uses in production.
		LOG("[skeletal] %s queried via context: iface=%p", iface.c_str(), originalInterface);
		skeletal::TryInstallPublicHooks(originalInterface);
	}

	return originalInterface;
}

void InjectHooks(ServerTrackedDeviceProvider *driver, vr::IVRDriverContext *pDriverContext)
{
	Driver = driver;

	auto err = MH_Initialize();
	if (err == MH_OK)
	{
		GetGenericInterfaceHook.CreateHookInObjectVTable(pDriverContext, 0, &DetourGetGenericInterface);
		IHook::Register(&GetGenericInterfaceHook);
		// Skeletal-hook subsystem: caches the driver pointer so its detours
		// can read finger-smoothing config. Must run AFTER MinHook init and
		// BEFORE any GetGenericInterface query for IVRDriverInput_003 /
		// IVRDriverInputInternal_*, both of which arrive through the
		// detour we just registered.
		skeletal::Init(driver);
	}
	else
	{
		LOG("MH_Initialize error: %s", MH_StatusToString(err));
	}
}

void DisableHooks()
{
	// 1. Remove the MinHook patches. After DestroyAll returns, no NEW callers
	//    can enter our detours via the hooked vtable slot (the slot is
	//    restored to point at the original function).
	// 2. Drain in-flight callers. MinHook does NOT wait for already-executing
	//    detour bodies to return — without this, the pose-update detour
	//    firing at ~kHz across all tracked devices has a window where a
	//    detour body is mid-execution while we tear down state below it. At
	//    SteamVR's driver-unload time that race is fatal: the DLL gets
	//    unmapped while a thread is still inside our code.
	// 3. Drop the skeletal subsystem's cached driver pointer + per-hand
	//    state. Safe now because (1)+(2) guarantee no skeletal detour body
	//    is in flight.
	// 4. Tear down MinHook itself.
	IHook::DestroyAll();
	InterfaceHooks::DrainInFlightDetours();
	skeletal::Shutdown();
	MH_Uninitialize();
}