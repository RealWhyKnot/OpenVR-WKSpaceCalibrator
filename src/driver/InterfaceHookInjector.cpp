#include "Logging.h"
#include "Hooking.h"
#include "InterfaceHookInjector.h"
#include "ServerTrackedDeviceProvider.h"
#include "SkeletalHookInjector.h"

#include <cstring>

static ServerTrackedDeviceProvider *Driver = nullptr;

static Hook<void*(*)(vr::IVRDriverContext *, const char *, vr::EVRInitError *)> 
	GetGenericInterfaceHook("IVRDriverContext::GetGenericInterface");

static Hook<void(*)(vr::IVRServerDriverHost *, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook005("IVRServerDriverHost005::TrackedDevicePoseUpdated");

static Hook<void(*)(vr::IVRServerDriverHost *, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook006("IVRServerDriverHost006::TrackedDevicePoseUpdated");

static void DetourTrackedDevicePoseUpdated005(vr::IVRServerDriverHost *_this, uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	//TRACE("ServerTrackedDeviceProvider::DetourTrackedDevicePoseUpdated(%d)", unWhichDevice);
	const vr::DriverPose_t* pNewPose = &newPose; // somehow newPose is nullptr sometimes??????
	if (pNewPose && unPoseStructSize == sizeof(vr::DriverPose_t)) {
		auto pose = newPose;
		if (Driver->HandleDevicePoseUpdated(unWhichDevice, pose))
		{
			TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
		}
	} else {
		// i think this would also cause issues
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
	}
}

static void DetourTrackedDevicePoseUpdated006(vr::IVRServerDriverHost *_this, uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	//TRACE("ServerTrackedDeviceProvider::DetourTrackedDevicePoseUpdated(%d)", unWhichDevice);
	const vr::DriverPose_t* pNewPose = &newPose; // somehow newPose is nullptr sometimes??????
	if (pNewPose && unPoseStructSize == sizeof(vr::DriverPose_t)) {
		auto pose = newPose;
		if (Driver->HandleDevicePoseUpdated(unWhichDevice, pose))
		{
			TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
		}
	}
	else {
		// i think this would also cause issues in steamvr tho
		TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
	}
}

static void *DetourGetGenericInterface(vr::IVRDriverContext *_this, const char *pchInterfaceVersion, vr::EVRInitError *peError)
{
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
	else if (iface == "IVRDriverInput_003")
	{
		// Capture the public IVRDriverInput vtable so SkeletalHookInjector can
		// parse the pimpl thunks and recover the IVRDriverInputInternal vtable
		// indices (no public header for Internal exists). We do NOT install any
		// hook on the public interface — M1 proved lighthouse never calls
		// through it; this is purely structural intel.
		LOG("[skeletal] IVRDriverInput_003 queried via context: iface=%p", originalInterface);
		skeletal::CapturePublicVTable(originalInterface);
	}
	else if (iface.find("IVRDriverInputInternal") != std::string::npos)
	{
		// Substring match because the version suffix on Internal interfaces
		// (typically "_XXX") shifts across SteamVR builds. The actual hook
		// install is idempotent per session — first non-null pointer wins.
		LOG("[skeletal] %s queried via context: iface=%p", iface.c_str(), originalInterface);

		// In the user's 2026-05-02 session the SC driver loaded, IVRDriverInputInternal
		// queries arrived 4 times via this hook, but IVRDriverInput_003 was never
		// queried — install bailed every time with "Internal pointer arrived
		// before public-stub thunks were parsed" and finger smoothing was dead
		// for the session. Root cause: the SC driver itself never calls
		// vr::VRDriverInput(), so OpenVR's lazy accessor never fires the
		// public query through SC's context, but the GetGenericInterface vtable
		// patch is shared across all driver contexts in the same vrserver, so
		// we still see Internal queries from other drivers landing here.
		//
		// Fix: proactively fetch IVRDriverInput_003 ourselves when the public
		// vtable hasn't been captured yet, so the thunk parser has the data it
		// needs before TryInstallInternalHook is asked to install.
		if (originalInterface && !skeletal::IsPublicVTableCaptured())
		{
			vr::EVRInitError fetchErr = vr::VRInitError_None;
			void *publicIface = GetGenericInterfaceHook.originalFunc(_this, "IVRDriverInput_003", &fetchErr);
			LOG("[skeletal] proactive IVRDriverInput_003 fetch: iface=%p err=%d", publicIface, (int)fetchErr);
			if (publicIface) {
				skeletal::CapturePublicVTable(publicIface);
			}
		}
		skeletal::TryInstallInternalHook(originalInterface);
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
	// IHook::DestroyAll removes all our MinHook patches first, including the
	// skeletal hooks. Once that returns no detour can fire, so it's safe to
	// drop the skeletal subsystem's cached pointer.
	IHook::DestroyAll();
	skeletal::Shutdown();
	MH_Uninitialize();
}