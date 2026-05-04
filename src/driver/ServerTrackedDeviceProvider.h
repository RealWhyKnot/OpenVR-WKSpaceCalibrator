#pragma once

#define EIGEN_MPL2_ONLY

#include "IPCServer.h"
#include "Protocol.h"
#include "IsometryTransform.h"

#include <Eigen/Dense>

#include <openvr_driver.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>


class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	////// Start vr::IServerTrackedDeviceProvider functions

	/** initializes the driver. This will be called before any other methods are called. */
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;

	/** cleans up the driver right before it is unloaded */
	virtual void Cleanup() override;

	/** Returns the version of the ITrackedDeviceServerDriver interface used by this driver */
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }

	/** Allows the driver do to some work in the main loop of the server. */
	virtual void RunFrame() { }

	/** Returns true if the driver wants to block Standby mode. */
	virtual bool ShouldBlockStandbyMode() { return false; }

	/** Called when the system is entering Standby mode. The driver should switch itself into whatever sort of low-power
	* state it has. */
	virtual void EnterStandby() { }

	/** Called when the system is leaving Standby mode. The driver should switch itself back to
	full operation. */
	virtual void LeaveStandby() { }

	////// End vr::IServerTrackedDeviceProvider functions

	ServerTrackedDeviceProvider() : server(this) { }
	void SetDeviceTransform(const protocol::SetDeviceTransform &newTransform);
	void SetTrackingSystemFallback(const protocol::SetTrackingSystemFallback &newFallback);
	bool HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);
	void HandleApplyRandomOffset();
	void HandleSetAlignmentSpeedParams(const protocol::AlignmentSpeedParams params) {
		alignmentSpeedParams = params;
	}

	// Finger-smoothing config cache. Written by IPCServer when the overlay
	// pushes a new config (rare — only on UI changes). Read by the
	// IVRDriverInputInternal::UpdateSkeletonComponent detour at hand-update
	// rate (~340 Hz/hand). Held under its OWN mutex distinct from
	// stateMutex so finger updates can never block the pose-update path,
	// and vice-versa.
	void SetFingerSmoothingConfig(const protocol::FingerSmoothingConfig &cfg);
	protocol::FingerSmoothingConfig GetFingerSmoothingConfig() const;

private:
	IPCServer server;
	protocol::DriverPoseShmem shmem;

	enum DeltaSize {
		TINY,
		SMALL,
		LARGE
	};

	struct DeviceTransform
	{
		bool enabled = false;
		bool quash = false;
		IsoTransform transform, targetTransform;
		double scale = 1.0;
		LARGE_INTEGER lastPoll{};
		DeltaSize currentRate = DeltaSize::TINY;
		// True when the slot's transform/targetTransform are tracking a tracking-
		// system fallback rather than an overlay-supplied per-ID value. Used to
		// snap on the first activation of fallback.
		bool fallbackActive = false;
		// When true, every pose update for this device gets its velocity /
		// Prediction-suppression strength on a 0..100 scale. The pose's
		// velocity / acceleration / poseTimeOffset fields are scaled by
		// (1 - smoothness/100) before any other processing. 0 = pose
		// untouched. 100 = fields zeroed (matches the old binary "freeze"
		// behaviour). The overlay enforces the hard block on HMD / ref /
		// target, so by the time we see a non-zero value here the sender
		// has already vetted that suppressing this device is safe.
		uint8_t predictionSmoothness = 0;

		// When true, BlendTransform's lerp toward targetTransform only advances
		// proportional to detected per-frame motion magnitude. A stationary user
		// (lying down, sitting still) sees no calibration drift even when the
		// math has updated — the catch-up happens during the user's next motion,
		// hidden by the natural movement instead of looking like a phantom body
		// shift. Default false; the overlay enables it per-device when the
		// recalibrateOnMovement profile setting is on.
		bool recalibrateOnMovement = false;

		// Previous-frame world-space pose, captured each call to BlendTransform
		// when recalibrateOnMovement is on. Used to compute per-frame motion
		// magnitude that gates the blend. blendMotionInitialized is reset to
		// false whenever recalibrateOnMovement transitions off so the first
		// frame after re-enable doesn't see a giant stale delta.
		Eigen::Vector3d lastBlendWorldPos = Eigen::Vector3d::Zero();
		Eigen::Quaterniond lastBlendWorldRot = Eigen::Quaterniond::Identity();
		bool blendMotionInitialized = false;
	};

	struct FallbackTransform
	{
		bool enabled = false;
		IsoTransform transform;
		double scale = 1.0;
		uint8_t predictionSmoothness = 0;
		bool recalibrateOnMovement = false;
	};

	// Tracking-system fallback slot. Stored as a fixed-size flat array indexed
	// by linear scan + memcmp on the system_name buffer (typical case is 2-3
	// systems and we never see more than a handful). Cache-line-friendly and
	// avoids the std::string + std::unordered_map heap traffic that otherwise
	// happens on every pose update for every disabled slot.
	struct FallbackSlot
	{
		// NUL-padded; full buffer is compared so a partial match against a
		// shorter name doesn't false-positive.
		char system_name[protocol::MaxTrackingSystemNameLen];
		FallbackTransform tf;
		bool occupied = false;
	};

	// Lookup state for the lazy VRProperties() tracking-system query in
	// HandleDevicePoseUpdated. The query has to fire from the pose-hook thread
	// because the overlay may not have called SetDeviceTransform for this slot
	// yet; without throttling it gets re-issued on every single pose update for
	// every empty slot up to k_unMaxTrackedDeviceCount.
	enum class LookupState : uint8_t {
		NotTried,
		Cached,
		Failed,
	};

	DeviceTransform transforms[vr::k_unMaxTrackedDeviceCount];
	Eigen::Vector3d debugTransform;
	Eigen::Quaterniond debugRotation;

	// Guards transforms[], deviceSystem[], systemFallbacks[] and the lookup
	// state against concurrent access. The IPC server thread mutates these via
	// SetDeviceTransform / SetTrackingSystemFallback while the pose-hook thread
	// reads them in HandleDevicePoseUpdated. std::string assignments + flat-
	// array writes during a concurrent read are UB without synchronisation.
	// Held briefly: hook thread copies the slot's transform + fallback target
	// into stack locals under the lock, then releases it for the math/blend.
	mutable std::mutex stateMutex;

	// Per-ID tracking-system name, populated from SetDeviceTransform messages.
	// Empty if the overlay hasn't told us yet for this slot.
	std::array<std::string, vr::k_unMaxTrackedDeviceCount> deviceSystem;

	// Per-ID state of the lazy VRProperties() tracking-system query. NotTried
	// means we haven't asked yet; Cached means deviceSystem[id] is populated;
	// Failed means the last query came up empty and we should back off.
	LookupState lookupState[vr::k_unMaxTrackedDeviceCount] = {};

	// Timestamp of the last failed VRProperties() query for each slot. Failed
	// queries are retried at most once per second so an unoccupied slot doesn't
	// hammer the property store on every pose update.
	LARGE_INTEGER lastLookupAttempt[vr::k_unMaxTrackedDeviceCount] = {};

	// Fixed-capacity flat list of tracking-system fallbacks. 8 is well above
	// any plausible deployment (HMD vendor + 1-2 controller systems + scratch).
	// Linear scanned in HandleDevicePoseUpdated; replaces the prior
	// std::unordered_map<std::string, FallbackTransform> which heap-allocated
	// on every key insert and required a hash + equality check on every pose
	// update for every disabled slot.
	static const size_t MaxFallbackSlots = 8;
	FallbackSlot systemFallbacks[MaxFallbackSlots] = {};

	// Cached QueryPerformanceFrequency value. QPF is constant for the lifetime
	// of the boot, so we capture it once in Init() rather than re-querying on
	// every BlendTransform call (~50-200 ns saved per pose update).
	LARGE_INTEGER qpcFreq = {};

	DeltaSize currentDeltaSpeed[vr::k_unMaxTrackedDeviceCount];

	protocol::AlignmentSpeedParams alignmentSpeedParams;

	// Finger-smoothing config packed into an atomic uint64_t. Single-writer
	// (IPC thread, on user UI input — rare) / many-reader (skeletal hook
	// detour, ~340 Hz/hand × 2 hands = 680 Hz). The previous version used
	// a mutex around the 6-byte struct; for a hot read at 680 Hz that's
	// gratuitous contention on a tiny POD that fits in a single cache line.
	// More importantly any future LOG() drift inside the critical section
	// would stall every skeletal update on a disk write.
	//
	// FingerSmoothingConfig is 6 bytes (1 master_enabled, 1 smoothness,
	// 2 finger_mask, 1 _reserved, with 1 byte trailing alignment padding).
	// memcpy into the lower 6 bytes of the uint64_t with the upper bytes
	// always zero. std::atomic<uint64_t> is always lock-free on x64 so the
	// hot-path read is a single mov + acquire fence.
	//
	// Default zero = {master_enabled=false, smoothness=0, finger_mask=0}
	// so the detour fast-paths to passthrough until the overlay has sent
	// a real config.
	mutable std::atomic<uint64_t>     fingerCfgPacked{0};

	// Look up an existing fallback slot by system name (linear scan + memcmp).
	// Returns nullptr if no slot is currently occupied with that name.
	FallbackSlot* FindFallbackSlot(const char* name, size_t len);
	const FallbackSlot* FindFallbackSlot(const char* name, size_t len) const;
	// Find or insert a fallback slot for the given name. Returns nullptr if the
	// flat array is at capacity.
	FallbackSlot* AcquireFallbackSlot(const char* name, size_t len);

	DeltaSize GetTransformDeltaSize(
		DeltaSize prior_delta,
		const IsoTransform& deviceWorldPose,
		const IsoTransform& src,
		const IsoTransform& target
	) const;

	double GetTransformRate(DeltaSize delta) const;

	void BlendTransform(DeviceTransform& device, const IsoTransform& deviceWorldPose) const;
	void ApplyTransform(DeviceTransform& device, vr::DriverPose_t& devicePose) const;
};