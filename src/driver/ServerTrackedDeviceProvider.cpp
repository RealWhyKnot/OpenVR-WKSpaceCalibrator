#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"
#include "IsometryTransform.h"
#include "MotionGate.h"  // ClassifyCorrection / StillFloor — option 3 per user 2026-05-04

#include <cstring>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	// QPF is constant for the lifetime of the boot; capture it once instead of
	// querying inside BlendTransform on every pose update.
	QueryPerformanceFrequency(&qpcFreq);

	// transforms[] elements are zero/identity-initialized via DeviceTransform's
	// default member initializers and IsoTransform's default constructor; a
	// memset would be undefined behavior because Eigen members aren't trivially
	// copyable. AlignmentSpeedParams is a plain aggregate of doubles so memset
	// here is safe.
	memset(&alignmentSpeedParams, 0, sizeof alignmentSpeedParams);

	alignmentSpeedParams.thr_rot_tiny = 0.1f * (EIGEN_PI / 180.0f);
	alignmentSpeedParams.thr_rot_small = 1.0f * (EIGEN_PI / 180.0f);
	alignmentSpeedParams.thr_rot_large = 5.0f * (EIGEN_PI / 180.0f);

	alignmentSpeedParams.thr_trans_tiny = 0.1f / 1000.0; // mm
	alignmentSpeedParams.thr_trans_small = 1.0f / 1000.0; // mm
	alignmentSpeedParams.thr_trans_large = 20.0f / 1000.0; // mm
	
	alignmentSpeedParams.align_speed_tiny = 0.05f;
	alignmentSpeedParams.align_speed_small = 0.2f;
	alignmentSpeedParams.align_speed_large = 2.0f;

	InjectHooks(this, pDriverContext);
	server.Run();
	shmem.Create(OPENVR_SPACECALIBRATOR_SHMEM_NAME);

	debugTransform = Eigen::Vector3d::Zero();
	debugRotation = Eigen::Quaterniond::Identity();

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	TRACE("ServerTrackedDeviceProvider::Cleanup()");

	// Order matters. The previous order (server.Stop -> shmem.Close ->
	// DisableHooks -> VR_CLEANUP) had a fatal race: DisableHooks removes
	// the MinHook patches but does NOT wait for in-flight detours to
	// return. SteamVR's pose-update detours fire ~kHz across all tracked
	// devices, the skeletal detour at ~340Hz/hand, so on every driver
	// unload there's a window where a detour body is still executing
	// inside our DLL while we tear down state below it -- and after
	// Cleanup returns SteamVR unmaps the DLL with that thread mid-call.
	//
	// New order:
	//   1. DisableHooks first -- removes patches AND drains in-flight
	//      detour callers before returning. After it returns no thread
	//      is executing inside any of our hook bodies.
	//   2. server.Stop -- joins the IPC worker thread (its own internal
	//      drain).
	//   3. shmem.Close -- safe now because no detour can read it.
	//   4. VR_CLEANUP_SERVER_DRIVER_CONTEXT -- finalize.
	DisableHooks();
	server.Stop();
	shmem.Close();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

namespace {


	vr::HmdQuaternion_t convert(const Eigen::Quaterniond& q) {
		vr::HmdQuaternion_t result;
		result.w = q.w();
		result.x = q.x();
		result.y = q.y();
		result.z = q.z();
		return result;
	}

	vr::HmdVector3_t convert(const Eigen::Vector3d& v) {
		vr::HmdVector3_t result;
		result.v[0] = (float) v.x();
		result.v[1] = (float) v.y();
		result.v[2] = (float) v.z();
		return result;
	}

	Eigen::Quaterniond convert(const vr::HmdQuaternion_t& q) {
		return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
	}

	Eigen::Vector3d convert(const vr::HmdVector3d_t& v) {
		return Eigen::Vector3d(v.v[0], v.v[1], v.v[2]);
	}

	Eigen::Vector3d convert(const double* arr) {
		return Eigen::Vector3d(arr[0], arr[1], arr[2]);
	}

	IsoTransform toIsoWorldTransform(const vr::DriverPose_t& pose) {
		Eigen::Quaterniond rot(pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x, pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z);
		Eigen::Vector3d trans(pose.vecWorldFromDriverTranslation[0], pose.vecWorldFromDriverTranslation[1], pose.vecWorldFromDriverTranslation[2]);

		return IsoTransform(rot, trans);
	}

	IsoTransform toIsoPose(const vr::DriverPose_t& pose) {
		auto worldXform = toIsoWorldTransform(pose);

		Eigen::Quaterniond rot(pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z);
		Eigen::Vector3d trans(pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]);

		return worldXform * IsoTransform(rot, trans);
	}
}


/**
 * This function heuristically evaluates the amount of drift between the src and target playspace transforms,
 * evaluated centered on the `pose` device transform. This is then used to control the speed of realignment.
 */
ServerTrackedDeviceProvider::DeltaSize ServerTrackedDeviceProvider::GetTransformDeltaSize(
	DeltaSize prior_delta,
	const IsoTransform& deviceWorldPose,
	const IsoTransform& src,
	const IsoTransform& target
) const {
	const auto src_pose = src * deviceWorldPose;
	const auto target_pose = target * deviceWorldPose;

	// Use .norm() (linear metres) here, NOT .squaredNorm(). The thresholds
	// (alignmentSpeedParams.thr_trans_*) are populated everywhere — driver
	// Init(), overlay defaults, the user-tunable UI sliders — as linear
	// distances in metres. Comparing squaredNorm against linear thresholds
	// silently squared the gate: a 20 mm threshold required a 141 mm offset
	// to trip, so currentRate was permanently TINY (= 0.05 lerp) for any
	// realistic continuous-cal correction. The runtime cost of the sqrt is
	// well under 1 µs per call; this hot path runs at ~kHz, the math budget
	// is microseconds. .angularDistance() is already linear (radians).
	const auto trans_delta = (src_pose.translation - target_pose.translation).norm();
	const auto rot_delta = src_pose.rotation.angularDistance(target_pose.rotation);

	DeltaSize trans_level, rot_level;

	if (trans_delta > alignmentSpeedParams.thr_trans_large) trans_level = DeltaSize::LARGE;
	else if (trans_delta > alignmentSpeedParams.thr_trans_small) trans_level = DeltaSize::SMALL;
	else trans_level = DeltaSize::TINY;

	if (rot_delta > alignmentSpeedParams.thr_rot_large) rot_level = DeltaSize::LARGE;
	else if (rot_delta > alignmentSpeedParams.thr_rot_small) rot_level = DeltaSize::SMALL;
	else rot_level = DeltaSize::TINY;

	if (trans_level == DeltaSize::TINY && rot_level == DeltaSize::TINY) return DeltaSize::TINY;
	else return std::max(prior_delta, std::max(trans_level, rot_level));
}

double ServerTrackedDeviceProvider::GetTransformRate(DeltaSize delta) const {
	switch (delta) {
	case DeltaSize::TINY: return alignmentSpeedParams.align_speed_tiny;
	case DeltaSize::SMALL: return alignmentSpeedParams.align_speed_small;
	default: return alignmentSpeedParams.align_speed_large;
	}
}

/**
 * Smoothly interpolates the device active transform towards the target transform.
 * When recalibrateOnMovement is enabled on the slot, the lerp rate is gated by
 * per-frame motion magnitude — a stationary device gets ~zero blend progress, a
 * moving one gets the full time-based rate. This hides calibration shifts in
 * the user's natural motion instead of producing visible "phantom drift" while
 * the user is still (a noticeable issue when lying down).
 *
 * 2026-05-04: per option 3 of feedback_calibration_blending_request.md, the
 * gate is now max(motionGate, regimeFloor) where regimeFloor depends on the
 * pending correction size (|targetTransform - transform|). This unfreezes
 * the lerp when the user is still — small corrections drift slowly (10%
 * floor), normal corrections at moderate speed (50%), and catastrophic
 * corrections (post-stall, Quest re-localization) effectively snap (90%).
 * Previously the lerp froze at 0 when the user wasn't moving and they had
 * to wave a controller before convergence resumed.
 */
void ServerTrackedDeviceProvider::BlendTransform(DeviceTransform& device, const IsoTransform &deviceWorldPose) const {
	LARGE_INTEGER timestamp;
	QueryPerformanceCounter(&timestamp);

	// qpcFreq captured once in Init(); QPF is constant per boot so re-querying
	// here would be wasted work on the pose-update hot path.
	double lerp = (timestamp.QuadPart - device.lastPoll.QuadPart) / (double)qpcFreq.QuadPart;
	device.lastPoll = timestamp;

	lerp *= GetTransformRate(device.currentRate);

	if (device.recalibrateOnMovement) {
		if (!device.blendMotionInitialized) {
			// First frame since the flag was enabled: capture the reference pose
			// and skip blend progress this tick — there's nothing to compute a
			// meaningful delta against yet, and we don't want a stale prior pose
			// (from before re-enable) to produce a giant phantom motion gate.
			device.lastBlendWorldPos = deviceWorldPose.translation;
			device.lastBlendWorldRot = deviceWorldPose.rotation;
			device.blendMotionInitialized = true;
			lerp = 0.0;
		} else {
			// Per-frame motion magnitude in normalized units. kPosFullScale and
			// kRotFullScale are the per-frame deltas at which the gate is fully
			// open — small typical-jitter motions produce partial gate, sustained
			// natural motion produces gate=1 (full time-based rate).
			constexpr double kPosFullScale = 0.005;   // 5 mm
			constexpr double kRotFullScale = 0.0175;  // ~1 deg in radians
			const double devPosDelta = (deviceWorldPose.translation - device.lastBlendWorldPos).norm();
			const double devRotDelta = deviceWorldPose.rotation.angularDistance(device.lastBlendWorldRot);
			const double motionGate = std::min(1.0,
				std::max(devPosDelta / kPosFullScale, devRotDelta / kRotFullScale));

			// Correction magnitude — how far the active transform has to travel
			// to reach the target. Distinct from the device-motion deltas above:
			// motionGate asks "is the user moving?", regime asks "how big is the
			// pending shift?". Convert to mm + degrees to match the thresholds
			// in MotionGate.h.
			const double correctionPosMm =
				(device.targetTransform.translation - device.transform.translation).norm() * 1000.0;
			const double correctionRotDeg =
				device.targetTransform.rotation.angularDistance(device.transform.rotation) * 180.0 / M_PI;
			const auto regime = spacecal::motiongate::ClassifyCorrection(
				correctionPosMm, correctionRotDeg);
			const double regimeFloor = spacecal::motiongate::StillFloor(regime);

			// Effective gate: take whichever is higher. When moving, motionGate
			// dominates (≈1); when still, regimeFloor sets the minimum so the
			// lerp doesn't freeze at 0.
			const double effectiveGate = std::max(motionGate, regimeFloor);
			lerp *= effectiveGate;

			device.lastBlendWorldPos = deviceWorldPose.translation;
			device.lastBlendWorldRot = deviceWorldPose.rotation;
		}
	} else if (device.blendMotionInitialized) {
		// Flag was on but is now off — reset so re-enabling later doesn't see a
		// stale prior pose.
		device.blendMotionInitialized = false;
	}

	if (lerp > 1.0)
		lerp = 1.0;
	if (lerp < 0 || isnan(lerp))
		lerp = 0;

	device.transform = device.transform.interpolateAround(lerp, device.targetTransform, deviceWorldPose.translation);
}

void ServerTrackedDeviceProvider::ApplyTransform(DeviceTransform& device, vr::DriverPose_t& devicePose) const {
	auto deviceWorldTransform = toIsoWorldTransform(devicePose);
	deviceWorldTransform = device.transform * deviceWorldTransform;
	devicePose.vecWorldFromDriverTranslation[0] = deviceWorldTransform.translation(0);
	devicePose.vecWorldFromDriverTranslation[1] = deviceWorldTransform.translation(1);
	devicePose.vecWorldFromDriverTranslation[2] = deviceWorldTransform.translation(2);
	devicePose.qWorldFromDriverRotation = convert(deviceWorldTransform.rotation);
}


inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t &lhs, const vr::HmdQuaternion_t &rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
	vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
	vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	if (newTransform.openVRID >= vr::k_unMaxTrackedDeviceCount) return;

	std::lock_guard<std::mutex> lock(stateMutex);

	auto &tf = transforms[newTransform.openVRID];
	const bool wasEnabled = tf.enabled;
	tf.enabled = newTransform.enabled;

	// Record the device's tracking system so we can match it against per-system
	// fallbacks for any future device that occupies this slot.
	{
		// target_system is a fixed-size buffer that may not be NUL-terminated if
		// fully populated; bound the read to the buffer length.
		size_t maxLen = sizeof newTransform.target_system;
		size_t len = 0;
		while (len < maxLen && newTransform.target_system[len] != '\0') ++len;
		deviceSystem[newTransform.openVRID].assign(newTransform.target_system, len);
		// Mark the lookup state so the pose-hook thread doesn't re-query the
		// property store for this slot. Empty target_system means "unknown" —
		// fall back to NotTried so the lazy lookup can still try (and then
		// throttle if it keeps failing).
		lookupState[newTransform.openVRID] = (len > 0) ? LookupState::Cached : LookupState::NotTried;
	}

	// Whenever the per-ID transform is updated (whether enabling or disabling), the
	// slot is no longer following a fallback. Subsequent HandleDevicePoseUpdated
	// calls will re-evaluate fallback eligibility.
	tf.fallbackActive = false;

	// Prediction-smoothness strength (0..100). Cheap to update unconditionally;
	// HandleDevicePoseUpdated reads it once per pose update.
	tf.predictionSmoothness = newTransform.predictionSmoothness;

	// Motion-gated blend. When the flag transitions off, reset the captured
	// previous-frame pose so a future re-enable doesn't see a stale delta.
	if (tf.recalibrateOnMovement && !newTransform.recalibrateOnMovement) {
		tf.blendMotionInitialized = false;
	}
	tf.recalibrateOnMovement = newTransform.recalibrateOnMovement;

	if (newTransform.updateTranslation) {
		tf.targetTransform.translation = convert(newTransform.translation);
		if (!newTransform.lerp) {
			tf.transform.translation = tf.targetTransform.translation;
		}
	}

	if (newTransform.updateRotation) {
		tf.targetTransform.rotation = convert(newTransform.rotation);

		if (!newTransform.lerp) {
			tf.transform.rotation = tf.targetTransform.rotation;
		}
	}

	if (newTransform.updateScale)
		tf.scale = newTransform.scale;

	tf.quash = newTransform.quash;

	// On enable transition, the slot's `transform` may be stale from a prior session
	// or never initialized. Snap to the target so we don't ramp in from a junk state.
	if (!wasEnabled && tf.enabled) {
		tf.transform = tf.targetTransform;
		// Forensic diagnostic for audit row #8 (project_upstream_regression_audit_2026-05-04).
		// If a sleeper bug ever puts a stale fallback transform into
		// `tf.transform` before this snap, the snap would lock-in the
		// staleness for the user. Surfacing every snap-on-enable lets
		// post-mortem grep `device_transform_snap_on_enable` reveal the
		// pattern. Once-per-event (driven by SetDeviceTransform IPC), so
		// no throttling needed.
		LOG("device_transform_snap_on_enable: id=%u target=(%.3f,%.3f,%.3f) prevFallbackActive=%d",
			(unsigned)newTransform.openVRID,
			tf.targetTransform.translation.x(),
			tf.targetTransform.translation.y(),
			tf.targetTransform.translation.z(),
			(int)tf.fallbackActive);
	}

	// On disable transition, drop any pending lerp target so a future re-enable
	// doesn't pick up where the last one left off.
	if (wasEnabled && !tf.enabled) {
		tf.targetTransform = tf.transform;
	}

	// Always reset the lerp clock and rate. If the device went offline for a long time,
	// the next BlendTransform would otherwise compute a huge dt and saturate to 1.0
	// (instant jump). Restarting the clock keeps lerp behavior bounded.
	QueryPerformanceCounter(&tf.lastPoll);
	tf.currentRate = DeltaSize::TINY;
}

ServerTrackedDeviceProvider::FallbackSlot* ServerTrackedDeviceProvider::FindFallbackSlot(const char* name, size_t len)
{
	if (len == 0 || len > protocol::MaxTrackingSystemNameLen) return nullptr;
	for (size_t i = 0; i < MaxFallbackSlots; ++i) {
		if (!systemFallbacks[i].occupied) continue;
		// Compare the full buffer length so a shorter prefix can't accidentally
		// match a longer occupant. The buffer is NUL-padded after assignment so
		// `len` bytes followed by a sentinel NUL is sufficient to distinguish.
		if (memcmp(systemFallbacks[i].system_name, name, len) == 0
			&& (len == protocol::MaxTrackingSystemNameLen || systemFallbacks[i].system_name[len] == '\0')) {
			return &systemFallbacks[i];
		}
	}
	return nullptr;
}

const ServerTrackedDeviceProvider::FallbackSlot* ServerTrackedDeviceProvider::FindFallbackSlot(const char* name, size_t len) const
{
	return const_cast<ServerTrackedDeviceProvider*>(this)->FindFallbackSlot(name, len);
}

ServerTrackedDeviceProvider::FallbackSlot* ServerTrackedDeviceProvider::AcquireFallbackSlot(const char* name, size_t len)
{
	if (len == 0 || len > protocol::MaxTrackingSystemNameLen) return nullptr;
	if (auto* existing = FindFallbackSlot(name, len)) return existing;
	for (size_t i = 0; i < MaxFallbackSlots; ++i) {
		if (!systemFallbacks[i].occupied) {
			memset(systemFallbacks[i].system_name, 0, sizeof systemFallbacks[i].system_name);
			memcpy(systemFallbacks[i].system_name, name, len);
			systemFallbacks[i].occupied = true;
			systemFallbacks[i].tf = FallbackTransform{};
			return &systemFallbacks[i];
		}
	}
	return nullptr;
}

void ServerTrackedDeviceProvider::SetTrackingSystemFallback(const protocol::SetTrackingSystemFallback& newFallback)
{
	size_t maxLen = sizeof newFallback.system_name;
	size_t len = 0;
	while (len < maxLen && newFallback.system_name[len] != '\0') ++len;
	if (len == 0) return;

	std::lock_guard<std::mutex> lock(stateMutex);

	if (!newFallback.enabled) {
		// Disabling the fallback. Drop any per-ID slots that were following it so
		// they don't keep applying the stale offset on subsequent pose updates.
		if (auto* slot = FindFallbackSlot(newFallback.system_name, len)) {
			slot->tf.enabled = false;
		}
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
			if (!transforms[id].fallbackActive) continue;
			const std::string& sys = deviceSystem[id];
			if (sys.size() == len && memcmp(sys.data(), newFallback.system_name, len) == 0) {
				transforms[id].fallbackActive = false;
				transforms[id].targetTransform = transforms[id].transform;
				transforms[id].currentRate = DeltaSize::TINY;
				QueryPerformanceCounter(&transforms[id].lastPoll);
			}
		}
		return;
	}

	auto* slot = AcquireFallbackSlot(newFallback.system_name, len);
	if (!slot) {
		// Flat array is full. With MaxFallbackSlots=8 this should never happen
		// in practice; log so we notice if it does.
		LOG("Fallback slot table full; ignoring fallback for new tracking system");
		return;
	}
	slot->tf.enabled = true;
	slot->tf.transform.translation = Eigen::Vector3d(
		newFallback.translation.v[0], newFallback.translation.v[1], newFallback.translation.v[2]);
	slot->tf.transform.rotation = Eigen::Quaterniond(
		newFallback.rotation.w, newFallback.rotation.x, newFallback.rotation.y, newFallback.rotation.z);
	slot->tf.scale = newFallback.scale;
	slot->tf.predictionSmoothness = newFallback.predictionSmoothness;
	slot->tf.recalibrateOnMovement = newFallback.recalibrateOnMovement;
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose)
{
	// Apply debug pose before anything else
	if (openVRID > 0) {
		auto dbgPos = convert(pose.vecPosition) + debugTransform;
		auto dbgRot = convert(pose.qRotation) * debugRotation;
		pose.qRotation = convert(dbgRot);
		pose.vecPosition[0] = dbgPos(0);
		pose.vecPosition[1] = dbgPos(1);
		pose.vecPosition[2] = dbgPos(2);
	}

	// Lock guards every read/write of transforms[], deviceSystem[], systemFallbacks
	// against the IPC server thread's SetDeviceTransform / SetTrackingSystemFallback
	// handlers. IPC writes are infrequent (once per ScanAndApplyProfile tick at
	// most ~1 Hz) and brief, so contention is negligible at the hook's hundreds-of-
	// pose-updates-per-second cadence. Held for the full hook body — simpler than
	// copy-out-under-lock + math-without-lock + write-back. shmem.SetPose writes
	// to a different-process ring buffer (overlay reader) so the mutex doesn't
	// synchronize that path; the lock only matters for in-process state.
	//
	// `unique_lock` (instead of `lock_guard`) so the lookup-fallback path below
	// can briefly release the mutex around its `vr::VRProperties()` call. That
	// OpenVR API call lands inside SteamVR and can block for milliseconds on
	// the runtime's own state lock. Holding our mutex across it stalled every
	// other tracked device's pose-update path — the documented cause of "70+
	// HMD stalls per session" the user reported. We re-acquire before touching
	// any in-process state again.
	std::unique_lock<std::mutex> lock(stateMutex);

	auto& tf = transforms[openVRID];

	// Native pose-prediction suppression. Scales the velocity / acceleration /
	// poseTimeOffset fields by (1 - smoothness/100), where smoothness comes from
	// either the per-ID slot or the matching tracking-system fallback (per-ID
	// wins when both are set). The user picks the value from a 0..100 slider in
	// the overlay's Prediction tab; 100 fully zeros the fields (matching the old
	// "freeze" behaviour) and effectively defeats SteamVR's pose extrapolation.
	// The HMD / calibration ref / calibration target are hard-blocked to 0
	// upstream by the overlay, so by the time we read smoothness here it's
	// already been vetted as safe to apply.
	uint8_t smoothness = tf.predictionSmoothness;
	if (smoothness == 0 && !tf.enabled && !deviceSystem[openVRID].empty()) {
		const auto& sys = deviceSystem[openVRID];
		const FallbackSlot* slot = FindFallbackSlot(sys.data(), sys.size());
		if (slot && slot->tf.enabled && slot->tf.predictionSmoothness > 0) {
			smoothness = slot->tf.predictionSmoothness;
		}
	}
	if (smoothness > 0) {
		// Clamp defensively -- a buggy overlay (or a stale-protocol mismatch)
		// shouldn't be able to push a value above 100 here.
		if (smoothness > 100) smoothness = 100;
		const double factor = 1.0 - (double)smoothness / 100.0;
		pose.vecVelocity[0] *= factor;
		pose.vecVelocity[1] *= factor;
		pose.vecVelocity[2] *= factor;
		pose.vecAcceleration[0] *= factor;
		pose.vecAcceleration[1] *= factor;
		pose.vecAcceleration[2] *= factor;
		pose.vecAngularVelocity[0] *= factor;
		pose.vecAngularVelocity[1] *= factor;
		pose.vecAngularVelocity[2] *= factor;
		pose.vecAngularAcceleration[0] *= factor;
		pose.vecAngularAcceleration[1] *= factor;
		pose.vecAngularAcceleration[2] *= factor;
		// poseTimeOffset is multiplicative on the predictor: zeroing it cancels
		// extrapolation; halving it halves the lookahead time. Same factor as
		// velocity for consistency.
		pose.poseTimeOffset *= factor;
	}

	shmem.SetPose(openVRID, pose);

	if (tf.quash) {
		pose.vecPosition[0] = -pose.vecWorldFromDriverTranslation[0];
		pose.vecPosition[1] = -pose.vecWorldFromDriverTranslation[1] + 9001; // put it 9001m above the origin
		pose.vecPosition[2] = -pose.vecWorldFromDriverTranslation[2];
		shmem.IncrementTelemetry(protocol::DriverPoseShmem::TELEMETRY_QUASH_APPLY);
	} else if (tf.enabled)
	{
		// @TODO: Offset, scale, and re-offset
		pose.vecPosition[0] *= tf.scale;
		pose.vecPosition[1] *= tf.scale;
		pose.vecPosition[2] *= tf.scale;

		auto deviceWorldPose = toIsoPose(pose);
		tf.currentRate = GetTransformDeltaSize(tf.currentRate, deviceWorldPose, tf.transform, tf.targetTransform);
		double lerp = GetTransformRate(tf.currentRate);

		BlendTransform(tf, deviceWorldPose);
		ApplyTransform(tf, pose);
		shmem.IncrementTelemetry(protocol::DriverPoseShmem::TELEMETRY_PER_ID_APPLY);
	}
	else
	{
		// Per-ID transform is disabled. Check for a per-tracking-system fallback —
		// this lets a tracker that connected after the last overlay scan inherit
		// the calibrated offset on its very first pose update.
		//
		// If the overlay hasn't told us this device's tracking system yet (no
		// SetDeviceTransform has arrived for this slot), query it directly via the
		// driver-side property API. Without throttling this fires for every
		// unoccupied slot up to k_unMaxTrackedDeviceCount on every pose update;
		// gate it on lookupState + a 1-second backoff for failures.
		if (deviceSystem[openVRID].empty() && lookupState[openVRID] != LookupState::Cached) {
			bool shouldTry = true;
			if (lookupState[openVRID] == LookupState::Failed) {
				LARGE_INTEGER now;
				QueryPerformanceCounter(&now);
				// Retry no more than once per second. qpcFreq.QuadPart is the
				// number of QPC ticks per second.
				if (qpcFreq.QuadPart > 0 &&
					(now.QuadPart - lastLookupAttempt[openVRID].QuadPart) < qpcFreq.QuadPart) {
					shouldTry = false;
				}
			}
			if (shouldTry) {
				// The `vr::VRProperties()` call below lands inside SteamVR and
				// can block for milliseconds. Drop our mutex for the duration
				// so other devices' pose-update paths aren't stalled behind us
				// (the cause of the user's reported 70+ HMD stalls/session).
				// We've already read everything we needed under the lock —
				// the in-process state we care about (deviceSystem[],
				// lookupState[], transforms[]) is untouched between unlock and
				// the re-lock below; reads/writes of `tf` earlier in the
				// function happened under the lock and reads/writes of
				// `tf.targetTransform` etc. below happen after we re-acquire.
				lock.unlock();

				std::string queriedSys;
				bool queryOk = false;
				if (auto* helpers = vr::VRProperties()) {
					auto handle = helpers->TrackedDeviceToPropertyContainer(openVRID);
					if (handle != vr::k_ulInvalidPropertyContainer) {
						vr::ETrackedPropertyError err = vr::TrackedProp_Success;
						std::string sys = helpers->GetStringProperty(handle, vr::Prop_TrackingSystemName_String, &err);
						if (err == vr::TrackedProp_Success && !sys.empty()) {
							queriedSys = std::move(sys);
							queryOk = true;
						}
					}
				}

				lock.lock();

				// Race check: while we were unlocked, the IPC server thread
				// could have populated this slot via SetDeviceTransform. If it
				// won the race, defer to its value rather than overwriting
				// with our (possibly stale) query result.
				if (lookupState[openVRID] != LookupState::Cached) {
					if (queryOk) {
						deviceSystem[openVRID] = std::move(queriedSys);
						lookupState[openVRID] = LookupState::Cached;
					} else {
						lookupState[openVRID] = LookupState::Failed;
						QueryPerformanceCounter(&lastLookupAttempt[openVRID]);
					}
				}
			}
		}

		if (deviceSystem[openVRID].empty()) {
			return true; // No system known yet; nothing to fall back to.
		}

		const auto& sys = deviceSystem[openVRID];
		FallbackSlot* slot = FindFallbackSlot(sys.data(), sys.size());
		if (slot && slot->tf.enabled) {
			const auto& fb = slot->tf;

			// Update the slot's blend target from the (possibly newly updated) fallback.
			tf.targetTransform = fb.transform;
			tf.scale = fb.scale;
			// Propagate motion-gate setting so the fallback path also gets the
			// movement-masked blend if the user has it enabled. Reset the captured
			// previous-frame pose if the flag is transitioning off.
			if (tf.recalibrateOnMovement && !fb.recalibrateOnMovement) {
				tf.blendMotionInitialized = false;
			}
			tf.recalibrateOnMovement = fb.recalibrateOnMovement;

			// First activation: snap so the device doesn't ramp in from an identity
			// or otherwise stale `transform` value.
			if (!tf.fallbackActive) {
				tf.transform = fb.transform;
				tf.fallbackActive = true;
				tf.currentRate = DeltaSize::TINY;
				QueryPerformanceCounter(&tf.lastPoll);
			}

			pose.vecPosition[0] *= tf.scale;
			pose.vecPosition[1] *= tf.scale;
			pose.vecPosition[2] *= tf.scale;

			auto deviceWorldPose = toIsoPose(pose);
			tf.currentRate = GetTransformDeltaSize(tf.currentRate, deviceWorldPose, tf.transform, tf.targetTransform);

			BlendTransform(tf, deviceWorldPose);
			ApplyTransform(tf, pose);
			shmem.IncrementTelemetry(protocol::DriverPoseShmem::TELEMETRY_FALLBACK_APPLY);
		}
		else if (tf.fallbackActive) {
			// Fallback was removed/disabled while we were following it. Clear our
			// blend state so a future re-enable starts clean.
			tf.fallbackActive = false;
			tf.targetTransform = tf.transform;
			tf.currentRate = DeltaSize::TINY;
			QueryPerformanceCounter(&tf.lastPoll);
		}
	}

	return true;
}

void ServerTrackedDeviceProvider::HandleApplyRandomOffset() {
	std::random_device gen;
	std::uniform_real_distribution<double> d(-1, 1);
	auto init = Eigen::Vector3d(d(gen), d(gen), d(gen));
	auto posOffset = init * 0.25f;

	debugTransform = posOffset;
	debugRotation = Eigen::Quaterniond::Identity();

	std::ostringstream oss;
	oss << "Applied random offset: " << posOffset << " from init " << init << std::endl;
	LOG("%s", oss.str().c_str());
}

void ServerTrackedDeviceProvider::SetFingerSmoothingConfig(const protocol::FingerSmoothingConfig &cfg)
{
	static_assert(sizeof(protocol::FingerSmoothingConfig) <= sizeof(uint64_t),
		"FingerSmoothingConfig must fit inside atomic<uint64_t>");

	// Pack cfg into a uint64_t. Zero-init the target so the upper bytes
	// (beyond sizeof(cfg) = 6) are always 0; the comparison below relies
	// on packed equality only changing when the meaningful bytes change.
	uint64_t newPacked = 0;
	std::memcpy(&newPacked, &cfg, sizeof(cfg));

	// Single atomic exchange gives us both the publish + the previous
	// value for change-detection logging without a separate load.
	const uint64_t oldPacked = fingerCfgPacked.exchange(newPacked, std::memory_order_acq_rel);

	// Log only on real changes so a slider drag (60 Hz no-op tick) doesn't
	// flood the log file. Packed equality covers all three meaningful
	// fields (master_enabled, smoothness, finger_mask) plus _reserved
	// which is always 0 -- so a non-zero diff means a real change.
	if (oldPacked != newPacked) {
		protocol::FingerSmoothingConfig prev{};
		std::memcpy(&prev, &oldPacked, sizeof(prev));
		LOG("[skeletal] SetFingerSmoothingConfig via IPC: enabled=%d smoothness=%u mask=0x%04x (was: enabled=%d smoothness=%u mask=0x%04x)",
			(int)cfg.master_enabled, (unsigned)cfg.smoothness, (unsigned)cfg.finger_mask,
			(int)prev.master_enabled, (unsigned)prev.smoothness, (unsigned)prev.finger_mask);
	}
}

protocol::FingerSmoothingConfig ServerTrackedDeviceProvider::GetFingerSmoothingConfig() const
{
	// Hot path: ~680 Hz (340 Hz × 2 hands) skeletal detour reads. Single
	// atomic load + acquire fence + memcpy = no contention, no syscalls,
	// no LOG()-inside-critical-section drift to worry about.
	const uint64_t packed = fingerCfgPacked.load(std::memory_order_acquire);
	protocol::FingerSmoothingConfig cfg{};
	std::memcpy(&cfg, &packed, sizeof(cfg));
	return cfg;
}