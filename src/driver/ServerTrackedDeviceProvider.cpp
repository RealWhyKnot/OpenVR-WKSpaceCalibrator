#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"
#include "IsometryTransform.h"

#include <random>

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

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
	server.Stop();
	shmem.Close();
	DisableHooks();
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

	const auto trans_delta = (src_pose.translation - target_pose.translation).squaredNorm();
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
 */
void ServerTrackedDeviceProvider::BlendTransform(DeviceTransform& device, const IsoTransform &deviceWorldPose) const {
	LARGE_INTEGER timestamp, freq;
	QueryPerformanceCounter(&timestamp);
	QueryPerformanceFrequency(&freq);

	double lerp = (timestamp.QuadPart - device.lastPoll.QuadPart) / (double)freq.QuadPart;
	device.lastPoll = timestamp;
	
	lerp *= GetTransformRate(device.currentRate);
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
	}

	// Whenever the per-ID transform is updated (whether enabling or disabling), the
	// slot is no longer following a fallback. Subsequent HandleDevicePoseUpdated
	// calls will re-evaluate fallback eligibility.
	tf.fallbackActive = false;

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

void ServerTrackedDeviceProvider::SetTrackingSystemFallback(const protocol::SetTrackingSystemFallback& newFallback)
{
	size_t maxLen = sizeof newFallback.system_name;
	size_t len = 0;
	while (len < maxLen && newFallback.system_name[len] != '\0') ++len;
	std::string name(newFallback.system_name, len);
	if (name.empty()) return;

	if (!newFallback.enabled) {
		// Disabling the fallback. Drop any per-ID slots that were following it so
		// they don't keep applying the stale offset on subsequent pose updates.
		auto it = systemFallbacks.find(name);
		if (it != systemFallbacks.end()) {
			it->second.enabled = false;
		}
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
			if (transforms[id].fallbackActive && deviceSystem[id] == name) {
				transforms[id].fallbackActive = false;
				transforms[id].targetTransform = transforms[id].transform;
				transforms[id].currentRate = DeltaSize::TINY;
				QueryPerformanceCounter(&transforms[id].lastPoll);
			}
		}
		return;
	}

	auto& fb = systemFallbacks[name];
	fb.enabled = true;
	fb.transform.translation = Eigen::Vector3d(
		newFallback.translation.v[0], newFallback.translation.v[1], newFallback.translation.v[2]);
	fb.transform.rotation = Eigen::Quaterniond(
		newFallback.rotation.w, newFallback.rotation.x, newFallback.rotation.y, newFallback.rotation.z);
	fb.scale = newFallback.scale;
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

	shmem.SetPose(openVRID, pose);

	auto& tf = transforms[openVRID];

	if (tf.quash) {
		pose.vecPosition[0] = -pose.vecWorldFromDriverTranslation[0];
		pose.vecPosition[1] = -pose.vecWorldFromDriverTranslation[1] + 9001; // put it 9001m above the origin
		pose.vecPosition[2] = -pose.vecWorldFromDriverTranslation[2];
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
	}
	else
	{
		// Per-ID transform is disabled. Check for a per-tracking-system fallback —
		// this lets a tracker that connected after the last overlay scan inherit
		// the calibrated offset on its very first pose update.
		//
		// If the overlay hasn't told us this device's tracking system yet (no
		// SetDeviceTransform has arrived for this slot), query it directly via the
		// driver-side property API so the fallback can apply on the *very* first
		// pose update — before any overlay scan has run. The query is lazy: cache
		// success in deviceSystem[id], and on failure leave it empty and try again
		// next tick (cheap; properties are an in-process lookup).
		if (deviceSystem[openVRID].empty()) {
			if (auto* helpers = vr::VRProperties()) {
				auto handle = helpers->TrackedDeviceToPropertyContainer(openVRID);
				if (handle != vr::k_ulInvalidPropertyContainer) {
					vr::ETrackedPropertyError err = vr::TrackedProp_Success;
					std::string sys = helpers->GetStringProperty(handle, vr::Prop_TrackingSystemName_String, &err);
					if (err == vr::TrackedProp_Success && !sys.empty()) {
						deviceSystem[openVRID] = std::move(sys);
					}
				}
			}
		}

		if (deviceSystem[openVRID].empty()) {
			return true; // No system known yet; nothing to fall back to.
		}

		auto it = systemFallbacks.find(deviceSystem[openVRID]);
		if (it != systemFallbacks.end() && it->second.enabled) {
			const auto& fb = it->second;

			// Update the slot's blend target from the (possibly newly updated) fallback.
			tf.targetTransform = fb.transform;
			tf.scale = fb.scale;

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