#include "stdafx.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "Configuration.h"
#include "IPCClient.h"
#include "CalibrationCalc.h"
#include "VRState.h"

#include <string>
#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include <GLFW/glfw3.h>

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

CalibrationContext CalCtx;
IPCClient Driver;
static protocol::DriverPoseShmem shmem;

namespace {
	CalibrationCalc calibration;

	inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
		vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
		vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
		auto rotatedVectorQuat = quat * vectorQuat * conjugate;
		return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
	}

	inline Eigen::Matrix3d quaternionRotateMatrix(const vr::HmdQuaternion_t& quat) {
		return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z).toRotationMatrix();
	}

	struct DSample
	{
		bool valid;
		Eigen::Vector3d ref, target;
	};

	bool StartsWith(const std::string& str, const std::string& prefix)
	{
		if (str.length() < prefix.length())
			return false;

		return str.compare(0, prefix.length(), prefix) == 0;
	}

	bool EndsWith(const std::string& str, const std::string& suffix)
	{
		if (str.length() < suffix.length())
			return false;

		return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
	}

	Eigen::Vector3d AxisFromRotationMatrix3(Eigen::Matrix3d rot)
	{
		return Eigen::Vector3d(rot(2, 1) - rot(1, 2), rot(0, 2) - rot(2, 0), rot(1, 0) - rot(0, 1));
	}

	double AngleFromRotationMatrix3(Eigen::Matrix3d rot)
	{
		return acos((rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) / 2.0);
	}

	vr::HmdQuaternion_t VRRotationQuat(const Eigen::Quaterniond& rotQuat)
	{

		vr::HmdQuaternion_t vrRotQuat;
		vrRotQuat.x = rotQuat.coeffs()[0];
		vrRotQuat.y = rotQuat.coeffs()[1];
		vrRotQuat.z = rotQuat.coeffs()[2];
		vrRotQuat.w = rotQuat.coeffs()[3];
		return vrRotQuat;
	}
	
	vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg)
	{
		auto euler = eulerdeg * EIGEN_PI / 180.0;

		Eigen::Quaterniond rotQuat =
			Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

		return VRRotationQuat(rotQuat);
	}

	vr::HmdVector3d_t VRTranslationVec(Eigen::Vector3d transcm)
	{
		auto trans = transcm * 0.01;
		vr::HmdVector3d_t vrTrans;
		vrTrans.v[0] = trans[0];
		vrTrans.v[1] = trans[1];
		vrTrans.v[2] = trans[2];
		return vrTrans;
	}

	DSample DeltaRotationSamples(Sample s1, Sample s2)
	{
		// Difference in rotation between samples.
		auto dref = s1.ref.rot * s2.ref.rot.transpose();
		auto dtarget = s1.target.rot * s2.target.rot.transpose();

		// When stuck together, the two tracked objects rotate as a pair,
		// therefore their axes of rotation must be equal between any given pair of samples.
		DSample ds;
		ds.ref = AxisFromRotationMatrix3(dref);
		ds.target = AxisFromRotationMatrix3(dtarget);

		// Reject samples that were too close to each other.
		auto refA = AngleFromRotationMatrix3(dref);
		auto targetA = AngleFromRotationMatrix3(dtarget);
		ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

		ds.ref.normalize();
		ds.target.normalize();
		return ds;
	}

	Pose ConvertPose(const vr::DriverPose_t &driverPose) {
		Eigen::Quaterniond driverToWorldQ(
			driverPose.qWorldFromDriverRotation.w,
			driverPose.qWorldFromDriverRotation.x,
			driverPose.qWorldFromDriverRotation.y,
			driverPose.qWorldFromDriverRotation.z
		);
		Eigen::Vector3d driverToWorldV(
			driverPose.vecWorldFromDriverTranslation[0],
			driverPose.vecWorldFromDriverTranslation[1],
			driverPose.vecWorldFromDriverTranslation[2]
		);

		Eigen::Quaterniond driverRot = driverToWorldQ * Eigen::Quaterniond(
			driverPose.qRotation.w,
			driverPose.qRotation.x,
			driverPose.qRotation.y,
			driverPose.qRotation.z
		);
		
		Eigen::Vector3d driverPos = driverToWorldV + driverToWorldQ * Eigen::Vector3d(
			driverPose.vecPosition[0],
			driverPose.vecPosition[1],
			driverPose.vecPosition[2]
		);

		Eigen::AffineCompact3d xform = Eigen::Translation3d(driverPos) * driverRot;

		return Pose(xform);
	}

	// Velocity-based extrapolation of a reference pose by `dtSeconds` (positive = forward
	// in time, negative = backward). Mutates `pose` in place.
	//
	// vecVelocity / vecAngularVelocity are in the driver's local frame (the same frame
	// as vecPosition / qRotation), so we apply them directly there and let
	// qWorldFromDriverRotation do the world-space projection downstream in ConvertPose.
	// This sidesteps the trap of rotating velocity into world space and then adding it
	// to a driver-space position.
	//
	// Returns true on success, false if the velocity data looks invalid (NaN, infinite,
	// or implausibly large). On false the caller should fall back to the un-extrapolated
	// pose; we never throw.
	inline bool ExtrapolateReferencePose(vr::DriverPose_t& pose, double dtSeconds) {
		if (dtSeconds == 0.0) return true;

		// Sanity-check the velocity components. A momentary tracking glitch can produce
		// NaN/inf or wildly large velocity values; applying those to the pose would
		// teleport the reference and pollute the sample.
		const double maxLinearMps = 50.0;        // ~180 km/h, far beyond any real head/tracker motion
		const double maxAngularRadps = 50.0;     // ~8 rev/s
		for (int i = 0; i < 3; i++) {
			double v = pose.vecVelocity[i];
			double w = pose.vecAngularVelocity[i];
			if (!std::isfinite(v) || !std::isfinite(w)) return false;
			if (std::fabs(v) > maxLinearMps) return false;
			if (std::fabs(w) > maxAngularRadps) return false;
		}

		// Linear extrapolation in driver-local space.
		pose.vecPosition[0] += pose.vecVelocity[0] * dtSeconds;
		pose.vecPosition[1] += pose.vecVelocity[1] * dtSeconds;
		pose.vecPosition[2] += pose.vecVelocity[2] * dtSeconds;

		// Angular extrapolation: vecAngularVelocity is axis-angle in radians/sec in the
		// driver-local frame. Build the corresponding small rotation deltaQ_local and
		// pre-multiply qRotation by it. qWorldFromDriverRotation downstream rotates the
		// updated qRotation into world space.
		double angSpeed = std::sqrt(
			pose.vecAngularVelocity[0] * pose.vecAngularVelocity[0] +
			pose.vecAngularVelocity[1] * pose.vecAngularVelocity[1] +
			pose.vecAngularVelocity[2] * pose.vecAngularVelocity[2]);
		if (angSpeed > 1e-9) {
			double angle = angSpeed * dtSeconds;
			double half = angle * 0.5;
			double s = std::sin(half);
			double axisX = pose.vecAngularVelocity[0] / angSpeed;
			double axisY = pose.vecAngularVelocity[1] / angSpeed;
			double axisZ = pose.vecAngularVelocity[2] / angSpeed;
			vr::HmdQuaternion_t deltaQ = { std::cos(half), axisX * s, axisY * s, axisZ * s };
			pose.qRotation = deltaQ * pose.qRotation;
		}

		return true;
	}

	bool CollectSample(const CalibrationContext& ctx)
	{
		vr::DriverPose_t reference, target;
		reference.poseIsValid = false;
		reference.result = vr::ETrackingResult::TrackingResult_Uninitialized;
		target.poseIsValid = false;
		target.result = vr::ETrackingResult::TrackingResult_Uninitialized;

		reference = ctx.devicePoses[ctx.referenceID];
		target = ctx.devicePoses[ctx.targetID];

		bool ok = true;
		if (!reference.poseIsValid && reference.result != vr::ETrackingResult::TrackingResult_Running_OK)
		{
			CalCtx.Log("Reference device is not tracking\n"); ok = false;
		}
		if (!target.poseIsValid && target.result != vr::ETrackingResult::TrackingResult_Running_OK)
		{
			CalCtx.Log("Target device is not tracking\n"); ok = false;
		}
		if (!ok)
		{
			if (CalCtx.state != CalibrationState::Continuous) {
				CalCtx.Log("Aborting calibration!\n");
				CalCtx.state = CalibrationState::None;
			}
			return false;
		}

		// Apply tracker offsets
		if (CalCtx.state == CalibrationState::Continuous || CalCtx.state == CalibrationState::ContinuousStandby) {
			reference.vecPosition[0] += ctx.continuousCalibrationOffset.x();
			reference.vecPosition[1] += ctx.continuousCalibrationOffset.y();
			reference.vecPosition[2] += ctx.continuousCalibrationOffset.z();
		}

		// Manual inter-system latency compensation. With a non-zero offset we shift the
		// reference pose forward/backward in time to align with the *effective* moment
		// the target sample represents. We use velocity extrapolation rather than a
		// history buffer (cheaper, bounded, and good enough for the small ±100 ms range
		// the UI exposes). If velocity data is invalid the reference pose is left
		// un-shifted for this tick — better one bad-but-bounded sample than a thrown
		// exception or a teleporting reference.
		//
		// Bit-for-bit identical behaviour to before this feature is preserved when
		// targetLatencyOffsetMs == 0: the shift in seconds is exactly 0.0 and
		// ExtrapolateReferencePose returns immediately without touching the pose. The
		// `if` below only enters when both that AND we have valid sample-time data,
		// i.e. when there's actually work to do.
		if (ctx.targetLatencyOffsetMs != 0.0
			&& ctx.devicePoseSampleTimes[ctx.referenceID].QuadPart != 0
			&& ctx.devicePoseSampleTimes[ctx.targetID].QuadPart != 0)
		{
			LARGE_INTEGER freq;
			QueryPerformanceFrequency(&freq);
			// Δshmem = how stale the reference pose is relative to the target pose
			// (target ahead → positive). Then we additionally subtract the user's
			// configured offset. We extrapolate the reference forward by this Δ to put
			// it on the same effective timeline as the target.
			double shmemDeltaSec =
				double(ctx.devicePoseSampleTimes[ctx.targetID].QuadPart -
					   ctx.devicePoseSampleTimes[ctx.referenceID].QuadPart)
				/ double(freq.QuadPart);
			double offsetSec = ctx.targetLatencyOffsetMs / 1000.0;
			// effectiveTargetTime = targetSampleTime - offset, so the reference needs to
			// move to (effectiveTargetTime) - referenceSampleTime = shmemDelta - offset.
			double dt = shmemDeltaSec - offsetSec;
			ExtrapolateReferencePose(reference, dt);
		}

		calibration.PushSample(Sample(
			ConvertPose(reference),
			ConvertPose(target),
			glfwGetTime()
		));

		return true;
	}

	bool AssignTargets() {
		auto state = VRState::Load();
		
		if (CalCtx.referenceID < 0) {
			CalCtx.referenceID = state.FindDevice(CalCtx.referenceStandby.trackingSystem, CalCtx.referenceStandby.model, CalCtx.referenceStandby.serial);
		}

		if (CalCtx.targetID < 0) {
			CalCtx.targetID = state.FindDevice(CalCtx.targetStandby.trackingSystem, CalCtx.targetStandby.model, CalCtx.targetStandby.serial);
		}

		for (int i = 0; i < CalCtx.MAX_CONTROLLERS; i++) {
			if (i < state.devices.size()
				&& state.devices[i].trackingSystem == CalCtx.targetTrackingSystem
				&& state.devices[i].deviceClass == vr::TrackedDeviceClass_Controller
				&& (state.devices[i].controllerRole == vr::TrackedControllerRole_LeftHand || state.devices[i].controllerRole == vr::TrackedControllerRole_RightHand))
			{
				CalCtx.controllerIDs[i] = state.devices[i].id;
			} else {
				CalCtx.controllerIDs[i] = -1;
			}
		}

		return CalCtx.referenceID >= 0 && CalCtx.targetID >= 0;
	}
}

void InitCalibrator()
{
	Driver.Connect();
	shmem.Open(OPENVR_SPACECALIBRATOR_SHMEM_NAME);
}

namespace {
	// Per-device cache of the last SetDeviceTransform we sent to the driver. Used to
	// suppress redundant IPC writes when ScanAndApplyProfile runs every tick during
	// continuous calibration.
	struct LastAppliedTransform {
		bool valid = false;
		protocol::SetDeviceTransform payload{ 0u, false };
	};
	LastAppliedTransform g_lastApplied[vr::k_unMaxTrackedDeviceCount];

	// Per-device serial cache. If a device ID gets reassigned to a different physical
	// device (battery dies, pairing changes, SteamVR re-enumerates), we want to clear
	// the stale per-ID state in the driver before applying any new transform.
	std::string g_lastSeenSerial[vr::k_unMaxTrackedDeviceCount];

	// Target system tracking. When the calibrated profile's target system changes
	// (or the profile is cleared), we invalidate all per-ID caches so the next scan
	// re-establishes correct enable/disable state.
	std::string g_lastTargetSystem;
	bool g_lastEnabled = false;

	// AlignmentSpeedParams dedupe — avoid spamming the driver with identical params.
	protocol::AlignmentSpeedParams g_lastAlignmentSpeed{};
	bool g_alignmentSpeedSent = false;

	// Last per-tracking-system fallback we sent to the driver (deduped).
	protocol::SetTrackingSystemFallback g_lastFallback{};
	bool g_lastFallbackSent = false;

	bool TransformPayloadEqual(const protocol::SetDeviceTransform& a, const protocol::SetDeviceTransform& b) {
		if (a.openVRID != b.openVRID) return false;
		if (a.enabled != b.enabled) return false;
		if (a.updateTranslation != b.updateTranslation) return false;
		if (a.updateRotation != b.updateRotation) return false;
		if (a.updateScale != b.updateScale) return false;
		if (a.lerp != b.lerp) return false;
		if (a.quash != b.quash) return false;
		if (a.scale != b.scale) return false;
		for (int i = 0; i < 3; i++) if (a.translation.v[i] != b.translation.v[i]) return false;
		if (a.rotation.w != b.rotation.w || a.rotation.x != b.rotation.x ||
			a.rotation.y != b.rotation.y || a.rotation.z != b.rotation.z) return false;
		if (memcmp(a.target_system, b.target_system, sizeof a.target_system) != 0) return false;
		return true;
	}

	bool FallbackPayloadEqual(const protocol::SetTrackingSystemFallback& a, const protocol::SetTrackingSystemFallback& b) {
		if (memcmp(a.system_name, b.system_name, sizeof a.system_name) != 0) return false;
		if (a.enabled != b.enabled) return false;
		if (a.scale != b.scale) return false;
		for (int i = 0; i < 3; i++) if (a.translation.v[i] != b.translation.v[i]) return false;
		if (a.rotation.w != b.rotation.w || a.rotation.x != b.rotation.x ||
			a.rotation.y != b.rotation.y || a.rotation.z != b.rotation.z) return false;
		return true;
	}

	void SetTargetSystemField(protocol::SetDeviceTransform& payload, const std::string& system) {
		// Copy bounded by the buffer size; leave any remaining bytes zero so the
		// driver can read up to the first NUL or buffer end.
		memset(payload.target_system, 0, sizeof payload.target_system);
		size_t copyLen = system.size();
		if (copyLen >= sizeof payload.target_system) copyLen = sizeof payload.target_system - 1;
		memcpy(payload.target_system, system.data(), copyLen);
	}

	void SendDeviceTransformIfChanged(uint32_t id, const protocol::SetDeviceTransform& payload) {
		if (id >= vr::k_unMaxTrackedDeviceCount) return;
		auto& cache = g_lastApplied[id];
		if (cache.valid && TransformPayloadEqual(cache.payload, payload)) {
			return;
		}
		protocol::Request req(protocol::RequestSetDeviceTransform);
		req.setDeviceTransform = payload;
		Driver.SendBlocking(req);
		cache.valid = true;
		cache.payload = payload;
	}

	void SendFallbackIfChanged(const std::string& systemName, bool enabled,
		const Eigen::Vector3d& translationCm, const Eigen::Quaterniond& rotation, double scale)
	{
		protocol::SetTrackingSystemFallback payload{};
		size_t copyLen = systemName.size();
		if (copyLen >= sizeof payload.system_name) copyLen = sizeof payload.system_name - 1;
		memcpy(payload.system_name, systemName.data(), copyLen);
		payload.enabled = enabled;
		Eigen::Vector3d trans = translationCm * 0.01; // cm -> m, matches per-ID convention
		payload.translation.v[0] = trans.x();
		payload.translation.v[1] = trans.y();
		payload.translation.v[2] = trans.z();
		payload.rotation.w = rotation.w();
		payload.rotation.x = rotation.x();
		payload.rotation.y = rotation.y();
		payload.rotation.z = rotation.z();
		payload.scale = scale;

		if (g_lastFallbackSent && FallbackPayloadEqual(g_lastFallback, payload)) return;

		protocol::Request req(protocol::RequestSetTrackingSystemFallback);
		req.setTrackingSystemFallback = payload;
		Driver.SendBlocking(req);
		g_lastFallback = payload;
		g_lastFallbackSent = true;
	}

	void InvalidateTransformCacheForId(uint32_t id) {
		if (id >= vr::k_unMaxTrackedDeviceCount) return;
		g_lastApplied[id].valid = false;
		g_lastSeenSerial[id].clear();
	}

	void InvalidateAllTransformCaches() {
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
			g_lastApplied[id].valid = false;
			g_lastSeenSerial[id].clear();
		}
		g_alignmentSpeedSent = false;
		g_lastFallbackSent = false;
	}
}

void ResetAndDisableOffsets(uint32_t id, const std::string& trackingSystem = "")
{
	vr::HmdVector3d_t zeroV;
	zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;

	vr::HmdQuaternion_t zeroQ;
	zeroQ.x = 0; zeroQ.y = 0; zeroQ.z = 0; zeroQ.w = 1;

	protocol::SetDeviceTransform payload{ id, false, zeroV, zeroQ, 1.0 };
	SetTargetSystemField(payload, trackingSystem);
	SendDeviceTransformIfChanged(id, payload);
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

void ScanAndApplyProfile(CalibrationContext &ctx)
{
	std::unique_ptr<char[]> buffer_array(new char [vr::k_unMaxPropertyStringSize]);
	char* buffer = buffer_array.get();
	ctx.enabled = ctx.validProfile;

	// If the calibrated target tracking system changed (or profile was loaded/cleared),
	// invalidate all per-ID caches so we re-establish correct state on every device.
	const bool targetSystemChanged = (ctx.targetTrackingSystem != g_lastTargetSystem);
	if (targetSystemChanged || ctx.enabled != g_lastEnabled) {
		// If we previously had a fallback registered for a now-stale system, tell
		// the driver to disable it so devices on that system stop receiving the
		// old offset. Done before InvalidateAllTransformCaches so the dedupe
		// shortcut doesn't suppress this.
		if (targetSystemChanged && !g_lastTargetSystem.empty() && g_lastFallbackSent && g_lastFallback.enabled) {
			protocol::SetTrackingSystemFallback disablePayload{};
			size_t copyLen = g_lastTargetSystem.size();
			if (copyLen >= sizeof disablePayload.system_name) copyLen = sizeof disablePayload.system_name - 1;
			memcpy(disablePayload.system_name, g_lastTargetSystem.data(), copyLen);
			disablePayload.enabled = false;
			disablePayload.rotation = { 1, 0, 0, 0 };
			disablePayload.scale = 1.0;
			protocol::Request req(protocol::RequestSetTrackingSystemFallback);
			req.setTrackingSystemFallback = disablePayload;
			Driver.SendBlocking(req);
		}

		InvalidateAllTransformCaches();
		g_lastTargetSystem = ctx.targetTrackingSystem;
		g_lastEnabled = ctx.enabled;
	}

	if (!g_alignmentSpeedSent || memcmp(&g_lastAlignmentSpeed, &ctx.alignmentSpeedParams, sizeof g_lastAlignmentSpeed) != 0) {
		protocol::Request setParamsReq(protocol::RequestSetAlignmentSpeedParams);
		setParamsReq.setAlignmentSpeedParams = ctx.alignmentSpeedParams;
		Driver.SendBlocking(setParamsReq);
		g_lastAlignmentSpeed = ctx.alignmentSpeedParams;
		g_alignmentSpeedSent = true;
	}

	// Push the per-tracking-system fallback so any device on `targetTrackingSystem`
	// that connects between scans inherits the calibrated offset on its first pose
	// update — without waiting for the next per-ID scan.
	if (ctx.enabled && !ctx.targetTrackingSystem.empty()) {
		auto euler = ctx.calibratedRotation * EIGEN_PI / 180.0;
		Eigen::Quaterniond rotQuat =
			Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
		SendFallbackIfChanged(ctx.targetTrackingSystem, true,
			ctx.calibratedTranslation, rotQuat, ctx.calibratedScale);
	}

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid) {
			// Device disappeared. Clear our cache for this slot so a future device
			// that gets assigned this ID starts from a known-clean state.
			if (!g_lastSeenSerial[id].empty() || g_lastApplied[id].valid) {
				InvalidateTransformCacheForId(id);
			}
			continue;
		}

		// Detect device-ID reuse: SteamVR can reassign an OpenVR ID to a different
		// physical device after the original disconnects. The driver's transforms[]
		// slot would otherwise apply the old offset to the new device.
		{
			char serialBuf[256] = {0};
			vr::ETrackedPropertyError serialErr = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, serialBuf, sizeof serialBuf, &serialErr);
			std::string serial = (serialErr == vr::TrackedProp_Success) ? std::string(serialBuf) : std::string();
			if (g_lastSeenSerial[id] != serial) {
				if (!g_lastSeenSerial[id].empty()) {
					// ID reassigned. Force a clean disable on the slot before any new
					// transform takes effect. Clear our local cache so the disable is
					// guaranteed to be sent (no dedupe match).
					g_lastApplied[id].valid = false;
					ResetAndDisableOffsets(id);
				}
				g_lastSeenSerial[id] = serial;
			}
		}

		/*if (deviceClass == vr::TrackedDeviceClass_HMD) // for debugging unexpected universe switches
		{
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			auto universeId = vr::VRSystem()->GetUint64TrackedDeviceProperty(id, vr::Prop_CurrentUniverseId_Uint64, &err);
			printf("uid %d err %d\n", universeId, err);
			ResetAndDisableOffsets(id);
			continue;
		}*/

		if (!ctx.enabled)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

		if (err != vr::TrackedProp_Success)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		std::string trackingSystem(buffer);

		if (id == vr::k_unTrackedDeviceIndex_Hmd)
		{
			//auto p = ctx.devicePoses[id].mDeviceToAbsoluteTracking.m;
			//printf("HMD %d: %f %f %f\n", id, p[0][3], p[1][3], p[2][3]);

			// Check if the current HMD is a Pimax crystal
			if (trackingSystem == "aapvr") {
				// HMD is a Pimax HMD
				vr::HmdMatrix34_t eyeToHeadLeft = vr::VRSystem()->GetEyeToHeadTransform(vr::Eye_Left);
				// Crystal's projection matrix is constant 0s or 1s except for [0][3], which stores the IPD offset from the nose
				bool isCrystalHmd =
					eyeToHeadLeft.m[0][0] == 1 && eyeToHeadLeft.m[0][1] == 0 && eyeToHeadLeft.m[0][2] == 0 &&                     // IPD
					eyeToHeadLeft.m[1][0] == 0 && eyeToHeadLeft.m[1][1] == 1 && eyeToHeadLeft.m[1][2] == 0 && eyeToHeadLeft.m[1][3] == 0 &&
					eyeToHeadLeft.m[2][0] == 0 && eyeToHeadLeft.m[2][1] == 0 && eyeToHeadLeft.m[2][2] == 1 && eyeToHeadLeft.m[2][3] == 0;

				if (isCrystalHmd) {
					// Move it outside the aapvr system ; we treat aapvr as if it were lighthouse
					trackingSystem = "Pimax Crystal HMD";
				}
			}

			if (trackingSystem != ctx.referenceTrackingSystem)
			{
				// Currently using an HMD with a different tracking system than the calibration.
				ctx.enabled = false;
			}

			ResetAndDisableOffsets(id, trackingSystem);
			continue;
		}

		// Detect Pimax crystal controllers and separate them too
		if (deviceClass == vr::TrackedDeviceClass_Controller) {
			if (trackingSystem == "oculus") {
				vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_RenderModelName_String, buffer, vr::k_unMaxPropertyStringSize, &err);
				std::string renderModel(buffer);
				vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ConnectedWirelessDongle_String, buffer, vr::k_unMaxPropertyStringSize, &err);
				std::string connectedWirelessDongle(buffer);

				// Check if the controller claims its an oculus controller but also pimax
				if (renderModel.find("{aapvr}") != std::string::npos &&
					renderModel.find("crystal") != std::string::npos &&
					connectedWirelessDongle.find("lighthouse") != std::string::npos) {
					trackingSystem = "Pimax Crystal Controllers";
				}
			}
		}

		if (trackingSystem != ctx.targetTrackingSystem)
		{
			ResetAndDisableOffsets(id, trackingSystem);
			continue;
		}

		const bool isFreshlyAdopted = !g_lastApplied[id].valid || !g_lastApplied[id].payload.enabled;

		protocol::SetDeviceTransform payload{
			id,
			true,
			VRTranslationVec(ctx.calibratedTranslation),
			VRRotationQuat(ctx.calibratedRotation),
			ctx.calibratedScale
		};
		// During continuous calibration, lerp toward the smoothly-updating target so
		// the active offset doesn't snap on every cycle. EXCEPT when this is a freshly
		// adopted device — those need to snap into place rather than ramping in from
		// identity, which would look like a slow drift to the user.
		payload.lerp = (CalCtx.state == CalibrationState::Continuous) && !isFreshlyAdopted;
		payload.quash = CalCtx.state == CalibrationState::Continuous && id == CalCtx.targetID && CalCtx.quashTargetInContinuous;
		SetTargetSystemField(payload, ctx.targetTrackingSystem);

		SendDeviceTransformIfChanged(id, payload);
	}

	if (ctx.enabled && ctx.chaperone.valid && ctx.chaperone.autoApply)
	{
		uint32_t quadCount = 0;
		vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

		// Heuristic: when SteamVR resets to a blank-ish chaperone, it uses empty geometry,
		// but manual adjustments (e.g. via a play space mover) will not touch geometry.
		if (quadCount != ctx.chaperone.geometry.size())
		{
			ApplyChaperoneBounds();
		}
	}
}

void StartCalibration() {
	CalCtx.hasAppliedCalibrationResult = false;
	AssignTargets();
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
	calibration.Clear();
	Metrics::WriteLogAnnotation("StartCalibration");
}

void StartContinuousCalibration() {
	CalCtx.hasAppliedCalibrationResult = false;
	AssignTargets();
	StartCalibration();
	CalCtx.state = CalibrationState::Continuous;
	calibration.setRelativeTransformation(CalCtx.refToTargetPose, CalCtx.relativePosCalibrated);
	calibration.lockRelativePosition = CalCtx.lockRelativePosition;
	if (CalCtx.lockRelativePosition) {
		CalCtx.Log("Relative position locked");
	}
	else {
		CalCtx.Log("Collecting initial samples...");
	}
	Metrics::WriteLogAnnotation("StartContinuousCalibration");
}

void EndContinuousCalibration() {
	CalCtx.state = CalibrationState::None;
	CalCtx.relativePosCalibrated = false;
	SaveProfile(CalCtx);
	Metrics::WriteLogAnnotation("EndContinuousCalibration");
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem())
		return;

	auto &ctx = CalCtx;
	if ((time - ctx.timeLastTick) < 0.05)
		return;

	if (ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby) {
		ctx.ClearLogOnMessage();

		if (CalCtx.requireTriggerPressToApply && (time - ctx.timeLastAssign) > 10) {
			// rescan devices every 10 seconds or so if we are using controller data
			ctx.timeLastAssign = time;
			AssignTargets();
		}
	}

	ctx.timeLastTick = time;
	shmem.ReadNewPoses([&](const protocol::DriverPoseShmem::AugmentedPose& augmented_pose) {
		if (augmented_pose.deviceId >= 0 && augmented_pose.deviceId <= vr::k_unMaxTrackedDeviceCount) {
			ctx.devicePoses[augmented_pose.deviceId] = augmented_pose.pose;
			// Track per-device shmem QPC timestamps so CollectSample can compute the
			// inter-system time delta when applying targetLatencyOffsetMs.
			ctx.devicePoseSampleTimes[augmented_pose.deviceId] = augmented_pose.sample_time;
		}
	});

	// Sample driver-side telemetry counters and push the per-tick deltas (in Hz)
	// into the metrics time series. Initialize the prior snapshot lazily on the
	// first valid sample so the first delta is zero rather than a huge spike
	// representing the entire driver-uptime accumulation.
	{
		static bool s_telemetryPrimed = false;
		static uint64_t s_lastFallback = 0, s_lastPerId = 0, s_lastQuash = 0;
		static double s_lastTelemetryTime = 0;

		uint64_t fallback = 0, perId = 0, quash = 0;
		if (shmem.GetTelemetry(fallback, perId, quash)) {
			Metrics::RecordTimestamp();
			double now = Metrics::CurrentTime;
			if (!s_telemetryPrimed) {
				s_telemetryPrimed = true;
				s_lastFallback = fallback;
				s_lastPerId = perId;
				s_lastQuash = quash;
				s_lastTelemetryTime = now;
			} else {
				double dt = now - s_lastTelemetryTime;
				if (dt > 1e-6) {
					Metrics::fallbackApplyRate.Push((fallback - s_lastFallback) / dt);
					Metrics::perIdApplyRate.Push((perId - s_lastPerId) / dt);
					Metrics::quashApplyRate.Push((quash - s_lastQuash) / dt);
				}
				s_lastFallback = fallback;
				s_lastPerId = perId;
				s_lastQuash = quash;
				s_lastTelemetryTime = now;
			}
		}
	}

	// check for non-updating headset tracking space (caused by quest out of bounds or taken off head for example) and abort everything for this tick
	auto p = ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].vecPosition;
	if ((p[0] == 0.0 && p[1] == 0.0 && p[2] == 0.0) || (ctx.xprev == p[0] && ctx.yprev == p[1] && ctx.zprev == p[2])) {
		// std::cerr << "HMD tracking didn't update, skipping update" << std::endl;
		ctx.consecutiveHmdStalls++;
		// After ~1.5s of stalled HMD tracking, the sample buffer no longer represents
		// reality (the user might have walked elsewhere, taken the headset off, etc).
		// Purge it and demote to ContinuousStandby so calibration restarts cleanly
		// when tracking returns. Without this, stale samples persist and the next
		// few calibration cycles either accept a bad estimate or sit rejecting
		// everything until the watchdog fires.
		const int MaxHmdStalls = 30;
		if (ctx.consecutiveHmdStalls == MaxHmdStalls) {
			calibration.Clear();
			if (ctx.state == CalibrationState::Continuous) {
				ctx.state = CalibrationState::ContinuousStandby;
				CalCtx.Log("HMD tracking lost — pausing continuous calibration\n");
			}
		}
		return;
	}
	ctx.consecutiveHmdStalls = 0;
	ctx.xprev = (float) p[0];
	ctx.yprev = (float) p[1];
	ctx.zprev = (float) p[2];

	// Run the scan in every state where a profile can be active. Previously the scan
	// was skipped once continuous calibration had a valid result, which meant a tracker
	// powered on mid-session never received its offset until calibration was restarted.
	// Per-ID dedupe inside ScanAndApplyProfile keeps IPC churn near zero when nothing
	// has changed.
	if (ctx.state == CalibrationState::None
		|| ctx.state == CalibrationState::ContinuousStandby
		|| ctx.state == CalibrationState::Continuous)
	{
		if ((time - ctx.timeLastScan) >= 1.0)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
	}

	if (ctx.state == CalibrationState::ContinuousStandby) {
		if (AssignTargets()) {
			StartContinuousCalibration();
		}
		else {
			ctx.wantedUpdateInterval = 0.5;
			ctx.Log("Waiting for devices...\n");
			return;
		}
	}

	if (ctx.state == CalibrationState::None) {
		ctx.wantedUpdateInterval = 1.0;
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = 0.1;

		if ((time - ctx.timeLastScan) >= 0.1)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	bool ok = true;

	if (ctx.referenceID == -1 || ctx.referenceID >= vr::k_unMaxTrackedDeviceCount) {
		CalCtx.Log("Missing reference device\n");
		ok = false;
	}
	if (ctx.targetID == -1 || ctx.targetID >= vr::k_unMaxTrackedDeviceCount)
	{
		CalCtx.Log("Missing target device\n");
		ok = false;
	}

	if (ctx.state == CalibrationState::Begin)
	{

		char referenceSerial[256], targetSerial[256];
		referenceSerial[0] = targetSerial[0] = 0;
		vr::VRSystem()->GetStringTrackedDeviceProperty(ctx.referenceID, vr::Prop_SerialNumber_String, referenceSerial, 256);
		vr::VRSystem()->GetStringTrackedDeviceProperty(ctx.targetID, vr::Prop_SerialNumber_String, targetSerial, 256);

		char buf[256];
		snprintf(buf, sizeof buf, "Reference device ID: %d, serial: %s\n", ctx.referenceID, referenceSerial);
		CalCtx.Log(buf);
		snprintf(buf, sizeof buf, "Target device ID: %d, serial %s\n", ctx.targetID, targetSerial);
		CalCtx.Log(buf);

		ScanAndApplyProfile(ctx);

		Metrics::jitterRef.Push(calibration.ReferenceJitter());
		Metrics::jitterRef.Push(calibration.TargetJitter());

		if (!CalCtx.ReferencePoseIsValidSimple())
		{
			CalCtx.Log("Reference device is not tracking\n"); ok = false;
		}

		if (!CalCtx.TargetPoseIsValidSimple())
		{
			CalCtx.Log("Target device is not tracking\n"); ok = false;
		}
		
		// @TOOD: Determine if the tracking is jittery
		if (calibration.ReferenceJitter() > ctx.jitterThreshold) {
			CalCtx.Log("Reference device is not tracking\n"); ok = false;
		}
		if (calibration.TargetJitter() > ctx.jitterThreshold) {
			CalCtx.Log("Target device is not tracking\n"); ok = false;
		}

		if (ok) {
			//ResetAndDisableOffsets(ctx.targetID);
			ctx.state = CalibrationState::Rotation;
			ctx.wantedUpdateInterval = 0.0;

			CalCtx.Log("Starting calibration...\n");
			return;
		}
	}

	if (!ok)
	{
		if (ctx.state != CalibrationState::Continuous) {
			ctx.state = CalibrationState::None;

			CalCtx.Log("Aborting calibration!\n");
		}
		return;
	}

	if (!CollectSample(ctx))
	{
		return;
	}

	CalCtx.Progress((int) calibration.SampleCount(), (int)CalCtx.SampleCount());

	if (calibration.SampleCount() < CalCtx.SampleCount()) return;
	while (calibration.SampleCount() > CalCtx.SampleCount()) calibration.ShiftSample();

	if (CalCtx.state == CalibrationState::Continuous && CalCtx.requireTriggerPressToApply && CalCtx.hasAppliedCalibrationResult) {
		bool triggerPressed = true;
		vr::VRControllerState_t state;
		for (int i = 0; i < CalCtx.MAX_CONTROLLERS; i++) {
			if (CalCtx.controllerIDs[i] >= 0) {
				vr::VRSystem()->GetControllerState(CalCtx.controllerIDs[i], &state, sizeof(state));
				triggerPressed &= state.rAxis[vr::k_eControllerAxis_TrackPad /* matches trigger on Index controllers?? */].x > 0.75f
					|| state.rAxis[vr::k_eControllerAxis_Trigger].x > 0.75f;
				//printf("Controller %d tracpad: %f\n", i, state.rAxis[vr::k_eControllerAxis_TrackPad].x);
				//printf("Controller %d trigger: %f\n", i, state.rAxis[vr::k_eControllerAxis_Trigger].x);
				if (!triggerPressed) {
					break;
				}
			}
		}

		if (!triggerPressed) {
			CalCtx.Log("Waiting for trigger press...\n");
			CalCtx.wasWaitingForTriggers = true;
			return;
		}

		if (CalCtx.wasWaitingForTriggers) {
			CalCtx.Log("Triggers pressed, continuing calibration...\n");
			CalCtx.wasWaitingForTriggers = false;
		}
	}

	LARGE_INTEGER start_time;
	QueryPerformanceCounter(&start_time);
		
	bool lerp = false;

	if (CalCtx.state == CalibrationState::Continuous) {
		CalCtx.messages.clear();
		calibration.enableStaticRecalibration = CalCtx.enableStaticRecalibration;
		calibration.lockRelativePosition = CalCtx.lockRelativePosition;
		calibration.ComputeIncremental(lerp, CalCtx.continuousCalibrationThreshold, CalCtx.maxRelativeErrorThreshold, CalCtx.ignoreOutliers);
	}
	else {
		calibration.enableStaticRecalibration = false;
		calibration.ComputeOneshot(CalCtx.ignoreOutliers);
	}

	if (calibration.isValid()) {
		ctx.calibratedRotation = calibration.EulerRotation();
		ctx.calibratedTranslation = calibration.Transformation().translation() * 100.0; // convert to cm units for profile storage
		ctx.refToTargetPose = calibration.RelativeTransformation();
		ctx.relativePosCalibrated = calibration.isRelativeTransformationCalibrated();

		auto vrTrans = VRTranslationVec(ctx.calibratedTranslation);
		auto vrRot = VRRotationQuat(Eigen::Quaterniond(calibration.Transformation().rotation()));

		ctx.validProfile = true;
		SaveProfile(ctx);

		ScanAndApplyProfile(ctx);

		CalCtx.hasAppliedCalibrationResult = true;

		CalCtx.Log("Finished calibration, profile saved\n");
	} else {
		CalCtx.Log("Calibration failed.\n");
	}

	LARGE_INTEGER end_time;
	QueryPerformanceCounter(&end_time);
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	double duration = (end_time.QuadPart - start_time.QuadPart) / (double)freq.QuadPart;
	Metrics::computationTime.Push(duration * 1000.0);

	// Hand the raw reference + target poses to the metrics writer so the v2 CSV
	// columns get filled. Reconstructing these in the replay harness (tools/replay/)
	// gives us the same Sample values that fed CalibrationCalc::PushSample, which
	// is the whole point of the harness — the metric-level columns alone aren't
	// enough to re-run the math offline.
	{
		const vr::DriverPose_t& refPose = ctx.devicePoses[ctx.referenceID];
		const vr::DriverPose_t& tgtPose = ctx.devicePoses[ctx.targetID];

		auto driverPoseToWorld = [](const vr::DriverPose_t& dp,
			Eigen::Vector3d& outTrans, Eigen::Quaterniond& outRot) {
			Eigen::Quaterniond worldFromDriver(
				dp.qWorldFromDriverRotation.w,
				dp.qWorldFromDriverRotation.x,
				dp.qWorldFromDriverRotation.y,
				dp.qWorldFromDriverRotation.z);
			Eigen::Vector3d worldFromDriverTrans(
				dp.vecWorldFromDriverTranslation[0],
				dp.vecWorldFromDriverTranslation[1],
				dp.vecWorldFromDriverTranslation[2]);
			Eigen::Quaterniond rot(dp.qRotation.w, dp.qRotation.x, dp.qRotation.y, dp.qRotation.z);
			Eigen::Vector3d pos(dp.vecPosition[0], dp.vecPosition[1], dp.vecPosition[2]);
			outRot = (worldFromDriver * rot).normalized();
			outTrans = worldFromDriverTrans + worldFromDriver * pos;
		};

		Eigen::Vector3d refT, tgtT;
		Eigen::Quaterniond refQ, tgtQ;
		driverPoseToWorld(refPose, refT, refQ);
		driverPoseToWorld(tgtPose, tgtT, tgtQ);

		// Map CalibrationState (Calibration.h) to TickPhase (CalibrationMetrics.h).
		// The two enums intentionally mirror each other; we don't share the type
		// so the metrics module doesn't need to include Calibration.h.
		Metrics::TickPhase phase = Metrics::TickPhase::None;
		switch (CalCtx.state) {
		case CalibrationState::None:              phase = Metrics::TickPhase::None; break;
		case CalibrationState::Begin:             phase = Metrics::TickPhase::Begin; break;
		case CalibrationState::Rotation:          phase = Metrics::TickPhase::Rotation; break;
		case CalibrationState::Translation:       phase = Metrics::TickPhase::Translation; break;
		case CalibrationState::Editing:           phase = Metrics::TickPhase::Editing; break;
		case CalibrationState::Continuous:        phase = Metrics::TickPhase::Continuous; break;
		case CalibrationState::ContinuousStandby: phase = Metrics::TickPhase::ContinuousStandby; break;
		}

		Metrics::SetTickRawPoses(refT, refQ, tgtT, tgtQ, phase);
	}

	Metrics::WriteLogEntry();
		
	if (CalCtx.state != CalibrationState::Continuous) {
		ctx.state = CalibrationState::None;
		calibration.Clear();
	}
	else {
		size_t drop_samples = CalCtx.SampleCount() / 10;
		for (int i = 0; i < drop_samples; i++) {
			calibration.ShiftSample();
		}
	}
}

void LoadChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

	CalCtx.chaperone.geometry.resize(quadCount);
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], &quadCount);
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->GetWorkingPlayAreaSize(&CalCtx.chaperone.playSpaceSize.v[0], &CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.chaperone.valid = true;
}

void ApplyChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();
	vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], (uint32_t)CalCtx.chaperone.geometry.size());
	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}

void DebugApplyRandomOffset() {
	protocol::Request req(protocol::RequestDebugOffset);
	Driver.SendBlocking(req);
}

int GetWatchdogResetCount() {
	return calibration.m_watchdogResets;
}