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
#include <algorithm>
#include <map>
#include <psapi.h>

#include <Eigen/Dense>
#include <GLFW/glfw3.h>

#pragma comment(lib, "psapi.lib")

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

CalibrationContext CalCtx;

// AdditionalCalibration's special members live inline in the header now --
// CalibrationCalc is complete at the include point, so the implicitly-defined
// destructor handles the unique_ptr just fine.

// Auto-lock detector tuning. The window must be long enough that brief tracking
// glitches don't push us into "rigidly attached" mode prematurely, but short
// enough that a real change (user repositioned the tracker) flips us back to
// unlocked within a few seconds. With samples coming in at the continuous-cal
// cadence (~2 Hz post-buffer-fill), a 30-sample window is ~15 seconds.
namespace {
	constexpr size_t kAutoLockHistoryMax = 30;
	constexpr size_t kAutoLockSamplesNeeded = 30;
	constexpr double kAutoLockTranslThreshM = 0.005;            // 5 mm
	constexpr double kAutoLockRotThreshRad = 1.0 * EIGEN_PI / 180.0; // 1 deg
}

void CalibrationContext::UpdateAutoLockDetector(
	const Eigen::AffineCompact3d& refWorld,
	const Eigen::AffineCompact3d& targetWorld)
{
	// Relative pose: target expressed in the reference's local frame. For a
	// rigidly attached pair (tracker glued to HMD), this is constant; for an
	// independent pair (tracker on hip vs HMD on head), it varies as the
	// user moves.
	const Eigen::AffineCompact3d rel = refWorld.inverse() * targetWorld;
	autoLockHistory.push_back(rel);
	while (autoLockHistory.size() > kAutoLockHistoryMax) autoLockHistory.pop_front();

	if (autoLockHistory.size() < kAutoLockSamplesNeeded) {
		// Not enough data yet -- stay in "not detected" state. AUTO mode
		// users see the calibration unlocked and re-solving until the
		// detector earns confidence.
		autoLockEffectivelyLocked = false;
		return;
	}

	// Translation variance: classic mean + std-dev on the translation
	// vector. Rigid attachment shows ~mm-level jitter from sensor noise;
	// independent devices can vary by tens of cm as the user moves.
	Eigen::Vector3d meanT = Eigen::Vector3d::Zero();
	for (const auto& a : autoLockHistory) meanT += a.translation();
	meanT /= (double)autoLockHistory.size();
	double translVar = 0.0;
	for (const auto& a : autoLockHistory) {
		const Eigen::Vector3d d = a.translation() - meanT;
		translVar += d.squaredNorm();
	}
	translVar /= (double)autoLockHistory.size();
	const double translStdDev = std::sqrt(translVar);

	// Rotation: max angular distance between any sample and the median.
	// We don't bother computing the proper Frechet mean on SO(3) -- the
	// median sample is good enough as a stand-in, and "max from median" is
	// a tighter bound than "max consecutive". For a true rigid attachment,
	// every sample sits within sensor jitter of the same rotation.
	const auto& medianRot = autoLockHistory[autoLockHistory.size() / 2].rotation();
	const Eigen::Quaterniond medianQ(medianRot);
	double rotMaxAngle = 0.0;
	for (const auto& a : autoLockHistory) {
		const Eigen::Quaterniond q(a.rotation());
		const double angle = medianQ.angularDistance(q);
		if (angle > rotMaxAngle) rotMaxAngle = angle;
	}

	autoLockEffectivelyLocked =
		(translStdDev < kAutoLockTranslThreshM) &&
		(rotMaxAngle < kAutoLockRotThreshRad);
}

void CalibrationContext::ResolveLockMode()
{
	switch (lockRelativePositionMode) {
	case LockMode::OFF:  lockRelativePosition = false; break;
	case LockMode::ON:   lockRelativePosition = true;  break;
	case LockMode::AUTO: lockRelativePosition = autoLockEffectivelyLocked; break;
	}
}

// Auto-speed resolver. Reads the recent jitter samples from Metrics:: and picks
// the calibration speed that should give a stable result without bogging down
// convergence. Sticky: requires the new bucket to be right for ~5 consecutive
// evaluations before switching, so transient noise doesn't oscillate the
// sample-buffer size. Hysteresis is baked in by separate up-shift / down-shift
// thresholds (1mm / 5mm) so a marginal value doesn't flap between buckets.
CalibrationContext::Speed CalibrationContext::ResolvedCalibrationSpeed() const {
	if (calibrationSpeed != AUTO) {
		return calibrationSpeed;
	}

	// Sticky state. Mutable-via-cast is fine here — these are pure caches.
	static Speed s_lastResolved = SLOW;          // safe default before first sample
	static Speed s_pendingResolved = SLOW;
	static int s_pendingTicks = 0;
	constexpr int kRequiredStableTicks = 5;      // ~5 samples of consistency
	constexpr double kFastThreshold = 0.001;     // 1 mm
	constexpr double kSlowThreshold = 0.005;     // 5 mm

	const double jRef = Metrics::jitterRef.last();
	const double jTgt = Metrics::jitterTarget.last();
	// Worst-of-the-two dominates: the noisiest tracker sets the cadence.
	const double j = std::max(jRef, jTgt);

	Speed candidate;
	if (j <= 0.0 || j < kFastThreshold) {
		candidate = FAST;
	} else if (j < kSlowThreshold) {
		candidate = SLOW;
	} else {
		candidate = VERY_SLOW;
	}

	if (candidate == s_lastResolved) {
		s_pendingResolved = candidate;
		s_pendingTicks = 0;
	} else if (candidate == s_pendingResolved) {
		++s_pendingTicks;
		if (s_pendingTicks >= kRequiredStableTicks) {
			s_lastResolved = candidate;
			s_pendingTicks = 0;
		}
	} else {
		s_pendingResolved = candidate;
		s_pendingTicks = 1;
	}

	return s_lastResolved;
}
IPCClient Driver;
static protocol::DriverPoseShmem shmem;

namespace {
	CalibrationCalc calibration;

	// Dedicated sample buffer + math instance for the T-pose-triggered silent
	// recalibration path (F). Lives separately from the primary `calibration`
	// global so passive collection while idle doesn't disturb a user-initiated
	// one-shot or continuous run, and vice versa.
	CalibrationCalc silentRecalCalc;

	// Time of the last accepted silent recal (seconds, glfwGetTime). Used to
	// throttle to at most one accepted recal per ~30 s — the user noticing a
	// micro-jump is more confusing than letting drift sit for an extra moment.
	double silentRecalLastAcceptedTime = -1e9;

	// T-pose hold tracking: when we first start observing T-pose conditions,
	// stamp the time. The hold must sustain for kTPoseHoldSeconds before we
	// fire. Reset to -1 whenever conditions break.
	double silentRecalTPoseHoldStartTime = -1.0;

	// E: residual-EMA drift monitor. Tracks an exponentially-weighted average of
	// the current calibration's RMS error against the silent-recal sample
	// buffer. When the EMA crosses kResidualEmaTriggerM and stays there, a
	// silent recal fires (subject to the same throttle and accept-if-better
	// gates as the T-pose path). Time of last EMA update is tracked separately
	// so the EMA decay rate is in real seconds, not ticks.
	double silentRecalResidualEMA = 0.0;
	double silentRecalLastEMATime = -1.0;
	double silentRecalLastResidualCheckTime = -1.0;
	bool silentRecalEMAPrimed = false;

	// G: floor-touch detector. When the calibrated target tracker's world-space
	// Y is within ±5 cm of zero AND it's been still for kFloorHoldSeconds, fire
	// a Y-only correction (subtracting the residual Y from calibratedTranslation).
	// Distinct from the full-recal triggers because we KNOW the target tracker
	// is on the floor at this moment -- we don't need to refit; we know the
	// answer for the Y axis.
	double silentRecalFloorTouchStartTime = -1.0;

	// H: idle-pose hold start time. When all tracked devices are stationary
	// for kIdleHoldSeconds, fire a silent recal -- the prior 30 s motion
	// in the rolling buffer provides the diversity needed to fit; the
	// idle moment is just the trigger. -1.0 = not currently idle.
	double silentRecalIdleHoldStartTime = -1.0;

	// K: hand-on-HMD hold start time. When any controller comes within 25 cm
	// of the HMD AND is still, fire a silent recal -- same buffer, different
	// trigger. Catches users adjusting their headset fit.
	double silentRecalHandOnHmdStartTime = -1.0;

	// N: HMD wake. Captured at the moment vr::VRSystem() reports
	// TrackedDeviceUserInteractionStarted for the HMD. After kWakeSettleSeconds
	// of post-wake activity (so the buffer has fresh motion), attempt a recal.
	// 0 = no pending wake.
	double silentRecalHmdWakeTime = 0.0;

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
		// Bit-for-bit identical behaviour to before this feature is preserved when the
		// active offset is 0: the shift in seconds is exactly 0.0 and
		// ExtrapolateReferencePose returns immediately without touching the pose. The
		// `if` below only enters when both that AND we have valid sample-time data,
		// i.e. when there's actually work to do.
		double activeOffsetMs = GetActiveLatencyOffsetMs(ctx);
		if (activeOffsetMs != 0.0
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
			double offsetSec = activeOffsetMs / 1000.0;
			// effectiveTargetTime = targetSampleTime - offset, so the reference needs to
			// move to (effectiveTargetTime) - referenceSampleTime = shmemDelta - offset.
			double dt = shmemDeltaSec - offsetSec;
			ExtrapolateReferencePose(reference, dt);
		}

		// Auto-detection feed: push linear-speed magnitudes for both devices into the
		// rolling history buffers. The cross-correlation in CalibrationTick consumes
		// these to estimate the lag between reference and target signal arrival.
		// Linear velocity is preferred over angular for these speed signals — angular
		// velocity from optical trackers is often filter-shaped (low-pass), which
		// blurs the transient that the cross-correlator looks for.
		{
			auto speedFromVel = [](const double v[3]) -> double {
				double s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
				if (!std::isfinite(s)) return 0.0;
				return s;
			};
			double refSpeed = speedFromVel(ctx.devicePoses[ctx.referenceID].vecVelocity);
			double tgtSpeed = speedFromVel(ctx.devicePoses[ctx.targetID].vecVelocity);
			double now = glfwGetTime();

			CalibrationContext& mctx = const_cast<CalibrationContext&>(ctx);
			mctx.refSpeedHistory.push_back(refSpeed);
			mctx.targetSpeedHistory.push_back(tgtSpeed);
			mctx.speedSampleTimes.push_back(now);
			while (mctx.refSpeedHistory.size() > CalibrationContext::kLatencyHistoryCapacity) {
				mctx.refSpeedHistory.pop_front();
			}
			while (mctx.targetSpeedHistory.size() > CalibrationContext::kLatencyHistoryCapacity) {
				mctx.targetSpeedHistory.pop_front();
			}
			while (mctx.speedSampleTimes.size() > CalibrationContext::kLatencyHistoryCapacity) {
				mctx.speedSampleTimes.pop_front();
			}
		}

		calibration.PushSample(Sample(
			ConvertPose(reference),
			ConvertPose(target),
			glfwGetTime()
		));

		// Feed the auto-lock detector with the same sample. We use the world
		// poses directly (not the post-calibration relative pose) so the
		// rigidity check is independent of the math's current solution --
		// the detector measures whether the two devices physically move
		// together, not whether the calibration thinks they do.
		Eigen::AffineCompact3d refWorld = Eigen::AffineCompact3d::Identity();
		refWorld.linear() = ConvertPose(reference).rot;
		refWorld.translation() = ConvertPose(reference).trans;
		Eigen::AffineCompact3d tgtWorld = Eigen::AffineCompact3d::Identity();
		tgtWorld.linear() = ConvertPose(target).rot;
		tgtWorld.translation() = ConvertPose(target).trans;
		const_cast<CalibrationContext&>(ctx).UpdateAutoLockDetector(refWorld, tgtWorld);

		// Push motion-coverage metrics for the live sample buffer. The Calibration
		// Progress popup reads these via Metrics:: and renders progress bars so
		// the user can see whether their motion is varied enough to fit a clean
		// calibration. Computed every CollectSample tick; cheap (linear scan).
		Metrics::translationDiversity.Push(calibration.TranslationDiversity());
		Metrics::rotationDiversity.Push(calibration.RotationDiversity());

		// Push observed jitter every tick so the AUTO calibration-speed selector
		// in ResolvedCalibrationSpeed sees a fresh value -- the previous push
		// site lived in the Begin state branch alone, where the buffer is still
		// empty (just-cleared) and so always pushed 0. AUTO would then read 0
		// from Metrics::jitterRef.last() and lock on FAST forever, regardless of
		// the user's actual tracker quality. Recomputing here is cheap (Welford
		// over the deque); skipping early ticks before two valid samples exist
		// keeps the reading honest -- WelfordStdMagnitude returns 0 on n<2 and
		// we don't want to advertise that as a meaningful "jitter".
		if (calibration.SampleCount() >= 2) {
			Metrics::jitterRef.Push(calibration.ReferenceJitter());
			Metrics::jitterTarget.Push(calibration.TargetJitter());
		}

		return true;
	}

	// === F: T-pose-triggered silent recalibration ============================
	// Forward declarations. AssignTargets is needed for standby-identity
	// resolution when IDs aren't set yet. InvalidateAllTransformCaches lives in
	// a later anonymous-namespace block alongside the per-ID dedupe cache;
	// referencing it from here forces a forward decl since C++ name lookup
	// doesn't reach across files-or-blocks for that symbol until link time.
	bool AssignTargets();
	void InvalidateAllTransformCaches();

	// Helper: pose for an OpenVR-ID-indexed device, using the same world-space
	// projection as the calibration math. Returns nullopt if the device isn't
	// present / not running OK. Used by both the passive sample collector and
	// the T-pose detector.
	struct WorldPose {
		Eigen::Affine3d pose;
		Eigen::Vector3d linVel;
	};
	bool TryGetWorldPose(int32_t id, WorldPose& out) {
		if (id < 0 || id >= (int32_t)vr::k_unMaxTrackedDeviceCount) return false;
		const auto& dp = CalCtx.devicePoses[id];
		if (!dp.poseIsValid || !dp.deviceIsConnected) return false;
		if (dp.result != vr::ETrackingResult::TrackingResult_Running_OK) return false;
		Pose p = ConvertPose(dp);
		out.pose = Eigen::Affine3d::Identity();
		out.pose.linear() = p.rot;
		out.pose.translation() = p.trans;
		out.linVel = Eigen::Vector3d(dp.vecVelocity[0], dp.vecVelocity[1], dp.vecVelocity[2]);
		return true;
	}

	// Locate the user's hand-controllers regardless of which tracking system
	// they live in. T-pose detection needs both hands. AssignTargets() also
	// fills CalCtx.controllerIDs[] but only with target-system controllers,
	// which is the wrong set for the typical lighthouse-HMD + slime-body
	// case where hands are in the reference system. We do our own scan.
	struct HandIDs { int32_t left = -1, right = -1; };
	HandIDs FindHandControllerIDs() {
		HandIDs h;
		if (!vr::VRSystem()) return h;
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
			if (vr::VRSystem()->GetTrackedDeviceClass(id) != vr::TrackedDeviceClass_Controller) continue;
			vr::TrackedPropertyError err = vr::TrackedProp_Success;
			int32_t role = vr::VRSystem()->GetInt32TrackedDeviceProperty(
				id, vr::Prop_ControllerRoleHint_Int32, &err);
			if (err != vr::TrackedProp_Success) continue;
			if (role == vr::TrackedControllerRole_LeftHand) h.left = (int32_t)id;
			else if (role == vr::TrackedControllerRole_RightHand) h.right = (int32_t)id;
		}
		return h;
	}

	// Push a passive sample into silentRecalCalc, throttled to one push per
	// ~50 ms (matches CalibrationTick's own gating). Sized to a 30-second
	// rolling window at that rate (~600 samples max, then the oldest gets
	// shifted out). Uses the primary calibration's referenceID/targetID so we
	// silently fit the same pair the user already has calibrated.
	void PassiveCollectSilentSample() {
		WorldPose ref, tgt;
		if (!TryGetWorldPose(CalCtx.referenceID, ref)) return;
		if (!TryGetWorldPose(CalCtx.targetID, tgt)) return;

		Pose refPose; refPose.rot = ref.pose.linear(); refPose.trans = ref.pose.translation();
		Pose tgtPose; tgtPose.rot = tgt.pose.linear(); tgtPose.trans = tgt.pose.translation();
		silentRecalCalc.PushSample(Sample(refPose, tgtPose, glfwGetTime()));

		// Cap the buffer at a fixed depth (matches the primary buffer's typical
		// size) so memory stays bounded even on very long sessions.
		constexpr size_t kMaxSilentSamples = 600;
		while (silentRecalCalc.SampleCount() > kMaxSilentSamples) {
			silentRecalCalc.ShiftSample();
		}
	}

	// T-pose detector. Returns true while the user holds the canonical T-pose
	// (HMD upright, both hands extended laterally at HMD height, all trackers
	// near-stationary). Fast to run -- a few dot products and norms per tick.
	// Tuning rationale lives next to each gate; the values match the plan's
	// detector criteria (pitch ±15°, lateral >50 cm, height ±30 cm,
	// velocity <5 cm/s on every tracked device).
	bool DetectTPose(const HandIDs& hands) {
		if (hands.left < 0 || hands.right < 0) return false;

		WorldPose hmd, lh, rh;
		if (!TryGetWorldPose(0, hmd)) return false; // HMD is always index 0
		if (!TryGetWorldPose(hands.left, lh)) return false;
		if (!TryGetWorldPose(hands.right, rh)) return false;

		// HMD upright: HMD's local +Y axis (up) must point near world +Y.
		// Equivalently, the dot of the rotated up-vector with world up must
		// exceed cos(15°) ~= 0.966.
		const Eigen::Vector3d hmdUp = hmd.pose.linear() * Eigen::Vector3d::UnitY();
		if (hmdUp.y() < 0.966) return false;

		// HMD forward and right axes for lateral-distance computation.
		const Eigen::Vector3d hmdRight = hmd.pose.linear() * Eigen::Vector3d::UnitX();
		const Eigen::Vector3d hmdPos = hmd.pose.translation();

		// Both hands at HMD height ±30 cm.
		const double dyL = lh.pose.translation().y() - hmdPos.y();
		const double dyR = rh.pose.translation().y() - hmdPos.y();
		if (std::abs(dyL) > 0.30 || std::abs(dyR) > 0.30) return false;

		// Lateral distance from HMD: project (hand - hmd) onto HMD's right axis.
		// Left hand should be in -right direction, right hand in +right, both
		// >= 50 cm.
		const Eigen::Vector3d toL = lh.pose.translation() - hmdPos;
		const Eigen::Vector3d toR = rh.pose.translation() - hmdPos;
		const double latL = toL.dot(hmdRight);  // expect strongly negative
		const double latR = toR.dot(hmdRight);  // expect strongly positive
		if (-latL < 0.50 || latR < 0.50) return false;

		// Stillness: every tracked device's reported linear velocity below
		// 5 cm/s. We check HMD + both hands + the calibrated target -- if any
		// of those move, the user isn't holding still in T-pose.
		auto stillness = [](const Eigen::Vector3d& v) {
			return v.norm() < 0.05;
		};
		if (!stillness(hmd.linVel) || !stillness(lh.linVel) || !stillness(rh.linVel)) return false;

		WorldPose tgt;
		if (TryGetWorldPose(CalCtx.targetID, tgt)) {
			if (!stillness(tgt.linVel)) return false;
		}
		return true;
	}

	// Apply silentRecalCalc's most recent ComputeOneshot result to the live
	// calibration if its RMS is meaningfully better than the current calibration's
	// RMS on the same sample buffer. "Meaningfully better" = at least 10%
	// improvement (per the open question in the drift plan -- conservative to
	// avoid replacing a good fit with a marginally-different one and producing
	// micro-jumps the user will notice).
	bool TryAcceptSilentRecal(double now, const char* source = "silent") {
		// Need motion coverage in the buffer or the math is solving against a
		// degenerate set of poses. Reuse the same diversity metrics that gate
		// one-shot calibration's UI feedback.
		if (silentRecalCalc.TranslationDiversity() < 0.50) return false;
		if (silentRecalCalc.RotationDiversity() < 0.50) return false;

		// Compose the current applied calibration into the same AffineCompact3d
		// space the math uses, then ask silentRecalCalc to score it on its
		// buffer. ValidateCalibration writes RMS to errorOut even when it
		// returns false (the bool only signals "passes the RMS gate"); we want
		// the raw number for comparison.
		const Eigen::Vector3d eulerRad = CalCtx.calibratedRotation * EIGEN_PI / 180.0;
		const Eigen::Quaterniond curRotQ =
			Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX());
		Eigen::AffineCompact3d curCal = Eigen::AffineCompact3d::Identity();
		curCal.linear() = curRotQ.toRotationMatrix();
		curCal.translation() = CalCtx.calibratedTranslation * 0.01;

		double curRMS = std::numeric_limits<double>::infinity();
		(void)silentRecalCalc.ValidateCalibration(curCal, &curRMS);

		// Run the silent recal. ignoreOutliers=false -- we want the gate to
		// fire on bad samples rather than fitting through them. quiet=true so
		// CalibrationCalc's user-facing "Not updating: ..." rejection messages
		// don't leak into the Calibration Progress popup; we have our own
		// accept/reject path with its own gates.
		if (!silentRecalCalc.ComputeOneshot(/*ignoreOutliers=*/false, /*quiet=*/true)) {
			return false;
		}

		const Eigen::AffineCompact3d newCal = silentRecalCalc.Transformation();
		double newRMS = std::numeric_limits<double>::infinity();
		(void)silentRecalCalc.ValidateCalibration(newCal, &newRMS);

		if (!std::isfinite(newRMS) || !std::isfinite(curRMS)) return false;
		if (newRMS >= curRMS * 0.9) return false; // need >=10% improvement

		// Decompose newCal into the cm + euler-deg encoding the rest of the
		// pipeline expects.
		CalCtx.calibratedTranslation = newCal.translation() * 100.0;
		CalCtx.calibratedRotation =
			Eigen::Matrix3d(newCal.linear()).eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;

		char logbuf[256];
		snprintf(logbuf, sizeof logbuf,
			"Silent recal accepted via %s (RMS %.4f m -> %.4f m)\n",
			source, curRMS, newRMS);
		CalCtx.Log(logbuf);
		char annotation[128];
		snprintf(annotation, sizeof annotation, "silent_recal: accepted via %s", source);
		Metrics::WriteLogAnnotation(annotation);

		silentRecalLastAcceptedTime = now;
		InvalidateAllTransformCaches();
		// Wipe the buffer so the next T-pose collects fresh post-recal motion
		// rather than scoring against samples that already produced this fit.
		silentRecalCalc.Clear();
		return true;
	}

	// E: update the rolling 30s EMA of the current calibration's RMS error
	// against the silent buffer. Returns the up-to-date EMA. When the buffer
	// doesn't have enough samples to score a calibration yet, returns 0 and
	// leaves the EMA un-primed -- the trigger upstream will see EMA below
	// threshold and not fire spuriously.
	double UpdateResidualEMA(double now) {
		// Need at least a handful of samples for ValidateCalibration to make
		// sense. The math itself only needs >0 but we want a meaningful score.
		if (silentRecalCalc.SampleCount() < 30) {
			silentRecalEMAPrimed = false;
			return 0.0;
		}

		const Eigen::Vector3d eulerRad = CalCtx.calibratedRotation * EIGEN_PI / 180.0;
		const Eigen::Quaterniond curRotQ =
			Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX());
		Eigen::AffineCompact3d curCal = Eigen::AffineCompact3d::Identity();
		curCal.linear() = curRotQ.toRotationMatrix();
		curCal.translation() = CalCtx.calibratedTranslation * 0.01;

		double instRMS = 0.0;
		(void)silentRecalCalc.ValidateCalibration(curCal, &instRMS);
		if (!std::isfinite(instRMS)) return silentRecalResidualEMA;

		if (!silentRecalEMAPrimed) {
			silentRecalResidualEMA = instRMS;
			silentRecalLastEMATime = now;
			silentRecalEMAPrimed = true;
			return silentRecalResidualEMA;
		}

		// Continuous-time EMA: y = (1-α) * y_prev + α * x, with α derived from
		// (dt / tau). Tau ~30 s gives the "30 s memory" the plan calls for;
		// using real seconds not tick counts keeps the half-life stable even
		// if the tick rate fluctuates.
		const double dt = std::max(0.0, now - silentRecalLastEMATime);
		const double tau = 30.0;
		const double alpha = 1.0 - std::exp(-dt / tau);
		silentRecalResidualEMA += alpha * (instRMS - silentRecalResidualEMA);
		silentRecalLastEMATime = now;
		return silentRecalResidualEMA;
	}

	// Forward decl: TrackingQualityOK is defined further down (it sits with
	// the rest of the P helpers and the original TickSilentTPoseRecal flow).
	// G/H/K/N's hold-fire paths gate on it, so we declare it up here.
	bool TrackingQualityOK();

	// Convenience: compose the live calibration from CalCtx into an Affine3d.
	// Used by G's floor-Y projection and any other path that needs to see what
	// world-space pose the driver is currently producing for a target tracker.
	Eigen::Affine3d ComposeLiveCalibration() {
		const Eigen::Vector3d eulerRad = CalCtx.calibratedRotation * EIGEN_PI / 180.0;
		const Eigen::Quaterniond rotQ =
			Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX());
		Eigen::Affine3d cal = Eigen::Affine3d::Identity();
		cal.linear() = rotQ.toRotationMatrix();
		cal.translation() = CalCtx.calibratedTranslation * 0.01;
		return cal;
	}

	// G: poll the floor-touch detector once per tick. If the calibrated target
	// tracker's world-space Y is near zero AND it's been still for ~1 s, the
	// tracker is most likely sitting on the floor: its true Y is 0, so the
	// calibration's translation Y component is off by exactly the reported Y.
	// Subtract that delta (in cm) from calibratedTranslation.y. Y-only -- we
	// don't touch rotation or X/Z. Cheap, non-disruptive, runs every tick.
	// Returns true if the correction fired this tick (so the caller knows to
	// short-circuit higher-cost recal paths for this frame).
	bool TickFloorTouchYAnchor(double now, bool throttled) {
		WorldPose tgt;
		if (!TryGetWorldPose(CalCtx.targetID, tgt)) {
			silentRecalFloorTouchStartTime = -1.0;
			return false;
		}
		const Eigen::Affine3d cal = ComposeLiveCalibration();
		const Eigen::Vector3d worldPos = cal * tgt.pose.translation();
		const double y = worldPos.y();

		// Bracket: |y| <= 5 cm captures both floor-touch and "calibration
		// dropped slightly below floor" cases. Wider would risk false-firing
		// on hand trackers held low; narrower would miss meaningful drift.
		constexpr double kFloorBandM = 0.05;
		const bool nearFloor = std::abs(y) < kFloorBandM;
		const bool still = tgt.linVel.norm() < 0.05;

		if (!nearFloor || !still) {
			silentRecalFloorTouchStartTime = -1.0;
			return false;
		}
		if (silentRecalFloorTouchStartTime < 0.0) {
			silentRecalFloorTouchStartTime = now;
		}
		// 1 s hold -- long enough to skip transient floor-brushes; short
		// enough that a tracker the user really left on the floor gets a
		// fix before the user moves on.
		if ((now - silentRecalFloorTouchStartTime) < 1.0) return false;
		if (throttled) return false;
		if (!TrackingQualityOK()) return false;

		// Don't apply sub-5mm corrections -- they're indistinguishable from
		// the tracker's noise floor and would just oscillate calibration Y
		// around the true value. Preserve the hold timer so we re-evaluate
		// every tick rather than waiting another full hold.
		if (std::abs(y) < 0.005) return false;

		// Apply the Y-only adjustment. y > 0 means tracker reports above the
		// floor (calibration too high), subtract. y < 0 means tracker reports
		// below floor (impossible physically -- calibration too low), add.
		const double oldYcm = CalCtx.calibratedTranslation.y();
		CalCtx.calibratedTranslation.y() = oldYcm - y * 100.0;

		// Reset the hold so we don't re-fire next tick; the user has to lift
		// the tracker and put it down again. Same throttle stamp as full recals
		// so an unrelated full recal doesn't immediately fire afterward.
		silentRecalFloorTouchStartTime = -1.0;
		silentRecalLastAcceptedTime = now;

		char logbuf[256];
		snprintf(logbuf, sizeof logbuf,
			"Floor-touch Y anchor: shifted calibration Y by %.1f cm (was %.1f, now %.1f)\n",
			-y * 100.0, oldYcm, CalCtx.calibratedTranslation.y());
		CalCtx.Log(logbuf);
		Metrics::WriteLogAnnotation("silent_recal: accepted via floor-touch");
		InvalidateAllTransformCaches();
		return true;
	}

	// H/K stillness helper. Returns true when HMD + target + (if present) both
	// hand controllers are reporting linear velocities below 5 cm/s. Used as
	// the trigger condition for both the idle-pose recal (H) and the hand-on-
	// HMD recal (K, plus a hand-near-HMD geometry check). The two share their
	// stillness check so the implementation tracks one well-defined notion of
	// "user is not moving."
	bool AllRelevantTrackersStill(const HandIDs& hands) {
		WorldPose hmd;
		if (!TryGetWorldPose(0, hmd)) return false;
		if (hmd.linVel.norm() >= 0.05) return false;

		WorldPose tgt;
		if (TryGetWorldPose(CalCtx.targetID, tgt)) {
			if (tgt.linVel.norm() >= 0.05) return false;
		}
		WorldPose lh, rh;
		if (hands.left >= 0 && TryGetWorldPose(hands.left, lh)) {
			if (lh.linVel.norm() >= 0.05) return false;
		}
		if (hands.right >= 0 && TryGetWorldPose(hands.right, rh)) {
			if (rh.linVel.norm() >= 0.05) return false;
		}
		return true;
	}

	// K geometry: at least one hand controller within 25 cm of the HMD. Used
	// alongside AllRelevantTrackersStill to identify the "user is touching/
	// adjusting their headset" moment.
	bool AnyHandNearHmd(const HandIDs& hands) {
		WorldPose hmd;
		if (!TryGetWorldPose(0, hmd)) return false;
		auto check = [&hmd](int32_t handId) {
			if (handId < 0) return false;
			WorldPose h;
			if (!TryGetWorldPose(handId, h)) return false;
			return (h.pose.translation() - hmd.pose.translation()).norm() < 0.25;
		};
		return check(hands.left) || check(hands.right);
	}

	// N: drain vr::VRSystem()'s event queue and watch for a wake event (HMD
	// proximity sensor reports the user just put the headset back on). The
	// wake timestamp is consumed downstream after a settle period. Called from
	// TickSilentTPoseRecal so we only consume events when in idle state --
	// during continuous, the math doesn't need wake-triggered recal.
	void PollSystemEventsForWake(double now) {
		if (!vr::VRSystem()) return;
		vr::VREvent_t ev;
		while (vr::VRSystem()->PollNextEvent(&ev, sizeof ev)) {
			if (ev.eventType == vr::VREvent_TrackedDeviceUserInteractionStarted
				&& ev.trackedDeviceIndex == vr::k_unTrackedDeviceIndex_Hmd)
			{
				silentRecalHmdWakeTime = now;
			}
		}
	}

	// P: gate auto-recal triggers behind tracking-quality. If either ref or
	// target's most recent pose is anything other than Running_OK, suppress.
	// Avoids fitting against samples that included a wireless dropout or
	// out-of-bounds frame.
	bool TrackingQualityOK() {
		auto ok = [](int32_t id) {
			if (id < 0 || id >= (int32_t)vr::k_unMaxTrackedDeviceCount) return false;
			const auto& dp = CalCtx.devicePoses[id];
			return dp.poseIsValid && dp.deviceIsConnected
				&& dp.result == vr::ETrackingResult::TrackingResult_Running_OK;
		};
		return ok(CalCtx.referenceID) && ok(CalCtx.targetID);
	}

	// Top-level driver for F (T-pose) and E (residual-EMA drift) silent recal.
	// Called from CalibrationTick when state == None.
	// Runs passive sample collection, watches for sustained T-pose, and
	// triggers TryAcceptSilentRecal when the hold completes.
	void TickSilentTPoseRecal(double now) {
		// IDs may not be resolved yet on a fresh launch with a saved profile
		// (the user hasn't started a calibration session, so StartCalibration's
		// AssignTargets() hasn't run). Resolve from standby identity once.
		// AssignTargets is a no-op for already-set IDs.
		if (CalCtx.referenceID < 0 || CalCtx.targetID < 0) {
			AssignTargets();
			if (CalCtx.referenceID < 0 || CalCtx.targetID < 0) return;
		}

		// Drop the buffer if the user re-targeted (different tracker selected,
		// or HMD index changed). Old samples are in the previous device's
		// reference frame and would corrupt any fit. Reset every per-trigger
		// hold timer along with it so we don't carry partial holds across
		// the discontinuity.
		static int32_t s_lastRefID = -2;
		static int32_t s_lastTgtID = -2;
		if (CalCtx.referenceID != s_lastRefID || CalCtx.targetID != s_lastTgtID) {
			silentRecalCalc.Clear();
			silentRecalTPoseHoldStartTime = -1.0;
			silentRecalIdleHoldStartTime = -1.0;
			silentRecalHandOnHmdStartTime = -1.0;
			silentRecalFloorTouchStartTime = -1.0;
			silentRecalHmdWakeTime = 0.0;
			silentRecalEMAPrimed = false;
			s_lastRefID = CalCtx.referenceID;
			s_lastTgtID = CalCtx.targetID;
		}

		// N: drain system events first, regardless of activity level, so the
		// wake transition gets captured the instant it fires -- the activity-
		// level gate below would race against it otherwise.
		PollSystemEventsForWake(now);

		// HMD-activity gate. When the user has the headset off (Idle / Standby /
		// timeout states), passive sample collection and the trigger logic
		// produce nonsense -- the user isn't actually doing VR. Skip everything
		// downstream. The wake event itself was captured above, so when the
		// user puts the headset back on, the level transitions to
		// UserInteraction and N's settle timer can run.
		const auto activity = vr::VRSystem()
			? vr::VRSystem()->GetTrackedDeviceActivityLevel(vr::k_unTrackedDeviceIndex_Hmd)
			: vr::k_EDeviceActivityLevel_Unknown;
		const bool hmdActive =
			activity == vr::k_EDeviceActivityLevel_UserInteraction ||
			activity == vr::k_EDeviceActivityLevel_UserInteraction_Timeout;
		if (!hmdActive) return;

		// Throttle: at most one accepted recal per 30 s.
		const bool throttled = (now - silentRecalLastAcceptedTime) < 30.0;

		PassiveCollectSilentSample();

		// G: floor-touch Y anchor. Cheapest of the lot -- one pose lookup,
		// one transform, no math beyond a subtraction. Run first so we don't
		// burn buffer on a full recal when a free Y fix is available.
		if (TickFloorTouchYAnchor(now, throttled)) {
			return;
		}

		// E: residual EMA. Updated at ~1 Hz to keep cost negligible. The EMA
		// itself decays in real seconds, not tick count.
		if (silentRecalLastResidualCheckTime < 0.0
			|| (now - silentRecalLastResidualCheckTime) >= 1.0)
		{
			silentRecalLastResidualCheckTime = now;
			const double ema = UpdateResidualEMA(now);

			// Trigger threshold per the drift plan: ~3 cm sustained EMA. With
			// a 30 s tau, "sustained 30 s above threshold" naturally translates
			// into "EMA above threshold" -- the EMA itself encodes the
			// sustained part. P-gate: don't fire if tracking is currently
			// degraded.
			constexpr double kResidualEmaTriggerM = 0.03;
			if (silentRecalEMAPrimed && ema > kResidualEmaTriggerM
				&& !throttled && TrackingQualityOK())
			{
				if (TryAcceptSilentRecal(now, "residual-EMA")) {
					silentRecalEMAPrimed = false;
					return;
				}
			}
		}

		const HandIDs hands = FindHandControllerIDs();

		// N: HMD wake. After ~5 s of post-wake activity (so the buffer has
		// fresh post-wake motion, not stale samples from before the headset
		// came off), fire one silent recal. The wake timestamp itself is
		// captured by PollSystemEventsForWake above.
		if (silentRecalHmdWakeTime > 0.0
			&& (now - silentRecalHmdWakeTime) > 5.0
			&& !throttled && TrackingQualityOK())
		{
			const double pendingWake = silentRecalHmdWakeTime;
			silentRecalHmdWakeTime = 0.0; // consume regardless of accept
			if (TryAcceptSilentRecal(now, "HMD-wake")) return;
			(void)pendingWake;
		}

		// F: T-pose detection.
		const bool tpose = DetectTPose(hands);
		if (tpose) {
			if (silentRecalTPoseHoldStartTime < 0.0) silentRecalTPoseHoldStartTime = now;
			// Plan calls for "1-2 sustained seconds". Pick 1.5 s.
			constexpr double kTPoseHoldSeconds = 1.5;
			if ((now - silentRecalTPoseHoldStartTime) >= kTPoseHoldSeconds
				&& !throttled && TrackingQualityOK())
			{
				silentRecalTPoseHoldStartTime = -1.0;
				if (TryAcceptSilentRecal(now, "T-pose")) return;
			}
		} else {
			silentRecalTPoseHoldStartTime = -1.0;
		}

		// K: hand-on-HMD. Strictly more specific than H -- requires both
		// stillness AND a hand within 25 cm of the HMD. Checked before H so
		// that the more meaningful trigger gets priority on the log /
		// diagnostics side.
		const bool handsStillNearHmd =
			AllRelevantTrackersStill(hands) && AnyHandNearHmd(hands);
		if (handsStillNearHmd) {
			if (silentRecalHandOnHmdStartTime < 0.0) silentRecalHandOnHmdStartTime = now;
			// 1 s hold -- shorter than T-pose because the user is actively
			// touching their head; this is a deliberate moment, not a held
			// posture.
			if ((now - silentRecalHandOnHmdStartTime) >= 1.0
				&& !throttled && TrackingQualityOK())
			{
				silentRecalHandOnHmdStartTime = -1.0;
				if (TryAcceptSilentRecal(now, "hand-on-HMD")) return;
			}
		} else {
			silentRecalHandOnHmdStartTime = -1.0;
		}

		// H: idle-pose. Plain stillness without the hand-on-HMD geometry.
		// Longer hold (5 s) to distinguish "user paused" from "user is just
		// between movements" -- a brief pause shouldn't trigger a recal.
		const bool allStill = AllRelevantTrackersStill(hands);
		if (allStill) {
			if (silentRecalIdleHoldStartTime < 0.0) silentRecalIdleHoldStartTime = now;
			if ((now - silentRecalIdleHoldStartTime) >= 5.0
				&& !throttled && TrackingQualityOK())
			{
				silentRecalIdleHoldStartTime = -1.0;
				if (TryAcceptSilentRecal(now, "idle-pose")) return;
			}
		} else {
			silentRecalIdleHoldStartTime = -1.0;
		}
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

	// Periodically scan running processes for known external "smooth tracking"
	// tools (OVR-SmoothTracking by yuumu, the most common one). When detected,
	// the overlay shows a warning banner asking the user to stop the external
	// tool and use our native per-tracker smoothness sliders instead. We DON'T
	// try to interop -- our pose-update hook scales velocity/acceleration in a
	// way that fights any external tool doing the same thing, and the resulting
	// behaviour is unpredictable. Better to tell the user clearly than ship
	// half-working interop.
	struct ExternalSmoothingTool {
		const wchar_t* exeName;
		const char* humanName;
	};
	static const ExternalSmoothingTool kKnownTools[] = {
		// Exact filenames we've seen in the wild. Case-insensitive comparison.
		// The actual published binary on yuumu's BOOTH ships as
		// "OpenVR-SmoothTracking.exe" (note: "Open" prefix, not "OVR").
		// We keep the older "OVR-..." names as fallbacks for renamed/repacked
		// variants and any third-party rebuild.
		{ L"OpenVR-SmoothTracking.exe", "OpenVR-SmoothTracking" },
		{ L"OpenVRSmoothTracking.exe",  "OpenVR-SmoothTracking" },
		{ L"OVR-SmoothTracking.exe",    "OVR-SmoothTracking" },
		{ L"OVRSmoothTracking.exe",     "OVR-SmoothTracking" },
		{ L"ovr_smooth_tracking.exe",   "OVR-SmoothTracking" },
	};

	// Substring patterns checked when no exact name matched. Lowercase comparison
	// against the lowercased process name. Catches future filename variants (e.g.
	// versioned releases like "OpenVR-SmoothTracking-v2.exe") without us having to
	// chase every rename. Kept narrow to avoid false positives: a process is
	// counted as a smoothing tool only if its name contains BOTH "smooth" and
	// "track" — the combination is specific enough that no unrelated software in
	// our use cases will trip the heuristic.
	struct SubstringSmoothingPattern {
		const wchar_t* requireA;
		const wchar_t* requireB;
		const char* humanName;
	};
	static const SubstringSmoothingPattern kSubstringTools[] = {
		{ L"smooth", L"track", "external smoothing tool" },
	};

	// Discrete cross-correlation between two equal-length speed series. Returns false
	// if there isn't enough signal energy to produce a trustworthy estimate. On
	// success, *lagSamplesOut is the lag (in samples) at which the correlation is
	// maximised, with sub-sample resolution from a quadratic fit around the peak.
	// Positive lag means the target signal lags the reference signal (target arrives later).
	//
	// Energy threshold: we require RMS speed > 0.1 m/s on both signals so we don't
	// chase noise when the user is standing still.
	bool EstimateLatencyLagSamples(
		const std::deque<double>& ref,
		const std::deque<double>& tgt,
		int maxTau,
		double* lagSamplesOut)
	{
		const size_t N = ref.size();
		if (N < (size_t)(2 * maxTau + 4) || tgt.size() != N) return false;

		// Energy gate: avoid biting on noise.
		double refE = 0, tgtE = 0;
		for (size_t i = 0; i < N; i++) { refE += ref[i] * ref[i]; tgtE += tgt[i] * tgt[i]; }
		double refRms = std::sqrt(refE / (double)N);
		double tgtRms = std::sqrt(tgtE / (double)N);
		const double kMinRmsSpeed = 0.1; // m/s
		if (refRms < kMinRmsSpeed || tgtRms < kMinRmsSpeed) return false;

		// Mean-subtract so a constant-offset signal doesn't dominate the correlation.
		double refMean = 0, tgtMean = 0;
		for (size_t i = 0; i < N; i++) { refMean += ref[i]; tgtMean += tgt[i]; }
		refMean /= (double)N; tgtMean /= (double)N;

		std::vector<double> r(N), t(N);
		for (size_t i = 0; i < N; i++) { r[i] = ref[i] - refMean; t[i] = tgt[i] - tgtMean; }

		// C(tau) = sum_k r[k] * t[k + tau], for tau in [-maxTau, +maxTau]. We index
		// t[k + tau] only when in range; the truncated sum is the standard biased
		// estimator and is fine since N >> maxTau.
		std::vector<double> C(2 * maxTau + 1, 0.0);
		int bestIdx = -1;
		double bestVal = -std::numeric_limits<double>::infinity();
		for (int tau = -maxTau; tau <= maxTau; tau++) {
			int idx = tau + maxTau;
			double sum = 0.0;
			int kStart = std::max(0, -tau);
			int kEnd = std::min((int)N, (int)N - tau);
			for (int k = kStart; k < kEnd; k++) {
				sum += r[k] * t[k + tau];
			}
			C[idx] = sum;
			if (sum > bestVal) { bestVal = sum; bestIdx = idx; }
		}

		if (bestIdx <= 0 || bestIdx >= (int)C.size() - 1) {
			// Peak at the edge — under-determined; fall back to integer lag.
			*lagSamplesOut = (double)(bestIdx - maxTau);
			return true;
		}

		// Quadratic interpolation around the integer peak for sub-sample resolution.
		// y0 = C[bestIdx-1], y1 = C[bestIdx], y2 = C[bestIdx+1].
		// Fractional offset of the parabolic peak = (y0 - y2) / (2*(y0 - 2y1 + y2)).
		double y0 = C[bestIdx - 1];
		double y1 = C[bestIdx];
		double y2 = C[bestIdx + 1];
		double denom = (y0 - 2.0 * y1 + y2);
		double frac = 0.0;
		if (std::fabs(denom) > 1e-12) {
			frac = 0.5 * (y0 - y2) / denom;
			// Bound the fractional shift; a parabolic peak should be within ±1 sample.
			if (!std::isfinite(frac) || std::fabs(frac) > 1.0) frac = 0.0;
		}
		*lagSamplesOut = (double)(bestIdx - maxTau) + frac;
		return true;
	}

	bool DetectExternalSmoothingTool(std::string& outName) {
		// EnumProcesses returns at most cb/sizeof(DWORD) PIDs; bump if it ever fills.
		DWORD pids[2048];
		DWORD bytesNeeded = 0;
		if (!EnumProcesses(pids, sizeof pids, &bytesNeeded)) return false;
		const DWORD count = bytesNeeded / sizeof(DWORD);

		for (DWORD i = 0; i < count; i++) {
			HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION | PROCESS_VM_READ, FALSE, pids[i]);
			if (!h) continue;
			wchar_t name[MAX_PATH] = {0};
			DWORD len = GetModuleBaseNameW(h, NULL, name, MAX_PATH);
			CloseHandle(h);
			if (len == 0) continue;

			// 1. Case-insensitive match against the exact-name table. Highest
			//    confidence; produces the canonical human name in the UI.
			for (const auto& tool : kKnownTools) {
				if (_wcsicmp(name, tool.exeName) == 0) {
					outName = tool.humanName;
					return true;
				}
			}

			// 2. Substring fallback. Lowercase the process name once, then
			//    require both required substrings to appear. Cheap; the loop
			//    only runs when the exact match above didn't fire.
			std::wstring lower(name, len);
			for (auto& c : lower) c = (wchar_t)towlower(c);
			for (const auto& pat : kSubstringTools) {
				if (lower.find(pat.requireA) != std::wstring::npos &&
				    lower.find(pat.requireB) != std::wstring::npos) {
					// Convert the actual filename to UTF-8 for display.
					// The user benefits from seeing the real binary name when
					// it's something we hadn't catalogued in kKnownTools.
					int n = WideCharToMultiByte(CP_UTF8, 0, name, (int)len, nullptr, 0, nullptr, nullptr);
					std::string utf8(n, '\0');
					WideCharToMultiByte(CP_UTF8, 0, name, (int)len, utf8.data(), n, nullptr, nullptr);
					outName = utf8;
					return true;
				}
			}
		}
		return false;
	}
}

void InitCalibrator()
{
	Driver.Connect();
	shmem.Open(OPENVR_SPACECALIBRATOR_SHMEM_NAME);
}

// Called by IPCClient::SendBlocking after a successful reconnect. vrserver crashing
// and respawning destroys the named file mapping that backs the shmem segment; the
// overlay's mapped view silently detaches and ReadNewPoses begins reading zeros.
// Tearing down and reopening the segment restores the link to the new driver process.
// On Open() failure we leave shmem in a closed state — the next reconnect will retry,
// and ReadNewPoses already guards against pData == nullptr by throwing.
void ReopenShmem()
{
	try {
		shmem.Close();
		shmem.Open(OPENVR_SPACECALIBRATOR_SHMEM_NAME);
	}
	catch (const std::exception& e) {
		// Close already happened; leave the segment closed and let the next reconnect retry.
		fprintf(stderr, "[ReopenShmem] failed to reopen pose shmem after reconnect: %s\n", e.what());
	}
}

double GetActiveLatencyOffsetMs(const CalibrationContext& ctx)
{
	return ctx.latencyAutoDetect ? ctx.estimatedLatencyOffsetMs : ctx.targetLatencyOffsetMs;
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
		if (a.predictionSmoothness != b.predictionSmoothness) return false;
		if (a.recalibrateOnMovement != b.recalibrateOnMovement) return false;
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
		if (a.predictionSmoothness != b.predictionSmoothness) return false;
		if (a.recalibrateOnMovement != b.recalibrateOnMovement) return false;
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

	// Per-tracking-system cache so multi-ecosystem setups (3+ systems) don't
	// thrash IPC: each system's fallback is compared against its OWN last-sent
	// value. The previous single-slot g_lastFallback worked when only one
	// system ever had a fallback active, but with extras we send N fallbacks
	// per scan tick, and a single-slot cache would miss on every other call.
	std::unordered_map<std::string, protocol::SetTrackingSystemFallback> g_lastFallbacksBySystem;

	void SendFallbackIfChanged(const std::string& systemName, bool enabled,
		const Eigen::Vector3d& translationCm, const Eigen::Quaterniond& rotation, double scale,
		uint8_t predictionSmoothness, bool recalibrateOnMovement)
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
		payload.predictionSmoothness = predictionSmoothness;
		payload.recalibrateOnMovement = recalibrateOnMovement;

		auto it = g_lastFallbacksBySystem.find(systemName);
		if (it != g_lastFallbacksBySystem.end() && FallbackPayloadEqual(it->second, payload)) {
			return;
		}

		protocol::Request req(protocol::RequestSetTrackingSystemFallback);
		req.setTrackingSystemFallback = payload;
		Driver.SendBlocking(req);
		g_lastFallbacksBySystem[systemName] = payload;
		// Legacy single-slot cache kept for any code that still reads it.
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

// Per-scan record of which device IDs (and their human-friendly identity) we
// applied a per-target-system transform to last time. Used to log
// adopted/disconnected events when the set changes scan-over-scan.
namespace {
	struct AdoptedTracker {
		std::string serial;
		std::string model;
	};
	// Indexed by OpenVR ID. Empty entries mean the slot was not adopted last scan.
	std::map<uint32_t, AdoptedTracker> g_lastAdoptedTrackers;
}

void ScanAndApplyProfile(CalibrationContext &ctx)
{
	std::unique_ptr<char[]> buffer_array(new char [vr::k_unMaxPropertyStringSize]);
	char* buffer = buffer_array.get();
	ctx.enabled = ctx.validProfile;

	// Snapshot of which IDs got adopted this scan and what serial/model they had.
	// Compared against g_lastAdoptedTrackers below to log new-adoption / disconnect events.
	std::map<uint32_t, AdoptedTracker> currentAdopted;

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
	// update — without waiting for the next per-ID scan. The fallback's freeze flag
	// fires whenever an external smoothing tool was detected and auto-suppress is on:
	// any newly-connected matching-system tracker (handled exclusively by the
	// fallback path until the next 1Hz scan tick promotes it to a per-ID transform)
	// gets clean-velocity behaviour from its very first pose update.
	if (ctx.enabled && !ctx.targetTrackingSystem.empty()) {
		auto euler = ctx.calibratedRotation * EIGEN_PI / 180.0;
		Eigen::Quaterniond rotQuat =
			Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
		// Per-tracking-system fallback never carries a smoothness value: the
		// fallback applies to ANY device of that system that doesn't have an
		// active per-ID transform, including potentially the HMD or a freshly-
		// connected reference/target which we hard-block from suppression. The
		// per-ID path below sends per-tracker smoothness; the fallback path
		// stays at 0 to avoid surprise-suppressing a device the user didn't
		// individually opt in.
		SendFallbackIfChanged(ctx.targetTrackingSystem, true,
			ctx.calibratedTranslation, rotQuat, ctx.calibratedScale,
			/*predictionSmoothness=*/0,
			ctx.recalibrateOnMovement);
	}

	// Multi-ecosystem extras: each entry contributes its own per-tracking-
	// system fallback, applied to every device of that system that lacks a
	// per-ID transform. Driver-side, these go into separate slots in the
	// systemFallbacks[8] array, so they don't interfere. Each entry's
	// per-system fallback is sent only when the entry itself is valid + enabled
	// AND its target tracking system is non-empty AND distinct from the
	// primary's (sending a duplicate fallback for the primary's system would
	// race the primary's send above and cause flicker).
	for (const auto& extra : ctx.additionalCalibrations) {
		if (!extra.enabled || !extra.valid) continue;
		if (extra.targetTrackingSystem.empty()) continue;
		if (extra.targetTrackingSystem == ctx.targetTrackingSystem) continue;

		auto eulerE = extra.calibratedRotation * EIGEN_PI / 180.0;
		Eigen::Quaterniond rotQuatE =
			Eigen::AngleAxisd(eulerE(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(eulerE(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(eulerE(2), Eigen::Vector3d::UnitX());
		SendFallbackIfChanged(extra.targetTrackingSystem, true,
			extra.calibratedTranslation, rotQuatE, extra.calibratedScale,
			/*predictionSmoothness=*/0,
			ctx.recalibrateOnMovement);
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

		// Native prediction-suppression. Looks up the per-tracker smoothness
		// (0..100) the user picked in the Prediction tab, with three hard
		// blocks: HMD, calibration reference, and calibration target. Those
		// must always run with raw poses -- suppressing the HMD causes judder
		// in the user's view, and suppressing the calibration trackers feeds
		// the math zeroed velocity (defeating cross-correlation latency
		// estimation, motion-gated blend, etc.).
		uint8_t smoothness = 0;
		if (!g_lastSeenSerial[id].empty()) {
			auto it = ctx.trackerSmoothness.find(g_lastSeenSerial[id]);
			if (it != ctx.trackerSmoothness.end()) {
				int v = it->second;
				if (v < 0) v = 0;
				if (v > 100) v = 100;
				smoothness = (uint8_t)v;
			}
		}
		const bool isHmd = vr::VRSystem()->GetTrackedDeviceClass(id)
			== vr::TrackedDeviceClass_HMD;
		const bool isRefOrTarget =
			static_cast<int32_t>(id) == ctx.referenceID
			|| static_cast<int32_t>(id) == ctx.targetID;
		if (isHmd || isRefOrTarget) {
			smoothness = 0;
		}
		payload.predictionSmoothness = smoothness;

		// Motion-gated blend — when on, the driver-side BlendTransform's lerp
		// only advances proportional to detected per-frame motion. Hides offset
		// shifts in the user's natural movement; eliminates "phantom drift" while
		// stationary. Default on at the profile level.
		payload.recalibrateOnMovement = ctx.recalibrateOnMovement;

		SendDeviceTransformIfChanged(id, payload);

		// Record this ID as adopted (it's receiving a per-target-system transform with
		// enabled=true). g_lastSeenSerial[id] is freshly populated above; pair it with
		// the model name for log readability. RenderModel falls back to empty string
		// on failure — we don't gate the log on that.
		AdoptedTracker tracker;
		tracker.serial = g_lastSeenSerial[id];
		char modelBuf[256] = {0};
		vr::ETrackedPropertyError modelErr = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, modelBuf, sizeof modelBuf, &modelErr);
		if (modelErr == vr::TrackedProp_Success) tracker.model = modelBuf;
		currentAdopted[id] = tracker;
	}

	// Diff against the previous scan: log new adoptions and disconnects. Skipped
	// when the profile is disabled so we don't spam the log on profile-clear.
	if (ctx.enabled) {
		for (const auto& kv : currentAdopted) {
			if (g_lastAdoptedTrackers.find(kv.first) == g_lastAdoptedTrackers.end()) {
				char buf[512];
				snprintf(buf, sizeof buf, "Adopted new tracker: %s/%s\n",
					kv.second.model.empty() ? "(unknown model)" : kv.second.model.c_str(),
					kv.second.serial.empty() ? "(no serial)" : kv.second.serial.c_str());
				CalCtx.Log(buf);
			}
		}
		for (const auto& kv : g_lastAdoptedTrackers) {
			if (currentAdopted.find(kv.first) == currentAdopted.end()) {
				char buf[512];
				snprintf(buf, sizeof buf, "Tracker disconnected: %s\n",
					kv.second.serial.empty() ? "(no serial)" : kv.second.serial.c_str());
				CalCtx.Log(buf);
			}
		}
	}
	g_lastAdoptedTrackers = std::move(currentAdopted);

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

	// Resolve LockMode -> lockRelativePosition every tick before any code
	// downstream reads the bool. The detector itself is updated in
	// CollectSample further down; this just transcribes mode + detector
	// state into the resolved field.
	ctx.ResolveLockMode();

	// Bounds-check the device IDs once at the top of the tick. Many code paths
	// downstream index devicePoses[ctx.referenceID] / devicePoses[ctx.targetID]
	// directly (CollectSample, the sample-history pose recording near the end of
	// this function, etc.), and a stale negative or out-of-range value reaches
	// for memory outside the array. We tolerate -1 (the not-yet-assigned sentinel)
	// because state machines below explicitly handle that, but anything else that
	// isn't in [0, k_unMaxTrackedDeviceCount) means we cannot run any per-device
	// logic this tick — bail out and try again next tick.
	const int32_t maxId = (int32_t)vr::k_unMaxTrackedDeviceCount;
	auto idInRangeOrUnset = [maxId](int32_t id) {
		return id == -1 || (id >= 0 && id < maxId);
	};
	if (!idInRangeOrUnset(ctx.referenceID) || !idInRangeOrUnset(ctx.targetID)) {
		// Defensive reset: a corrupted ID is unrecoverable for this tick. Don't
		// touch state — we just skip the tick so the next AssignTargets() call can
		// reseat the IDs cleanly.
		return;
	}

	if (ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby) {
		ctx.ClearLogOnMessage();

		if (CalCtx.requireTriggerPressToApply && (time - ctx.timeLastAssign) > 10) {
			// rescan devices every 10 seconds or so if we are using controller data
			ctx.timeLastAssign = time;
			AssignTargets();
		}
	}

	// Sudden-tracking-shift watchdog. The 50-rejection watchdog inside CalibrationCalc
	// only fires after ~25s of consistent rejection; that's appropriate for genuinely
	// degraded calibration but too slow for catastrophic geometry shifts (a lighthouse
	// gets bumped, a tracker goes through a portal, etc.). Here we look at the recent
	// error_currentCal time series: when the last error sample is more than 5x the
	// 30-tick rolling median for 3 consecutive ticks, the calibration is almost
	// certainly invalid even if CalibrationCalc still considers it valid. We force a
	// Clear() and demote to ContinuousStandby so the next AssignTargets cycle starts a
	// fresh calibration. Done from the overlay side (not CalibrationCalc) so we don't
	// touch shared math code.
	{
		static int s_consecutiveBadTicks = 0;
		static double s_lastErrorTs = 0.0;
		const auto& errSeries = Metrics::error_currentCal;
		const int N = errSeries.size();
		if (N >= 5 && calibration.isValid()) {
			// Only count an excursion once per new error sample (the time series is
			// shared and may not advance every tick).
			double thisTs = errSeries[N - 1].first;
			if (thisTs > s_lastErrorTs) {
				s_lastErrorTs = thisTs;
				const int window = std::min(30, N);
				std::vector<double> tail;
				tail.reserve(window);
				for (int i = N - window; i < N; i++) tail.push_back(errSeries[i].second);
				std::sort(tail.begin(), tail.end());
				double median = tail[tail.size() / 2];
				double current = errSeries[N - 1].second;
				if (median > 1e-9 && current > 5.0 * median) {
					s_consecutiveBadTicks++;
				} else {
					s_consecutiveBadTicks = 0;
				}
				if (s_consecutiveBadTicks >= 3) {
					CalCtx.Log("Tracking geometry shifted — restarting calibration\n");
					calibration.Clear();
					ctx.state = CalibrationState::ContinuousStandby;
					ctx.relativePosCalibrated = false;
					s_consecutiveBadTicks = 0;
				}
			}
		} else {
			s_consecutiveBadTicks = 0;
		}
	}

	// Latency cross-correlation. Once per second, if the rolling speed buffers
	// are full and the user has actually been moving (RMS speed > 0.1 m/s on both
	// signals), compute a discrete cross-correlation and update the EMA estimate.
	// The active value is then used by CollectSample on subsequent ticks when
	// latencyAutoDetect is on.
	if ((time - ctx.timeLastLatencyEstimate) > 1.0
		&& ctx.refSpeedHistory.size() >= CalibrationContext::kLatencyHistoryCapacity
		&& ctx.targetSpeedHistory.size() >= CalibrationContext::kLatencyHistoryCapacity
		&& ctx.speedSampleTimes.size() >= CalibrationContext::kLatencyHistoryCapacity)
	{
		ctx.timeLastLatencyEstimate = time;
		double lagSamples = 0.0;
		const int kMaxTau = 10;
		if (EstimateLatencyLagSamples(ctx.refSpeedHistory, ctx.targetSpeedHistory, kMaxTau, &lagSamples)) {
			// Convert sample lag to ms using the *empirical* sample rate from the
			// timestamp ring. This is more honest than assuming a fixed 20 Hz: the
			// rate is whatever CollectSample is being called at right now.
			double dur =
				ctx.speedSampleTimes.back() - ctx.speedSampleTimes.front();
			size_t intervals = ctx.speedSampleTimes.size() - 1;
			if (dur > 1e-3 && intervals > 0) {
				double sampleRateHz = (double)intervals / dur;
				double lagMs = lagSamples * 1000.0 / sampleRateHz;
				// Bound the per-update step to keep one bad correlation from
				// teleporting the offset estimate.
				if (std::isfinite(lagMs) && std::fabs(lagMs) <= 200.0) {
					ctx.estimatedLatencyOffsetMs = 0.7 * ctx.estimatedLatencyOffsetMs + 0.3 * lagMs;
				}
			}
		}
	}

	// Re-scan for known external smoothing tools every 5 seconds. Cheap; the
	// process enumeration is bounded and the result feeds both the UI banner and
	// the auto-suppress logic in ScanAndApplyProfile.
	if ((time - ctx.timeLastSmoothingScan) > 5.0) {
		ctx.timeLastSmoothingScan = time;
		std::string detectedName;
		bool detected = DetectExternalSmoothingTool(detectedName);
		if (detected != ctx.externalSmoothingDetected || detectedName != ctx.externalSmoothingToolName) {
			ctx.externalSmoothingDetected = detected;
			ctx.externalSmoothingToolName = detected ? detectedName : std::string();

			// Annotate the debug log on every detection state change. This is
			// invaluable when triaging "smoothing tool wasn't detected" reports
			// — the log will show whether the detector saw the process at all.
			char ann[256];
			if (detected) {
				snprintf(ann, sizeof ann,
					"external_smoothing_detected: %s", detectedName.c_str());
			} else {
				snprintf(ann, sizeof ann, "external_smoothing_cleared");
			}
			Metrics::WriteLogAnnotation(ann);

			if (detected) {
				char msg[256];
				snprintf(msg, sizeof msg,
					"%s detected -- not supported. Stop it and use the per-tracker smoothness sliders in the Prediction tab instead.\n",
					detectedName.c_str());
				CalCtx.Log(msg);
			}
		}
	}

	ctx.timeLastTick = time;
	shmem.ReadNewPoses([&](const protocol::DriverPoseShmem::AugmentedPose& augmented_pose) {
		if (augmented_pose.deviceId >= 0 && augmented_pose.deviceId < (int)vr::k_unMaxTrackedDeviceCount) {
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
			// Annotate the log so the post-stall garbage rows have an obvious cause.
			Metrics::WriteLogAnnotation("hmd_stall: tracking lost, sample buffer purged");
		}
		return;
	}
	if (ctx.consecutiveHmdStalls > 0) {
		// Annotate recovery so the log shows the gap clearly.
		char buf[128];
		snprintf(buf, sizeof buf,
		         "hmd_stall_recovered after %d ticks", ctx.consecutiveHmdStalls);
		Metrics::WriteLogAnnotation(buf);
	}
	ctx.consecutiveHmdStalls = 0;
	ctx.xprev = (float) p[0];
	ctx.yprev = (float) p[1];
	ctx.zprev = (float) p[2];

	// HMD recenter compensation. When the HMD's pose in reference-tracking world
	// jumps by more than legitimate motion can produce in one tick (>30 cm or
	// >30 deg), interpret it as a driver-side re-origin (Quest Home-button
	// recenter, Oculus Link reset, etc.) and apply the same delta to every
	// stored calibrated transform so non-HMD trackers stay aligned with the
	// user's body. Without this, every recenter breaks tracking visibly and
	// the user has to manually recalibrate.
	//
	// Gates: master kill switch silentRecalEnabled must be on (default off
	// after Phase 1+2 produced worse tracking in real-world testing); a
	// profile must be loaded; previous HMD frame must have been valid AND
	// recent (<0.5 s) so we don't conflate tracking-dropout return with a
	// recenter; calibration must not be actively running (the cal math pins
	// the reference frame across collection and a mid-buffer compensation
	// would corrupt samples).
	if (ctx.silentRecalEnabled) {
		static bool s_havePrevHmdPose = false;
		static Eigen::Affine3d s_prevHmdPose = Eigen::Affine3d::Identity();
		static double s_prevHmdTime = 0.0;

		const auto& hmdRaw = ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
		const bool hmdGood = hmdRaw.poseIsValid && hmdRaw.deviceIsConnected
			&& hmdRaw.result == vr::ETrackingResult::TrackingResult_Running_OK;

		if (hmdGood) {
			Pose hmdPoseWorld = ConvertPose(hmdRaw);
			Eigen::Affine3d hmdPose = Eigen::Affine3d::Identity();
			hmdPose.linear() = hmdPoseWorld.rot;
			hmdPose.translation() = hmdPoseWorld.trans;

			const bool calibrationLoaded = ctx.validProfile && ctx.enabled;
			const bool inActiveCal =
				ctx.state == CalibrationState::Begin ||
				ctx.state == CalibrationState::Rotation ||
				ctx.state == CalibrationState::Translation ||
				ctx.state == CalibrationState::Continuous;

			if (s_havePrevHmdPose && (time - s_prevHmdTime) < 0.5
				&& calibrationLoaded && !inActiveCal)
			{
				const Eigen::Vector3d posDelta = hmdPose.translation() - s_prevHmdPose.translation();
				const double posMag = posDelta.norm();

				const Eigen::Quaterniond qNew(hmdPose.linear());
				const Eigen::Quaterniond qOld(s_prevHmdPose.linear());
				Eigen::Quaterniond rotDelta = qNew * qOld.conjugate();
				rotDelta.normalize();
				const double angRad = 2.0 * std::acos(std::min(1.0, std::abs(rotDelta.w())));

				constexpr double kPosJumpM = 0.30;
				constexpr double kAngJumpRad = 30.0 * EIGEN_PI / 180.0;

				if (posMag > kPosJumpM || angRad > kAngJumpRad) {
					// Rigid recenter delta in ref-tracking-world: new = delta * old.
					const Eigen::Affine3d delta = hmdPose * s_prevHmdPose.inverse();

					// Apply: new_R = delta_R * R, new_T = delta_R * T + delta_T.
					// transCm is in centimeters, rotDeg is intrinsic Euler ZYX
					// in degrees -- the same encoding used everywhere else for
					// the stored calibration result.
					auto applyDelta = [&delta](Eigen::Vector3d& transCm, Eigen::Vector3d& rotDeg) {
						const Eigen::Vector3d eulerRad = rotDeg * EIGEN_PI / 180.0;
						const Eigen::Quaterniond rotQ =
							Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
							Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
							Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX());
						const Eigen::Affine3d cal = Eigen::Translation3d(transCm * 0.01) * rotQ;
						const Eigen::Affine3d newCal = delta * cal;
						transCm = newCal.translation() * 100.0;
						const Eigen::Matrix3d newRot = newCal.linear();
						rotDeg = newRot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
					};

					applyDelta(ctx.calibratedTranslation, ctx.calibratedRotation);
					for (auto& extra : ctx.additionalCalibrations) {
						if (extra.valid) {
							applyDelta(extra.calibratedTranslation, extra.calibratedRotation);
						}
					}

					char logbuf[256];
					snprintf(logbuf, sizeof logbuf,
						"HMD recenter detected (%.0f cm, %.0f deg jump) - calibration delta-corrected\n",
						posMag * 100.0, angRad * 180.0 / EIGEN_PI);
					ctx.Log(logbuf);
					Metrics::WriteLogAnnotation("hmd_recenter: calibration delta-corrected");

					InvalidateAllTransformCaches();

					// The silent-recal sample buffer's existing samples were
					// collected in the OLD reference-tracking-world. Mixing
					// those with post-recenter samples produces a corrupted
					// fit that mostly captures the recenter delta itself.
					// Drop the buffer + EMA and let it re-fill from scratch.
					// Reset every per-trigger hold timer too: a hold that
					// was in progress was being timed against the old frame
					// and shouldn't carry over.
					silentRecalCalc.Clear();
					silentRecalEMAPrimed = false;
					silentRecalTPoseHoldStartTime = -1.0;
					silentRecalIdleHoldStartTime = -1.0;
					silentRecalHandOnHmdStartTime = -1.0;
					silentRecalFloorTouchStartTime = -1.0;
					silentRecalHmdWakeTime = 0.0;
				}
			}

			s_prevHmdPose = hmdPose;
			s_prevHmdTime = time;
			s_havePrevHmdPose = true;
		} else {
			// Tracking is not Running_OK. Drop the cached prev so we don't
			// compute a bogus delta on the first good frame after recovery.
			s_havePrevHmdPose = false;
		}
	}

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
		// Phase 1+2 silent drift-correction subsystem. Master kill switch is
		// CalCtx.silentRecalEnabled (default OFF) -- the subsystem produced
		// worse tracking in real-world testing than leaving the calibration
		// alone, and its internal "Not updating: ..." diagnostics leaked into
		// the user-facing Calibration Progress popup. Off-by-default keeps
		// one-shot users on the well-tested pre-Phase-1 behaviour; opt-in
		// from the Advanced tab.
		if (ctx.silentRecalEnabled && ctx.validProfile && ctx.enabled) {
			TickSilentTPoseRecal(time);
		}
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

		// (Removed: the original code pushed jitter here, in the Begin state,
		// before CollectSample had ever populated calibration.m_samples. The
		// pushed value was always 0.0 from an empty buffer, which then
		// poisoned ResolvedCalibrationSpeed's read of Metrics::jitterRef.last()
		// and locked AUTO onto FAST regardless of real tracker quality. The
		// authoritative push now lives at the end of CollectSample, so jitter
		// reflects the live buffer every active-calibration tick.)

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

	// One-shot motion-variety gate. Continuous mode bypasses this -- it has its
	// own incremental accept/reject loop that doesn't need a "stop here" signal.
	// One-shot's prior behaviour was "buffer fills -> run ComputeOneshot once,
	// pass or fail". With AUTO calibration speed picking VERY_SLOW (500 samples
	// = ~25 s) for noisy IMU rigs, the user could wave for 25 seconds and then
	// get rejected because their motion only covered one axis -- a frustrating
	// outcome the program could have prevented.
	//
	// New flow: once the buffer is full (noise averaging satisfied), we still
	// require both motion-coverage diversities >= 70 % before running the math.
	// Below that threshold, drop the oldest sample and keep collecting -- the
	// buffer rolls so fresh motion replaces stale, the bars in the popup keep
	// climbing, and the user finishes naturally when their motion is varied
	// enough. Both "have we waved enough" (buffer) and "varied enough" (bars)
	// gates are visible in the popup so the user knows what's holding them up.
	if (CalCtx.state != CalibrationState::Continuous
		&& CalCtx.state != CalibrationState::ContinuousStandby)
	{
		constexpr double kAutoFinishDiversity = 0.70;
		if (calibration.TranslationDiversity() < kAutoFinishDiversity
			|| calibration.RotationDiversity() < kAutoFinishDiversity)
		{
			calibration.ShiftSample(); // drop oldest, keep buffer rolling
			return;
		}
	}

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
		// User-toggled "Pause updates" from the continuous-cal UI: keep the
		// already-applied driver offset live, skip any new solve cycle so the
		// math doesn't fight the user trying to inspect the current result.
		if (!CalCtx.calibrationPaused) {
			calibration.ComputeIncremental(lerp, CalCtx.continuousCalibrationThreshold, CalCtx.maxRelativeErrorThreshold, CalCtx.ignoreOutliers);
		}

		// Multi-ecosystem extras: each runs its own continuous calibration
		// loop in parallel with the primary, against the SAME reference
		// device (the HMD) and its own target. Each extra has its own
		// sample buffer (extra.calc) so noisy samples on one don't taint
		// another. Cheap -- the math is bounded by sample-buffer size and
		// runs at the same low cadence as the primary.
		for (auto& extra : CalCtx.additionalCalibrations) {
			if (!extra.enabled) continue;
			if (extra.referenceID < 0 || extra.targetID < 0) continue;
			if (extra.referenceID >= maxId || extra.targetID >= maxId) continue;

			const auto& refPose = CalCtx.devicePoses[extra.referenceID];
			const auto& tgtPose = CalCtx.devicePoses[extra.targetID];
			if (!refPose.poseIsValid || !tgtPose.poseIsValid) continue;
			if (refPose.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;
			if (tgtPose.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;

			Sample s(ConvertPose(refPose), ConvertPose(tgtPose), glfwGetTime());
			extra.calc->PushSample(s);
			while (extra.calc->SampleCount() > CalCtx.SampleCount()) extra.calc->ShiftSample();

			// Per-extra auto-lock detector update.
			Eigen::AffineCompact3d refW = Eigen::AffineCompact3d::Identity();
			refW.linear() = ConvertPose(refPose).rot;
			refW.translation() = ConvertPose(refPose).trans;
			Eigen::AffineCompact3d tgtW = Eigen::AffineCompact3d::Identity();
			tgtW.linear() = ConvertPose(tgtPose).rot;
			tgtW.translation() = ConvertPose(tgtPose).trans;
			const Eigen::AffineCompact3d rel = refW.inverse() * tgtW;
			extra.autoLockHistory.push_back(rel);
			while (extra.autoLockHistory.size() > 30) extra.autoLockHistory.pop_front();
			// (mirrors the primary detector; a tiny duplication is fine for now)
			if (extra.autoLockHistory.size() >= 30) {
				Eigen::Vector3d meanT = Eigen::Vector3d::Zero();
				for (const auto& a : extra.autoLockHistory) meanT += a.translation();
				meanT /= (double)extra.autoLockHistory.size();
				double translVar = 0.0;
				for (const auto& a : extra.autoLockHistory) {
					translVar += (a.translation() - meanT).squaredNorm();
				}
				translVar /= (double)extra.autoLockHistory.size();
				const double translStd = std::sqrt(translVar);
				const auto& medRot = extra.autoLockHistory[extra.autoLockHistory.size() / 2].rotation();
				Eigen::Quaterniond medQ(medRot);
				double rotMaxAng = 0.0;
				for (const auto& a : extra.autoLockHistory) {
					double ang = medQ.angularDistance(Eigen::Quaterniond(a.rotation()));
					if (ang > rotMaxAng) rotMaxAng = ang;
				}
				extra.autoLockEffectivelyLocked =
					(translStd < 0.005) && (rotMaxAng < 1.0 * EIGEN_PI / 180.0);
			}
			// Resolve effective lock for this extra.
			switch (extra.lockMode) {
			case 0:  extra.lockRelativePosition = false; break;
			case 1:  extra.lockRelativePosition = true; break;
			default: extra.lockRelativePosition = extra.autoLockEffectivelyLocked; break;
			}

			extra.calc->lockRelativePosition = extra.lockRelativePosition;
			extra.calc->enableStaticRecalibration = CalCtx.enableStaticRecalibration;

			if (!CalCtx.calibrationPaused && extra.calc->SampleCount() >= CalCtx.SampleCount()) {
				bool extraLerp = false;
				if (extra.calc->ComputeIncremental(extraLerp,
						CalCtx.continuousCalibrationThreshold,
						CalCtx.maxRelativeErrorThreshold,
						CalCtx.ignoreOutliers)) {
					if (extra.calc->isValid()) {
						extra.calibratedRotation = extra.calc->EulerRotation();
						extra.calibratedTranslation =
							extra.calc->Transformation().translation() * 100.0;
						extra.refToTargetPose = extra.calc->RelativeTransformation();
						extra.relativePosCalibrated = extra.calc->isRelativeTransformationCalibrated();
						extra.valid = true;
					}
				}
			}
		}
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