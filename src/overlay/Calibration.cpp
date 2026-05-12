#include "stdafx.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "Configuration.h"
#include "IPCClient.h"
#include "CalibrationCalc.h"
#include "VRState.h"
#include "WedgeDetector.h"   // ShouldFireRuntimeWedgeRecovery, kMaxPlausibleCalibrationMagnitudeCm
#include "GeometryShiftDetector.h"  // IsCurrentErrorSpike, ShouldFireGeometryShiftRecovery
#include "MotionGate.h"      // ShouldBlendCycle -- auto-recovery snap decision
#include "LatencyEstimator.h"  // spacecal::latency::EstimateLagTimeDomain / EstimateLagGccPhat
#include "TiltDiagnostic.h"    // spacecal::gravity::EvaluateTilt -- sustained gravity-disagreement annotation
#include "Wizard.h"          // spacecal::wizard::IsActive -- gate the runtime wedge detector
                             // off while the user is mid-setup so it can't disrupt them.
#include "RestLockedYaw.h"   // spacecal::rest_yaw::* -- rest-locked yaw drift correction
                             // (continuous-cal-OFF mode). Default OFF; opt-in via Experimental tab.
#include "RecoveryDeltaBuffer.h" // spacecal::recovery_delta::* -- predictive pre-correction
                             // from the rolling buffer of 30 cm relocalization events.
#include "ReanchorChiSquareDetector.h" // spacecal::reanchor_chi::* -- sub-30 cm re-anchor
                             // sub-detector. Detection-only; freezes recs A/C on candidate.

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <map>

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

// Forward declaration. Defined alongside the auto-recovery snapshot APIs
// near the bottom of the file but called from TickHmdRelocalizationDetector
// (Quest re-localization auto-recovery) and CalibrationTick (runtime wedge
// detector), both of which sit above the definition.
static void RecoverFromWedgedCalibration(const char* userFacingMessage);

// Runtime wedge detector state. Persists across CalibrationTick calls;
// updated by spacecal::wedge::ShouldFireRuntimeWedgeRecovery (header-only,
// pure). Sentinel <0 means "no active wedge candidate"; otherwise stores
// the glfwGetTime() timestamp of the first tick magnitude crossed the bound.
// File scope (not anonymous namespace) so CalibrationTick can read+write it
// directly from earlier in the translation unit.
static double g_runtimeWedgeSince = -1.0;

// CPU-pressure diagnostic. Samples GetProcessTimes() once per CalibrationTick
// (~20 Hz). Computes the % of total CPU the SC process used over the wall-clock
// delta since the last sample, divided by the logical-processor count so 100%
// means "fully saturating one core's worth of compute". Maintains a 5-second
// EMA so a one-tick computationTime spike doesn't false-trigger; emits a
// `cpu_pressure_warning_on` annotation when the EMA crosses 50% and a
// `_off` annotation when it falls back below 30% (hysteresis stops flapping).
//
// Pure diagnostic. No behavior change. The emit-once-on-transition pattern
// matches the existing gravity_disagreement_sustained_on/off annotations.
struct CpuPressureState {
    bool initialized = false;
    bool alarmed = false;
    uint64_t lastCpuNs = 0;
    uint64_t lastWallNs = 0;
    double emaPct = 0.0;       // 0..100 in single-core-equivalent percent
    int logicalProcessors = 1;
};
static CpuPressureState g_cpuPressureState;

constexpr double kCpuPressureOnThresholdPct  = 50.0;
constexpr double kCpuPressureOffThresholdPct = 30.0;
constexpr double kCpuPressureEmaTimeConstSec = 5.0;
constexpr double kCpuPressureSpikeMs         = 200.0;  // single-tick spike threshold

inline uint64_t FileTimeToNs100(const FILETIME& ft) {
    return ((uint64_t)ft.dwHighDateTime << 32) | ft.dwLowDateTime;  // units of 100 ns
}

// Samples this tick's CPU usage + folds into the EMA. Emits transition
// annotations on EMA crossing and a one-shot `cpu_pressure_spike` on any
// single ComputeIncremental that took >= kCpuPressureSpikeMs.
static void TickCpuPressureMonitor(double computationTimeMs, double now_s) {
    auto& s = g_cpuPressureState;
    FILETIME ftCreate, ftExit, ftKernel, ftUser;
    if (!GetProcessTimes(GetCurrentProcess(), &ftCreate, &ftExit, &ftKernel, &ftUser)) {
        return;  // bail silently; this is diagnostic only
    }
    const uint64_t cpuNow = FileTimeToNs100(ftKernel) + FileTimeToNs100(ftUser);
    LARGE_INTEGER pcNow;
    QueryPerformanceCounter(&pcNow);
    LARGE_INTEGER pcFreq;
    QueryPerformanceFrequency(&pcFreq);
    const uint64_t wallNow = (uint64_t)((pcNow.QuadPart * 10'000'000ull) / pcFreq.QuadPart);  // 100ns

    if (!s.initialized) {
        SYSTEM_INFO si{};
        GetSystemInfo(&si);
        s.logicalProcessors = si.dwNumberOfProcessors > 0 ? (int)si.dwNumberOfProcessors : 1;
        s.lastCpuNs = cpuNow;
        s.lastWallNs = wallNow;
        s.initialized = true;
        return;
    }

    const uint64_t cpuDelta = cpuNow >= s.lastCpuNs ? cpuNow - s.lastCpuNs : 0;
    const uint64_t wallDelta = wallNow >= s.lastWallNs ? wallNow - s.lastWallNs : 0;
    s.lastCpuNs = cpuNow;
    s.lastWallNs = wallNow;
    if (wallDelta == 0) return;

    const double instPct = 100.0 * (double)cpuDelta /
        ((double)wallDelta * (double)s.logicalProcessors);

    // EMA toward instPct with time constant kCpuPressureEmaTimeConstSec.
    const double dtSec = (double)wallDelta / 1.0e7;
    const double alpha = 1.0 - std::exp(-dtSec / kCpuPressureEmaTimeConstSec);
    s.emaPct = s.emaPct + alpha * (instPct - s.emaPct);

    // EMA-crossing transition annotations with hysteresis.
    if (!s.alarmed && s.emaPct > kCpuPressureOnThresholdPct) {
        s.alarmed = true;
        char buf[256];
        snprintf(buf, sizeof buf,
            "cpu_pressure_warning_on: ema_pct=%.1f inst_pct=%.1f cores=%d"
            " upstream=%d gcc_phat=%d cusum=%d velocity_aware=%d tukey=%d kalman=%d"
            " auto_detect_latency=%d state=%d",
            s.emaPct, instPct, s.logicalProcessors,
            (int)CalCtx.useUpstreamMath,
            (int)CalCtx.useGccPhatLatency, (int)CalCtx.useCusumGeometryShift,
            (int)CalCtx.useVelocityAwareWeighting, (int)CalCtx.useTukeyBiweight,
            (int)CalCtx.useBlendFilter, (int)CalCtx.latencyAutoDetect,
            (int)CalCtx.state);
        Metrics::WriteLogAnnotation(buf);
    } else if (s.alarmed && s.emaPct < kCpuPressureOffThresholdPct) {
        s.alarmed = false;
        char buf[128];
        snprintf(buf, sizeof buf,
            "cpu_pressure_warning_off: ema_pct=%.1f", s.emaPct);
        Metrics::WriteLogAnnotation(buf);
    }

    // Per-tick spike annotation. Independent of the EMA; a single
    // ComputeIncremental that took >= 200 ms is worth surfacing on its own
    // because it indicates a stall-class event regardless of session-average
    // CPU. Throttled to one annotation per 5 seconds so a sustained-high
    // window doesn't flood.
    static double s_lastSpikeAnnotation = -1e9;
    if (computationTimeMs >= kCpuPressureSpikeMs && (now_s - s_lastSpikeAnnotation) >= 5.0) {
        s_lastSpikeAnnotation = now_s;
        char buf[256];
        snprintf(buf, sizeof buf,
            "cpu_pressure_spike: computationTime_ms=%.1f ema_pct=%.1f"
            " upstream=%d gcc_phat=%d cusum=%d velocity_aware=%d tukey=%d kalman=%d state=%d",
            computationTimeMs, s.emaPct,
            (int)CalCtx.useUpstreamMath,
            (int)CalCtx.useGccPhatLatency, (int)CalCtx.useCusumGeometryShift,
            (int)CalCtx.useVelocityAwareWeighting, (int)CalCtx.useTukeyBiweight,
            (int)CalCtx.useBlendFilter, (int)CalCtx.state);
        Metrics::WriteLogAnnotation(buf);
    }
}

// Auto-recovery snap flag (option-3 bundle, 2026-05-04). Set true by
// RecoverFromWedgedCalibration so the next ScanAndApplyProfile cycle sends
// every device transform with payload.lerp=false, which routes through the
// driver's existing snap path in SetDeviceTransform (transform := target,
// no blending). Without this, the post-recovery cal would be smoothly
// interpolated through whatever wrong steady-state the driver had cached
// -- defeating the point of recovery.
//
// Rationale (per feedback_calibration_blending_request.md): blending is
// for smooth steady-state convergence. Recovery events are catastrophic
// state changes -- smoothing them is wrong. Snap on the next tick, then
// resume normal blending.
//
// Cleared by ScanAndApplyProfile after consuming. One-shot.
static bool g_snapNextProfileApply = false;

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

	const bool prevLocked = autoLockEffectivelyLocked;
	autoLockEffectivelyLocked =
		(translStdDev < kAutoLockTranslThreshM) &&
		(rotMaxAngle < kAutoLockRotThreshRad);

	// Forensic diagnostic for audit row #11 (project_upstream_regression_audit_2026-05-04).
	// Fork-only AUTO mode flips lockRelativePosition mid-session based on
	// observed relative-pose stability. A flip while the user thinks they're
	// in OFF mode (e.g. from misreading the UI) would change which constraint
	// CalibrateByRelPose is gated against. Fires only on transitions, so
	// healthy AUTO sessions produce one annotation per actual flip -- not a
	// flood. Use grep `auto_lock_flip:` on session logs to spot any flapping.
	if (prevLocked != autoLockEffectivelyLocked) {
		char flipbuf[200];
		snprintf(flipbuf, sizeof flipbuf,
			"auto_lock_flip: previous=%d now=%d translStdDev=%.4fm rotMaxAng=%.4frad samples=%zu",
			(int)prevLocked, (int)autoLockEffectivelyLocked,
			translStdDev, rotMaxAngle, autoLockHistory.size());
		Metrics::WriteLogAnnotation(flipbuf);
	}
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

	// Sticky state. Mutable-via-cast is fine here -- these are pure caches.
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
SCIPCClient Driver;
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
		// Normalise the composed quaternion. Eigen's quaternion multiplication
		// does not auto-normalise, so over a multi-minute session the cumulative
		// scale drift from non-unit-norm input quaternions (some drivers emit
		// quaternions slightly off the unit sphere -- Quest Pro's IMU fusion
		// occasionally produces tiny scale errors per frame) compounds into a
		// non-orthonormal rotation matrix at xform.linear(), which the Kabsch /
		// SVD downstream silently treats as a mild shear. The metrics-side
		// composer at the bottom of CalibrationTick already does this; the
		// calibration-side now matches.
		driverRot.normalize();

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

		// Validity gate. Previously this was `if (!poseIsValid && result != Running_OK)`,
		// i.e. "fail only when BOTH signals say bad" -- permissive. The case the
		// permissive form silently accepts is the one we care about: Quest Pro
		// (and other inside-out drivers) regularly report `poseIsValid == false`
		// for one tick during a relocalization while still claiming
		// `result == Running_OK`. The pose for that single tick is genuinely
		// invalid -- using it injects a phantom translation that the calibration
		// math sees as legitimate motion and tries to fit. Tighten to `||`:
		// reject if EITHER signal says bad.
		//
		// To watch the impact: every time this gate now rejects a sample that
		// the old gate would have accepted, we annotate. If those annotations
		// show up only in known-bad sessions, the change is a net win; if they
		// show up at every tick of a healthy session, the gate is too tight and
		// we revisit.
		bool ok = true;
		const bool refSilentInvalid = !reference.poseIsValid &&
			reference.result == vr::ETrackingResult::TrackingResult_Running_OK;
		const bool tgtSilentInvalid = !target.poseIsValid &&
			target.result == vr::ETrackingResult::TrackingResult_Running_OK;
		if (!reference.poseIsValid || reference.result != vr::ETrackingResult::TrackingResult_Running_OK)
		{
			CalCtx.Log("Reference device is not tracking\n"); ok = false;
		}
		if (!target.poseIsValid || target.result != vr::ETrackingResult::TrackingResult_Running_OK)
		{
			CalCtx.Log("Target device is not tracking\n"); ok = false;
		}
		if (refSilentInvalid || tgtSilentInvalid) {
			Metrics::WriteLogAnnotation(refSilentInvalid && tgtSilentInvalid
				? "silent_invalid_pose_rejected: ref+tgt"
				: refSilentInvalid
					? "silent_invalid_pose_rejected: ref"
					: "silent_invalid_pose_rejected: tgt");
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
		// history buffer (cheaper, bounded, and good enough for the small +/-100 ms range
		// the UI exposes). If velocity data is invalid the reference pose is left
		// un-shifted for this tick -- better one bad-but-bounded sample than a thrown
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
			// Delta shmem = how stale the reference pose is relative to the target pose
			// (target ahead -> positive). Then we additionally subtract the user's
			// configured offset. We extrapolate the reference forward by this delta to put
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
		// Linear velocity is preferred over angular for these speed signals -- angular
		// velocity from optical trackers is often filter-shaped (low-pass), which
		// blurs the transient that the cross-correlator looks for.
		auto speedFromVel = [](const double v[3]) -> double {
			double s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
			if (!std::isfinite(s)) return 0.0;
			return s;
		};
		const double refSpeed = speedFromVel(ctx.devicePoses[ctx.referenceID].vecVelocity);
		const double tgtSpeed = speedFromVel(ctx.devicePoses[ctx.targetID].vecVelocity);
		{
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
			glfwGetTime(),
			refSpeed,
			tgtSpeed
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
		Metrics::translationAxisRangesCm.Push(calibration.TranslationAxisRangesCm());

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

	// External smoothing-tool detection (kKnownTools, kSubstringTools,
	// DetectExternalSmoothingTool) relocated to the Smoothing overlay on
	// 2026-05-11 (Protocol v12 migration). The Smoothing plugin scans on
	// its own Tick and surfaces the banner inside its Prediction sub-tab.

	// Discrete cross-correlation between two equal-length speed series. Returns false
	// if there isn't enough signal energy to produce a trustworthy estimate. On
	// success, *lagSamplesOut is the lag (in samples) at which the correlation is
	// maximised, with sub-sample resolution from a quadratic fit around the peak.
	// Positive lag means the target signal lags the reference signal (target arrives later).
	//
	// 2026-05-05: math extracted to spacecal::latency::EstimateLagTimeDomain in
	// LatencyEstimator.h so it can be unit-tested directly. This wrapper
	// dispatches to either the original time-domain CC (default) or to the
	// GCC-PHAT variant (Knapp & Carter 1976) based on `useGccPhat`. GCC-PHAT
	// is opt-in until real-session evidence confirms the whitened-spectrum
	// estimate is preferable; the math review pinned time-domain CC as
	// "empirically validated, not a sore point".
	bool EstimateLatencyLagSamples(
		const std::deque<double>& ref,
		const std::deque<double>& tgt,
		int maxTau,
		bool useGccPhat,
		double* lagSamplesOut)
	{
		if (useGccPhat) {
			return spacecal::latency::EstimateLagGccPhat(ref, tgt, maxTau, lagSamplesOut);
		}
		return spacecal::latency::EstimateLagTimeDomain(ref, tgt, maxTau, lagSamplesOut);
	}

}

void InitCalibrator()
{
	Driver.Connect();
	shmem.Open(OPENVR_PAIRDRIVER_SHMEM_NAME);
	// Finger smoothing config is now owned by the Smoothing overlay
	// (Protocol v12, 2026-05-11). Its plugin pushes the persisted config
	// on its own driver connect; SC no longer participates in that path.
}

// Called by SCIPCClient::SendBlocking after a successful reconnect. vrserver crashing
// and respawning destroys the named file mapping that backs the shmem segment; the
// overlay's mapped view silently detaches and ReadNewPoses begins reading zeros.
// Tearing down and reopening the segment restores the link to the new driver process.
// On Open() failure we leave shmem in a closed state -- the next reconnect will retry,
// and ReadNewPoses already guards against pData == nullptr by throwing.
void ReopenShmem()
{
	try {
		shmem.Close();
		shmem.Open(OPENVR_PAIRDRIVER_SHMEM_NAME);
	}
	catch (const std::exception& e) {
		// Close already happened; leave the segment closed and let the next reconnect retry.
		fprintf(stderr, "[ReopenShmem] failed to reopen pose shmem after reconnect: %s\n", e.what());
	}
}

double GetActiveLatencyOffsetMs(const CalibrationContext& ctx)
{
	if (ctx.useUpstreamMath) return 0.0;
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

	// AlignmentSpeedParams dedupe -- avoid spamming the driver with identical params.
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

	// === Base station drift correction (one-shot mode) =======================
	//
	// Watches TrackingReference (Lighthouse base station) poses for runtime
	// universe shifts. Base stations are physically static -- their poses in
	// the runtime's tracking universe only change when the runtime re-origins
	// (chaperone reset, SetSeatedZeroPose, etc.). When ALL base stations in a
	// tracking system shift by the same rigid delta D between two consecutive
	// ticks, that's a re-origin -- we apply D (or D-^-1, depending on which
	// system shifted) to the stored calibration so body trackers stay aligned
	// with the user's physical position.
	//
	// Why this works where the deleted "HMD pose jumped" heuristic (D in the
	// old plan) didn't: HMD pose changes during natural user motion. Base
	// stations don't. The signal-to-noise ratio is dramatically higher.
	//
	// Failure mode is benign: if OpenVR doesn't update base station poses on
	// some recenter event, the detector simply doesn't fire. Worst case it
	// catches nothing; never makes tracking worse.
	struct BaseStationCacheEntry {
		std::string trackingSystem;
		Eigen::Affine3d pose;
	};
	std::map<std::string, BaseStationCacheEntry> baseStationCache; // serial -> entry
	double lastBaseStationShiftAcceptedTime = -1e9;

	// Tolerance for "this delta is significant (above pose noise)": 1 mm,
	// 0.05 deg. Static base station poses are byte-stable in the OpenVR
	// runtime under normal operation, so any motion above this is signal.
	constexpr double kBsDeltaSignificanceTransM = 0.001;
	constexpr double kBsDeltaSignificanceRotRad = 0.05 * EIGEN_PI / 180.0;

	// Tolerance for "all base stations moved by the same delta" (consensus):
	// 5 mm, 0.5 deg. Slightly looser than the significance threshold to
	// allow for sub-tick interpolation differences across base stations.
	constexpr double kBsConsensusTransM = 0.005;
	constexpr double kBsConsensusRotRad = 0.5 * EIGEN_PI / 180.0;

	double RigidDeltaAngleRad(const Eigen::Affine3d& delta) {
		const Eigen::Quaterniond q(delta.linear());
		return 2.0 * std::acos(std::min(1.0, std::abs(q.w())));
	}

	// Apply a universe shift D to every calibration whose ref or target
	// tracking system matches `system`. The math:
	//   reference world shifted by D => R_new = D * R_old
	//   target    world shifted by D => R_new = R_old * D-^-1
	// Primary calibration uses CalCtx.referenceTrackingSystem and
	// CalCtx.targetTrackingSystem. Each AdditionalCalibration only stores
	// targetTrackingSystem; its reference is implicitly the same as the
	// primary's (always HMD-side).
	void ApplyUniverseShiftToCalibrations(const Eigen::Affine3d& D, const std::string& system) {
		auto applyDelta = [&D](Eigen::Vector3d& transCm, Eigen::Vector3d& rotDeg, bool refSide) {
			const Eigen::Vector3d eulerRad = rotDeg * EIGEN_PI / 180.0;
			const Eigen::Quaterniond rotQ =
				Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
				Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX());
			const Eigen::Affine3d cal = Eigen::Translation3d(transCm * 0.01) * rotQ;
			const Eigen::Affine3d newCal = refSide ? (D * cal) : (cal * D.inverse());
			transCm = newCal.translation() * 100.0;
			rotDeg = newCal.linear().eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
		};

		const bool primaryRefShift   = (system == CalCtx.referenceTrackingSystem);
		const bool primaryTargetShift = (system == CalCtx.targetTrackingSystem);
		if (primaryRefShift) {
			applyDelta(CalCtx.calibratedTranslation, CalCtx.calibratedRotation, /*refSide=*/true);
		} else if (primaryTargetShift) {
			applyDelta(CalCtx.calibratedTranslation, CalCtx.calibratedRotation, /*refSide=*/false);
		}

		// Additional calibrations: their reference IS the primary's reference,
		// so a reference-side shift hits all of them. Their target is per-entry.
		for (auto& extra : CalCtx.additionalCalibrations) {
			if (!extra.valid) continue;
			if (primaryRefShift) {
				applyDelta(extra.calibratedTranslation, extra.calibratedRotation, /*refSide=*/true);
			} else if (extra.targetTrackingSystem == system) {
				applyDelta(extra.calibratedTranslation, extra.calibratedRotation, /*refSide=*/false);
			}
		}

		char logbuf[256];
		const double angDeg = RigidDeltaAngleRad(D) * 180.0 / EIGEN_PI;
		snprintf(logbuf, sizeof logbuf,
			"Universe shift detected in %s system (%.1f cm, %.1f deg) - "
			"calibration delta-corrected from base stations\n",
			system.c_str(), D.translation().norm() * 100.0, angDeg);
		CalCtx.Log(logbuf);
		Metrics::WriteLogAnnotation("base_station_shift: calibration delta-corrected");

		InvalidateAllTransformCaches();
	}

	// Per-tick detector. Cheap: a few property reads + matrix arithmetic per
	// base station. Skipped entirely when the toggle is off or no base stations
	// are present.
	void TickBaseStationDrift(double now) {
		if (!CalCtx.baseStationDriftCorrectionEnabled) return;
		if (!vr::VRSystem()) return;
		if (!CalCtx.validProfile || !CalCtx.enabled) return;

		// Throttle: at most one accepted shift per 5 seconds. A second shift
		// arriving rapidly after the first usually means the first applied
		// delta was wrong (we've already cached the post-shift poses, so a
		// follow-up shift would be a brand-new event). Throttling avoids
		// thrashing the calibration on a runtime that's still settling.
		const bool throttled = (now - lastBaseStationShiftAcceptedTime) < 5.0;

		struct Observation {
			std::string serial;
			std::string system;
			Eigen::Affine3d pose;
		};
		std::vector<Observation> current;
		current.reserve(8);

		char buf[vr::k_unMaxPropertyStringSize] = {};
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
			if (vr::VRSystem()->GetTrackedDeviceClass(id) != vr::TrackedDeviceClass_TrackingReference)
				continue;
			const auto& dp = CalCtx.devicePoses[id];
			if (!dp.poseIsValid || !dp.deviceIsConnected) continue;
			if (dp.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;

			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String,
				buf, sizeof buf, &err);
			if (err != vr::TrackedProp_Success) continue;
			std::string serial = buf;

			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String,
				buf, sizeof buf, &err);
			if (err != vr::TrackedProp_Success) continue;
			std::string system = buf;

			Pose p = ConvertPose(dp);
			Eigen::Affine3d pose = Eigen::Affine3d::Identity();
			pose.linear() = p.rot;
			pose.translation() = p.trans;

			current.push_back({serial, system, pose});
		}

		// No base stations -> AUTO mode self-disables, leaving the cache empty.
		if (current.empty()) {
			baseStationCache.clear();
			return;
		}

		// Group observations by tracking system. A user with multiple
		// Lighthouse setups (rare) might have base stations from different
		// systems; each system's universe is independent so they're checked
		// separately.
		std::map<std::string, std::vector<const Observation*>> bySystem;
		for (const auto& obs : current) bySystem[obs.system].push_back(&obs);

		for (const auto& [system, obsPtrs] : bySystem) {
			// Only base stations belonging to a system that's actually part
			// of the active calibration are useful for drift correction.
			// Stations from an unrelated third system can't tell us anything
			// about our calibration's coordinate frames.
			const bool refSystem = (system == CalCtx.referenceTrackingSystem);
			bool targetSystem = (system == CalCtx.targetTrackingSystem);
			if (!targetSystem) {
				for (const auto& extra : CalCtx.additionalCalibrations) {
					if (extra.valid && extra.targetTrackingSystem == system) {
						targetSystem = true;
						break;
					}
				}
			}
			if (!refSystem && !targetSystem) continue;

			// Compute deltas vs. previous tick. Only base stations we've
			// previously seen can produce a delta; first-sighting entries
			// just populate the cache.
			std::vector<Eigen::Affine3d> deltas;
			deltas.reserve(obsPtrs.size());
			for (const auto* obs : obsPtrs) {
				auto it = baseStationCache.find(obs->serial);
				if (it == baseStationCache.end()) continue;
				if (it->second.trackingSystem != system) continue;
				deltas.push_back(obs->pose * it->second.pose.inverse());
			}

			if (!throttled && !deltas.empty()) {
				// Significance: at least one delta meaningfully bigger than
				// pose noise. Otherwise everything's stationary -- nothing
				// to do.
				bool anySignificant = false;
				for (const auto& d : deltas) {
					if (d.translation().norm() > kBsDeltaSignificanceTransM
						|| RigidDeltaAngleRad(d) > kBsDeltaSignificanceRotRad)
					{
						anySignificant = true;
						break;
					}
				}

				// Consensus: all deltas approximately equal (= same rigid
				// universe shift). For multi-base-station setups this is
				// the principled disambiguator between "universe shifted"
				// and "one base station's pose was internally refined".
				//
				// Single-base-station setups can't cross-validate, so we
				// require a substantial shift (>= 10 cm or >= 5 deg) to
				// reduce false positives from runtime base-station
				// refinement.
				bool consensus = true;
				if (anySignificant) {
					if (deltas.size() >= 2) {
						const Eigen::Affine3d& ref = deltas[0];
						for (size_t i = 1; i < deltas.size(); ++i) {
							const Eigen::Affine3d diff = deltas[i] * ref.inverse();
							if (diff.translation().norm() > kBsConsensusTransM
								|| RigidDeltaAngleRad(diff) > kBsConsensusRotRad)
							{
								consensus = false;
								break;
							}
						}
					} else {
						const Eigen::Affine3d& only = deltas[0];
						const double angRad = RigidDeltaAngleRad(only);
						if (only.translation().norm() < 0.10
							&& angRad < 5.0 * EIGEN_PI / 180.0)
						{
							consensus = false;
						}
					}
				}

				if (anySignificant && consensus) {
					ApplyUniverseShiftToCalibrations(deltas[0], system);
					lastBaseStationShiftAcceptedTime = now;
					// Cache update happens unconditionally below, picking
					// up the post-shift poses as the new baseline.
				}
			}
		}

		// Refresh cache for the next tick. We always update -- even when
		// throttled, so that when the throttle releases we're comparing
		// against the most recent poses, not pre-shift ones.
		baseStationCache.clear();
		for (const auto& obs : current) {
			baseStationCache[obs.serial] = {obs.system, obs.pose};
		}
	}

	// === Hybrid HMD-relocalization detector (logging-only) ===================
	//
	// Triple-AND signal that the HMD's tracking system (e.g. Quest SLAM)
	// re-localized -- its reported pose in the OpenVR universe jumps even
	// though physically the user didn't move:
	//
	//   1. HMD pose changed by >5 cm (translation) since the previous tick.
	//   2. Base stations are stable (max delta <1 mm). Rules out a SteamVR
	//      universe re-origin -- that's the other detector's domain.
	//   3. Body trackers in a DIFFERENT tracking system from the HMD didn't
	//      follow the HMD jump (their per-tick deltas are all <5 cm). Rules
	//      out natural fast user motion -- when the user moves their head,
	//      the body trackers (worn on the body) move along with it.
	//
	// The conjunction can't false-fire on natural motion: real motion has
	// the HMD and body trackers moving together, so condition 3 fails. Only
	// a re-localization (HMD jumps independently of body) trips all three.
	//
	// This is the LOGGING-ONLY first cut. When the trigger fires we emit a
	// `# [time] hmd_relocalization_detected: dx=<m> dy=<m> dz=<m> dt=<rad>`
	// annotation to the debug log and update the cache. We do NOT modify R
	// or touch the chaperone -- the user wants to gather real-world data
	// confirming the trigger fires only on actual events before we take
	// corrective action.
	//
	// Requires >=2 base stations to even consider firing -- on Quest-only
	// setups (no Lighthouse) the second condition can't be cross-checked
	// and the detector is silent.
	//
	// Runs in continuous AND one-shot None mode. Skipped during active
	// calibration sub-states (Begin / Rotation / Translation) where the
	// HMD is being deliberately moved.

	struct RelocalizationDetectorState {
		bool havePrevHmd = false;
		Eigen::Affine3d prevHmd = Eigen::Affine3d::Identity();
		std::string hmdTrackingSystem;
		// Per-device previous poses, keyed by OpenVR ID. ID is the right
		// key here (not serial) because we re-read every tick and the OpenVR
		// IDs are stable within a session for the duration of our use.
		std::map<int32_t, Eigen::Vector3d> prevBodyTrans;
		double lastFireTime = -1e9;

		// Snapshot of the most recent fire's measured deltas, exposed via
		// LastDetectedRelocalization() so the UI can surface "your last
		// detected drift event was X cm at T seconds ago" alongside the
		// recenter button.
		Eigen::Vector3d lastFireDelta = Eigen::Vector3d::Zero();
		double lastFireRotRad = 0.0;

		// Last time auto-recovery actually clobbered the calibration. Throttled
		// separately from the 5-second logging fire -- the cost of a too-eager
		// auto-recover is much higher than a too-eager log line. Continuous-cal
		// needs uninterrupted time to converge after each recover, so we keep
		// at least 30s between consecutive auto-recovers.
		double lastAutoRecoverTime = -1e9;

		// Last time HMD tracking transitioned from valid -> invalid (a stall
		// began). Used to enforce a post-stall grace period: a "relocalization"
		// detected within seconds of a stall recovery is more likely the Quest
		// settling its post-stall pose than a true SLAM teleport. The existing
		// stall-recovery flow ALREADY calls StartContinuousCalibration to
		// bootstrap, so a same-instant auto-recover would just double-tap that
		// flow AND save an empty profile.
		//
		// Set to "now" whenever we see hmdValid transition false (stall begin).
		// Auto-recovery checks (now - lastHmdInvalidTime) and refuses to fire
		// if the post-stall window hasn't elapsed.
		//
		// Triggered the first false-positive on 2026-05-02 (build 2026.5.2.8):
		// 3.5s stall ended at t=1527.69, the existing flow auto-restarted
		// continuous-cal at t=1527.69, my detector saw a 29cm pose delta at
		// t=1527.74, auto-recovery fired and clobbered the working cal.
		// Adding this gate so the same scenario can't repeat.
		//
		// NOTE: claimed-but-empirically-false: "stamped on every non-OK tick
		// below". 2026-05-04 logs show this stays at the session-start
		// sentinel even across a 94-second stall, meaning either the
		// !hmdValid branch isn't being entered during real stalls, or
		// devicePoses[Hmd] reports valid-but-stale during stalls. Tracking
		// as a follow-up; new debug logging (reloc_tick, reloc_hmd_invalid_
		// stamped) added in the same change set as this comment will surface
		// the data needed to fix it.
		double lastHmdInvalidTime = -1e9;

		// --- Auto-recovery snapshot: pre-clear state so Undo can restore.
		// Captured INSIDE the auto-recover action block, just before
		// calibration.Clear() runs. Snapshot is "valid" until the user
		// hits Undo (which restores + invalidates) or Dismiss (which only
		// hides the UI banner; the snapshot stays so a later Undo button
		// rendered through some other path would still work, though there
		// currently isn't one).
		struct AutoRecoverySnapshot {
			bool valid = false;
			Eigen::AffineCompact3d refToTargetPose = Eigen::AffineCompact3d::Identity();
			bool relativePosCalibrated         = false;
			bool hasAppliedCalibrationResult   = false;
		};
		AutoRecoverySnapshot lastAutoRecoverSnapshot;

		// Set by DismissAutoRecoveryBanner. Reset to false on every new
		// auto-recover firing so subsequent recoveries get their own banner
		// even if the user dismissed an earlier one.
		bool autoRecoverBannerDismissed = false;

		// --- DIAGNOSTIC ONLY (no gating). Per-base-station HMD-relative
		// distance from the previous tick, used to dump per-base distance
		// jumps into the log when hmd_relocalization_detected fires. The
		// fd81e83 commit tried to USE this as a corroboration gate, but
		// the math is broken for cross-system setups (Quest+Lighthouse)
		// because hmdPose and base poses live in different tracking-system
		// frames in CalCtx.devicePoses. Reverted as a gate; kept here only
		// as diagnostic data so a future log diff can see what cross-frame
		// geometry the detector observed.
		std::map<std::string, double> prevHmdToBaseDist;

		// Throttle for the per-tick reloc_tick diagnostic log. 1 Hz cap so
		// we always have a recent baseline state in the log without flooding.
		double lastTickLogTime = -1e9;

		// Throttle for the per-tick reloc_hmd_invalid_stamped diagnostic log
		// inside the !hmdValid branch. Same 1 Hz cap so a long stall produces
		// 1 line per second instead of 60+.
		double lastInvalidLogTime = -1e9;
	};
	RelocalizationDetectorState g_relocDetector;

	// Rest-locked yaw drift correction state. Per-target-tracker phase
	// machine + locked world-frame orientation. Cleared on AssignTargets
	// reseats, target ID change, or pose-validity loss. The ID-keyed map is
	// indexed by OpenVR device ID; entries are removed when the device's
	// pose goes invalid. Activates only when CalCtx.restLockedYawEnabled is
	// true and the calibration state is not Continuous (continuous-cal
	// already handles drift in its own loop). Math is in
	// src/overlay/RestLockedYaw.h.
	std::unordered_map<uint32_t, spacecal::rest_yaw::RestState> g_restStates;
	double g_restLockedYawLastTickTime = -1.0;
	double g_restLockedYawLastLogTime  = -1e9;

	// Predictive recovery (rec C). Each RecoverFromWedgedCalibration fire
	// pushes the HMD-jump direction and magnitude into this ring; the per-
	// tick predictive apply reads from it. Cleared on session end. Math is
	// in src/overlay/RecoveryDeltaBuffer.h.
	spacecal::recovery_delta::Buffer g_recoveryDeltaBuffer;
	double g_predictiveRecoveryLastTickTime = -1.0;
	double g_predictiveRecoveryLastLogTime  = -1e9;

	// Chi-square re-anchor sub-detector (rec F). Tracks rolling HMD pose
	// history, predicts via velocity, computes Mahalanobis distance against
	// online residual variance. When fired, freezes recs A and C for 500 ms
	// so the existing 30 cm detector has a clean window to confirm.
	spacecal::reanchor_chi::DetectorState g_reanchorChiState;
	double g_reanchorChiLastTickTime = -1.0;
	double g_reanchorChiLastLogTime  = -1e9;

	constexpr double kRelocHmdJumpM       = 0.05;   // 5 cm
	constexpr double kRelocBodyMaxDeltaM  = 0.05;   // 5 cm (any body tracker moving more = rule out)
	constexpr double kRelocBaseStableM    = 0.001;  // 1 mm
	constexpr double kRelocThrottleSec    = 5.0;
	constexpr int    kRelocMinBaseStations = 2;

	// Auto-recovery thresholds. Stricter than the logging trigger because the
	// cost of a false-positive auto-recovery (clobbering a working cal) is
	// much higher than a false-positive log line. The user's reported wedged
	// event was 86 cm; the false-positive that clobbered a working cal was
	// 29 cm right after a stall. So we need BOTH a higher magnitude floor AND
	// a post-stall lockout.
	//
	// Threshold: 30 cm (raised from 15 cm after the 2026-05-02 false-positive).
	// SLAM noise is typically <2 cm/tick, real teleport events tend to be
	// 30-100+ cm. 30 cm filters out the post-stall-settling case (which can
	// look like 20-30 cm) without losing the catastrophic-wedge case (60+ cm).
	//
	// Startup grace: don't fire in the first 30 s of the session. Pose data
	// settling + initial driver bootstrap can produce spurious large deltas
	// in the first few ticks; better to let the system stabilise before we
	// start invalidating.
	//
	// Throttle: at least 30 s between auto-recovers. Continuous-cal needs
	// uninterrupted time to converge after each recovery; if we re-fire too
	// quickly we'd interrupt the convergence and prevent the calibration
	// from ever stabilising.
	//
	// Post-stall grace: 10 s after an HMD-tracking-lost event, refuse to
	// auto-recover. The stall flow has its own restart logic; let it converge
	// before considering further intervention. Long enough that even a chunky
	// post-stall pose-settling jitter window doesn't trigger us. (Was briefly
	// raised to 30 s in fd81e83 alongside the per-base corroboration gate;
	// reverted with that gate in 2026-05-04 -- the 30 s value never actually
	// applied in practice because lastHmdInvalidTime doesn't advance during
	// real stalls, see TODO above. Restoring 10 s as the original behavior
	// while we figure out the timestamp-not-updating problem separately.)
	constexpr double kRelocAutoRecoverThresholdM   = 0.30;  // 30 cm (was 15, too low)
	constexpr double kRelocAutoRecoverStartupSec   = 30.0;  // 30 s startup grace
	constexpr double kRelocAutoRecoverThrottleSec  = 30.0;  // 30 s between recovers
	constexpr double kRelocAutoRecoverPostStallSec = 10.0;  // 10 s after stall recovery

	void TickHmdRelocalizationDetector(double now) {
		if (!vr::VRSystem()) return;

		// Skip during active one-shot calibration sub-states (the HMD is
		// being deliberately swung around). Continuous, ContinuousStandby,
		// None, Editing all OK.
		if (CalCtx.state == CalibrationState::Begin
		 || CalCtx.state == CalibrationState::Rotation
		 || CalCtx.state == CalibrationState::Translation) {
			// Reset cache so the post-calibration baseline is fresh.
			g_relocDetector.havePrevHmd = false;
			g_relocDetector.prevBodyTrans.clear();
			return;
		}

		auto& s = g_relocDetector;

		// HMD pose. Index 0 by the static_assert above.
		const auto& hmdRaw = CalCtx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
		const bool hmdValid = hmdRaw.poseIsValid && hmdRaw.deviceIsConnected
			&& hmdRaw.result == vr::ETrackingResult::TrackingResult_Running_OK;

		// Diagnostic: per-tick state baseline, throttled to 1 Hz. Future
		// triage of "tracking went weird" reports can grep `reloc_tick` to
		// see what the detector observed across a problematic window. Also
		// the only way to confirm the function is being called every tick
		// (vs. some upstream skipping it during stalls -- which is what we
		// suspect is preventing lastHmdInvalidTime from advancing during
		// real stalls, see comment on the field).
		if ((now - s.lastTickLogTime) >= 1.0) {
			s.lastTickLogTime = now;
			char tickbuf[256];
			snprintf(tickbuf, sizeof tickbuf,
				"reloc_tick: hmdValid=%d havePrevHmd=%d state=%d hmdRaw.result=%d hmdRaw.poseIsValid=%d hmdRaw.deviceIsConnected=%d lastHmdInvalidTime=%.3f secSinceStall=%.2f",
				(int)hmdValid, (int)s.havePrevHmd, (int)CalCtx.state,
				(int)hmdRaw.result, (int)hmdRaw.poseIsValid, (int)hmdRaw.deviceIsConnected,
				s.lastHmdInvalidTime, now - s.lastHmdInvalidTime);
			Metrics::WriteLogAnnotation(tickbuf);
		}

		if (!hmdValid) {
			// Tracking dropout. Drop the cache so we don't compare across
			// the dropout (which would produce a spurious jump). Also stamp
			// the dropout time -- the auto-recovery gate uses it to enforce
			// a post-stall grace window so we don't double-tap the existing
			// stall-recovery flow's StartContinuousCalibration.

			// Diagnostic: log every entry into the !hmdValid branch (1 Hz
			// throttled). This is the data we need to debug why
			// lastHmdInvalidTime doesn't advance during real stalls --
			// either this branch isn't being entered (devicePoses[Hmd]
			// reports stale-but-valid during stalls), or it IS being
			// entered but the stamp isn't reaching the auto-recovery gate
			// for some other reason. Either way, having a log line per
			// stall second will surface the truth on the next reproduction.
			if ((now - s.lastInvalidLogTime) >= 1.0) {
				s.lastInvalidLogTime = now;
				char invbuf[224];
				snprintf(invbuf, sizeof invbuf,
					"reloc_hmd_invalid_stamped: now=%.3f prev_lastHmdInvalidTime=%.3f hmdRaw.result=%d hmdRaw.poseIsValid=%d hmdRaw.deviceIsConnected=%d",
					now, s.lastHmdInvalidTime,
					(int)hmdRaw.result, (int)hmdRaw.poseIsValid, (int)hmdRaw.deviceIsConnected);
				Metrics::WriteLogAnnotation(invbuf);
			}

			s.havePrevHmd = false;
			s.prevBodyTrans.clear();
			s.lastHmdInvalidTime = now;
			return;
		}

		// HMD's tracking system, cached once. Used to identify "body trackers
		// in a DIFFERENT system" below.
		if (s.hmdTrackingSystem.empty()) {
			char buf[vr::k_unMaxPropertyStringSize] = {};
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(
				vr::k_unTrackedDeviceIndex_Hmd,
				vr::Prop_TrackingSystemName_String,
				buf, sizeof buf, &err);
			if (err != vr::TrackedProp_Success) return;
			s.hmdTrackingSystem = buf;
		}

		Pose hmdPoseWorld = ConvertPose(hmdRaw);
		Eigen::Affine3d hmdPose = Eigen::Affine3d::Identity();
		hmdPose.linear() = hmdPoseWorld.rot;
		hmdPose.translation() = hmdPoseWorld.trans;

		// Base stations: count + max delta vs last tick. Reuse the same
		// cache the existing universe-shift detector populates -- but we
		// can't read its state directly without coupling to it, so do a
		// fresh scan here (cheap; ~4 base stations max).
		//
		// Also track per-base HMD-relative distance (cur + prev) for
		// DIAGNOSTIC dumping in the reloc_base_dists log when a fire
		// event triggers below. NOT used as a gate (the fd81e83 attempt
		// at gating on this was reverted -- HMD and base poses live in
		// different tracking-system frames in CalCtx.devicePoses, so
		// the cross-frame distance has no consistent physical meaning).
		// Kept here purely so the log shows what the detector saw.
		double bsMaxDelta = 0.0;
		int bsCount = 0;
		std::map<std::string, double> currentHmdToBaseDist;
		char propBuf[vr::k_unMaxPropertyStringSize] = {};
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
			if (vr::VRSystem()->GetTrackedDeviceClass(id) != vr::TrackedDeviceClass_TrackingReference) continue;
			const auto& dp = CalCtx.devicePoses[id];
			if (!dp.poseIsValid || !dp.deviceIsConnected) continue;
			if (dp.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;
			++bsCount;
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String,
				propBuf, sizeof propBuf, &err);
			if (err != vr::TrackedProp_Success) continue;
			std::string serial = propBuf;
			Pose p = ConvertPose(dp);
			currentHmdToBaseDist[serial] = (hmdPose.translation() - p.trans).norm();
			auto it = baseStationCache.find(serial); // populated by TickBaseStationDrift
			if (it == baseStationCache.end()) continue;
			double d = (p.trans - it->second.pose.translation()).norm();
			if (d > bsMaxDelta) bsMaxDelta = d;
		}

		// Body trackers in OTHER tracking system(s). For Quest HMD + Lighthouse
		// trackers, this picks up the Lighthouse trackers; for Lighthouse HMD +
		// Quest trackers, it picks up the Quest trackers. Either way, the trackers
		// we care about are the ones that DIDN'T re-localize when the HMD did.
		double bodyMaxDelta = 0.0;
		std::map<int32_t, Eigen::Vector3d> currentBodyTrans;
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
			const auto cls = vr::VRSystem()->GetTrackedDeviceClass(id);
			if (cls != vr::TrackedDeviceClass_GenericTracker
				&& cls != vr::TrackedDeviceClass_Controller) continue;
			const auto& dp = CalCtx.devicePoses[id];
			if (!dp.poseIsValid || !dp.deviceIsConnected) continue;
			if (dp.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String,
				propBuf, sizeof propBuf, &err);
			if (err != vr::TrackedProp_Success) continue;
			if (std::string(propBuf) == s.hmdTrackingSystem) continue; // skip same-system devices
			Pose p = ConvertPose(dp);
			currentBodyTrans[(int32_t)id] = p.trans;
			auto it = s.prevBodyTrans.find((int32_t)id);
			if (it == s.prevBodyTrans.end()) continue;
			double d = (p.trans - it->second).norm();
			if (d > bodyMaxDelta) bodyMaxDelta = d;
		}

		// Trigger evaluation. Need a previous HMD pose and >=2 base stations
		// (so condition 2 isn't trivially passing). Note we still update the
		// cache even when no trigger fires -- always tracking the latest
		// values so the next tick's delta is over a one-tick interval.
		bool fired = false;
		if (s.havePrevHmd && bsCount >= kRelocMinBaseStations) {
			const double hmdDelta = (hmdPose.translation() - s.prevHmd.translation()).norm();
			const Eigen::Quaterniond qNew(hmdPose.linear());
			const Eigen::Quaterniond qOld(s.prevHmd.linear());
			Eigen::Quaterniond rotDelta = qNew * qOld.conjugate();
			rotDelta.normalize();
			const double angRad = 2.0 * std::acos(std::min(1.0, std::abs(rotDelta.w())));

			const bool throttled = (now - s.lastFireTime) < kRelocThrottleSec;
			if (!throttled
				&& hmdDelta > kRelocHmdJumpM
				&& bodyMaxDelta < kRelocBodyMaxDeltaM
				&& bsMaxDelta < kRelocBaseStableM)
			{
				const Eigen::Vector3d dpos = hmdPose.translation() - s.prevHmd.translation();
				char logbuf[256];
				snprintf(logbuf, sizeof logbuf,
					"hmd_relocalization_detected: dx=%.4f dy=%.4f dz=%.4f dt=%.4f"
					" (hmdDelta=%.3f bodyMax=%.3f bsMax=%.4f bsCount=%d)",
					dpos.x(), dpos.y(), dpos.z(), angRad,
					hmdDelta, bodyMaxDelta, bsMaxDelta, bsCount);
				Metrics::WriteLogAnnotation(logbuf);

				// Diagnostic: per-base distance dump alongside every fire,
				// so a future log can show whether the cross-frame HMD-to-
				// base distance jumped on this event (it usually doesn't on
				// cross-system Quest+Lighthouse setups -- that's why the
				// fd81e83 corroboration gate had to be reverted). Format
				// chunks one base station per snprintf so the line stays
				// bounded.
				{
					char baseBuf[1024];
					int written = snprintf(baseBuf, sizeof baseBuf, "reloc_base_dists:");
					for (const auto& kv : currentHmdToBaseDist) {
						auto prevIt = s.prevHmdToBaseDist.find(kv.first);
						double prev = (prevIt != s.prevHmdToBaseDist.end()) ? prevIt->second : -1.0;
						double jump = (prev >= 0.0) ? std::abs(kv.second - prev) : 0.0;
						int n = snprintf(baseBuf + written, sizeof(baseBuf) - written,
							" {serial=%s prev=%.3f cur=%.3f jump=%.3f}",
							kv.first.c_str(), prev, kv.second, jump);
						if (n <= 0 || (size_t)(written + n) >= sizeof baseBuf) break;
						written += n;
					}
					Metrics::WriteLogAnnotation(baseBuf);
				}

				// "Who moved" diagnostic: when relocalization fires, integrate
				// each tracking system's reported velocity over the last tick
				// and compare to the observed pose delta. Quest-re-anchor
				// signature: the HMD's velocity-integrated displacement is
				// near zero (the Quest didn't physically move; its world
				// frame jumped). A genuine fast head movement that slipped
				// through the gates would have velocity-integrated displacement
				// comparable to the observed delta. This is logging-only --
				// the auto-recover decision still runs from the existing
				// triple-AND gate; the annotation just gives the log enough
				// data to distinguish "Quest re-anchored" from "base station
				// bumped" from "false-positive on fast natural motion".
				{
					const double dt = std::max(1e-3, now - s.lastTickLogTime);  // ~ tick interval
					auto vmag = [](const double v[3]) -> double {
						return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
					};
					auto vmagFinite = [&](const double v[3]) -> double {
						const double m = vmag(v);
						return std::isfinite(m) ? m : 0.0;
					};
					const auto& hmdRawNow = CalCtx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
					const double hmdSpeed = vmagFinite(hmdRawNow.vecVelocity);
					const double hmdImuDisp = hmdSpeed * dt;

					// Body-tracker (other-system) integrated displacement: max
					// across all body trackers. If this is near zero, no body
					// device physically moved either, so the geometry shift
					// is purely a world-frame change of one of the systems.
					double bodyMaxImuDisp = 0.0;
					int bodyCount = 0;
					for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
						const auto cls = vr::VRSystem()->GetTrackedDeviceClass(id);
						if (cls != vr::TrackedDeviceClass_GenericTracker
						 && cls != vr::TrackedDeviceClass_Controller) continue;
						const auto& dp = CalCtx.devicePoses[id];
						if (!dp.poseIsValid || !dp.deviceIsConnected) continue;
						if (dp.result != vr::ETrackingResult::TrackingResult_Running_OK) continue;
						const double d = vmagFinite(dp.vecVelocity) * dt;
						if (d > bodyMaxImuDisp) bodyMaxImuDisp = d;
						++bodyCount;
					}

					// Ratio interpretation (for the human grepping this later):
					//   hmdRatio = observedHmdDelta / hmdImuDisp
					//     >> 1: HMD's world frame jumped without device moving
					//           (most likely Quest re-anchor)
					//     ~= 1: HMD device physically moved (gate should have
					//           rejected on bodyMaxDelta but did not -- worth
					//           investigating; possible false positive)
					//   bodyRatio similar for the body trackers.
					const double hmdRatio = (hmdImuDisp > 1e-6) ? (hmdDelta / hmdImuDisp) : -1.0;
					char wmoBuf[320];
					snprintf(wmoBuf, sizeof wmoBuf,
						"who_moved: dt=%.4f hmd_observed=%.3f hmd_imu_disp=%.4f hmd_ratio=%.2f"
						" body_observed_max=%.3f body_imu_disp_max=%.4f body_count=%d",
						dt, hmdDelta, hmdImuDisp, hmdRatio,
						bodyMaxDelta, bodyMaxImuDisp, bodyCount);
					Metrics::WriteLogAnnotation(wmoBuf);
				}

				s.lastFireTime = now;
				s.lastFireDelta = dpos;
				s.lastFireRotRad = angRad;
				fired = true;
			}
		}

		// =================================================================
		// AUTO-RECOVERY: clobber the wedged calibration when a real
		// re-localization is detected.
		//
		// Background: continuous-cal can converge to a "self-consistent fit
		// at the wrong offset" after a Quest re-localization. Once wedged,
		// continuous-cal cannot recover on its own -- the saved relative
		// pose constraint pulls every refinement back to the bad neighborhood.
		// Until 2026-05-02 the only fix was to restart the overlay and
		// re-do calibration manually. This block makes recovery automatic.
		//
		// Entry conditions are intentionally stricter than the logging fire:
		//   - hmdDelta >= 15 cm (logging fires at 5 cm)
		//   - State == Continuous or ContinuousStandby (don't surprise the
		//     user mid-wizard or in None mode where there's nothing to fix)
		//   - Session has been running >= 30 s (avoid bootstrap noise)
		//   - >= 30 s since last auto-recover (let cal converge between resets)
		//
		// The triple-AND of the underlying detector means we already have
		// strong evidence this is a real re-localization, not natural motion.
		// The extra gates here are about being conservative on the *action*,
		// since clobbering a working cal is bad and we want zero false-fires.
		//
		// Recovery procedure:
		//   1. calibration.Clear() -- wipes m_estimatedTransformation,
		//      m_isValid, m_samples, m_refToTargetPose, m_relativePosCalibrated.
		//   2. CalCtx.refToTargetPose / relativePosCalibrated reset, so the
		//      restart in step 4 doesn't immediately re-apply the saved bad
		//      relative-pose constraint via setRelativeTransformation.
		//   3. SaveProfile() -- persist the cleared state. Without this, a
		//      subsequent program restart would re-load the bad cal from
		//      disk and the recovery would only have helped the live session.
		//   4. StartContinuousCalibration() -- restart cold. Continuous-cal
		//      will bootstrap from new pose pairs and converge to the
		//      correct (post-relocalization) calibration within seconds.
		//
		// User feedback: the fresh-start period IS visible -- body trackers
		// will appear at their lighthouse-system positions for a few seconds
		// before the new fit locks in. That's a much better outcome than
		// "calibration is permanently 86 cm off until you restart the
		// overlay manually."
		// Auto-recover gate. Each clause excludes a specific false-positive
		// scenario; if you change one, document why.
		const double currentHmdDelta = s.havePrevHmd
			? (hmdPose.translation() - s.prevHmd.translation()).norm()
			: 0.0;
		const double secSinceStall   = now - s.lastHmdInvalidTime;
		const bool postStallGrace    = secSinceStall < kRelocAutoRecoverPostStallSec;
		const bool stateOK           = (CalCtx.state == CalibrationState::Continuous
		                              || CalCtx.state == CalibrationState::ContinuousStandby);
		const bool magnitudeOK       = currentHmdDelta >= kRelocAutoRecoverThresholdM;
		const bool startupOK         = now >= kRelocAutoRecoverStartupSec;
		const bool throttleOK        = (now - s.lastAutoRecoverTime) >= kRelocAutoRecoverThrottleSec;

		// If `fired` is true (a relocalization log line was emitted) but a
		// gate blocked the recovery, log WHY -- gives us debug evidence for
		// every borderline event so we can tune thresholds against real data
		// instead of guessing. Throttled to once per fire (the fire itself
		// is throttled to 5s by the existing code), so this won't flood.
		if (fired && (!magnitudeOK || !stateOK || !startupOK || !throttleOK || postStallGrace)) {
			char skipbuf[384];
			snprintf(skipbuf, sizeof skipbuf,
				"auto_recover_skipped: hmdDelta=%.3f magnitudeOK=%d stateOK=%d startupOK=%d throttleOK=%d postStallGrace=%d (secSinceStall=%.2f) state=%d",
				currentHmdDelta, (int)magnitudeOK, (int)stateOK, (int)startupOK, (int)throttleOK,
				(int)postStallGrace, secSinceStall, (int)CalCtx.state);
			Metrics::WriteLogAnnotation(skipbuf);
		}

		if (fired
			&& magnitudeOK
			&& stateOK
			&& startupOK
			&& throttleOK
			&& !postStallGrace)
		{
			const double hmdDelta = currentHmdDelta;
			const Eigen::Vector3d dpos = hmdPose.translation() - s.prevHmd.translation();
			char logbuf[384];
			snprintf(logbuf, sizeof logbuf,
				"auto_recover_from_relocalization: hmdDelta=%.3f dpos=(%.3f,%.3f,%.3f) rotRad=%.3f"
				" priorState=%s priorValid=%d secSinceStall=%.2f -> calibration cleared, continuous-cal restarting",
				hmdDelta, dpos.x(), dpos.y(), dpos.z(), s.lastFireRotRad,
				(CalCtx.state == CalibrationState::Continuous) ? "Continuous" : "ContinuousStandby",
				(int)calibration.isValid(), secSinceStall);
			Metrics::WriteLogAnnotation(logbuf);

			// Step 0 (audit UX #3): snapshot the pre-recovery calibration
			// state so the UI's "Undo" button can restore it. Snapshot the
			// CalCtx fields the recovery is about to clear -- restoring
			// these is sufficient to put the user back in the wedged
			// calibration (which is, after all, what they were running
			// happily in until the false-positive auto-recovery clobbered
			// it). Reset the dismissed flag so this new event gets a
			// fresh banner even if a previous one was dismissed.
			s.lastAutoRecoverSnapshot.valid                      = true;
			s.lastAutoRecoverSnapshot.refToTargetPose            = CalCtx.refToTargetPose;
			s.lastAutoRecoverSnapshot.relativePosCalibrated      = CalCtx.relativePosCalibrated;
			s.lastAutoRecoverSnapshot.hasAppliedCalibrationResult = CalCtx.hasAppliedCalibrationResult;
			s.autoRecoverBannerDismissed                         = false;

			// Steps 1-4: wipe + restart cold. The helper does
			// calibration.Clear() + zero CalCtx fields (incl. calibratedTranslation
			// / calibratedRotation, which Clear() doesn't touch -- this was the
			// 2026-05-03 SaveProfile-persisted-wedge bug; see the helper's
			// comment) + StartContinuousCalibration() + posts the user-facing
			// message AFTER the restart (StartContinuousCalibration clears
			// CalCtx.messages internally). Step 5 (the user banner) is folded
			// in via the helper's userFacingMessage argument.
			//
			// SaveProfile is intentionally NOT called here. The next valid
			// ComputeIncremental will write the post-recovery values via the
			// existing path at the end of CalibrationTick, which is exactly
			// what we want.

			// Rec C: push the HMD-jump direction and magnitude into the
			// rolling buffer before recovery clears state. Subsequent ticks
			// can predict the next jump from these accumulated events and
			// apply a small fraction as a bounded-rate translation nudge,
			// shrinking the magnitude of the next observed event if the
			// drift trend is consistent.
			spacecal::recovery_delta::Push(g_recoveryDeltaBuffer, dpos, now);
			{
				char b[200];
				snprintf(b, sizeof b,
					"[drift][recovery-buffer] event_pushed mag_cm=%.2f dpos=(%.3f,%.3f,%.3f) live_count=%zu",
					hmdDelta * 100.0, dpos.x(), dpos.y(), dpos.z(),
					spacecal::recovery_delta::LiveCount(g_recoveryDeltaBuffer));
				Metrics::WriteLogAnnotation(b);
			}

			char uimsg[128];
			snprintf(uimsg, sizeof uimsg,
				"Quest re-localized (%.0f cm jump). Recalibrating...\n",
				hmdDelta * 100.0);
			RecoverFromWedgedCalibration(uimsg);

			s.lastAutoRecoverTime = now;
		}

		// Update cache for next tick.
		s.prevHmd = hmdPose;
		s.havePrevHmd = true;
		s.prevBodyTrans = std::move(currentBodyTrans);
		// Diagnostic-only: per-base distance cache for next-tick comparison
		// in the reloc_base_dists log.
		s.prevHmdToBaseDist = std::move(currentHmdToBaseDist);
	}
}

// Classify a tracking-system name string into a coarse class. The name comes
// from OpenVR's Prop_TrackingSystemName_String for the device. Per-class rate
// caps live in spacecal::rest_yaw::RateCaps; the dominant axis of variation
// in drift rate is sensor class, not individual unit (Borenstein & Ojeda
// 2009/2010 iHDE; SlimeVR v0.16.0 release notes).
static spacecal::rest_yaw::TrackingSystemClass ClassifyTrackingSystem(const std::string& name) {
	if (name.empty()) return spacecal::rest_yaw::TrackingSystemClass::Unknown;
	std::string lower = name;
	std::transform(lower.begin(), lower.end(), lower.begin(),
		[](unsigned char c) { return (char)std::tolower(c); });
	if (lower.find("lighthouse") != std::string::npos) return spacecal::rest_yaw::TrackingSystemClass::Lighthouse;
	if (lower.find("oculus") != std::string::npos)     return spacecal::rest_yaw::TrackingSystemClass::Quest;
	if (lower.find("quest") != std::string::npos)      return spacecal::rest_yaw::TrackingSystemClass::Quest;
	if (lower.find("slime") != std::string::npos)      return spacecal::rest_yaw::TrackingSystemClass::SlimeVR;
	return spacecal::rest_yaw::TrackingSystemClass::Unknown;
}

// Lift a target-tracking-system pose into world frame: world_q = qWorldFromDriver * qRotation.
static inline Eigen::Quaterniond WorldRotationFromPose(const vr::DriverPose_t& p) {
	const Eigen::Quaterniond qWorld(
		p.qWorldFromDriverRotation.w,
		p.qWorldFromDriverRotation.x,
		p.qWorldFromDriverRotation.y,
		p.qWorldFromDriverRotation.z);
	const Eigen::Quaterniond qRot(
		p.qRotation.w,
		p.qRotation.x,
		p.qRotation.y,
		p.qRotation.z);
	return qWorld * qRot;
}

// Rest-locked yaw drift correction tick. Runs after TickHmdRelocalizationDetector
// when CalCtx.restLockedYawEnabled is true and the calibration state is not
// Continuous. Updates per-target-tracker rest state, fuses yaw drift signals
// from all AT_REST trackers via the rec I weighted-mean shape, and applies a
// bounded-rate yaw nudge to ctx.calibratedRotation(1).
//
// Why not run during Continuous: continuous-cal already corrects drift in its
// own loop; running both produces oscillation unless one is gain-limited well
// below the other (basic IMC). Q1 supersession (research synthesis 2026-05-07)
// proposes a 1/10-rate watchdog mode for Continuous; that is deferred -- v1
// hard-skips Continuous to keep behavior strictly opt-in additive.
//
// Sign convention: ctx.calibratedRotation is in degrees, Euler order Z-Y-X
// (component 1 is yaw about Y). The applied step is
// -SignedYawDelta(currentWorld, lockedWorld), i.e., "subtract drift to
// compensate." If the live test shows the wrong sign, flip here -- the toggle
// is OFF by default so a wrong-sign build cannot regress users.
static void TickRestLockedYaw(double now) {
	if (!CalCtx.restLockedYawEnabled || CalCtx.useUpstreamMath) {
		// Toggle OFF: clear state so a future toggle-on starts fresh.
		if (!g_restStates.empty()) g_restStates.clear();
		g_restLockedYawLastTickTime = -1.0;
		return;
	}

	// Skip during Continuous and during active one-shot sub-states. Allowed
	// states: None, Editing, ContinuousStandby, post-completion idle.
	if (CalCtx.state == CalibrationState::Continuous
	 || CalCtx.state == CalibrationState::Begin
	 || CalCtx.state == CalibrationState::Rotation
	 || CalCtx.state == CalibrationState::Translation) {
		if (!g_restStates.empty()) g_restStates.clear();
		g_restLockedYawLastTickTime = -1.0;
		return;
	}

	// Need a valid calibration to nudge.
	if (!CalCtx.validProfile) return;

	// Compute dt. First tick after enable produces dt = 0; we still update
	// phase state so the next tick has a reference, but skip the apply.
	double dt = 0.0;
	if (g_restLockedYawLastTickTime > 0.0) {
		dt = now - g_restLockedYawLastTickTime;
	}
	g_restLockedYawLastTickTime = now;

	// Walk all valid target-system devices. Per rec I (research synthesis
	// 2026-05-07), multi-tracker fusion via Markley matrix-weighted average
	// composes contributions from every AT_REST tracker. Each contribution
	// carries a class weight (Lighthouse 1.0, Quest 0.6, SlimeVR 0.3),
	// an age weight exp(-age/120 s), and a quality weight 1/(1+sigma^2).
	// The yaw-only collapse (all contributions are pure yaw rotations)
	// reduces the symmetric 4x4 Markley eigenproblem to a 1-D weighted mean
	// implemented in spacecal::rest_yaw::FuseYawContributionsRad.
	std::vector<spacecal::rest_yaw::YawContribution> contributions;
	std::vector<uint32_t> seenIds;
	seenIds.reserve(vr::k_unMaxTrackedDeviceCount);

	auto considerDevice = [&](uint32_t id, const std::string& trackingSystem) {
		const auto& tp = CalCtx.devicePoses[id];
		if (!tp.poseIsValid || !tp.deviceIsConnected
		 || tp.result != vr::ETrackingResult::TrackingResult_Running_OK) {
			auto it = g_restStates.find(id);
			if (it != g_restStates.end()) {
				if (it->second.haveLock) {
					char b[160];
					snprintf(b, sizeof b,
						"[drift][rest-detector] device=%u lock_dropped reason=pose_invalid result=%d",
						id, (int)tp.result);
					Metrics::WriteLogAnnotation(b);
				}
				g_restStates.erase(it);
			}
			return;
		}
		const Eigen::Quaterniond worldRot = WorldRotationFromPose(tp);
		auto& rest = g_restStates[id];
		const bool wasLocked = rest.haveLock;
		const auto priorPhase = rest.phase;
		rest = spacecal::rest_yaw::UpdatePhase(rest, worldRot, now, dt);
		if (rest.phase == spacecal::rest_yaw::RestPhase::AtRest && rest.haveLock) {
			if (!wasLocked) {
				// Stamp the lock time on first transition into AtRest. Used
				// for the age-weight term in rec I's fusion.
				rest.phaseEnteredAt = now;
				char b[200];
				snprintf(b, sizeof b,
					"[drift][rest-detector] device=%u lock_acquired phase=AtRest world_yaw_deg=%.4f tracking_system=%s",
					id, std::atan2(2.0 * (worldRot.w() * worldRot.y() + worldRot.z() * worldRot.x()),
					                1.0 - 2.0 * (worldRot.x() * worldRot.x() + worldRot.y() * worldRot.y())) * 180.0 / EIGEN_PI,
					trackingSystem.c_str());
				Metrics::WriteLogAnnotation(b);
			}
			const double yawErrRad = spacecal::rest_yaw::SignedYawDeltaRad(rest.lockedRot, worldRot);
			const auto cls = ClassifyTrackingSystem(trackingSystem);
			spacecal::rest_yaw::YawContribution contrib;
			contrib.yawErrRad = -yawErrRad; // sign: subtract drift to compensate
			contrib.cls = cls;
			const double ageSec = std::max(0.0, now - rest.phaseEnteredAt);
			// v1 quality is a constant proxy; the Cramer-Rao 1/(1+sigma^2)
			// term needs per-tracker residual variance, which is not yet
			// tracked. Use 1.0 as a placeholder so age and class weighting
			// dominate; promote to real variance when residual tracking
			// per device lands.
			contrib.weight = spacecal::rest_yaw::ClassWeight(cls)
			               * spacecal::rest_yaw::AgeWeight(ageSec)
			               * spacecal::rest_yaw::QualityWeight(0.0);
			contributions.push_back(contrib);
			seenIds.push_back(id);
		} else if (wasLocked && rest.phase != spacecal::rest_yaw::RestPhase::AtRest) {
			// Phase exited AtRest. Log the transition so a session log can
			// reconstruct the lock lifecycle without grepping for tick lines.
			const char* reason = (priorPhase == spacecal::rest_yaw::RestPhase::AtRest)
				? "motion_detected" : "phase_reset";
			char b[160];
			snprintf(b, sizeof b,
				"[drift][rest-detector] device=%u lock_released reason=%s phase=%d",
				id, reason, (int)rest.phase);
			Metrics::WriteLogAnnotation(b);
		}
	};

	const int32_t targetID = CalCtx.targetID;
	if (targetID >= 0 && targetID < (int32_t)vr::k_unMaxTrackedDeviceCount) {
		considerDevice((uint32_t)targetID, CalCtx.targetTrackingSystem);
	}
	// Multi-ecosystem extras: each entry's targetID is a distinct device on a
	// potentially distinct tracking system. Adding their contributions makes
	// rec A robust against a rig where the primary target's IMU drifts but
	// auxiliary trackers stay anchored.
	for (const auto& extra : CalCtx.additionalCalibrations) {
		if (!extra.enabled || !extra.valid) continue;
		const int32_t exId = extra.targetID;
		if (exId < 0 || exId >= (int32_t)vr::k_unMaxTrackedDeviceCount) continue;
		// Skip if already considered (primary and an extra mapped to the same ID).
		bool already = false;
		for (auto sid : seenIds) if (sid == (uint32_t)exId) { already = true; break; }
		if (already) continue;
		considerDevice((uint32_t)exId, extra.targetTrackingSystem);
	}

	if (contributions.empty()) {
		return;
	}

	// Markley fusion. Weighted mean over yaw-only contributions; weights
	// already include class * age * quality.
	const double meanErrRad = spacecal::rest_yaw::FuseYawContributionsRad(contributions);

	// Apply the per-class cap of the highest-trust contribution. Rationale:
	// the cap has to be small enough that the worst tracker in the pool can
	// not inject bias faster than the sensor can drift, but the dominant
	// source of bias-cancellation is the highest-trust class so cap by
	// THAT class's expected drift rate.
	spacecal::rest_yaw::RateCaps caps;
	double capDegPerSec = caps.global_ceiling_deg_per_sec;
	for (const auto& c : contributions) {
		const double cls_cap = spacecal::rest_yaw::CapForClass(c.cls, caps);
		if (cls_cap < capDegPerSec) capDegPerSec = cls_cap;
	}

	if (dt <= 0.0) return;
	const double stepRad = spacecal::rest_yaw::ApplyBoundedYawStep(meanErrRad, dt, capDegPerSec);
	const double stepDeg = stepRad * (180.0 / EIGEN_PI);

	CalCtx.calibratedRotation(1) += stepDeg;

	// 1 Hz throttled telemetry. step_deg = bounded; err_deg = unbounded
	// (so the log shows whether the cap is doing work).
	if ((now - g_restLockedYawLastLogTime) >= 1.0) {
		g_restLockedYawLastLogTime = now;
		char buf[200];
		snprintf(buf, sizeof buf,
			"[drift][rest-yaw] tick step_deg=%.5f err_deg=%.5f locked_trackers=%zu cap_deg_per_sec=%.4f",
			stepDeg, meanErrRad * 180.0 / EIGEN_PI, contributions.size(), capDegPerSec);
		Metrics::WriteLogAnnotation(buf);
	}
}

// Chi-square re-anchor sub-detector tick. Detection-only: fires the freeze
// window for recs A and C so they suspend for 500 ms after a candidate. The
// existing 30 cm relocalization detector remains the only path to actual
// recovery. Returns true if rec A and rec C should skip their tick.
static bool TickReanchorChiSquare(double now) {
	if (!CalCtx.reanchorChiSquareEnabled) {
		spacecal::reanchor_chi::Reset(g_reanchorChiState);
		g_reanchorChiLastTickTime = -1.0;
		return false;
	}

	double dt = 0.0;
	if (g_reanchorChiLastTickTime > 0.0) {
		dt = now - g_reanchorChiLastTickTime;
	}
	g_reanchorChiLastTickTime = now;

	const auto& hmdRaw = CalCtx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
	const bool hmdValid = hmdRaw.poseIsValid && hmdRaw.deviceIsConnected
		&& hmdRaw.result == vr::ETrackingResult::TrackingResult_Running_OK;
	if (!hmdValid) return false;

	const Eigen::Quaterniond worldFromDriver(
		hmdRaw.qWorldFromDriverRotation.w,
		hmdRaw.qWorldFromDriverRotation.x,
		hmdRaw.qWorldFromDriverRotation.y,
		hmdRaw.qWorldFromDriverRotation.z);
	const Eigen::Vector3d worldFromDriverT(
		hmdRaw.vecWorldFromDriverTranslation[0],
		hmdRaw.vecWorldFromDriverTranslation[1],
		hmdRaw.vecWorldFromDriverTranslation[2]);
	const Eigen::Quaterniond rot(hmdRaw.qRotation.w, hmdRaw.qRotation.x, hmdRaw.qRotation.y, hmdRaw.qRotation.z);
	const Eigen::Vector3d pos(hmdRaw.vecPosition[0], hmdRaw.vecPosition[1], hmdRaw.vecPosition[2]);
	const Eigen::Vector3d worldT = worldFromDriverT + worldFromDriver * pos;
	const Eigen::Quaterniond worldR = (worldFromDriver * rot).normalized();

	const bool fired = spacecal::reanchor_chi::TickAndCheckCandidate(
		g_reanchorChiState, worldT, worldR, now, dt);
	if (fired && (now - g_reanchorChiLastLogTime) >= 1.0) {
		g_reanchorChiLastLogTime = now;
		char buf[200];
		snprintf(buf, sizeof buf,
			"[drift][reanchor-chi-square] fire chi_sq=%.3f threshold=%.3f freeze_until=%.3f",
			g_reanchorChiState.lastChiSquared,
			spacecal::reanchor_chi::kChiSquare6DoF_p1e4,
			g_reanchorChiState.freezeUntil);
		Metrics::WriteLogAnnotation(buf);
	}
	return spacecal::reanchor_chi::IsFrozen(g_reanchorChiState, now);
}

// Predictive recovery pre-correction tick. Runs after TickRestLockedYaw when
// CalCtx.predictiveRecoveryEnabled is true. Reads the rolling buffer of
// recovery events from g_recoveryDeltaBuffer; if the gate (>= 3 events,
// consistent direction) passes, applies a bounded-rate translation nudge
// to ctx.calibratedTranslation. The 30 cm relocalization detector is the
// high-SNR signal source -- rec C only chooses how to extrapolate between
// events.
//
// Bounded twice: kAmount = 0.10 fraction of predicted magnitude per event,
// AND kPredictiveRateCapMps = 0.001 m/s (1 mm/s) per-tick rate cap. Either
// gate alone would prevent the deleted Phase 1+2 silent-recal failure mode;
// together they make the worst-case bias mathematically bounded.
//
// Sign convention: subtract the predicted drift from calibratedTranslation
// to compensate. If a real-session test surfaces the wrong sign, flip here.
constexpr double kPredictiveRateCapMps = 0.001; // 1 mm/s

static void TickPredictiveRecovery(double now) {
	if (!CalCtx.predictiveRecoveryEnabled) {
		spacecal::recovery_delta::Clear(g_recoveryDeltaBuffer);
		g_predictiveRecoveryLastTickTime = -1.0;
		return;
	}
	if (!CalCtx.validProfile) return;

	// Same continuous-cal coexistence rule as rec A: skip during active one-
	// shot sub-states; allow during Continuous, ContinuousStandby, None,
	// Editing. The predictive nudge IS a hint to continuous-cal -- if both
	// run, continuous-cal's per-tick fit will dominate the next tick if they
	// disagree (the nudge becomes a small perturbation absorbed by the EMA
	// blend).
	if (CalCtx.state == CalibrationState::Begin
	 || CalCtx.state == CalibrationState::Rotation
	 || CalCtx.state == CalibrationState::Translation) {
		return;
	}

	double dt = 0.0;
	if (g_predictiveRecoveryLastTickTime > 0.0) {
		dt = now - g_predictiveRecoveryLastTickTime;
	}
	g_predictiveRecoveryLastTickTime = now;

	const Eigen::Vector3d step = spacecal::recovery_delta::ComputePerTickNudge(
		g_recoveryDeltaBuffer, now, dt, kPredictiveRateCapMps);

	if (step.norm() <= 0.0) return;

	// calibratedTranslation is in centimeters (per Configuration.cpp:255 and
	// the publish path's *100.0 conversion). Convert step (meters) to cm.
	CalCtx.calibratedTranslation -= step * 100.0;

	if ((now - g_predictiveRecoveryLastLogTime) >= 1.0) {
		g_predictiveRecoveryLastLogTime = now;
		const size_t live = spacecal::recovery_delta::LiveCount(g_recoveryDeltaBuffer);
		char buf[200];
		snprintf(buf, sizeof buf,
			"[drift][predictive-recovery] apply step_m=(%.6f,%.6f,%.6f) step_norm_mm=%.4f buffer_live=%zu",
			step.x(), step.y(), step.z(), step.norm() * 1000.0, live);
		Metrics::WriteLogAnnotation(buf);
	}
}

void DumpDriftSubsystemState() {
	const double now = glfwGetTime();

	{
		char b[280];
		snprintf(b, sizeof b,
			"[drift][state-dump] header now=%.3f state=%d "
			"rest_locked_yaw=%d predictive_recovery=%d reanchor_chi_square=%d",
			now, (int)CalCtx.state,
			(int)CalCtx.restLockedYawEnabled,
			(int)CalCtx.predictiveRecoveryEnabled,
			(int)CalCtx.reanchorChiSquareEnabled);
		Metrics::WriteLogAnnotation(b);
	}

	// Rest detector: one line per tracker.
	{
		char b[200];
		snprintf(b, sizeof b,
			"[drift][state-dump] rest_detector tracker_count=%zu last_tick=%.3f",
			g_restStates.size(), g_restLockedYawLastTickTime);
		Metrics::WriteLogAnnotation(b);
	}
	for (const auto& kv : g_restStates) {
		const auto& s = kv.second;
		char b[280];
		snprintf(b, sizeof b,
			"[drift][state-dump] rest_detector device=%u phase=%d have_lock=%d phase_entered_at=%.3f locked_yaw_deg=%.4f",
			kv.first,
			(int)s.phase,
			(int)s.haveLock,
			s.phaseEnteredAt,
			std::atan2(2.0 * (s.lockedRot.w() * s.lockedRot.y() + s.lockedRot.z() * s.lockedRot.x()),
			            1.0 - 2.0 * (s.lockedRot.x() * s.lockedRot.x() + s.lockedRot.y() * s.lockedRot.y())) * 180.0 / EIGEN_PI);
		Metrics::WriteLogAnnotation(b);
	}

	// Recovery delta buffer: header + per-event lines.
	{
		char b[200];
		snprintf(b, sizeof b,
			"[drift][state-dump] recovery_buffer total_count=%zu live_count=%zu last_apply_tick=%.3f",
			g_recoveryDeltaBuffer.count,
			spacecal::recovery_delta::LiveCount(g_recoveryDeltaBuffer),
			g_predictiveRecoveryLastTickTime);
		Metrics::WriteLogAnnotation(b);
	}
	const size_t live = spacecal::recovery_delta::LiveCount(g_recoveryDeltaBuffer);
	for (size_t i = 0; i < live; ++i) {
		const auto& ev = g_recoveryDeltaBuffer.events[i];
		char b[260];
		snprintf(b, sizeof b,
			"[drift][state-dump] recovery_event idx=%zu timestamp=%.3f mag_cm=%.2f dir=(%.4f,%.4f,%.4f)",
			i, ev.timestamp, ev.magnitude * 100.0,
			ev.direction.x(), ev.direction.y(), ev.direction.z());
		Metrics::WriteLogAnnotation(b);
	}

	// Chi-square detector summary.
	{
		char b[280];
		snprintf(b, sizeof b,
			"[drift][state-dump] reanchor_chi history_count=%zu variance_count=%d last_chi_sq=%.3f freeze_until=%.3f frozen_now=%d",
			g_reanchorChiState.historyCount,
			g_reanchorChiState.varianceCount,
			g_reanchorChiState.lastChiSquared,
			g_reanchorChiState.freezeUntil,
			(int)spacecal::reanchor_chi::IsFrozen(g_reanchorChiState, now));
		Metrics::WriteLogAnnotation(b);
	}

	{
		char b[120];
		snprintf(b, sizeof b, "[drift][state-dump] footer end_now=%.3f", now);
		Metrics::WriteLogAnnotation(b);
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

	// Auto-recovery snap (option-3 bundle, 2026-05-04). RecoverFromWedgedCalibration
	// sets g_snapNextProfileApply=true so the very next profile-apply cycle sends
	// every per-ID payload with lerp=false (driver snaps transform := target rather
	// than smoothly interpolating). Fallback payloads have no lerp field so they
	// can't snap directly -- but in practice every device that needs the cal has a
	// per-ID slot by the time recovery fires, so per-ID snap covers the user-visible
	// case. Captured at the top of the function and consumed at the end so all
	// per-ID sends in this cycle see the same value.
	const bool snapThisCycle = g_snapNextProfileApply;

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
	// update -- without waiting for the next per-ID scan. The fallback's freeze flag
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
		// adopted device -- those need to snap into place rather than ramping in from
		// identity, which would look like a slow drift to the user. ALSO except when
		// auto-recovery just fired (snapThisCycle): the recovery's brand-new cal must
		// land discontinuously, blending it would defeat the recovery.
		// Decision routed through the pure helper so test_motion_gate.cpp pins
		// the contract.
		payload.lerp = spacecal::motiongate::ShouldBlendCycle(
			/*inContinuousState=*/CalCtx.state == CalibrationState::Continuous,
			/*isFreshlyAdopted=*/isFreshlyAdopted,
			/*snapThisCycle=*/snapThisCycle);
		payload.quash = CalCtx.state == CalibrationState::Continuous && id == CalCtx.targetID && CalCtx.quashTargetInContinuous;
		SetTargetSystemField(payload, ctx.targetTrackingSystem);

		// predictionSmoothness moved to the Smoothing overlay on 2026-05-11
		// (Protocol v12). The driver ignores this field on SetDeviceTransform
		// from v12 onward; SC sends 0 to keep wire layout stable.
		payload.predictionSmoothness = 0;

		// Motion-gated blend -- when on, the driver-side BlendTransform's lerp
		// only advances proportional to detected per-frame motion. Hides offset
		// shifts in the user's natural movement; eliminates "phantom drift" while
		// stationary. Default on at the profile level.
		payload.recalibrateOnMovement = ctx.recalibrateOnMovement;

		SendDeviceTransformIfChanged(id, payload);

		// Record this ID as adopted (it's receiving a per-target-system transform with
		// enabled=true). g_lastSeenSerial[id] is freshly populated above; pair it with
		// the model name for log readability. RenderModel falls back to empty string
		// on failure -- we don't gate the log on that.
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

	// Consume the one-shot auto-recovery snap flag -- only after every per-ID
	// payload in this cycle has been sent, so the snap reaches all devices.
	// Subsequent cycles return to normal lerp behaviour.
	if (snapThisCycle) {
		g_snapNextProfileApply = false;
		Metrics::WriteLogAnnotation("auto_recovery_snap_consumed: post-recovery profile sent with payload.lerp=false");
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

	// One-shot session-start config dump. Fires on the first non-skipped
	// CalibrationTick after the profile has been loaded, so the annotation
	// reflects the user's actual saved settings. Captures every experimental
	// toggle + the load-bearing tunables. Lets a session reader skip the
	// "what version of the math is running" reverse-derivation from code.
	{
		static bool s_loggedConfigDump = false;
		if (!s_loggedConfigDump) {
			s_loggedConfigDump = true;
			char dumpBuf[512];
			snprintf(dumpBuf, sizeof dumpBuf,
				"session_config_dump: upstream=%d gcc_phat=%d cusum=%d velocity_aware=%d tukey=%d kalman=%d"
				" auto_detect_latency=%d ignore_outliers=%d static_recal=%d"
				" recalibrate_on_movement=%d cal_speed=%.2f jitter_threshold=%.2f",
				(int)ctx.useUpstreamMath,
				(int)ctx.useGccPhatLatency, (int)ctx.useCusumGeometryShift,
				(int)ctx.useVelocityAwareWeighting, (int)ctx.useTukeyBiweight,
				(int)ctx.useBlendFilter,
				(int)ctx.latencyAutoDetect, (int)ctx.ignoreOutliers,
				(int)ctx.enableStaticRecalibration, (int)ctx.recalibrateOnMovement,
				(double)ctx.calibrationSpeed, (double)ctx.jitterThreshold);
			Metrics::WriteLogAnnotation(dumpBuf);
		}
	}

	// Bounds-check the device IDs once at the top of the tick. Many code paths
	// downstream index devicePoses[ctx.referenceID] / devicePoses[ctx.targetID]
	// directly (CollectSample, the sample-history pose recording near the end of
	// this function, etc.), and a stale negative or out-of-range value reaches
	// for memory outside the array. We tolerate -1 (the not-yet-assigned sentinel)
	// because state machines below explicitly handle that, but anything else that
	// isn't in [0, k_unMaxTrackedDeviceCount) means we cannot run any per-device
	// logic this tick -- bail out and try again next tick.
	const int32_t maxId = (int32_t)vr::k_unMaxTrackedDeviceCount;
	auto idInRangeOrUnset = [maxId](int32_t id) {
		return id == -1 || (id >= 0 && id < maxId);
	};
	if (!idInRangeOrUnset(ctx.referenceID) || !idInRangeOrUnset(ctx.targetID)) {
		// Defensive reset: a corrupted ID is unrecoverable for this tick. Don't
		// touch state -- we just skip the tick so the next AssignTargets() call can
		// reseat the IDs cleanly.
		return;
	}

	// Hybrid HMD-relocalization detector. Runs in every state where a tick
	// proceeds; the function itself skips active calibration sub-states
	// where the HMD is being deliberately moved. Logging-only -- emits a
	// `# [time] hmd_relocalization_detected: ...` annotation when the
	// triple-AND trigger fires, but doesn't modify R or the chaperone yet.
	// We run this BEFORE TickBaseStationDrift so the base-station cache it
	// shares is populated with the previous tick's poses (the relocalization
	// detector compares against THIS tick's poses; the universe-shift
	// detector cares about pose changes between consecutive ticks).
	TickHmdRelocalizationDetector(time);

	// Chi-square re-anchor sub-detector (rec F). When fired, returns true so
	// recs A and C skip their tick for the freeze window. Detection-only:
	// the 30 cm detector above is still the only path to actual recovery.
	const bool restAndRecCFrozen = TickReanchorChiSquare(time);

	// Rest-locked yaw drift correction (rec A). Opt-in via Experimental tab,
	// default OFF. Skips Continuous mode (continuous-cal already handles drift)
	// and active one-shot sub-states. When at-rest signal is available, applies
	// a bounded-rate yaw nudge to ctx.calibratedRotation(1); the existing
	// publish path picks up the change via SendFallbackIfChanged.
	if (!restAndRecCFrozen) {
		TickRestLockedYaw(time);
	}

	// Predictive recovery pre-correction (rec C). Opt-in via Experimental tab,
	// default OFF. Reads the rolling buffer of 30 cm relocalization events and
	// applies a bounded-rate translation nudge if the gate passes.
	if (!restAndRecCFrozen) {
		TickPredictiveRecovery(time);
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
		static spacecal::geometry_shift::CusumState s_cusumState;
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

				bool fire = false;
				if (ctx.useCusumGeometryShift) {
					// Page CUSUM: accumulates (current - baseline - drift) per
					// tick, fires when the running sum crosses threshold. Median
					// stays as the baseline so the test is centered on the recent
					// no-shift behavior. Fire path resets the CUSUM state to zero
					// inside UpdateCusumGeometryShift.
					fire = spacecal::geometry_shift::UpdateCusumGeometryShift(
						s_cusumState, current, median);
					// Mirror the consecutive-bad-tick counter to zero in this path
					// so a toggle flip mid-session doesn't leave stale state.
					s_consecutiveBadTicks = 0;
				} else {
					if (spacecal::geometry_shift::IsCurrentErrorSpike(current, median)) {
						s_consecutiveBadTicks++;
					} else {
						s_consecutiveBadTicks = 0;
					}
					fire = spacecal::geometry_shift::ShouldFireGeometryShiftRecovery(s_consecutiveBadTicks);
					// Reset CUSUM accumulator when the legacy path is active so a
					// later toggle flip starts from a clean state rather than a
					// stale running-sum from before the toggle change.
					s_cusumState.S = 0.0;
				}

				if (fire) {
					CalCtx.Log("Tracking geometry shifted -- restarting calibration\n");
					calibration.Clear();
					ctx.state = CalibrationState::ContinuousStandby;
					ctx.relativePosCalibrated = false;
					s_consecutiveBadTicks = 0;
					s_cusumState.S = 0.0;
				}
			}
		} else {
			s_consecutiveBadTicks = 0;
			s_cusumState.S = 0.0;
		}
	}

	// Latency cross-correlation. Once per second, if the rolling speed buffers
	// are full and the user has actually been moving (RMS speed > 0.1 m/s on both
	// signals), compute a discrete cross-correlation and update the EMA estimate.
	// The active value is then used by CollectSample on subsequent ticks when
	// latencyAutoDetect is on.
	if (!ctx.useUpstreamMath
		&& (time - ctx.timeLastLatencyEstimate) > 1.0
		&& ctx.refSpeedHistory.size() >= CalibrationContext::kLatencyHistoryCapacity
		&& ctx.targetSpeedHistory.size() >= CalibrationContext::kLatencyHistoryCapacity
		&& ctx.speedSampleTimes.size() >= CalibrationContext::kLatencyHistoryCapacity)
	{
		ctx.timeLastLatencyEstimate = time;
		double lagSamples = 0.0;
		const int kMaxTau = 10;
		if (EstimateLatencyLagSamples(ctx.refSpeedHistory, ctx.targetSpeedHistory, kMaxTau, ctx.useGccPhatLatency, &lagSamples)) {
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
					const double prevEma = ctx.estimatedLatencyOffsetMs;
					ctx.estimatedLatencyOffsetMs = 0.7 * ctx.estimatedLatencyOffsetMs + 0.3 * lagMs;
					// Forensic diagnostic for audit row #12 (project_upstream_regression_audit_2026-05-04).
					// The cross-correlation buffer doesn't reset across HMD
					// stalls -- `dur` here can span the stall duration while
					// `intervals` is bounded by buffer size, producing a
					// deflated sample rate and inflating lagMs. The 200 ms
					// clamp above bounds this, but the EMA can still drift
					// for a few cycles. Log every successful update with
					// dur and sampleRateHz so post-stall anomalies show up
					// in the data. Throttled by the existing 1 s gate at
					// line 2167 (timeLastLatencyEstimate).
					char latbuf[224];
					snprintf(latbuf, sizeof latbuf,
						"latency_ema_update: lagMs=%.2f sampleRateHz=%.2f intervals=%zu dur=%.2fs prev=%.2f new=%.2f",
						lagMs, sampleRateHz, intervals, dur,
						prevEma, ctx.estimatedLatencyOffsetMs);
					Metrics::WriteLogAnnotation(latbuf);
				}
			}
		}
	}

	// External smoothing-tool detection moved to the Smoothing overlay's
	// Tick (Protocol v12, 2026-05-11); its plugin scans on its own 5-second
	// cadence and surfaces the banner inside its Prediction sub-tab.

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
		// Counter is preserved for the existing diagnostic UI in
		// UserInterface.cpp ("Stall purge: N events") and the
		// "hmd_stall_recovered after N ticks" log annotation below.
		ctx.consecutiveHmdStalls++;
		// REVERTED 2026-05-04: previously, after MaxHmdStalls=30 ticks of stalled
		// HMD tracking, the sample buffer was purged via calibration.Clear() and
		// state was demoted to ContinuousStandby. The intent was "stale samples no
		// longer represent reality" -- but the actual effect was much worse than
		// the problem it solved: on stall recovery, StartContinuousCalibration()
		// re-applies the saved refToTargetPose warm-start (relativePosCalibrated
		// is NOT reset, asymmetric vs the geometry-shift detector at line 2120
		// which DOES reset it), and continuous-cal converges from new post-stall
		// samples against that stale constraint. Each HMD-off/on cycle landed at
		// a slightly different local minimum; SaveProfile persisted it; cumulative
		// drift across many cycles wedged the saved profile.
		//
		// Empirical evidence (spacecal_log.2026-05-04T17-14-50.txt): two HMD off/on
		// events at t=1918 (56 ticks) and t=2096 (95 ticks) each produced a 7-9 cm
		// Z-axis shift in posOffset_currentCal IMMEDIATELY post-recovery, with the
		// cal magnitude climbing toward the wedge bound across the session.
		// Upstream (hyblocker) just `return`s on stall -- no clear, no demote -- and
		// the user reports this drift didn't happen on the old fork.
		//
		// Now matching upstream behavior: just return. Stale samples in the rolling
		// buffer naturally age out as fresh ones come in post-stall; the existing
		// rolling-window solver handles the transition without a discrete reset.
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
		// Base station drift correction (one-shot mode only): catch SteamVR
		// universe shifts so body trackers stay aligned with the user's
		// physical position. No-op when no base stations are detected.
		// Continuous mode is intentionally skipped -- the live solver would
		// converge through the shift on its own within a few seconds.
		TickBaseStationDrift(time);

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

	// Two-phase one-shot motion-variety gate. Continuous mode bypasses this --
	// it has its own incremental accept/reject loop that doesn't need a "stop
	// here" signal.
	//
	// Earlier this was a single combined gate ("both diversities >= 70 % or
	// keep rolling"). That trapped users in an unwinnable game with the
	// rolling 250-sample buffer: rotate first -> rotation samples age out
	// before translation samples accumulate; translate first -> vice versa.
	// The user-visible symptom was the "Translation %" bar that "never
	// reaches 100" because the buffer recycled rotation-rich content out
	// before the user could fill the translation half.
	//
	// Two-phase flow:
	//   Rotation phase: gate on rotationDiversity only. Buffer rolls until
	//   the user has rotated through >= 90 deg between some pair of samples.
	//   When the gate passes, freeze the buffer (FreezeRotationPhaseSamples)
	//   and transition to Translation. The freeze preserves the rotation
	//   samples for the final solve regardless of how slowly the user fills
	//   the translation half.
	//
	//   Translation phase: gate on translationDiversity only, computed on a
	//   fresh live buffer. Buffer rolls until the user has waved >= 30 cm on
	//   every axis. When the gate passes, fall through to ComputeOneshot,
	//   which splices the frozen rotation samples back in for the math.
	if (CalCtx.state == CalibrationState::Rotation) {
		constexpr double kPhaseDiversity = 0.70;
		if (calibration.RotationDiversity() < kPhaseDiversity) {
			calibration.ShiftSample();
			return;
		}
		// Rotation phase complete. Freeze the buffer, transition state, and
		// return so the next CollectSample tick starts populating a fresh
		// translation-phase buffer. The popup's Rotation% bar will visually
		// drop to 0 (the new buffer is empty) but the UI latches it at 100%
		// while CalCtx.state == Translation so the user sees the achievement
		// preserved.
		calibration.FreezeRotationPhaseSamples();
		CalCtx.state = CalibrationState::Translation;
		CalCtx.Log("Rotation phase complete. Now wave the tracker through ~30 cm on every axis.\n");
		Metrics::WriteLogAnnotation("RotationPhaseFrozen");
		return;
	}
	if (CalCtx.state == CalibrationState::Translation) {
		constexpr double kPhaseDiversity = 0.70;
		if (calibration.TranslationDiversity() < kPhaseDiversity) {
			calibration.ShiftSample();
			return;
		}
		// Translation diversity satisfied -- fall through to ComputeOneshot
		// below. ComputeOneshot's RotationFreezeSplice will prepend the
		// frozen rotation samples for the duration of the solve.
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
	calibration.useVelocityAwareWeighting = CalCtx.useVelocityAwareWeighting && !CalCtx.useUpstreamMath;
	calibration.useTukeyBiweight = CalCtx.useTukeyBiweight && !CalCtx.useUpstreamMath;
	calibration.useBlendFilter = CalCtx.useBlendFilter && !CalCtx.useUpstreamMath;

	if (CalCtx.state == CalibrationState::Continuous) {
		CalCtx.messages.clear();
		calibration.enableStaticRecalibration = CalCtx.enableStaticRecalibration;
		calibration.lockRelativePosition = CalCtx.lockRelativePosition;
		// User-toggled "Pause updates" from the continuous-cal UI: keep the
		// already-applied driver offset live, skip any new solve cycle so the
		// math doesn't fight the user trying to inspect the current result.
		if (!CalCtx.calibrationPaused) {
			calibration.ComputeIncremental(lerp, CalCtx.continuousCalibrationThreshold, CalCtx.maxRelativeErrorThreshold, CalCtx.ignoreOutliers);

			// Sustained gravity-axis disagreement diagnostic. Push the per-tick
			// residual pitch+roll reading into a rolling 60s window and ask
			// the pure-helper decision (TiltDiagnostic.h) whether the median
			// has crossed the 1.0 deg sustained threshold. Logging-only:
			// surfaces "your two systems disagree about which way is down"
			// as a sustained signal so the user can re-run room setup. No
			// calibration behavior change. See project_future_improvements_
			// 2026-05-05.md for the eventual Pacher-2021 RFU-style
			// correction that this diagnostic is the prerequisite for.
			{
				const double tiltDeg = calibration.m_residualPitchRollDeg;
				if (std::isfinite(tiltDeg) && tiltDeg >= 0.0) {
					CalCtx.tiltDiagnosticWindow.push_back({time, tiltDeg});
					const double cutoff = time - spacecal::gravity::kSustainedWindowSeconds;
					while (!CalCtx.tiltDiagnosticWindow.empty()
						&& CalCtx.tiltDiagnosticWindow.front().timestamp_s < cutoff) {
						CalCtx.tiltDiagnosticWindow.pop_front();
					}
					const auto decision = spacecal::gravity::EvaluateTilt(
						CalCtx.tiltDiagnosticWindow, time, CalCtx.tiltSustainedAlarmed);
					if (decision.sustainedDisagreement != CalCtx.tiltSustainedAlarmed) {
						CalCtx.tiltSustainedAlarmed = decision.sustainedDisagreement;
						CalCtx.tiltLastAnnotatedMedian = decision.medianDeg;
						char tbuf[224];
						snprintf(tbuf, sizeof tbuf,
							"gravity_disagreement_%s: median_tilt_deg=%.3f window_samples=%zu threshold=%.2f",
							decision.sustainedDisagreement ? "sustained_on" : "sustained_off",
							decision.medianDeg, CalCtx.tiltDiagnosticWindow.size(),
							spacecal::gravity::kSustainedTiltThresholdDeg);
						Metrics::WriteLogAnnotation(tbuf);
					}
				}
			}
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

			auto vmag = [](const double v[3]) -> double {
				const double s = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
				return std::isfinite(s) ? s : 0.0;
			};
			Sample s(ConvertPose(refPose), ConvertPose(tgtPose), glfwGetTime(),
			         vmag(refPose.vecVelocity), vmag(tgtPose.vecVelocity));
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

		// Runtime wedge detector REMOVED 2026-05-05 -- fired in a 3-fire
		// reset loop on the user's Quest+Lighthouse setup whose legitimate
		// continuous-cal convergence values (~265-295 cm) sit above any
		// fixed magnitude threshold we picked. See
		// project_wedge_guard_removed_2026-05-05.md (memory). Quest
		// re-localization auto-recovery in TickHmdRelocalizationDetector
		// (HMD-jump signal, not magnitude) is unchanged and still active.
	} else {
		CalCtx.Log("Calibration failed.\n");
	}

	LARGE_INTEGER end_time;
	QueryPerformanceCounter(&end_time);
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	double duration = (end_time.QuadPart - start_time.QuadPart) / (double)freq.QuadPart;
	const double computationTimeMs = duration * 1000.0;
	Metrics::computationTime.Push(computationTimeMs);

	// CPU-pressure diagnostic. Sampled at the end of each CalibrationTick so the
	// computationTime above is in scope for the per-tick spike check. Pure
	// logging: emits cpu_pressure_warning_on/_off transitions when the
	// 5-second EMA of process CPU% crosses 50%/30% (with hysteresis), and a
	// throttled cpu_pressure_spike on any single ComputeIncremental >= 200 ms.
	TickCpuPressureMonitor(computationTimeMs, time);

	// Hand the raw reference + target poses to the metrics writer so the v2 CSV
	// columns get filled. Reconstructing these in the replay harness (tools/replay/)
	// gives us the same Sample values that fed CalibrationCalc::PushSample, which
	// is the whole point of the harness -- the metric-level columns alone aren't
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

bool LastDetectedRelocalization(double& outAgeSeconds, double& outDeltaMeters,
                                double& outDeltaDegrees) {
	if (g_relocDetector.lastFireTime <= -1e8) return false;
	const double now = glfwGetTime();
	outAgeSeconds = now - g_relocDetector.lastFireTime;
	outDeltaMeters = g_relocDetector.lastFireDelta.norm();
	outDeltaDegrees = g_relocDetector.lastFireRotRad * 180.0 / EIGEN_PI;
	return true;
}

bool LastAutoRecoveryActive(double& outAge, double& outDeltaMeters) {
	auto& s = g_relocDetector;
	if (!s.lastAutoRecoverSnapshot.valid)   return false;
	if (s.autoRecoverBannerDismissed)       return false;
	if (s.lastAutoRecoverTime <= -1e8)      return false;
	const double now = glfwGetTime();
	const double age = now - s.lastAutoRecoverTime;
	// 60 s sticky window matches the audit suggestion. After that the
	// banner self-dismisses; the snapshot itself stays valid in case some
	// other path wants to expose Undo (currently nothing does).
	if (age > 60.0) return false;
	outAge          = age;
	outDeltaMeters  = s.lastFireDelta.norm();
	return true;
}

bool UndoLastAutoRecovery() {
	auto& s = g_relocDetector;
	if (!s.lastAutoRecoverSnapshot.valid) return false;

	// Diagnostic: dump the snapshot we're about to restore. Useful when the
	// user reports "I hit Undo and tracking is still wrong" -- we want to
	// see what state we put them back into. Logged BEFORE the restore so
	// even if the restore is a no-op for some reason, we have the data.
	{
		const auto& snap = s.lastAutoRecoverSnapshot;
		const Eigen::Vector3d t = snap.refToTargetPose.translation();
		char dumpbuf[256];
		snprintf(dumpbuf, sizeof dumpbuf,
			"undo_snapshot_dump (Undo): refToTarget_t=(%.3f,%.3f,%.3f) magnitude=%.3f relativePosCalibrated=%d hasAppliedCalibrationResult=%d",
			t.x(), t.y(), t.z(), t.norm(),
			(int)snap.relativePosCalibrated, (int)snap.hasAppliedCalibrationResult);
		Metrics::WriteLogAnnotation(dumpbuf);
	}

	// Restore the three CalCtx fields the recovery had cleared. The
	// continuous-cal tick will then re-apply the saved relative-pose
	// constraint via setRelativeTransformation on the next frame, putting
	// the body trackers back where they were before the (false-positive)
	// auto-recover clobbered the calibration.
	CalCtx.refToTargetPose             = s.lastAutoRecoverSnapshot.refToTargetPose;
	CalCtx.relativePosCalibrated       = s.lastAutoRecoverSnapshot.relativePosCalibrated;
	CalCtx.hasAppliedCalibrationResult = s.lastAutoRecoverSnapshot.hasAppliedCalibrationResult;

	// Invalidate the snapshot so a second Undo click does nothing.
	s.lastAutoRecoverSnapshot.valid    = false;
	s.autoRecoverBannerDismissed       = true;

	Metrics::WriteLogAnnotation("auto_recover_undone: pre-recovery calibration restored from snapshot");
	CalCtx.Log("Auto-recovery undone. Restored pre-recovery calibration.\n");
	return true;
}

void DismissAutoRecoveryBanner() {
	g_relocDetector.autoRecoverBannerDismissed = true;
}

// Wedge recovery -- the canonical wipe routine. Used by both the Quest
// re-localization auto-recovery (TickHmdRelocalizationDetector) and the
// runtime wedge detector (CalibrationTick post-cal-update block).
//
// Wipes:
//   - calibration.m_estimatedTransformation, m_isValid, m_samples, etc.
//   - CalCtx.refToTargetPose (warm-start for the relative-pose constraint)
//   - CalCtx.relativePosCalibrated (so StartContinuousCalibration's
//     setRelativeTransformation call passes `false` and doesn't re-anchor
//     to the wedged value)
//   - CalCtx.hasAppliedCalibrationResult (lets the trigger-press gate
//     re-arm if the user has it enabled)
//   - CalCtx.calibratedTranslation / calibratedRotation (the values that
//     are actually persisted to the saved profile and applied to the
//     driver via ScanAndApplyProfile). project_auto_recovery_2026-05-03.md
//     called out that calibration.Clear() doesn't touch these -- leaving
//     them wedged here is what made the earlier SaveProfile-after-Clear
//     persist bad state. Zero them explicitly.
//
// Does NOT call SaveProfile. The next continuous-cal tick that produces
// a valid result will overwrite the on-disk profile via the existing
// path at the end of CalibrationTick (~line 2620), at which point we'll
// be persisting the post-recovery values, not the wedged ones.
//
// `userFacingMessage` is appended to CalCtx.messages AFTER the restart
// (StartContinuousCalibration internally clears the buffer, so it must be
// last). Pass nullptr to suppress the user-facing log if the trigger is
// ambient/silent (e.g. a runtime wedge clear that shouldn't surface UI text
// per the 2026-05-04 "user notices nothing" directive). The metrics
// annotation is the caller's responsibility -- this helper deliberately
// doesn't write one so each caller's grep key can differ.
static void RecoverFromWedgedCalibration(const char* userFacingMessage) {
	// Capture the prior cal state BEFORE we discard it, so the log line
	// records what we just threw away. Anyone reading the session log
	// later can reconstruct "the cal we cleared was X cm with Y mm RMS"
	// without having to grep for the latest values from earlier in the
	// file.
	{
		const double priorTransMagCm = calibration.Transformation().translation().norm() * 100.0;
		const Eigen::Vector3d priorEulerDeg = calibration.EulerRotation();
		const bool priorWasValid = calibration.isValid();
		const double priorBufferSamples = static_cast<double>(calibration.SampleCount());
		char priorBuf[320];
		snprintf(priorBuf, sizeof priorBuf,
			"recovery_prior_state: was_valid=%d trans_mag_cm=%.2f euler_deg=(%.2f,%.2f,%.2f)"
			" sample_count=%.0f relativePosCalibrated=%d",
			(int)priorWasValid, priorTransMagCm,
			priorEulerDeg.x(), priorEulerDeg.y(), priorEulerDeg.z(),
			priorBufferSamples, (int)CalCtx.relativePosCalibrated);
		Metrics::WriteLogAnnotation(priorBuf);
	}

	calibration.Clear();
	CalCtx.refToTargetPose             = Eigen::AffineCompact3d::Identity();
	CalCtx.relativePosCalibrated       = false;
	CalCtx.hasAppliedCalibrationResult = false;
	CalCtx.calibratedTranslation       = Eigen::Vector3d::Zero();
	CalCtx.calibratedRotation          = Eigen::Vector3d::Zero();

	// Snap the next ScanAndApplyProfile send (one-shot). The driver's
	// SetDeviceTransform handler will see payload.lerp=false and assign
	// transform := targetTransform directly, bypassing BlendTransform.
	// Without this, the recovery's brand-new cal would be smoothly
	// interpolated through the driver's stale cached transform -- which
	// defeats the point of recovery (we WANT a discontinuity here).
	g_snapNextProfileApply = true;

	StartContinuousCalibration();

	if (userFacingMessage != nullptr) {
		CalCtx.Log(userFacingMessage);
	}
}

// Manual playspace recenter: shift the standing zero pose so the user's
// current physical HMD position becomes the chaperone center. Called from
// the "Recenter playspace" UI button. Y is preserved (don't recalibrate
// floor) and rotation is preserved (don't change yaw); only X/Z translate.
//
// Math derivation (so the next person editing this can double-check):
//
//   SZP is the standing-universe origin's pose expressed in the raw
//   tracking universe. SZP = [R_szp | t_szp; 0 | 1]. A device's pose in
//   the standing universe is then:
//
//     standing_pose = SZP^-1 * raw_pose
//                   -> translation = R_szp^-1 * (raw_pos - t_szp)
//
//   We want the user's HMD standing-pose translation to become (0, y, 0)
//   in X/Z while preserving Y. Setting:
//
//     t_szp_new = t_szp_old + R_szp * (hmd_standing_x, 0, hmd_standing_z)
//
//   makes hmd_standing_new = hmd_standing_old - (hmd_standing_x, 0, hmd_standing_z)
//                          = (0, hmd_standing_y, 0).
//
//   For the common case where SZP's rotation is yaw-only (room
//   calibration's typical state), R_szp * (x, 0, z) is just (x, 0, z)
//   rotated about the Y axis -- so we have to apply the rotation, not
//   just add the X/Z components. (An earlier draft of this function got
//   that wrong; the addition only happened to be correct when yaw was
//   exactly 0.)
//
// Returns true on success. Failures: VRChaperoneSetup unavailable, HMD
// pose not valid.
bool RecenterPlayspaceToCurrentHmd() {
	if (!vr::VRSystem() || !vr::VRChaperoneSetup()) return false;

	// Read HMD's current pose in the standing universe. With predictionSecs=0
	// we get the "now" pose -- no extrapolation, which is what we want for
	// a one-shot snapshot anchor.
	vr::TrackedDevicePose_t poses[1] = {};
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
		vr::TrackingUniverseStanding, 0.0f, poses, 1);
	if (!poses[0].bPoseIsValid
		|| poses[0].eTrackingResult != vr::TrackingResult_Running_OK) {
		CalCtx.Log("Recenter playspace: HMD pose not valid; aborting.\n");
		return false;
	}
	const auto& hmdMat = poses[0].mDeviceToAbsoluteTracking;
	const float hmdStandingX = hmdMat.m[0][3];
	const float hmdStandingZ = hmdMat.m[2][3];

	// Defensive: throw away any pending working copy so we read the live
	// state, not whatever some other code path is in the middle of editing.
	// Mirrors what ApplyChaperoneBounds does.
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	vr::HmdMatrix34_t szp{};
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&szp);

	// Delta = R_szp * (hmd_standing_x, 0, hmd_standing_z). Use the X/Z columns
	// of R_szp's row-major 3x3 (the [_][0] and [_][2] columns).
	const float dx = szp.m[0][0] * hmdStandingX + szp.m[0][2] * hmdStandingZ;
	const float dz = szp.m[2][0] * hmdStandingX + szp.m[2][2] * hmdStandingZ;
	// Y delta would be szp.m[1][0] * hmdStandingX + szp.m[1][2] * hmdStandingZ.
	// For yaw-only SZP that's 0 (Y-axis row is (0, 1, 0)). For tilted SZPs
	// it's small and cancels naturally if the user wants Y preserved.
	// Either way we do NOT modify Y to keep floor calibration intact.
	const float oldTx = szp.m[0][3];
	const float oldTz = szp.m[2][3];

	szp.m[0][3] = oldTx + dx;
	szp.m[2][3] = oldTz + dz;

	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&szp);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);

	// Log enough that a debug-log reader can verify the action did what was
	// intended: HMD pose pre-shift, delta applied, expected post-shift HMD
	// position (~0, y, ~0 in X/Z if the math is right).
	char logbuf[256];
	snprintf(logbuf, sizeof logbuf,
		"playspace_recenter: hmd_standing=(%.3f, %.3f, %.3f) -> "
		"szp_translation_delta=(%.3f, 0, %.3f); szp_translation %.3f,%.3f -> %.3f,%.3f",
		hmdStandingX, hmdMat.m[1][3], hmdStandingZ,
		dx, dz,
		oldTx, oldTz, szp.m[0][3], szp.m[2][3]);
	CalCtx.Log(logbuf);
	Metrics::WriteLogAnnotation(logbuf);

	return true;
}
