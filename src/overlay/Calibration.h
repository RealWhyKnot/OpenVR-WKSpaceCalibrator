#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Windows.h>
#include <openvr.h>
#include <vector>
#include <deque>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "Protocol.h"
// We hold a unique_ptr<CalibrationCalc> in AdditionalCalibration. unique_ptr's
// implicit destructor needs the pointee type complete at the destructor's
// site, including in any TU that destroys an instance (test/replay stubs that
// declare a global CalibrationContext do, transitively). Pulling the full
// header in here -- rather than forward-declaring + defining the destructor
// in Calibration.cpp -- keeps the destructor available everywhere without
// dragging the stubs through extra link steps.
#include "CalibrationCalc.h"
#include "TiltDiagnostic.h"  // spacecal::gravity::TiltSample for the diagnostic window

enum class CalibrationState
{
	None,
	Begin,
	Rotation,
	Translation,
	Editing,
	Continuous,
	ContinuousStandby,
};

// Persistent identity for a tracked device. Used to re-resolve the live
// vr device ID after restart / reconnection by matching serial. Defined
// here (above AdditionalCalibration) so the additional-calibration struct
// can hold one without a forward-decl dance.
struct StandbyDevice {
	std::string trackingSystem;
	std::string model, serial;
};

// One additional calibration entry for the multi-ecosystem case. Each
// AdditionalCalibration aligns one non-HMD tracking system to the HMD's
// tracking system, independently of the "primary" calibration carried in the
// singular fields of CalibrationContext.
//
// Example: a user with a Quest HMD + SlimeVR body trackers + a single Vive
// tracker glued to the headset has the SlimeVR alignment as the primary, and
// the Vive alignment as one entry in additionalCalibrations.
//
// Each entry runs its own continuous-calibration loop with its own sample
// buffer (the unique_ptr<CalibrationCalc> is per-entry). The driver sees them
// via per-tracking-system fallback transforms -- it doesn't care how many
// entries the overlay tracks; it just applies whatever fallbacks arrive over
// IPC, one per tracking system name.
struct AdditionalCalibration {
	// Identification.
	std::string targetTrackingSystem;

	// Standby record for the target device, used to re-resolve targetID
	// after restart / device reconnection (matched by serial). Reference
	// standby isn't stored here -- the reference is always the HMD, looked
	// up at scan time, same as the primary calibration.
	StandbyDevice targetStandby;

	// Live IDs, refreshed each scan tick.
	int32_t referenceID = -1;
	int32_t targetID = -1;

	// Calibration result, in the same units as the primary.
	Eigen::Vector3d calibratedRotation = Eigen::Vector3d::Zero();
	Eigen::Vector3d calibratedTranslation = Eigen::Vector3d::Zero();
	double calibratedScale = 1.0;
	Eigen::Vector3d continuousCalibrationOffset = Eigen::Vector3d::Zero();

	// Per-extra lock + relative-pose state. Same semantics as the primary's
	// lockRelativePositionMode and friends.
	int lockMode = 2; // 0=OFF, 1=ON, 2=AUTO. int instead of LockMode to keep
	                  // the forward-decl situation simple here.
	Eigen::AffineCompact3d refToTargetPose = Eigen::AffineCompact3d::Identity();
	bool relativePosCalibrated = false;
	std::deque<Eigen::AffineCompact3d> autoLockHistory;
	bool autoLockEffectivelyLocked = false;

	// Resolved (effective) lock state -- ResolveLockMode mirror for extras.
	bool lockRelativePosition = false;

	// Per-extra math state. Pointer (not value) so the type can stay
	// forward-declared in this header. Constructed lazily in Calibration.cpp.
	std::unique_ptr<CalibrationCalc> calc;

	// True once a calibration has been computed for this entry. Until then,
	// no fallback is sent for this tracking system.
	bool valid = false;

	// True when this entry is participating in continuous mode. Set once
	// the wizard finishes calibrating the entry; cleared when the user
	// removes the entry.
	bool enabled = true;

	// Defaulted in-class because CalibrationCalc is a complete type at this
	// point (we included its header above), so unique_ptr's destructor is
	// inlinable. Out-of-line definitions in Calibration.cpp would only show
	// up to TUs that link the overlay -- the test/replay stubs that pull
	// Calibration.h transitively need the destructor available everywhere.
	AdditionalCalibration() : calc(std::make_unique<CalibrationCalc>()) {}
	~AdditionalCalibration() = default;
	AdditionalCalibration(const AdditionalCalibration&) = delete;
	AdditionalCalibration& operator=(const AdditionalCalibration&) = delete;
	AdditionalCalibration(AdditionalCalibration&&) noexcept = default;
	AdditionalCalibration& operator=(AdditionalCalibration&&) noexcept = default;
};

struct CalibrationContext
{
	CalibrationState state = CalibrationState::None;
	int32_t referenceID = -1, targetID = -1;

	static const size_t MAX_CONTROLLERS = 8;
	int32_t controllerIDs[MAX_CONTROLLERS];

	StandbyDevice targetStandby, referenceStandby;

	Eigen::Vector3d calibratedRotation;
	Eigen::Vector3d calibratedTranslation;
	double calibratedScale;

	std::string referenceTrackingSystem;
	std::string targetTrackingSystem;

	bool enabled = false;
	bool validProfile = false;
	bool clearOnLog = false;
	bool quashTargetInContinuous = false;
	double timeLastTick = 0, timeLastScan = 0, timeLastAssign = 0;
	bool ignoreOutliers = false;
	double wantedUpdateInterval = 1.0;
	float jitterThreshold = 3.0f;

	bool requireTriggerPressToApply = false;
	bool wasWaitingForTriggers = false;
	bool hasAppliedCalibrationResult = false;

	// One-shot mode: detect SteamVR universe shifts (chaperone reset, seated
	// zero pose reset, etc.) by watching the poses of TrackingReference
	// devices (Lighthouse base stations). When a uniform rigid delta moves
	// every base station in the same tracking system between two consecutive
	// ticks, that's a universe re-origin -- apply the inverse to the stored
	// calibration so body trackers stay aligned with the user's physical
	// position. AUTO (true): runs when ≥2 TrackingReference devices are
	// detected for the relevant system; otherwise no-ops. OFF (false): never
	// runs. Default AUTO. No effect in continuous mode -- continuous already
	// updates each tick and would converge through the shift anyway.
	bool baseStationDriftCorrectionEnabled = true;

	float xprev, yprev, zprev;
	int consecutiveHmdStalls = 0;

	float continuousCalibrationThreshold;
	float maxRelativeErrorThreshold = 0.005f;
	Eigen::Vector3d continuousCalibrationOffset;

	// Manual per-target-system end-to-end-latency offset (milliseconds). When non-zero,
	// CollectSample extrapolates the most recent reference pose forward/backward by the
	// time delta between the reference and target shmem sample timestamps plus this
	// offset, using the reference pose's reported linear/angular velocity. This
	// compensates for systems with different latencies (e.g. a wireless tracker
	// running ~10–30 ms behind a Lighthouse-tracked reference). Default 0 produces
	// identical behaviour to before the offset was introduced. Auto-detection (see
	// latencyAutoDetect / estimatedLatencyOffsetMs below) can override this at runtime.
	double targetLatencyOffsetMs = 0.0;

	// When true, the cross-correlation latency auto-detector overrides
	// targetLatencyOffsetMs at each scan tick with the most recent estimate.
	// When false, the manual targetLatencyOffsetMs value is used.
	bool latencyAutoDetect = false;
	// EMA of the estimated latency offset in milliseconds, produced by the
	// auto-detector (see refSpeedHistory / targetSpeedHistory below). Persisted
	// across overlay restarts so the auto-detect value is restored when the
	// user re-enables latencyAutoDetect.
	double estimatedLatencyOffsetMs = 0.0;

	// Ring buffers of recent reference / target device linear-speed magnitudes,
	// timestamped with the glfw time at the moment CollectSample pushed them. The
	// auto-detector cross-correlates these once per second to estimate the time
	// shift that maximises signal alignment; that lag is converted to ms and
	// fed into estimatedLatencyOffsetMs through an EMA. Capacity targets ~5 s
	// of history at the calibrator's natural sample rate. The buffers are
	// trimmed in CollectSample when they exceed kLatencyHistoryCapacity below.
	static const size_t kLatencyHistoryCapacity = 100;
	std::deque<double> refSpeedHistory;
	std::deque<double> targetSpeedHistory;
	std::deque<double> speedSampleTimes;
	double timeLastLatencyEstimate = 0.0;

	// Opt-in switch for the GCC-PHAT latency estimator (Knapp & Carter 1976)
	// alongside the original time-domain cross-correlator. Default OFF: the
	// 2026-05-04 math review pinned the existing time-domain CC as
	// "empirically validated, not a sore point", so we keep it as the
	// default and make GCC-PHAT a logged-side-by-side alternative until
	// real-session evidence shows the whitened-spectrum estimate is
	// preferable. Both algorithms are pure helpers in
	// src/overlay/LatencyEstimator.h. Persisted via Configuration.cpp.
	bool useGccPhatLatency = false;

	// Opt-in switch for the CUSUM geometry-shift detector (Page 1954)
	// alternative to the default 5x-rolling-median rule. Both share the
	// same "fire -> Clear() + demote to Standby" recovery action; only the
	// per-tick decision differs. CUSUM gives a tunable false-alarm rate
	// via standard ARL tables (kCusumDriftMm, kCusumThreshold in
	// GeometryShiftDetector.h). Default OFF; the rolling-median rule has
	// not misfired in observed 35.6 MB of session logs. Persisted as
	// geometry_shift_use_cusum in profile JSON.
	bool useCusumGeometryShift = false;

	// Opt-in switch for velocity-aware outlier weighting in the IRLS
	// translation solve. When on, the per-pair Cauchy threshold c is
	// scaled DOWN with motion magnitude: c_pair = c0 / (1 + kappa *
	// v_pair / v_ref) where v_pair = max(refSpeed, targetSpeed) across
	// the pair. Stationary pairs keep c0 (high-residual stays informative
	// — "the cal is wrong here"); fast-motion pairs get a sharper cutoff
	// that suppresses high-residual rows as glitches. Default OFF; only
	// worth turning on if motion-glitch failure modes are observed
	// (current logs show axis_variance_low dominating, which this does
	// not address). Persisted as irls_velocity_aware in profile JSON.
	bool useVelocityAwareWeighting = false;

	// Opt-in switch for the Tukey biweight + Qn-scale path in the IRLS
	// translation solve. Replaces the default Cauchy + MAD pair with a
	// redescending kernel (large residuals get exactly zero weight) and
	// a 50% breakdown scale estimator (no symmetry assumption, no MAD
	// floor saturation). Default OFF; the Cauchy + MAD path has been
	// adequate on observed Quest+Lighthouse residual distributions.
	// Persisted as irls_use_tukey in profile JSON.
	bool useTukeyBiweight = false;

	// Opt-in switch for the Kalman-filter blend at publish (replaces the
	// single-step EMA at alpha=0.3 in CalibrationCalc::ComputeIncremental
	// with a 4-state filter on yaw + translation, with proper process and
	// measurement covariances). Default OFF; the EMA path is the
	// validated default. Filter divergence is detected per-tick via hard
	// caps on per-component innovation; on divergence the filter resets
	// to the candidate and the EMA path runs that tick as a graceful
	// fallback. Persisted as blend_use_kalman in profile JSON.
	bool useBlendFilter = false;

	// Opt-in switch for the rest-locked yaw drift correction. Per-tracker
	// rest detector locks the orientation 1 s of dwell; subsequent at-rest
	// ticks compare current vs locked rotation and apply a bounded-rate
	// yaw nudge (per-class cap, global ceiling) to the active SE(3).
	// Activates only outside Continuous mode (continuous-cal already
	// handles drift in its own loop). Default OFF; flips to ON only after
	// a real-session test against the four-tier success criterion. The
	// math lives in src/overlay/RestLockedYaw.h. Persisted as
	// rest_locked_yaw in profile JSON.
	bool restLockedYawEnabled = false;

	// Rolling window of per-solve residual pitch+roll readings (degrees), used
	// by spacecal::gravity::EvaluateTilt to flag sustained gravity-axis
	// disagreement between the reference and target tracking systems. Pushed
	// each ComputeIncremental tick that produces a candidate; trimmed to the
	// last kSustainedWindowSeconds of history. Logging-only diagnostic:
	// surfaces "your two systems disagree about which way is down by X
	// degrees" as a sustained signal so the user can correct (e.g. re-run
	// room setup) -- the calibration math itself is unchanged.
	std::deque<spacecal::gravity::TiltSample> tiltDiagnosticWindow;
	bool tiltSustainedAlarmed = false;
	double tiltLastAnnotatedMedian = -1.0;

	protocol::AlignmentSpeedParams alignmentSpeedParams;
	bool enableStaticRecalibration;

	// "Lock relative position" -- freezes the relative pose between the
	// reference and target devices once it has been calibrated. When locked,
	// continuous calibration only updates the world anchor frame, not the
	// relationship between the two trackers themselves.  Useful when the
	// target is rigidly attached to the reference (a tracker glued to your
	// HMD, taped to a controller, etc).
	//
	// Tristate:
	//   OFF  -- never lock; continuous calibration is free to re-solve the
	//           relative pose on every cycle.
	//   ON   -- always lock once a relative pose has been recorded.
	//   AUTO -- (default) detect rigid attachment from observed relative-pose
	//           variance.  Starts unlocked; flips to "effectively locked"
	//           once the relative pose has stayed stable (~5mm translation,
	//           ~1deg rotation) for a sustained window of accepted samples.
	//           Flips back to unlocked if the variance climbs again, e.g.
	//           the user repositioned the tracker.
	enum class LockMode : int { OFF = 0, ON = 1, AUTO = 2 };
	LockMode lockRelativePositionMode = LockMode::AUTO;

	// Resolved/effective lock state -- recomputed each tick from
	// lockRelativePositionMode + the auto-lock detector's verdict.  Existing
	// math code reads this field, so the resolver layer keeps the math
	// untouched while the user-facing knob becomes a tristate.
	bool lockRelativePosition = false;

	// Auto-lock detector state.  Tracks the most recent relative-pose samples
	// (ref^-1 * target) on a sliding window.  When their variance stays below
	// the rigidity thresholds for kAutoLockSamplesNeeded consecutive accepted
	// samples, autoLockEffectivelyLocked flips true.  Cleared on profile
	// reload / Clear() so a fresh calibration starts unlocked.
	std::deque<Eigen::AffineCompact3d> autoLockHistory;
	bool autoLockEffectivelyLocked = false;

	// Multi-ecosystem extras: each entry aligns an additional non-HMD tracking
	// system to the HMD's tracking system. Empty for the typical 1-or-2-system
	// case. The wizard appends entries here as it walks the user through each
	// detected non-HMD system. In continuous mode every entry's calibration
	// runs independently in parallel with the primary.
	std::vector<AdditionalCalibration> additionalCalibrations;

	// Wizard state. Persisted as a flag in the profile so we only auto-show
	// the wizard the first time. The user can re-launch from a button in
	// Advanced.
	bool wizardCompleted = false;

	// Push a fresh ref+target world pose into the auto-lock detector.
	// Computes the relative pose (ref^-1 * target), appends it to the
	// history, and re-evaluates rigidity.  Called from the calibration tick
	// each time a new sample is collected.
	void UpdateAutoLockDetector(const Eigen::AffineCompact3d& refWorld,
	                            const Eigen::AffineCompact3d& targetWorld);

	// Resolve `lockRelativePosition` from `lockRelativePositionMode` + the
	// auto-lock detector.  Cheap; safe to call every tick.  Math code reads
	// `lockRelativePosition`, not the mode -- keeping the math layer ignorant
	// of the tristate.
	void ResolveLockMode();

	// "Recalibrate on movement" — gates the driver-side BlendTransform's lerp
	// progress on detected per-frame motion magnitude. With this on, a user who
	// is lying still won't see calibration drift even when the math is updating;
	// the catch-up happens during their next motion, hidden by the natural
	// movement instead of looking like phantom body shifts. Default ON because
	// the failure mode it prevents (visible drift while motionless) is more
	// common in practice than the rare case where you actually want instant
	// updates while stationary.
	bool recalibrateOnMovement = true;

	// UI-only flag toggled by the "Pause updates" button on the Status tab.
	// While true the overlay-side calibration tick is expected to skip the
	// ComputeIncremental call so the current driver-applied offset stays put
	// — useful when something looks momentarily wrong and the user wants to
	// freeze the live view to investigate rather than have it self-correct
	// out from under them. Default false (live updates).
	bool calibrationPaused = false;
	// Status-tab UI state: collapses the busier sliders into an "Advanced
	// settings" section. Persisting this is intentional — a user who opened
	// it once probably wants it open next session too.
	bool showAdvancedSettings = false;

	// Native prediction-suppression (see wiki/Prediction-Suppression). Scales
	// velocity/acceleration on per-device pose updates inside our SteamVR
	// driver, which lets the user trade smoothness for raw responsiveness.
	// Per-tracker because not every device wants the same setting -- e.g. a
	// hip tracker that's barely moving wants more smoothing than a wrist
	// tracker that's swinging fast.
	//
	// trackerSmoothness: serial number -> 0..100 strength. Anything not in
	// the map (or with value 0) gets no suppression. The HMD and the active
	// calibration reference + target trackers are hard-blocked at the UI and
	// at ScanAndApplyProfile -- their entries are forced to 0 even if the
	// map contains a non-zero value, because suppressing them corrupts either
	// the user's view or the calibration math.
	std::unordered_map<std::string, int> trackerSmoothness;

	// Finger smoothing for Index Knuckles. Driver hooks the per-frame bone
	// arrays via IVRDriverInputInternal::UpdateSkeletonComponent and slerps
	// each bone toward the incoming pose with the configured strength.
	// Default-OFF so a user who only cares about calibration sees zero
	// behaviour change. Settings persist across profile clears (they're a
	// preference, not calibration data tied to a tracker serial).
	//
	// fingerSmoothingEnabled: master kill-switch (false = passthrough).
	// fingerSmoothingStrength: 0..100. 0 = no smoothing; 100 = max
	//   (slerp factor 0.05, never fully freezes). Linear in between.
	// fingerSmoothingMask: per-finger enable bits, see protocol::kAllFingersMask.
	//   Defaults to all 10 fingers enabled.
	bool     fingerSmoothingEnabled  = false;
	int      fingerSmoothingStrength = 50;
	uint16_t fingerSmoothingMask     = 0x03FF;

	// Set by the periodic external-tool detector. Read by the UI (warning
	// banner). Third-party tools fight our suppression; we don't try to
	// interop -- we just tell the user to stop the external tool.
	bool externalSmoothingDetected = false;
	// Name of the detected external tool (for the warning text). Empty when
	// nothing is detected.
	std::string externalSmoothingToolName;
	// Time-of-last-detector-run, in glfw seconds. CalibrationTick re-runs the
	// scan every ~5 seconds to keep detection responsive without burning CPU
	// on a tight process-enumeration loop.
	double timeLastSmoothingScan = 0;

	Eigen::AffineCompact3d refToTargetPose = Eigen::AffineCompact3d::Identity();
	bool relativePosCalibrated = false;

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2,
		// Picks one of the above each tick based on observed reference+target jitter.
		// Lets a casual user not have to think about it: the program watches its own
		// noise floor and slows the calibration down when conditions are bad.
		AUTO = 3,
	};
	Speed calibrationSpeed = AUTO;

	vr::DriverPose_t devicePoses[vr::k_unMaxTrackedDeviceCount];

	// Per-device shmem-side QPC timestamps captured alongside the most recent pose.
	// Populated by CalibrationTick when ingesting AugmentedPose entries from the
	// driver shared-memory ring; consumed by CollectSample to compute the inter-system
	// time delta used for velocity extrapolation when targetLatencyOffsetMs != 0.
	LARGE_INTEGER devicePoseSampleTimes[vr::k_unMaxTrackedDeviceCount];

	CalibrationContext() {
		calibratedScale = 1.0;
		memset(devicePoses, 0, sizeof(devicePoses));
		memset(devicePoseSampleTimes, 0, sizeof(devicePoseSampleTimes));
		ResetConfig();
	}

	void ResetConfig() {
		alignmentSpeedParams.thr_rot_tiny = 0.49f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_small = 0.5f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_large = 5.0f * (EIGEN_PI / 180.0f);

		alignmentSpeedParams.thr_trans_tiny = 0.98f / 1000.0; // mm
		alignmentSpeedParams.thr_trans_small = 1.0f / 1000.0; // mm
		alignmentSpeedParams.thr_trans_large = 20.0f / 1000.0; // mm

		alignmentSpeedParams.align_speed_tiny = 1.0f;
		alignmentSpeedParams.align_speed_small = 1.0f;
		alignmentSpeedParams.align_speed_large = 2.0f;

		continuousCalibrationThreshold = 1.5f;
		maxRelativeErrorThreshold = 0.005f;
		jitterThreshold = 3.0f;

		continuousCalibrationOffset = Eigen::Vector3d::Zero();

		// Static recalibration: when Lock relative position has identified a
		// rigid attachment (Lock=ON or Lock=AUTO and the auto-detector fired),
		// snap to the locked relative pose if the live solver diverges from it.
		// No-op for independent devices (no locked relative pose to snap to),
		// so leaving it on by default is safe and accelerates recovery from
		// brief tracking glitches on rigid setups. The user can still flip it
		// off in Advanced if they want pure incremental behaviour.
		enableStaticRecalibration = true;

		// Default AUTO. Failure mode is benign: with no base stations
		// detected, the detector simply doesn't fire.
		baseStationDriftCorrectionEnabled = true;
	}

	struct Chaperone
	{
		bool valid = false;
		bool autoApply = true;
		std::vector<vr::HmdQuad_t> geometry;
		vr::HmdMatrix34_t standingCenter = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
		};
		vr::HmdVector2_t playSpaceSize = { 0.0f, 0.0f };
	} chaperone;

	void ClearLogOnMessage() {
		clearOnLog = true;
	}

	void Clear()
	{
		chaperone.geometry.clear();
		chaperone.standingCenter = vr::HmdMatrix34_t();
		chaperone.playSpaceSize = vr::HmdVector2_t();
		chaperone.valid = false;

		calibratedRotation = Eigen::Vector3d();
		calibratedTranslation = Eigen::Vector3d();
		calibratedScale = 1.0;
		referenceTrackingSystem = "";
		targetTrackingSystem = "";
		enabled = false;
		validProfile = false;
		refToTargetPose = Eigen::AffineCompact3d::Identity();

		// Per-profile fields added by recent passes. Without these resets
		// the user's stale latency/suppression settings would carry over after a
		// profile clear and silently apply to the next calibration session.
		trackerSmoothness.clear();
		targetLatencyOffsetMs = 0.0;
		latencyAutoDetect = false;
		estimatedLatencyOffsetMs = 0.0;
		refSpeedHistory.clear();
		targetSpeedHistory.clear();
		speedSampleTimes.clear();
		timeLastLatencyEstimate = 0.0;
		// Runtime UI state — pausing on an empty profile makes no sense.
		calibrationPaused = false;
		// Default this back ON when clearing — it's the safer setting and
		// matches the construction-time default.
		recalibrateOnMovement = true;
		// Auto-lock detector resets to "no observations yet". The user's lock
		// preference (mode) is intentionally NOT reset -- it's a setting, not
		// calibration data, and a user who deliberately set ON or OFF wants
		// that to persist across profile clears.
		lockRelativePosition = false;
		autoLockHistory.clear();
		autoLockEffectivelyLocked = false;
		// Note: showAdvancedSettings and the externalSmoothing* runtime-detection
		// fields are intentionally NOT reset -- they're user preferences and
		// detector state that span profiles.
		// No calibration was performed — relative pose is NOT calibrated. The
		// previous value here was `true`, which left a stale-identity-matrix
		// believed-good and caused StartContinuousCalibration to pass `true` to
		// setRelativeTransformation downstream.
		relativePosCalibrated = false;
		// Continuous-mode runtime offset. ResetConfig() at construction zeroes
		// this; without resetting on Clear() too, a leftover offset from a
		// previous session biased every reference pose in the next continuous
		// calibration ("everything looks consistently 5–10mm off and won't
		// converge" symptom). The offset is a runtime adjustment, not a
		// persistent profile setting — it has no business surviving a Clear().
		continuousCalibrationOffset = Eigen::Vector3d::Zero();
	}

	// Resolve the user's selected speed to a concrete FAST/SLOW/VERY_SLOW. When
	// the user picks AUTO, we look at the recent observed jitter on both
	// reference and target trackers and pick the buffer size that matches:
	//   - clean trackers (sub-mm jitter) get FAST so calibration converges quickly
	//   - typical setups (1-5mm) get SLOW
	//   - noisy / reflective rooms / drifty IMU (>5mm) get VERY_SLOW
	// This is sticky-by-default: we only re-evaluate every few seconds and
	// require the new bucket to have been right for a while before switching,
	// so the buffer size doesn't oscillate during transient noise.
	Speed ResolvedCalibrationSpeed() const;

	size_t SampleCount()
	{
		switch (ResolvedCalibrationSpeed())
		{
		case FAST:
			return 100;
		case SLOW:
			return 250;
		case VERY_SLOW:
			return 500;
		default:
			return 100;
		}
	}

	struct Message
	{
		enum Type
		{
			String,
			Progress
		} type = String;

		Message(Type type) : type(type), progress(0), target(0) { }

		std::string str;
		int progress, target;
	};

	std::deque<Message> messages;

	void Log(const std::string &msg)
	{
		if (clearOnLog) {
			messages.clear();
			clearOnLog = false;
		}

		if (messages.empty() || messages.back().type == Message::Progress)
			messages.push_back(Message(Message::String));

		OutputDebugStringA(msg.c_str());

		messages.back().str += msg;
		std::cerr << msg;

		while (messages.size() > 15) messages.pop_front();
	}

	void Progress(int current, int target)
	{
		if (messages.empty() || messages.back().type == Message::String)
			messages.push_back(Message(Message::Progress));

		messages.back().progress = current;
		messages.back().target = target;
	}

	bool TargetPoseIsValidSimple() const {
		return targetID >= 0 && targetID < (int32_t)vr::k_unMaxTrackedDeviceCount
			&& devicePoses[targetID].poseIsValid && devicePoses[targetID].result == vr::ETrackingResult::TrackingResult_Running_OK;
	}

	bool ReferencePoseIsValidSimple() const {
		return referenceID >= 0 && referenceID < (int32_t)vr::k_unMaxTrackedDeviceCount
			&& devicePoses[referenceID].poseIsValid && devicePoses[referenceID].result == vr::ETrackingResult::TrackingResult_Running_OK;
	}
};

extern CalibrationContext CalCtx;

void InitCalibrator();
void CalibrationTick(double time);
void StartCalibration();
void StartContinuousCalibration();
void EndContinuousCalibration();
void LoadChaperoneBounds();
void ApplyChaperoneBounds();

void PushCalibrationApplyTime();
void ShowCalibrationDebug(int r, int c);
void DebugApplyRandomOffset();

// Accessor for the session-counter of stuck-loop watchdog firings. The
// underlying CalibrationCalc instance lives in an anonymous namespace inside
// Calibration.cpp, so we expose this via a free function.
int GetWatchdogResetCount();

// Most recent HMD-relocalization detection event. Returns true if any event
// has been logged this session and populates out parameters with the time
// since the event (seconds), the translation magnitude (meters), and the
// rotation magnitude (degrees). Returns false if the detector hasn't fired
// at all this session.
bool LastDetectedRelocalization(double& outAgeSeconds, double& outDeltaMeters,
                                double& outDeltaDegrees);

// Recent auto-recovery info: returns true if auto-recovery clobbered the
// calibration in the last 60 seconds AND the user hasn't dismissed the
// banner. Populates outAge (seconds since recovery fired) and outDeltaMeters
// (the HMD jump magnitude that triggered the recovery). Used by the UI to
// render a sticky banner with Undo + Dismiss buttons.
bool LastAutoRecoveryActive(double& outAge, double& outDeltaMeters);

// Restore the pre-recovery refToTargetPose / relativePosCalibrated /
// hasAppliedCalibrationResult, taking the user back to the calibration
// state that was in effect before the auto-recovery cleared it. Returns
// true if the snapshot existed and was restored, false if no recovery
// has happened yet or undo was already applied. Idempotent: a second
// click is a no-op.
bool UndoLastAutoRecovery();

// Hide the recovery banner without undoing. The recovered calibration
// continues; only the UI banner disappears.
void DismissAutoRecoveryBanner();

// Manual playspace recenter: shift the standing zero pose so the user's
// current HMD position becomes the chaperone center. X and Z translate;
// Y (floor) and rotation are preserved. Used by the "Recenter playspace"
// UI button. Returns true on success.
bool RecenterPlayspaceToCurrentHmd();

// Re-open the driver pose shared-memory segment. The IPC client invokes this
// after a successful reconnect to vrserver: when vrserver crashes and respawns,
// the named-mapping the overlay had open is destroyed, the mapped view detaches
// silently, and ReadNewPoses() begins yielding zeros. Re-opening picks up the
// new mapping the freshly-respawned driver creates.
void ReopenShmem();

// Returns the latency offset (in ms) that should currently be applied to
// reference-pose extrapolation. When ctx.latencyAutoDetect is true, this is
// the auto-detected EMA value (estimatedLatencyOffsetMs); otherwise it is the
// user-supplied manual value (targetLatencyOffsetMs). Centralising the choice
// here keeps the manual/auto switch a single read.
double GetActiveLatencyOffsetMs(const CalibrationContext& ctx);