#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Windows.h>
#include <openvr.h>
#include <vector>
#include <deque>
#include <set>
#include <string>

#include "Protocol.h"

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

struct StandbyDevice {
	std::string trackingSystem;
	std::string model, serial;
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

	protocol::AlignmentSpeedParams alignmentSpeedParams;
	bool enableStaticRecalibration;
	bool lockRelativePosition = false;

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

	// Native prediction-suppression (see wiki/Prediction-Suppression). Replaces
	// external tools like OVR-SmoothTracking by zeroing velocity/acceleration on
	// per-device pose updates inside our SteamVR driver, which is the same trick
	// those tools use. Driver-side per-slot freezePrediction is the actual control
	// surface; these overlay-side fields are the user-visible knobs that drive it.
	//
	// suppressedSerials: serial numbers of devices the user has explicitly opted
	// in for. ScanAndApplyProfile sends freezePrediction=true to any device whose
	// Prop_SerialNumber_String is in this set.
	std::set<std::string> suppressedSerials;
	// When true and an external smoothing tool is detected running, automatically
	// suppress prediction on the calibration reference + target trackers (they're
	// the ones whose pose the math reads, so they matter most). Default on so a
	// user who installs OVR-SmoothTracking and forgets gets clean math by default;
	// the in-app warning tells them what we did.
	bool autoSuppressOnExternalTool = true;
	// Set by the periodic external-tool detector. Read by the UI (status banner)
	// and by ScanAndApplyProfile (gates the auto-suppress behaviour above).
	bool externalSmoothingDetected = false;
	// Name of the detected external tool (for the warning text). Empty when nothing
	// is detected.
	std::string externalSmoothingToolName;
	// Time-of-last-detector-run, in glfw seconds. CalibrationTick re-runs the scan
	// every ~5 seconds to keep detection responsive without burning CPU on a tight
	// process-enumeration loop.
	double timeLastSmoothingScan = 0;

	Eigen::AffineCompact3d refToTargetPose = Eigen::AffineCompact3d::Identity();
	bool relativePosCalibrated = false;

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2
	};
	Speed calibrationSpeed = FAST;

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

		enableStaticRecalibration = false;
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

		// Per-profile fields added by recent agent passes. Without these resets
		// the user's stale latency/suppression settings would carry over after a
		// profile clear and silently apply to the next calibration session.
		suppressedSerials.clear();
		targetLatencyOffsetMs = 0.0;
		latencyAutoDetect = false;
		estimatedLatencyOffsetMs = 0.0;
		refSpeedHistory.clear();
		targetSpeedHistory.clear();
		speedSampleTimes.clear();
		timeLastLatencyEstimate = 0.0;
		// Runtime UI state — pausing on an empty profile makes no sense.
		calibrationPaused = false;
		// Note: autoSuppressOnExternalTool, showAdvancedSettings, and the
		// externalSmoothing* runtime-detection fields are intentionally NOT reset
		// — they're user preferences / detector state that span profiles.
		// No calibration was performed — relative pose is NOT calibrated. The
		// previous value here was `true`, which left a stale-identity-matrix
		// believed-good and caused StartContinuousCalibration to pass `true` to
		// setRelativeTransformation downstream.
		relativePosCalibrated = false;
	}

	size_t SampleCount()
	{
		switch (calibrationSpeed)
		{
		case FAST:
			return 100;
		case SLOW:
			return 250;
		case VERY_SLOW:
			return 500;
		}
		return 100;
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