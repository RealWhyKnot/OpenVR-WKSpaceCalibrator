#pragma once

#include <Eigen/Dense>
#include <openvr.h>
#include <string>
#include <vector>
#include <deque>
#include <iostream>

#include "BlendFilter.h"  // spacecal::blendfilter::State (member of CalibrationCalc)

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() { }
	Pose(const Eigen::AffineCompact3d& transform) {
		rot = transform.rotation();
		trans = transform.translation();
	}
	
	Pose(vr::HmdMatrix34_t hmdMatrix)
	{
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rot(i, j) = hmdMatrix.m[i][j];
			}
		}
		trans = Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]);
	}
	Pose(vr::HmdQuaternion_t rot, const double *trans) {
		this->rot = Eigen::Matrix3d(Eigen::Quaterniond(rot.w, rot.x, rot.y, rot.z));
		this->trans = Eigen::Vector3d(trans[0], trans[1], trans[2]);
	}
	Pose(double x, double y, double z) : trans(Eigen::Vector3d(x, y, z)) { }

	Eigen::Matrix4d ToAffine() const {
		Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				matrix(i, j) = rot(i, j);
			}
			matrix(i, 3) = trans(i);
		}

		return matrix;
	}
};

struct Sample
{
	Pose ref, target;
	bool valid;
	double timestamp;
	// Linear-speed magnitudes (m/s) of the reference and target devices at
	// sample-collection time. Sourced from vr::DriverPose_t::vecVelocity in
	// the production CollectSample path; left at zero for replay-harness and
	// test paths that don't have velocity data. Used by the velocity-aware
	// outlier weighting (default-off experimental flag) to scale the per-row
	// IRLS Cauchy threshold so high-residual samples taken during fast motion
	// can be suppressed as glitches while stationary high-residual samples
	// stay informative.
	double refSpeed = 0.0;
	double targetSpeed = 0.0;
	Sample() : valid(false), timestamp(0) { }
	Sample(Pose ref, Pose target, double timestamp) : valid(true), ref(ref), target(target), timestamp(timestamp){ }
	Sample(Pose ref, Pose target, double timestamp, double refSpeed, double targetSpeed)
		: valid(true), ref(ref), target(target), timestamp(timestamp),
		  refSpeed(refSpeed), targetSpeed(targetSpeed) { }
};

class CalibrationCalc {
public:
	static const double AxisVarianceThreshold;

	bool enableStaticRecalibration;
	bool lockRelativePosition = false;
	// Opt-in: when true, the per-pair IRLS Cauchy threshold scales with the
	// pair's motion magnitude so stationary high-residual rows stay
	// informative while fast-motion high-residual rows are aggressively
	// suppressed as glitches. Set by the caller per-tick from
	// CalCtx.useVelocityAwareWeighting; default-off path is unchanged.
	bool useVelocityAwareWeighting = false;
	// Opt-in: when true, the IRLS robust kernel switches from Cauchy + MAD
	// to Tukey biweight + Qn-scale. Tukey is redescending (large residuals
	// -> exactly zero weight); Qn replaces MAD with a no-symmetry,
	// no-saturation 50%-breakdown estimator. Set by the caller per-tick
	// from CalCtx.useTukeyBiweight; default-off path is unchanged.
	bool useTukeyBiweight = false;
	// Opt-in: when true, the publish-time blend uses a Kalman filter on
	// (yaw, tx, ty, tz) instead of the single-step EMA at alpha=0.3.
	// Filter state is stored on CalibrationCalc (m_blendFilter) so it
	// persists across ticks. Set by the caller per-tick from
	// CalCtx.useBlendFilter; default-off path is unchanged.
	bool useBlendFilter = false;
	
	const Eigen::AffineCompact3d Transformation() const 
	{
		return m_estimatedTransformation;
	}

	const Eigen::Vector3d EulerRotation() const {
		auto rot = m_estimatedTransformation.rotation();
		return rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
	}

	bool isValid() const {
		return m_isValid;
	}
	
	const Eigen::AffineCompact3d RelativeTransformation() const 
	{
		return m_refToTargetPose;
	}

	bool isRelativeTransformationCalibrated() const
	{
		return m_relativePosCalibrated;
	}

	void setRelativeTransformation(const Eigen::AffineCompact3d transform, bool calibrated)
	{
		m_refToTargetPose = transform;
		m_relativePosCalibrated = calibrated;
	}

	void PushSample(const Sample& sample);
	void Clear();

	double ReferenceJitter() const;
	double TargetJitter() const;

	// Motion-coverage metrics on the current sample buffer. Used during one-
	// shot calibration to give the user real-time feedback on whether their
	// figure-8 motion has produced enough variety to fit a good calibration.
	// Each returns a 0.0..1.0 score where 1.0 means "you've covered enough
	// motion that the math will produce a clean fit; you can stop now".
	//
	// TranslationDiversity is the smallest per-axis position range divided by
	// a target range (~30cm). Penalises planar / single-axis motion -- you
	// want all three axes to have meaningful spread, not just the one the
	// user is comfortable swinging.
	//
	// RotationDiversity is the maximum angular distance between any two
	// sampled target rotations, divided by a target angle (~90 deg). Even
	// one wide rotation pair is enough to constrain yaw cleanly; we don't
	// need full hemispheric coverage like translation.
	double TranslationDiversity() const;
	double RotationDiversity() const;

	// Per-axis (X, Y, Z) bounding-box ranges of the target tracker across the
	// live sample buffer, in centimetres. UI uses this to point the user at
	// the limiting axis when TranslationDiversity() is below 1.0; whichever
	// component is smallest is the bottleneck (the diversity score is
	// component-min / 30 cm). Returns zero vector when fewer than two valid
	// samples are available.
	Eigen::Vector3d TranslationAxisRangesCm() const;

	bool ComputeOneshot(const bool ignoreOutliers);
	bool ComputeIncremental(bool &lerp, double threshold, double relPoseMaxError, const bool ignoreOutliers);

	size_t SampleCount() const {
		return m_samples.size();
	}

	void ShiftSample() {
		if (!m_samples.empty()) m_samples.pop_front();
	}

	// Two-phase one-shot calibration support. The Calibration.cpp state machine
	// runs Begin → Rotation → Translation; at the Rotation→Translation
	// transition we move the rotation-phase samples into m_rotationFrozen so
	// the live buffer can refill with translation-phase samples without aging
	// the rotation samples out. ComputeOneshot then prepends the frozen samples
	// onto m_samples for the duration of the solve, so all internal helpers
	// (DetectOutliers, CalibrateRotation, ComputeAxisVariance, etc.) see the
	// unified buffer without modification. Cleared by Clear().
	void FreezeRotationPhaseSamples();
	bool HasFrozenRotationSamples() const { return !m_rotationFrozen.empty(); }
	size_t FrozenRotationSampleCount() const { return m_rotationFrozen.size(); }

	CalibrationCalc() : m_isValid(false), m_calcCycle(0), enableStaticRecalibration(true) {}

	// Debug fields
	Eigen::Vector3d m_posOffset;
	double m_axisVariance = 0.0;
	long m_calcCycle;

	// Smallest/largest singular-value ratio of the 2D Kabsch cross-covariance from the
	// most recent CalibrateRotation. Near-zero means the user only rotated in one
	// axis (degenerate motion), so the yaw solution is ill-conditioned. Set by
	// CalibrateRotation, consulted by ComputeIncremental to reject bad solutions.
	mutable double m_rotationConditionRatio = 0.0;

	// Diagnostics: number of consecutive ComputeIncremental rejections, and the last
	// time we successfully collected a sample. Used by the stuck-loop watchdog to
	// drop m_isValid (and trigger ContinuousStandby) when continuous calibration can
	// no longer produce a better estimate but isn't admitting it.
	int m_consecutiveRejections = 0;
	double m_lastSuccessfulIncrementalTime = 0.0;
	double m_lastSampleTime = 0.0;
	int m_watchdogResets = 0;

	// Short tag describing why the most recent ComputeIncremental rejected (or
	// "" if the last call accepted). Surfaced in the debug log so a stuck-loop
	// row can be traced back to the gate that tripped: "below_floor_or_worse",
	// "axis_variance_low", "rotation_planar", "translation_planar",
	// "rms_above_gate", "healthy_below_floor", "validate_failed".
	// Distinct from the older enum-typed `m_lastRejectReason` further down,
	// which tracks ValidateCalibration's RMS-gate decisions specifically.
	std::string m_rejectReasonTag;

	// Latch that prevents the "watchdog_skipped: ... healthy" annotation from
	// repeating on every tick once the healthy-skip path has fired. Cleared on
	// the next successful accept so a subsequent fresh stuck-run still gets one
	// annotation. Strictly diagnostic; doesn't affect math.
	bool m_healthyHoldAnnotated = false;

private:
	bool m_isValid;
	Eigen::AffineCompact3d m_estimatedTransformation;
	bool m_relativePosCalibrated = false;

	// Pre-allocated scratch matrices reused across solver invocations. With
	// continuous calibration ticking at ~2 Hz against a 200-sample buffer, the
	// per-call heap churn from local Eigen::MatrixXd / Eigen::VectorXd
	// allocations was on the order of ~100 KB per tick. These members are
	// resized in place; resize() is a no-op when the requested dimensions
	// already match, so the steady-state cost is just the solve itself.
	mutable Eigen::MatrixXd m_coefficientsTrans;
	mutable Eigen::VectorXd m_constantsTrans;
	mutable Eigen::VectorXd m_weightsTrans;
	mutable Eigen::MatrixXd m_outlierCoefficients;
	mutable Eigen::VectorXd m_outlierConstraints;

	// Smallest/largest singular-value ratio of the translation LS coefficient
	// matrix from the most recent CalibrateTranslation. Mirrors
	// m_rotationConditionRatio: near-zero means the user moved through too few
	// independent directions to constrain the translation, so the result is
	// dominated by noise. Set by CalibrateTranslation, consulted by
	// ComputeIncremental to reject ill-conditioned solves.
public:
	mutable double m_translationConditionRatio = 0.0;

	// Residual pitch+roll (in degrees) of the most recent SO(3) Kabsch fit. If
	// this is large (~> 2 deg), the reference and target spaces' gravity axes
	// don't agree, which the yaw-only solver cannot represent — we log a hint
	// for the user but don't reject the solution.
	mutable double m_residualPitchRollDeg = 0.0;

	// SO(3) Kabsch result + validity, computed in DetectOutliers and reused by
	// CalibrateRotation for the yaw projection (item #3 from the math review).
	// Without this DetectOutliers and CalibrateRotation each ran their own
	// Kabsch SVD, with CalibrateRotation throwing away the Y axis before the
	// SVD step — that 2D simplification leaks any pitch/roll discrepancy into
	// the yaw answer. The shared 3D fit projected to yaw is the principled
	// answer.
	mutable Eigen::Matrix3d m_so3KabschResult = Eigen::Matrix3d::Identity();
	mutable bool m_so3KabschValid = false;

	// Diagnostics: which gate caused the most recent ValidateCalibration to
	// reject. Set by ValidateCalibration and consulted by ComputeOneshot for
	// branching the user-facing log line.
	enum class RejectReason {
		None,
		RmsTooHigh,
		// (other gates live further out; ValidateCalibration only checks RMS today)
	};
	mutable RejectReason m_lastRejectReason = RejectReason::None;
	mutable double m_lastRejectRms = 0.0;
	mutable double m_lastRejectRmsThreshold = 0.0;

private:

	/*
	 * This affine transform estimates the pose of the target within the reference device's local pose space.
	 * That is to say, it's given by transforming the target world pose by the inverse reference pose.
	 */
	Eigen::AffineCompact3d m_refToTargetPose = Eigen::AffineCompact3d::Identity();

	// Kalman-filter state for the opt-in blend path. Persists across
	// ComputeIncremental calls; reset by Clear() and on detected divergence.
	// Accessed only when useBlendFilter is true; otherwise it sits idle.
	spacecal::blendfilter::State m_blendFilter;
	double m_blendFilterLastUpdateTime = 0.0;

	std::deque<Sample> m_samples;

	// Frozen rotation-phase samples (see FreezeRotationPhaseSamples comment in
	// the public section). Empty during continuous calibration and during the
	// one-shot Rotation phase; populated only between the Rotation→Translation
	// transition and the final ComputeOneshot. Cleared by Clear().
	std::deque<Sample> m_rotationFrozen;

	std::vector<bool> DetectOutliers() const;
	Eigen::Vector3d CalibrateRotation(const bool ignoreOutliers) const;
	Eigen::Vector3d CalibrateTranslation(const Eigen::Matrix3d &rotation) const;
	Eigen::Vector3d CalibrateTranslationLegacyPairwise(const Eigen::Matrix3d &rotation) const;

	Eigen::AffineCompact3d ComputeCalibration(const bool ignoreOutliers) const;

	double RetargetingErrorRMS(const Eigen::Vector3d& hmdToTargetPos, const Eigen::AffineCompact3d& calibration) const;
	Eigen::Vector3d ComputeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const;

	Eigen::Vector4d ComputeAxisVariance(const Eigen::AffineCompact3d& calibration) const;

	void ComputeInstantOffset();

	Eigen::AffineCompact3d EstimateRefToTargetPose(const Eigen::AffineCompact3d& calibration) const;
	bool CalibrateByRelPose(Eigen::AffineCompact3d &out) const;

public:
	// Hoisted to public so the in-app replay panel + the standalone replay
	// CLI can score arbitrary candidate transforms against the current sample
	// buffer without needing to call the private ComputeIncremental flow.
	[[nodiscard]] bool ValidateCalibration(const Eigen::AffineCompact3d& calibration, double *errorOut = nullptr, Eigen::Vector3d* posOffsetV = nullptr);
};