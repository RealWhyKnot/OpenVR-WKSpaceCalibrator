#pragma once

#include <Eigen/Dense>
#include <openvr.h>
#include <vector>
#include <deque>
#include <iostream>

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
	Sample() : valid(false), timestamp(0) { }
	Sample(Pose ref, Pose target, double timestamp) : valid(true), ref(ref), target(target), timestamp(timestamp){ }
};

class CalibrationCalc {
public:
	static const double AxisVarianceThreshold;

	bool enableStaticRecalibration;
	bool lockRelativePosition = false;
	
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

	bool ComputeOneshot(const bool ignoreOutliers);
	bool ComputeIncremental(bool &lerp, double threshold, double relPoseMaxError, const bool ignoreOutliers);

	size_t SampleCount() const {
		return m_samples.size();
	}

	void ShiftSample() {
		if (!m_samples.empty()) m_samples.pop_front();
	}

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

	std::deque<Sample> m_samples;

	std::vector<bool> DetectOutliers() const;
	Eigen::Vector3d CalibrateRotation(const bool ignoreOutliers) const;
	Eigen::Vector3d CalibrateTranslation(const Eigen::Matrix3d &rotation) const;

	Eigen::AffineCompact3d ComputeCalibration(const bool ignoreOutliers) const;

	double RetargetingErrorRMS(const Eigen::Vector3d& hmdToTargetPos, const Eigen::AffineCompact3d& calibration) const;
	Eigen::Vector3d ComputeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const;

	Eigen::Vector4d ComputeAxisVariance(const Eigen::AffineCompact3d& calibration) const;

	[[nodiscard]] bool ValidateCalibration(const Eigen::AffineCompact3d& calibration, double *errorOut = nullptr, Eigen::Vector3d* posOffsetV = nullptr);
	void ComputeInstantOffset();

	Eigen::AffineCompact3d EstimateRefToTargetPose(const Eigen::AffineCompact3d& calibration) const;
	bool CalibrateByRelPose(Eigen::AffineCompact3d &out) const;
};