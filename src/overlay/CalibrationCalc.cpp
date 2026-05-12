#include "CalibrationCalc.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "Protocol.h"
#include "WatchdogDecisions.h"  // ShouldClearViaWatchdog, IsCalibrationHealthy
#include "RobustScale.h"        // Qn, TukeyWeight, kQnConsistency, kTukeyTune (opt-in IRLS path)
#include "BlendFilter.h"        // Kalman-filter blend (opt-in publish path)
#include "TranslationSolveDirect.h"

#include <chrono>  // steady_clock for throttled diagnostic logs in
                   // CalibrateRotation / CalibrateTranslation. The throttle
                   // keeps the per-tick solver loop from flooding the log.

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

namespace {

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
		const double c = (rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) * 0.5;
		return std::acos(std::clamp(c, -1.0, 1.0));
	}

	vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg)
	{
		auto euler = eulerdeg * EIGEN_PI / 180.0;

		Eigen::Quaterniond rotQuat =
			Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

		vr::HmdQuaternion_t vrRotQuat;
		vrRotQuat.x = rotQuat.coeffs()[0];
		vrRotQuat.y = rotQuat.coeffs()[1];
		vrRotQuat.z = rotQuat.coeffs()[2];
		vrRotQuat.w = rotQuat.coeffs()[3];
		return vrRotQuat;
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
}

const double CalibrationCalc::AxisVarianceThreshold = 0.001;
void CalibrationCalc::PushSample(const Sample& sample) {
	m_samples.push_back(sample);
	if (sample.valid && sample.timestamp > m_lastSampleTime) {
		m_lastSampleTime = sample.timestamp;
	}
}

void CalibrationCalc::Clear() {
	m_estimatedTransformation.setIdentity();
	m_isValid = false;
	m_samples.clear();
	m_rotationFrozen.clear();
	m_axisVariance = 0.0;
	m_refToTargetPose = Eigen::AffineCompact3d::Identity();
	m_relativePosCalibrated = false;
	m_rotationConditionRatio = 0.0;
	m_consecutiveRejections = 0;
	// Kalman blend filter resets here so post-Clear (recovery, geometry-shift,
	// stuck-loop watchdog) restarts seed the filter from the next accept.
	spacecal::blendfilter::Reset(m_blendFilter);
	m_blendFilterLastUpdateTime = 0.0;
	// `m_lastSampleTime` and `m_lastSuccessfulIncrementalTime` deliberately retained
	// across Clear() so the watchdog can still see the gap if continuous calibration
	// is restarted faster than fresh samples can be collected.
}

void CalibrationCalc::FreezeRotationPhaseSamples() {
	// Move the live sample buffer into the frozen-rotation slot so the next
	// CollectSample tick starts a fresh translation-phase buffer. ComputeOneshot
	// later splices these back in for the duration of the solve so the math sees
	// rotation+translation samples as a single unified deque.
	m_rotationFrozen = std::move(m_samples);
	m_samples.clear();   // explicit; std::deque move-from is empty per the standard but be defensive
}

std::vector<bool> CalibrationCalc::DetectOutliers() const {
	// Use bigger step to get a rough rotation.
	std::vector<DSample> deltas;
	const size_t step = 5;
	for (size_t i = 0; i < m_samples.size(); i += step) {
		for (size_t j = 0; j < i; j += step)
		{
			auto delta = DeltaRotationSamples(m_samples[i], m_samples[j]);
			if (delta.valid) {
				deltas.push_back(delta);
			}
		}
	}

	// With fewer than ~6 samples the step=5 pair extraction can't produce any
	// delta-rotations, so the Kabsch SVD below would run on empty input and the
	// extrinsic-quaternion construction further down would feed `Eigen::Quaterniond`
	// a non-orthogonal/NaN-laden rotation matrix — which asserts in debug builds and
	// produces undefined behaviour in release. Short-circuit: with too little data
	// to make an outlier judgement we accept everything; the caller's own validity
	// checks (RMS error, axis variance, condition ratio) will catch garbage solves.
	if (deltas.empty()) {
		m_so3KabschResult = Eigen::Matrix3d::Identity();
		m_so3KabschValid = false;
		m_residualPitchRollDeg = 0.0;
		return std::vector<bool>(m_samples.size(), true);
	}

	// Kabsch algorithm
	Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);
	Eigen::Vector3d refCentroid(0, 0, 0), targetCentroid(0, 0, 0);

	for (size_t i = 0; i < deltas.size(); i++) {
		refPoints.row(i) = deltas[i].ref;
		refCentroid += deltas[i].ref;
		targetPoints.row(i) = deltas[i].target;
		targetCentroid += deltas[i].target;
	}

	refCentroid /= (double)deltas.size();
	targetCentroid /= (double)deltas.size();

	for (size_t i = 0; i < deltas.size(); i++) {
		refPoints.row(i) -= refCentroid;
		targetPoints.row(i) -= targetCentroid;
	}

	auto crossCV = refPoints.transpose() * targetPoints;

	Eigen::BDCSVD<Eigen::MatrixXd> bdcsvd;
	auto svd = bdcsvd.compute(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
	if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0) {
		i(2, 2) = -1;
	}

	Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
	rot.transposeInPlace();

	// Stash the 3D Kabsch result so CalibrateRotation can reuse it for the
	// SO(3) yaw projection (item #3) instead of re-deriving a 2D fit.
	m_so3KabschResult = rot;
	m_so3KabschValid = true;

	// Compute residual pitch+roll. If the recovered SO(3) rotation has any
	// non-yaw component above ~2 deg, the reference and target spaces' gravity
	// axes don't agree — log the discrepancy so the user knows yaw-only
	// alignment is leaving error on the table. We don't reject; the yaw
	// projection below still works, just less accurately.
	{
		// Yaw is the rotation about the world-up (Y) axis. Pitch+roll magnitude
		// = angle between ([0,1,0]) and ([0,1,0]) after the rotation, projected
		// onto the YZ/XY plane respectively. A simpler proxy: the angle between
		// the rotation's Y-column and (0,1,0).
		Eigen::Vector3d yColumn = rot.col(1);
		double yDot = std::max(-1.0, std::min(1.0, yColumn(1)));
		double tiltRad = std::acos(yDot);
		m_residualPitchRollDeg = tiltRad * 180.0 / EIGEN_PI;
	}

	// Optimize an extrinsic from reference to target.
	// Detect the outliers by comparing the extrinsic computed from each pair of
	// rotations to the optimized extrinsic. Iterate up to MaxIters times: each pass
	// recomputes the average extrinsic from the previously-marked valid samples and
	// re-marks based on the refined average. A single pass is fragile when >20% of
	// samples are outliers (the average is dragged toward the bad samples), so this
	// gives the threshold a chance to settle.
	const double threshold = 0.99;
	const int MaxIters = 4;

	std::vector<Eigen::Quaterniond> perSampleQuat(m_samples.size());
	for (size_t i = 0; i < m_samples.size(); i++) {
		Eigen::Matrix3d rotExtTmp = (m_samples[i].ref.rot.transpose() * rot * m_samples[i].target.rot);
		Eigen::Quaterniond q(rotExtTmp);
		q.normalize();
		perSampleQuat[i] = q;
	}

	std::vector<bool> valids(m_samples.size(), true);

	for (int iter = 0; iter < MaxIters; iter++) {
		// Collect quaternions from currently-valid samples.
		size_t validCount = 0;
		for (size_t i = 0; i < valids.size(); i++) if (valids[i]) validCount++;
		if (validCount == 0) break;

		// Markley eigenvector mean: accumulate the 4x4 outer-product matrix
		// M = sum(q * q^T) for each valid quaternion (w,x,y,z), then take
		// the eigenvector of the largest eigenvalue as the mean quaternion.
		// Eigen sorts eigenvalues ascending, so the largest is column 3.
		// This is numerically stable across antipodal-flip boundaries where
		// the arithmetic mean would collapse to near-zero and normalize
		// poorly. Convention (w,x,y,z) matches PoseAverager::Push.
		Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
		for (size_t i = 0; i < m_samples.size(); i++) {
			if (!valids[i]) continue;
			Eigen::Vector4d v(
				perSampleQuat[i].w(), perSampleQuat[i].x(),
				perSampleQuat[i].y(), perSampleQuat[i].z());
			M += v * v.transpose();
		}
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
		solver.compute(M);
		const Eigen::Vector4d& ev = solver.eigenvectors().col(3);
		Eigen::Quaterniond quatExt(ev(0), ev(1), ev(2), ev(3));
		quatExt.normalize();

		// Re-mark every sample (valid set can grow as well as shrink across iterations,
		// since a previously-rejected sample can become valid once outliers stop
		// pulling the average).
		bool changed = false;
		for (size_t i = 0; i < m_samples.size(); i++) {
			double cosHalfAngle = perSampleQuat[i].w() * quatExt.w() + perSampleQuat[i].vec().dot(quatExt.vec());
			bool nowValid = std::abs(cosHalfAngle) >= threshold;
			if (nowValid != valids[i]) {
				changed = true;
				valids[i] = nowValid;
			}
		}
		if (!changed) break;
	}

	return valids;
}

Eigen::Vector3d CalibrationCalc::CalibrateRotation(const bool ignoreOutliers) const {
	std::vector<DSample> deltas;
	std::vector<bool> valids = DetectOutliers();

	for (size_t i = 0; i < m_samples.size(); i++) {
		for (size_t j = 0; j < i; j++) {
			if ( ignoreOutliers && (!valids[i] || !valids[j])) {
				continue;
			}
			auto delta = DeltaRotationSamples(m_samples[i], m_samples[j]);
			if (delta.valid) {
				deltas.push_back(delta);
			}
		}
	}
	//char buf[256];
	//snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", m_samples.size(), deltas.size());
	//CalCtx.Log(buf);

	// Guard against the degenerate empty-deltas case (too few samples, or all
	// pairs failed the rotation-magnitude validity check). Without this we'd
	// divide by deltas.size() == 0 below and feed NaN-laden centroids into the
	// SVD. Returning zero euler with condition ratio = 0 lets ComputeIncremental's
	// degenerate-motion guard reject the result naturally.
	if (deltas.empty()) {
		m_rotationConditionRatio = 0.0;
		return Eigen::Vector3d::Zero();
	}

	// Item #3: full SO(3) Kabsch + yaw projection. The previous implementation
	// dropped Y from the rotation deltas before SVD, which leaks 1-2 deg of
	// system-gravity-misalignment into the recovered yaw. Instead we run the
	// full 3D Kabsch fit (already computed in DetectOutliers and stashed in
	// m_so3KabschResult) and project to yaw via the Rodrigues identity:
	//   yaw = atan2(R(2,0) - R(0,2), R(0,0) + R(2,2))
	// This gives the closed-form yaw component of an SO(3) rotation, isolating
	// it from any pitch/roll discrepancy between the two spaces' gravity axes.
	// We still compute a 3D cross-covariance SVD on the filtered (post-outlier-
	// rejection) deltas so the rotation we project from reflects the user's
	// actual sample set rather than the rough step=5 fit DetectOutliers used.

	// Build 3D point clouds and centroids from the (filtered) deltas.
	Eigen::MatrixXd refPoints3(deltas.size(), 3), targetPoints3(deltas.size(), 3);
	Eigen::Vector3d refCentroid3(0, 0, 0), targetCentroid3(0, 0, 0);

	for (size_t i = 0; i < deltas.size(); i++) {
		refPoints3.row(i) = deltas[i].ref;
		refCentroid3 += deltas[i].ref;
		targetPoints3.row(i) = deltas[i].target;
		targetCentroid3 += deltas[i].target;
	}

	refCentroid3 /= (double)deltas.size();
	targetCentroid3 /= (double)deltas.size();

	for (size_t i = 0; i < deltas.size(); i++) {
		refPoints3.row(i) -= refCentroid3;
		targetPoints3.row(i) -= targetCentroid3;
	}

	auto crossCV3 = refPoints3.transpose() * targetPoints3;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd3(crossCV3, Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Record the rotation-condition ratio in the yaw plane. We extract this
	// from the 3D singular values projected onto X and Z: the smallest of the
	// two yaw-plane singular values vs the largest. (The Y-axis singular value
	// describes the gravity-tilt component, which is informational, not part
	// of the yaw conditioning question.)
	{
		const auto& sv = svd3.singularValues();
		// SVD orders singular values in decreasing magnitude. For the
		// "did the user rotate around enough yaw-plane axes" gate we use the
		// ratio s_min/s_max. With 3 axes available, the smallest is the worst
		// case — if the rotation was mostly single-axis, two of the three sv's
		// will be small and the ratio will be ~0.
		double smax = sv.size() > 0 ? sv(0) : 0.0;
		double smin = sv.size() > 0 ? sv(sv.size() - 1) : 0.0;
		m_rotationConditionRatio = (smax > 0.0) ? (smin / smax) : 0.0;
	}

	Eigen::Matrix3d so3i = Eigen::Matrix3d::Identity();
	if ((svd3.matrixU() * svd3.matrixV().transpose()).determinant() < 0) {
		so3i(2, 2) = -1;
	}
	Eigen::Matrix3d R = svd3.matrixV() * so3i * svd3.matrixU().transpose();
	// Transpose to match the DetectOutliers convention. The raw Kabsch product
	// above maps target deltas onto ref deltas; the prior 2D code's
	// atan2(rot(1,0), rot(0,0)) returned +yaw when the true calibration was
	// +yaw — implying the 2D `rot` was already in the inverse direction. To
	// keep the Rodrigues yaw-projection sign aligned with the existing test
	// fixtures (RecoversPureYaw expects +30 deg for a +30 deg true rotation),
	// we transpose here. DetectOutliers does the same transposeInPlace() at
	// the equivalent point, so this also keeps both cached fits consistent.
	R.transposeInPlace();

	// Update the cached SO(3) result with the filtered fit (DetectOutliers'
	// step=5 result was a rough estimate; this one was solved over the full
	// pruned delta set).
	m_so3KabschResult = R;
	m_so3KabschValid = true;

	// Recompute residual pitch+roll diagnostic on the filtered fit.
	{
		Eigen::Vector3d yColumn = R.col(1);
		double yDot = std::max(-1.0, std::min(1.0, yColumn(1)));
		double tiltRad = std::acos(yDot);
		m_residualPitchRollDeg = tiltRad * 180.0 / EIGEN_PI;
	}

	if (m_residualPitchRollDeg > 2.0) {
		char buf[256];
		snprintf(buf, sizeof buf,
			"system gravity axes appear misaligned (residual pitch+roll = %.2f deg)\n",
			m_residualPitchRollDeg);
		CalCtx.Log(buf);
	}

	// Yaw extraction by Y-axis swing-twist decomposition of the SO(3)
	// rotation. For a unit quaternion this is algebraically identical to
	// atan2(R(0,2) - R(2,0), R(0,0) + R(2,2)); both yield the closed-form
	// yaw rotation closest to R under Frobenius distance. Swing-twist is
	// kept for quaternion-convention clarity. The accuracy improvement
	// over the pre-9f46548 code came from fitting full SO(3) with Kabsch
	// before projecting to yaw, not from the projection formula itself.
	//
	// Sign convention: with R in target -> ref direction (after the
	// transpose above) and Eigen's right-handed Y-up convention, the
	// twist quaternion's y component sign matches the Rodrigues atan2
	// sign for the zero-pitch/roll case.
	Eigen::Quaterniond q(R);
	Eigen::Quaterniond twistY(q.w(), 0.0, q.y(), 0.0);
	twistY.normalize();
	double yaw = 2.0 * std::atan2(twistY.y(), twistY.w());

	// Diagnostic: also compute the previous Rodrigues yaw projection so we
	// can monitor the empirical delta between the two methods on real
	// inputs. Throttled to once per 2 s so a multi-Hz solver loop doesn't
	// flood the log. The 2026-05-04 audit-fix-cleanup work added this so
	// future "calibration looks off" reports have data to back-check whether
	// the swing-twist switch is producing meaningfully different yaw than
	// the Rodrigues approximation would have.
	{
		static auto s_lastLog = std::chrono::steady_clock::time_point{};
		auto nowTp = std::chrono::steady_clock::now();
		if (nowTp - s_lastLog >= std::chrono::seconds(2)) {
			s_lastLog = nowTp;
			const double rodriguesYaw = std::atan2(R(0, 2) - R(2, 0), R(0, 0) + R(2, 2));
			char yawbuf[160];
			snprintf(yawbuf, sizeof yawbuf,
				"cal_rotation_yaw: rodrigues_yaw=%.4f swingtwist_yaw=%.4f delta=%.4f (degrees: rod=%.3f st=%.3f)",
				rodriguesYaw, yaw, yaw - rodriguesYaw,
				rodriguesYaw * 180.0 / EIGEN_PI, yaw * 180.0 / EIGEN_PI);
			Metrics::WriteLogAnnotation(yawbuf);
		}
	}

	// Convert to degrees
	Eigen::Vector3d euler(0.0, yaw * 180.0 / EIGEN_PI, 0.0);

	//snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
	//CalCtx.Log(buf);
	return euler;
}

Eigen::Vector3d CalibrationCalc::CalibrateTranslationLegacyPairwise(const Eigen::Matrix3d &rotation) const
{
	// Each delta-pair carries an associated weight derived from the smaller of the
	// two rotation magnitudes between samples i and j. Pairs with tiny rotation
	// contribute mostly noise to the translation solve (the LS rows are small),
	// so we down-weight them with sqrt(weight) row-scaling.
	struct DeltaRow {
		Eigen::Vector3d c;
		Eigen::Matrix3d q;
		double weight;
		// Max linear speed (m/s) across the four pose readings (ref/target,
		// sample i / sample j) that produced this row. Zero when sample
		// velocity wasn't recorded (replay harness, tests). Consumed only
		// by the velocity-aware IRLS path; the default Cauchy path ignores it.
		double pairSpeedMax;
	};
	std::vector<DeltaRow> deltas;
	deltas.reserve(m_samples.size() * m_samples.size());

	for (size_t i = 0; i < m_samples.size(); i++)
	{
		Sample s_i = m_samples[i];
		s_i.target.rot = rotation * s_i.target.rot;
		s_i.target.trans = rotation * s_i.target.trans;

		for (size_t j = 0; j < i; j++)
		{
			Sample s_j = m_samples[j];
			s_j.target.rot = rotation * s_j.target.rot;
			s_j.target.trans = rotation * s_j.target.trans;

			// Weight = min(refAngle, targetAngle), clamped to a small floor so we
			// never fully zero a row (preserves rank). Both angles are in radians.
			// (This is the *initial* weight for the IRLS warm start below; later
			// passes overwrite it with Cauchy weights derived from residuals.)
			double refA = AngleFromRotationMatrix3(s_i.ref.rot * s_j.ref.rot.transpose());
			double targetA = AngleFromRotationMatrix3(s_i.target.rot * s_j.target.rot.transpose());
			double weight = std::min(refA, targetA);
			if (!std::isfinite(weight) || weight < 0.01) weight = 0.01;

			// Maximum recorded linear speed across the pair. ANY mover is
			// enough to mark the pair as "in motion"; only when all four
			// readings were stationary do we treat residuals as ground-truth
			// signal that the calibration is wrong.
			const double pairSpeed = std::max(std::max(m_samples[i].refSpeed, m_samples[i].targetSpeed),
			                                  std::max(m_samples[j].refSpeed, m_samples[j].targetSpeed));

			auto QAi = s_i.ref.rot.transpose();
			auto QAj = s_j.ref.rot.transpose();
			auto dQA = QAj - QAi;
			auto CA = QAj * (s_j.ref.trans - s_j.target.trans) - QAi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back({ CA, dQA, weight, pairSpeed });

			auto QBi = s_i.target.rot.transpose();
			auto QBj = s_j.target.rot.transpose();
			auto dQB = QBj - QBi;
			auto CB = QBj * (s_j.ref.trans - s_j.target.trans) - QBi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back({ CB, dQB, weight, pairSpeed });
		}
	}

	// With < 2 samples no pair-based deltas exist; the LS system below would be
	// 0x3 and the QR solve is undefined on empty input. Fall through with zero
	// translation; the caller's validation gate will reject the result.
	if (deltas.empty()) {
		m_translationConditionRatio = 0.0;
		return Eigen::Vector3d::Zero();
	}

	const Eigen::Index nRows = static_cast<Eigen::Index>(deltas.size() * 3);

	// Item #2: pre-allocated members reused across calls. resize() is a no-op
	// when dimensions match, so the steady-state per-tick allocation is zero.
	m_coefficientsTrans.resize(nRows, 3);
	m_constantsTrans.resize(nRows);
	m_weightsTrans.resize(static_cast<Eigen::Index>(deltas.size()));

	// Pack the unweighted system once. IRLS reweights via row-scaling on the
	// already-built coefficient/constant blocks each pass, so we don't need to
	// rebuild the per-pair geometry.
	Eigen::MatrixXd baseCoefficients(nRows, 3);
	Eigen::VectorXd baseConstants(nRows);
	for (size_t i = 0; i < deltas.size(); i++)
	{
		for (int axis = 0; axis < 3; axis++)
		{
			baseConstants(static_cast<Eigen::Index>(i) * 3 + axis) = deltas[i].c(axis);
			baseCoefficients.row(static_cast<Eigen::Index>(i) * 3 + axis) = deltas[i].q.row(axis);
		}
		// Initial weight = 1 (unweighted first pass per item #4 spec).
		m_weightsTrans(static_cast<Eigen::Index>(i)) = 1.0;
	}

	// Item #4: IRLS with Cauchy weight. The previous min-rotation-magnitude
	// weight was a heuristic that didn't adapt to per-pair residual noise — a
	// few large-magnitude deltas with bad position data could still pull the
	// solution. Cauchy (Lorentzian) is a monotonically-descending M-estimator:
	// large residuals get progressively smaller but never-zero weights, so
	// heavy-tailed jitter (Slime IMU translations, USB-glitched frames) stops
	// dominating the LS sum. (Earlier comment said "redescending"; that was
	// wrong. Cauchy is monotonically descending; Tukey biweight is the
	// canonical redescending alternative.) The cosine-similarity outlier
	// rejection in DetectOutliers becomes redundant after this — keep it as
	// belt-and-braces; the math review notes considering removal as a follow-up.
	const int kMaxIters = 5;
	const double kWeightChangeThreshold = 0.01; // 1%
	const double kMadFloor = 1e-3;              // 1mm — avoids div-by-zero when residuals collapse
	// Tuning constant for the Cauchy weight w_i = 1 / (1 + (r_i/c)^2).
	// NOTE: 1.345 is Huber's 95% Gaussian-efficiency constant, NOT Cauchy's
	// (Cauchy 95% efficiency is ~2.3849). The mislabel is historical; we
	// keep the value because it is empirically stable on the residual
	// distributions we observe in real sessions. Switching to true Cauchy
	// tuning or to Tukey biweight (c=4.685) would be a behavior change
	// and needs its own regression-test pass before merging.
	const double kCauchyTune = 1.345;

	Eigen::Vector3d trans = Eigen::Vector3d::Zero();
	Eigen::VectorXd prevWeights = m_weightsTrans;

	for (int iter = 0; iter < kMaxIters; iter++) {
		// Apply current weights as sqrt-row-scaling.
		for (size_t i = 0; i < deltas.size(); i++) {
			double sw = std::sqrt(std::max(0.0, m_weightsTrans(static_cast<Eigen::Index>(i))));
			for (int axis = 0; axis < 3; axis++) {
				const Eigen::Index r = static_cast<Eigen::Index>(i) * 3 + axis;
				m_constantsTrans(r) = baseConstants(r) * sw;
				m_coefficientsTrans.row(r) = baseCoefficients.row(r) * sw;
			}
		}

		// Item #1: colPivHouseholderQr is ~5-10x faster than BDCSVD on 3-column
		// systems and equally accurate. Use it for every IRLS iteration.
		Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr = m_coefficientsTrans.colPivHouseholderQr();
		trans = qr.solve(m_constantsTrans);

		// Item #6 (audit Math #2): condition guard via true SVD on the 3x3
		// normal matrix. Replaces the previous min|R(i,i)| / max|R(i,i)|
		// proxy from the QR decomposition's R triangle: that proxy is
		// notoriously loose because ColPivHouseholderQR's column pivoting
		// can re-order R's diagonal so |R(0,0)| isn't the largest singular
		// value, and the diagonal magnitudes are NOT the singular values
		// in general -- they only track the singular-value extrema for
		// well-conditioned systems.
		//
		// JacobiSVD on the 3x3 normal matrix m_coefficientsTrans^T *
		// m_coefficientsTrans gives exact singular values, sorted descending.
		// For an Nx3 matrix A, sv(A^T A) = sv(A)^2, so the condition ratio
		// of A is sqrt(svN/sv0) where svN is the smallest singular value and
		// sv0 the largest. The condition-number squaring (sv(A) -> sv(A)^2)
		// costs ~half the precision but JacobiSVD on a 3x3 matrix is well
		// within double-precision headroom for the gate range we care about
		// (gate at ratio ~0.05 = condition number 20, squared = 400, way
		// above any precision floor).
		//
		// Same [0,1] semantic as the prior proxy: 1.0 = perfectly
		// conditioned, 0.0 = singular. Existing 0.05 hard-gate at the
		// caller continues to work without re-tuning.
		//
		// Conditioning ratio compute moved out of the per-iter branch (rec G,
		// 2026-05-07): it was previously gated on iter == kMaxIters - 1, which
		// the convergence check at line 744 routinely short-circuits past.
		// The result was a stale (often zero) ratio reaching the caller's
		// hard-gate. Compute below the loop instead so every solve emits a
		// fresh ratio.

		// Compute per-pair residuals (we collapse the per-axis residuals into a
		// single magnitude per delta-row triple via the L2 norm — this gives
		// each delta-pair a single weight, which is what the Cauchy formula
		// expects).
		Eigen::VectorXd resid = baseCoefficients * trans - baseConstants;
		Eigen::VectorXd perPair(static_cast<Eigen::Index>(deltas.size()));
		for (size_t i = 0; i < deltas.size(); i++) {
			double s2 = 0.0;
			for (int axis = 0; axis < 3; axis++) {
				double r = resid(static_cast<Eigen::Index>(i) * 3 + axis);
				s2 += r * r;
			}
			perPair(static_cast<Eigen::Index>(i)) = std::sqrt(s2);
		}

		// Scale estimate. Default path: MAD via two nth_element passes (O(N)).
		// Opt-in path (useTukeyBiweight): Qn-scale (Rousseeuw-Croux 1993),
		// O(N^2) over the per-pair residuals. Both produce a sigma estimate
		// that the weight function consumes. Qn has 50% breakdown without
		// requiring symmetry of the residual distribution and does not
		// saturate at the kMadFloor.
		double scale = 0.0;
		if (useTukeyBiweight) {
			std::vector<double> resids(deltas.size());
			for (size_t i = 0; i < deltas.size(); i++) {
				resids[i] = perPair(static_cast<Eigen::Index>(i));
			}
			scale = spacecal::robust::Qn(resids);
			if (!(scale > kMadFloor)) scale = kMadFloor;
		} else {
			std::vector<double> absResid(deltas.size());
			for (size_t i = 0; i < deltas.size(); i++) absResid[i] = std::abs(perPair(static_cast<Eigen::Index>(i)));
			std::nth_element(absResid.begin(), absResid.begin() + absResid.size() / 2, absResid.end());
			const double median = absResid[absResid.size() / 2];

			// MAD = median(|r_i - median(r)|). Reuse absResid storage with the
			// shifted residuals.
			for (size_t i = 0; i < deltas.size(); i++) absResid[i] = std::abs(perPair(static_cast<Eigen::Index>(i)) - median);
			std::nth_element(absResid.begin(), absResid.begin() + absResid.size() / 2, absResid.end());
			scale = absResid[absResid.size() / 2];
			if (!(scale > kMadFloor)) scale = kMadFloor;
		}

		// Tuning constant for the chosen kernel. Cauchy 95% efficiency
		// historically uses Huber's c=1.345 here (a relabel; see
		// CalibrationCalc.cpp:560 comment); Tukey biweight 95% efficiency
		// uses c=4.685 paired with Qn.
		const double tuneConstant = useTukeyBiweight
			? spacecal::robust::kTukeyTune
			: kCauchyTune;
		const double c0 = tuneConstant * scale;

		// Velocity-aware per-row scaling (opt-in). When useVelocityAwareWeighting
		// is on, divide the per-pair threshold by (1 + kappa * v / vRef) so
		// fast-motion pairs get a SHARPER cutoff and stationary pairs keep
		// the standard c0. Direction follows sore-point #12 in the math
		// rundown: stationary high-residual is informative ("cal is wrong
		// here"), motion high-residual is a glitch (suppress). Composes with
		// either kernel.
		constexpr double kVelocityKappa = 2.0;
		constexpr double kVelocityRefMps = 0.3;  // m/s; brisk arm-swing speed

		// Apply weights: Cauchy or Tukey biweight per the toggle.
		for (size_t i = 0; i < deltas.size(); i++) {
			double cThis = c0;
			if (useVelocityAwareWeighting) {
				const double v = std::max(0.0, deltas[i].pairSpeedMax);
				const double vScale = 1.0 + kVelocityKappa * v / kVelocityRefMps;
				cThis = c0 / vScale;
				if (!(cThis > 1e-9)) cThis = 1e-9;
			}
			const double r = perPair(static_cast<Eigen::Index>(i));
			double w = 0.0;
			if (useTukeyBiweight) {
				w = spacecal::robust::TukeyWeight(r, cThis);
			} else {
				const double rOverC = r / cThis;
				w = 1.0 / (1.0 + rOverC * rOverC);
			}
			m_weightsTrans(static_cast<Eigen::Index>(i)) = w;
		}

		// Convergence check: max relative weight change < 1%.
		double maxDelta = 0.0;
		for (Eigen::Index i = 0; i < m_weightsTrans.size(); i++) {
			double prev = prevWeights(i);
			double cur = m_weightsTrans(i);
			double denom = std::max(std::abs(prev), 1e-9);
			double rel = std::abs(cur - prev) / denom;
			if (rel > maxDelta) maxDelta = rel;
		}
		prevWeights = m_weightsTrans;
		if (iter > 0 && maxDelta < kWeightChangeThreshold) break;
	}

	// Conditioning ratio computed once after IRLS terminates (rec G, 2026-05-07).
	// Uses the final weighted system that produced `trans`. JacobiSVD on the
	// 3x3 normal matrix m_coefficientsTrans^T * m_coefficientsTrans gives
	// exact singular values; the [0,1] ratio sqrt(sv(2)/sv(0)) is the same
	// semantic the prior block emitted. Computing here (not gated on
	// iter == kMaxIters - 1) makes the ratio reliable when IRLS converges
	// early -- the previous gate left the value stale, often at 0.0, which
	// the rec G hard-reject in ComputeOneshot mistook for "zero-rank
	// covariance" on legitimate fits.
	{
		Eigen::Matrix3d normal = m_coefficientsTrans.transpose() * m_coefficientsTrans;
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(normal);
		const auto& sv = svd.singularValues();
		m_translationConditionRatio = (sv(0) > 0.0) ? std::sqrt(sv(2) / sv(0)) : 0.0;

		// Diagnostic: log the condition ratio + sample count so we can observe
		// how often the gate is near-singular (ratio below the 0.05 threshold)
		// on real data. Throttled to once per 2 s so a per-tick solver does
		// not flood the log.
		static auto s_lastCondLog = std::chrono::steady_clock::time_point{};
		auto nowTp = std::chrono::steady_clock::now();
		if (nowTp - s_lastCondLog >= std::chrono::seconds(2)) {
			s_lastCondLog = nowTp;
			const double kGateThreshold = 0.05;
			const bool pass = m_translationConditionRatio >= kGateThreshold;
			char condbuf[224];
			snprintf(condbuf, sizeof condbuf,
				"cal_translation_cond: svd_ratio=%.6f threshold=%.6f pass=%d sample_count=%lld sv=(%.6e, %.6e, %.6e)",
				m_translationConditionRatio, kGateThreshold, (int)pass,
				(long long)deltas.size(), sv(0), sv(1), sv(2));
			Metrics::WriteLogAnnotation(condbuf);
		}
	}

	auto transcm = trans * 100.0;
	(void)transcm; // kept for parity with the prior debug logging block

	//char buf[256];
	//snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
	//CalCtx.Log(buf);
	return trans;
}

Eigen::Vector3d CalibrationCalc::CalibrateTranslation(const Eigen::Matrix3d &rotation) const
{
	std::vector<Sample> sampleVec(m_samples.begin(), m_samples.end());
	if (CalCtx.useUpstreamMath) {
		const auto result = spacecal::translation::CalibrateTranslationUpstream(sampleVec, rotation);
		m_translationConditionRatio = result.conditionRatio;
		return result.translation;
	}
	if (CalCtx.useLegacyMath) {
		return CalibrateTranslationLegacyPairwise(rotation);
	}
	spacecal::translation::DirectOptions opts;
	opts.useTukeyBiweight          = useTukeyBiweight;
	opts.useVelocityAwareWeighting = useVelocityAwareWeighting;
	const auto result = spacecal::translation::SolveDirect(sampleVec, rotation, opts);
	m_translationConditionRatio = result.conditionRatio;
	return result.translation;
}


namespace {
	Pose ApplyTransform(const Pose& originalPose, const Eigen::AffineCompact3d& transform) {
		Pose pose(originalPose);
		pose.rot = transform.rotation() * pose.rot;
		pose.trans = transform * pose.trans;
		return pose;
	}

	 Pose ApplyTransform(const Pose & originalPose, const Eigen::Vector3d & vrTrans, const Eigen::Matrix3d & rotMat) {
		Pose pose(originalPose);
		pose.rot = rotMat * pose.rot;
		pose.trans = vrTrans + (rotMat * pose.trans);
		return pose;
	}
}

Eigen::AffineCompact3d CalibrationCalc::ComputeCalibration(const bool ignoreOutliers) const {
	Eigen::Vector3d rotation = CalibrateRotation(ignoreOutliers);
	Eigen::Matrix3d rotationMat = quaternionRotateMatrix(VRRotationQuat(rotation));
	Eigen::Vector3d translation = CalibrateTranslation(rotationMat);
	
	Eigen::AffineCompact3d rot(rotationMat);
	Eigen::Translation3d trans(translation);

	return trans * rot;
}



double CalibrationCalc::RetargetingErrorRMS(
	const Eigen::Vector3d& hmdToTargetPos,
	const Eigen::AffineCompact3d& calibration
) const {
	double errorAccum = 0;
	int sampleCount = 0;

	for (auto& sample : m_samples) {
		if (!sample.valid) continue;

		// Apply transformation
		const auto updatedPose = ApplyTransform(sample.target, calibration);

		const Eigen::Vector3d hmdPoseSpace = sample.ref.rot * hmdToTargetPos + sample.ref.trans;

		// Compute error term
		double error = (updatedPose.trans - hmdPoseSpace).squaredNorm();
		errorAccum += error;
		sampleCount++;
	}

	if (sampleCount == 0) return std::numeric_limits<double>::infinity();
	return sqrt(errorAccum / sampleCount);
}

// Welford's online algorithm for the position-jitter standard deviation across
// the current sample buffer. Returns the magnitude of the per-axis std-dev
// vector, in meters. Used as the per-tracker noise floor that informs the
// rejection threshold and the auto-calibration-speed heuristic.
//
// Bug fixed 2026-04-28: the original implementation divided by `sampleCount`
// (the count BEFORE the current sample was added) instead of `sampleCount + 1`
// (Welford's k). Off-by-one in the mean update inflated the variance by a
// factor of ~2x at small N and asymptotically less. Symptom: jitter values in
// the debug log were reading ~2 km instead of a few cm.
// Per-sample tracking-noise magnitude via second-difference variance.
//
// Why second differences and not raw position std-dev: the original metric
// computed Welford std-dev of every sample's position across the buffer,
// which conflated tracking noise with user motion. A buffer spanning 1 m
// of head-waving has 30+ cm "jitter" -- enough to permanently pin the
// AUTO calibration-speed selector at VERY_SLOW, regardless of how clean
// the actual tracking is. The user reported tracking that "isn't that
// bad" stuck on VERY_SLOW; the metric, not their hardware, was at fault.
//
// The fix: for each consecutive triple p[i-1], p[i], p[i+1], compute the
// second difference Δ²p = p[i+1] - 2 p[i] + p[i-1]. This is zero for
// any linear motion (constant velocity) and small for any bounded human
// acceleration; tracking noise dominates the signal at typical sample
// rates because acceleration * dt² is sub-millimetre while tracking
// noise is order-of-magnitude bigger on most rigs.
//
// For independent zero-mean Gaussian noise with per-axis std σ, the
// second difference has per-axis variance (1²+2²+1²)σ² = 6σ². The
// magnitude squared expected value is therefore 3·6σ² = 18σ², so
// √(mean|Δ²p|² / 6) recovers the magnitude form √3·σ (matching the
// old return shape -- per-axis-std sum-of-squares root). Threshold
// constants in ResolvedCalibrationSpeed (1 mm / 5 mm) didn't have to
// change; they were always written for noise, just paired with the
// wrong metric.
static double SampleNoiseStdMagnitude(const std::deque<Sample>& samples,
                                      bool useTarget) {
	double sumSq = 0;
	int n = 0;
	for (size_t i = 1; i + 1 < samples.size(); ++i) {
		if (!samples[i-1].valid || !samples[i].valid || !samples[i+1].valid) continue;
		const Eigen::Vector3d& p0 = useTarget ? samples[i-1].target.trans : samples[i-1].ref.trans;
		const Eigen::Vector3d& p1 = useTarget ? samples[i].target.trans   : samples[i].ref.trans;
		const Eigen::Vector3d& p2 = useTarget ? samples[i+1].target.trans : samples[i+1].ref.trans;
		const Eigen::Vector3d a = p2 - 2.0 * p1 + p0;
		sumSq += a.squaredNorm();
		++n;
	}
	if (n < 1) return 0.0;
	return std::sqrt(sumSq / static_cast<double>(n) / 6.0);
}

double CalibrationCalc::ReferenceJitter() const {
	return SampleNoiseStdMagnitude(m_samples, /*useTarget=*/false);
}

double CalibrationCalc::TargetJitter() const {
	return SampleNoiseStdMagnitude(m_samples, /*useTarget=*/true);
}

double CalibrationCalc::TranslationDiversity() const {
	// Per-axis bounding box of the target tracker translation across valid
	// samples. The smallest axis-range relative to a desired total spread
	// is the "weakest link" -- a user who waved on X+Y but never on Z gets
	// a low score regardless of how much they waved on the other two.
	if (m_samples.size() < 2) return 0.0;
	constexpr double kInf = std::numeric_limits<double>::infinity();
	Eigen::Vector3d minPos(kInf, kInf, kInf);
	Eigen::Vector3d maxPos(-kInf, -kInf, -kInf);
	int n = 0;
	for (const auto& s : m_samples) {
		if (!s.valid) continue;
		minPos = minPos.cwiseMin(s.target.trans);
		maxPos = maxPos.cwiseMax(s.target.trans);
		++n;
	}
	if (n < 2) return 0.0;
	const Eigen::Vector3d range = maxPos - minPos;
	// 30cm spread per axis is the sweet spot where the translation LS is
	// well-conditioned for a typical room-scale setup. Going much wider
	// doesn't help; going narrower starts to leak per-axis correlation noise
	// into the fitted offset.
	constexpr double kDesiredAxisRange = 0.30;
	const double minAxis = range.minCoeff();
	const double score = minAxis / kDesiredAxisRange;
	return std::min(std::max(score, 0.0), 1.0);
}

Eigen::Vector3d CalibrationCalc::TranslationAxisRangesCm() const {
	// Same bounding-box scan as TranslationDiversity, but returns the per-axis
	// ranges in centimetres rather than collapsing them to a single score. The
	// UI tooltip uses these to tell the user which axis is the bottleneck when
	// the Translation% bar is stuck below 100. Whichever component is smallest
	// is what's pinning the score (= min component / 30 cm).
	if (m_samples.size() < 2) return Eigen::Vector3d::Zero();
	Eigen::Vector3d minPos = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
	Eigen::Vector3d maxPos = -minPos;
	int n = 0;
	for (const auto& s : m_samples) {
		if (!s.valid) continue;
		minPos = minPos.cwiseMin(s.target.trans);
		maxPos = maxPos.cwiseMax(s.target.trans);
		++n;
	}
	if (n < 2) return Eigen::Vector3d::Zero();
	return (maxPos - minPos) * 100.0;
}

double CalibrationCalc::RotationDiversity() const {
	// Maximum angular distance between any two sampled target rotations.
	// One pair with a wide angular separation is enough to anchor yaw; we
	// scale toward 90 degrees as the "fully covered" point. This is much
	// less stringent than the full SO(3) Kabsch needs for a clean fit, but
	// matches the practical observation that even modest rotation variety
	// constrains the calibration's rotation component well.
	if (m_samples.size() < 2) return 0.0;
	constexpr double kDesiredMaxAngle = EIGEN_PI / 2.0; // 90 deg
	Eigen::Quaterniond first;
	bool haveFirst = false;
	double maxAngle = 0.0;
	for (const auto& s : m_samples) {
		if (!s.valid) continue;
		const Eigen::Quaterniond q(s.target.rot);
		if (!haveFirst) { first = q; haveFirst = true; continue; }
		const double a = first.angularDistance(q);
		if (a > maxAngle) maxAngle = a;
	}
	const double score = maxAngle / kDesiredMaxAngle;
	return std::min(std::max(score, 0.0), 1.0);
}

Eigen::Vector3d CalibrationCalc::ComputeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const {
	Eigen::Vector3d accum = Eigen::Vector3d::Zero();
	int sampleCount = 0;

	for (auto& sample : m_samples) {
		if (!sample.valid) continue;

		// Apply transformation
		const auto updatedPose = ApplyTransform(sample.target, calibration);

		// Now move the transform from world to HMD space
		const auto hmdOriginPos = updatedPose.trans - sample.ref.trans;
		const auto hmdSpace = sample.ref.rot.inverse() * hmdOriginPos;

		accum += hmdSpace;
		sampleCount++;
	}

	if (sampleCount == 0) return Eigen::Vector3d::Zero();
	accum /= sampleCount;

	return accum;
}

Eigen::Vector4d CalibrationCalc::ComputeAxisVariance(
	const Eigen::AffineCompact3d& calibration
) const {
	// We want to determine if the user rotated in enough axis to find a unique solution.
	// It's sufficient to rotate in two axis - this is because once we constrain the mapping
	// of those two orthogonal basis vectors, the third is determined by the cross product of
	// those two basis vectors. So, the question we then have to answer is - after accounting for
	// translational movement of the HMD itself, are we too close to having only moved on a plane?

	// To determine this, we perform primary component analysis on the rotation quaternions themselves.
	// Since an angle axis quaternion is defined as the sum of Qidentity*cos(angle/2) + Qaxis*sin(angle/2),
	// we expect that rotations around a single axis will have two primary components: One corresponding
	// to the identity component, and one to the axis component. Thus, we check the variance (eigenvalue) of
	// the third primary component to see if we've moved in two axis.
	std::ostringstream dbgStream;

	std::vector<Eigen::Vector4d> points;

	Eigen::Vector4d mean = Eigen::Vector4d::Zero();

	for (auto& sample : m_samples) {
		if (!sample.valid) continue;

		auto q = Eigen::Quaterniond(sample.target.rot);
		auto point = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
		mean += point;

		points.push_back(point);
	}
	if (points.empty()) return Eigen::Vector4d::Zero();
	mean /= (double) points.size();

	// Compute covariance matrix
	Eigen::Matrix4d covMatrix = Eigen::Matrix4d::Zero();

	for (auto& point : points) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				covMatrix(i, j) += (point(i) - mean(i)) * (point(j) - mean(j));
			}
		}
	}
	covMatrix /= (double) points.size();

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
	solver.compute(covMatrix);

	return solver.eigenvalues();
}

[[nodiscard]] bool CalibrationCalc::ValidateCalibration(const Eigen::AffineCompact3d &calibration, double *error, Eigen::Vector3d *posOffsetV) {
	bool ok = true;

	const auto posOffset = ComputeRefToTargetOffset(calibration);

	if (posOffsetV) *posOffsetV = posOffset;

	// char buf[256];
	//snprintf(buf, sizeof buf, "HMD to target offset: (%.2f, %.2f, %.2f)\n", posOffset(0), posOffset(1), posOffset(2));
	//CalCtx.Log(buf);

	double rmsError = RetargetingErrorRMS(posOffset, calibration);
	//snprintf(buf, sizeof buf, "Position error (RMS): %.3f\n", rmsError);
	//CalCtx.Log(buf);

	// Item #7: dynamic RMS gate. The previous fixed 0.1 m threshold was too
	// loose: a Slime IMU rig with ~5 mm jitter can easily produce a "valid"
	// RMS of 30 mm that's just noise, not a meaningful fit. Scale the gate to
	// the input jitter (the same noise the LS solver was fed) plus a 5 mm
	// floor — below that floor we'd be rejecting good fits because the
	// trackers are inherently more precise than the threshold pretends.
	// 3 sigma covers ~99.7% of Gaussian noise; combined ref/target jitter is
	// the L2 sum of independent stddevs.
	double jRef = ReferenceJitter();
	double jTgt = TargetJitter();
	double rmsThreshold = std::max(0.005, 3.0 * std::sqrt(jRef * jRef + jTgt * jTgt));
	// Surface the dynamic threshold for triage: a "validate_failed" row is
	// opaque without knowing whether the floor was at 5 mm (low-jitter) or
	// 15 mm (high-jitter), and whether the candidate's RMS was honestly above
	// it or just lost a noise-jitter coin flip.
	Metrics::validateRmsThresholdMm.Push(rmsThreshold * 1000.0);

	m_lastRejectRms = rmsError;
	m_lastRejectRmsThreshold = rmsThreshold;
	if (rmsError > rmsThreshold) {
		ok = false;
		m_lastRejectReason = RejectReason::RmsTooHigh;
	} else {
		m_lastRejectReason = RejectReason::None;
	}

	if (error) *error = rmsError;

	return ok;
}


// Given:
//   R - the reference pose (in reference world space)
//   T - the target pose (in target world space)
//   C - the true calibration (target world -> reference world)
// We assume that there is some "static target pose" S s.t.:
// R * S = C * T (we'll call this the static target pose)
// To compute S:
// S = R^-1 * C * T
// To compute C:
// R * S * T^-1 = C

namespace {
	class PoseAverager {
	private:
		Eigen::Matrix<double, 4, Eigen::Dynamic> quatAvg;
		Eigen::Vector3d accum = Eigen::Vector3d::Zero();
		int i = 0;
	public:
		PoseAverager(size_t n_samples) {
			quatAvg.resize(4, n_samples);
		}

		template<typename P>
		void Push(const P &pose) {
			const Eigen::Quaterniond rot(pose.rotation());
			quatAvg.col(i++) = Eigen::Vector4d(rot.w(), rot.x(), rot.y(), rot.z());
			accum += pose.translation();
		}

		Eigen::AffineCompact3d Average() {
			// https://stackoverflow.com/a/27410865/36723
			auto quatT = quatAvg.transpose();
			Eigen::Matrix4d quatMul = quatAvg * quatT;
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
			solver.compute(quatMul);

			Eigen::Vector4d quatAvgV = solver.eigenvectors().col(3).real().normalized();
			Eigen::Quaterniond avgQ(quatAvgV(0), quatAvgV(1), quatAvgV(2), quatAvgV(3));
			avgQ.normalize();

			Eigen::AffineCompact3d pose(avgQ);
			pose.pretranslate(accum * (1.0 / i));

			return pose;
		}

		template<typename XS, typename F>
		static Eigen::AffineCompact3d AverageFor(const XS& samples, const F& poseProvider) {
			int sampleCount = 0;

			for (auto& sample : samples) {
				if (!sample.valid) continue;

				sampleCount++;
			}

			PoseAverager accum(sampleCount);

			for (auto& sample : samples) {
				if (!sample.valid) continue;
				auto pose = poseProvider(sample);
				accum.Push(pose);
			}

			return accum.Average();
		}
	};
}

// S = R^-1 * C * T
Eigen::AffineCompact3d CalibrationCalc::EstimateRefToTargetPose(const Eigen::AffineCompact3d &calibration) const {
	auto avg = PoseAverager::AverageFor(m_samples, [&](const auto& sample) {
		return Eigen::Affine3d(sample.ref.ToAffine().inverse() * calibration * sample.target.ToAffine());
	});

#if 0
	Eigen::Vector3d eulerAvgQ = avg.rotation().eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
	Eigen::Vector3d trans = Eigen::Vector3d(avg.translation());

	std::ostringstream oss;
	oss << "==========================================================================================\n";
	oss << "Avg rot: " << eulerAvgQ.x() << ", " << eulerAvgQ.y() << ", " << eulerAvgQ.z() << "\n";
	oss << "Avg trans:\n" << trans.x() << ", " << trans.y() << ", " << trans.z() << "\n";
	OutputDebugStringA(oss.str().c_str());
#endif
	return avg;
}

// S = R^-1 * C * T
// R * S * T^-1 = C

// R * (R^-1 * C * T) * T^-1 = C

/*
 * This calibration routine attempts to use the estimated refToTargetPose to derive the
 * playspace calibration based on the relative position of reference and target device.
 * This computation can be performed even when the devices are not moving.
 */
bool CalibrationCalc::CalibrateByRelPose(Eigen::AffineCompact3d &out) const {
	// R * S * T^-1 = C
	out = PoseAverager::AverageFor(m_samples, [&](const auto& sample) {
		return Eigen::AffineCompact3d(sample.ref.ToAffine() * m_refToTargetPose * sample.target.ToAffine().inverse());
	});

	return true;
}



namespace {
	// RAII helper that temporarily prepends frozen rotation-phase samples onto
	// a live sample buffer for the duration of a one-shot solve. ComputeOneshot
	// (and every helper it calls — DetectOutliers, CalibrateRotation,
	// ComputeAxisVariance, ValidateCalibration, ComputeRefToTargetOffset,
	// ComputeInstantOffset) iterates m_samples directly. Rather than threading
	// a "use union" parameter through every call site, we splice the frozen
	// samples in at construction and pop them off in the destructor. Result:
	// the math sees rotation+translation samples as one deque without any
	// internal helper having to know about the phase split. ComputeOneshot's
	// metric pushes (samplesInBuffer, posOffset_lastSample, etc.) also reflect
	// the unified buffer, which is what we want for triage — the math ran on
	// the union, so the diagnostic numbers should describe the union.
	//
	// Frozen samples go at the FRONT so m_samples.back() (used by
	// ComputeInstantOffset) still references the most recent live sample.
	class RotationFreezeSplice {
		std::deque<Sample>& m_live;
		size_t m_count;
	public:
		RotationFreezeSplice(std::deque<Sample>& live, const std::deque<Sample>& frozen)
			: m_live(live), m_count(frozen.size())
		{
			if (m_count) m_live.insert(m_live.begin(), frozen.begin(), frozen.end());
		}
		~RotationFreezeSplice() {
			if (m_count) m_live.erase(m_live.begin(), m_live.begin() + m_count);
		}
		RotationFreezeSplice(const RotationFreezeSplice&) = delete;
		RotationFreezeSplice& operator=(const RotationFreezeSplice&) = delete;
	};
}

bool CalibrationCalc::ComputeOneshot(const bool ignoreOutliers) {
	// Splice in any frozen rotation-phase samples for the duration of this
	// solve. No-op when the user is in continuous mode or did a single-phase
	// one-shot (m_rotationFrozen empty). See the RotationFreezeSplice comment
	// above for the rationale.
	RotationFreezeSplice splice(m_samples, m_rotationFrozen);

	// Below ~6 samples, the step=5 outlier-detection produces no deltas, several
	// downstream solves end up with empty matrices, and validation degenerates.
	// Refuse to attempt a calibration on too-small inputs rather than letting
	// the math fall through to NaN / divide-by-zero territory. The Calibration.cpp
	// caller already gates on SampleCount() (100+) so production never hits this
	// branch — it's a safety net for direct callers (the replay harness, tests).
	if (m_samples.size() < 6) {
		CalCtx.Log("Not updating: too few samples to compute a calibration\n");
		return false;
	}

	auto calibration = ComputeCalibration(ignoreOutliers);

	double rmsError = INFINITY;
	bool valid = ValidateCalibration(calibration, &rmsError);

	// Compute the auxiliary gates that ComputeIncremental uses, so the failure
	// log line for one-shot can name the *actual* problem instead of a generic
	// "low quality" shrug. (Item #5 from the math review.)
	double newVariance = ComputeAxisVariance(calibration)(1);
	const double RotationConditionMin = 0.05;

	// Rec G: Fisher-rank observability gate. The deleted Phase 1+2 silent-
	// recal (2026-04-29 main 9fab09d) accepted candidates from stationary
	// buffers because RMS was tautologically small. The actual degenerate
	// case is m_rotationConditionRatio == 0.0 (zero-rank cross-covariance,
	// from CalibrateRotation's empty-deltas early return -- no pair of
	// samples with > 23 deg of rotation between them). Hard-reject in that
	// case even when ValidateCalibration's RMS gate would pass. The
	// existing 0.05 threshold below stays as a rejection-reason label
	// rather than a hard gate so legitimate-but-marginal motion does not
	// regress; the hard reject only fires on the truly unobservable
	// stationary case (Nobre & Heckman 2017/2018 FastCal).
	if (valid && m_rotationConditionRatio == 0.0) {
		valid = false;
	}
	if (valid && m_translationConditionRatio == 0.0) {
		valid = false;
	}

	if (valid) {
		m_estimatedTransformation = calibration; // @NOTE: Normal calibration
		m_isValid = true;
		return true;
	}
	else {
		// Item #5: branch the user-facing log on which threshold tripped. The
		// generic "low-quality calibration result" was actionable for nobody —
		// these messages tell the user what motion to add.
		char buf[256];
		double priorRms = INFINITY;
		Eigen::Vector3d priorOffset;
		if (m_isValid) {
			(void)ValidateCalibration(m_estimatedTransformation, &priorRms, &priorOffset);
		}

		if (m_isValid && rmsError * 1.5 > priorRms) {
			snprintf(buf, sizeof buf,
				"Not updating: candidate RMS %.1fmm not better than active %.1fmm by 1.5x\n",
				rmsError * 1000.0, priorRms * 1000.0);
			CalCtx.Log(buf);
		}
		else if (newVariance < AxisVarianceThreshold) {
			CalCtx.Log("Not updating: rotation single-axis (move tracker through more orientations)\n");
		}
		else if (m_rotationConditionRatio < RotationConditionMin) {
			// Catch-all for both empty-deltas (ratio == 0) and ill-conditioned (small but nonzero).
			CalCtx.Log("Not updating: motion too planar (rotate around more axes)\n");
		}
		else if (m_translationConditionRatio < RotationConditionMin) {
			CalCtx.Log("Translation conditioning poor — need more diverse motion\n");
		}
		else {
			snprintf(buf, sizeof buf,
				"Not updating: RMS %.1fmm above gate %.1fmm\n",
				rmsError * 1000.0, m_lastRejectRmsThreshold * 1000.0);
			CalCtx.Log(buf);
		}
		return false;
	}
}

void CalibrationCalc::ComputeInstantOffset() {
	const auto &latestSample = m_samples.back();

	// Apply transformation
	const auto updatedPose = ApplyTransform(latestSample.target, m_estimatedTransformation);

	// Now move the transform from world to HMD space
	const auto hmdOriginPos = updatedPose.trans - latestSample.ref.trans;
	const auto hmdSpace = latestSample.ref.rot.inverse() * hmdOriginPos;
	
	Metrics::posOffset_lastSample.Push(hmdSpace * 1000);
}

bool CalibrationCalc::ComputeIncremental(bool &lerp, double threshold, double relPoseMaxError, const bool ignoreOutliers) {
	Metrics::RecordTimestamp();

	// Same minimum-sample guard as ComputeOneshot; see comment there.
	if (m_samples.size() < 6) {
		return false;
	}

	if (lockRelativePosition) {
		Eigen::AffineCompact3d byRelPose;
		double relPoseError = INFINITY;
		Eigen::Vector3d relPosOffset;
		if (CalibrateByRelPose(byRelPose) &&
			ValidateCalibration(byRelPose, &relPoseError, &relPosOffset)) {

			Metrics::posOffset_byRelPose.Push(relPosOffset * 1000);
			Metrics::error_byRelPose.Push(relPoseError * 1000);

			m_isValid = true;
			m_estimatedTransformation = byRelPose;
			return true;
		}
	}

	// Push debugging counters every tick so the CSV row is self-describing
	// without having to chase down annotation-line context. samplesInBuffer
	// is the deque size at compute time; watchdogResetCount is monotonic.
	Metrics::samplesInBuffer.Push((double)m_samples.size());
	Metrics::watchdogResetCount.Push((double)m_watchdogResets);

	double priorCalibrationError = INFINITY;
	Eigen::Vector3d priorPosOffset;
	if (m_isValid && ValidateCalibration(m_estimatedTransformation, &priorCalibrationError, &priorPosOffset)) {
		Metrics::posOffset_currentCal.Push(priorPosOffset * 1000);
		Metrics::error_currentCal.Push(priorCalibrationError * 1000);
	}

	double newError = INFINITY;
	bool newCalibrationValid = false;
	Eigen::AffineCompact3d byRelPose;
	Eigen::AffineCompact3d calibration;
	bool usingRelPose = false;
	double relPoseError = INFINITY;

	if (enableStaticRecalibration && CalibrateByRelPose(byRelPose)) {
		Eigen::Vector3d relPosOffset;
		if (ValidateCalibration(byRelPose, &relPoseError, &relPosOffset)) {
			Metrics::posOffset_byRelPose.Push(relPosOffset * 1000);
			Metrics::error_byRelPose.Push(relPoseError * 1000);

			if (relPoseError < 0.010 || m_relativePosCalibrated && relPoseError < 0.025) {
				if (relPoseError * threshold >= priorCalibrationError) {
					return false;
				}

				if (relPoseError > relPoseMaxError) {
					return false;
				}

				newCalibrationValid = true;
				usingRelPose = true;
				newError = relPoseError;
				calibration = byRelPose;

				// Forensic diagnostic for audit row #6 (project_upstream_regression_audit_2026-05-04):
				// fork flipped enableStaticRecalibration default false→true, which
				// makes this byRelPose-shortcut path active for lock-OFF setups
				// where the underlying refToTargetPose may not be a meaningful
				// constraint. If a sleeper regression manifests as cal drift in
				// lock-OFF sessions, the rate of this annotation will reveal it.
				// Throttled 5 s — ComputeIncremental can fire at 10+ Hz.
				static auto s_lastUsingRelPoseLog = std::chrono::steady_clock::time_point{};
				const auto nowTpRP = std::chrono::steady_clock::now();
				if (nowTpRP - s_lastUsingRelPoseLog >= std::chrono::seconds(5)) {
					s_lastUsingRelPoseLog = nowTpRP;
					char rpbuf[200];
					snprintf(rpbuf, sizeof rpbuf,
						"usingRelPose_fired: relPoseError=%.3fmm priorError=%.3fmm relPosCal=%d lockRel=%d",
						relPoseError * 1000.0, priorCalibrationError * 1000.0,
						(int)m_relativePosCalibrated, (int)lockRelativePosition);
					Metrics::WriteLogAnnotation(rpbuf);
				}
			}
		}
	}

	double newVariance = 0;
	bool shouldRapidCorrect = true;
	// Threshold below which the 2D Kabsch yaw solution is considered too ill-conditioned
	// to trust. Empirically: if min/max singular value < 0.05, the user moved on
	// roughly one axis only and the yaw is dominated by noise.
	const double RotationConditionMin = 0.05;
	if (!newCalibrationValid) {
		calibration = ComputeCalibration(ignoreOutliers);

		newVariance = ComputeAxisVariance(calibration)(1);
		Metrics::axisIndependence.Push(newVariance);
		Metrics::rotationConditionRatio.Push(m_rotationConditionRatio);

		if (newVariance < AxisVarianceThreshold && newVariance < m_axisVariance) {
			newCalibrationValid = false;
			shouldRapidCorrect = false;
			m_rejectReasonTag = "axis_variance_low";
		} else if (m_rotationConditionRatio < RotationConditionMin) {
			// Degenerate motion guard. ComputeCalibration above always updates
			// m_rotationConditionRatio for THIS tick — either to a real ratio when
			// the SVD ran, or to 0.0 when CalibrateRotation hit the empty-deltas
			// early return (no pair of samples with >23deg of rotation between
			// them). The previous gate was `> 0.0 && < RotationConditionMin`
			// which inadvertently SKIPPED itself in the empty-deltas case — i.e.
			// the worst possible conditioning silently passed. Let the gate trip
			// uniformly: any ratio below threshold (including exactly zero) means
			// the rotation solve is untrustworthy this tick.
			CalCtx.Log("Not updating: motion too planar (rotate around more axes)\n");
			newCalibrationValid = false;
			m_rejectReasonTag = m_rotationConditionRatio == 0.0
				? "rotation_no_deltas"
				: "rotation_planar";
		} else if (m_translationConditionRatio < RotationConditionMin) {
			// Mirror of the rotation guard. Same rationale — if the QR solve was
			// rank-deficient (or empty), don't trust the translation. Including
			// the explicit-zero case so empty-deltas doesn't bypass the gate.
			CalCtx.Log("Translation conditioning poor — need more diverse motion\n");
			newCalibrationValid = false;
			m_rejectReasonTag = m_translationConditionRatio == 0.0
				? "translation_no_deltas"
				: "translation_planar";
		} else {
			newCalibrationValid = ValidateCalibration(calibration, &newError, &m_posOffset);
			Metrics::posOffset_rawComputed.Push(m_posOffset * 1000);
			if (!newCalibrationValid) m_rejectReasonTag = "validate_failed";
		}

		if (m_isValid) {
			// Reject the new estimate if it's worse than what we already have, BUT
			// floor the comparison: once the prior is sub-mm, the gate `prior < new
			// * 1.5` becomes "new must be < prior/1.5" — i.e. better than tracker
			// jitter floor, which no honest sample can clear. Symptom in v2026.4.28.x
			// logs: error_currentCal converges to ~0.6mm, then every subsequent
			// sample is rejected, the stuck-loop watchdog spurious-fires every ~25s,
			// and the post-clear recompute often produces garbage. Floor at
			// kRejectionFloor — anything tighter than that is noise, not signal.
			//
			// Only run this check when newError is finite. If newError is INFINITY,
			// no candidate was actually computed this tick (the rotation- or
			// translation-condition guard above bailed out and never set newError),
			// so the comparison `effectivePrior < INFINITY * threshold` is trivially
			// true. Doing the rejection here in that case is harmless math-wise --
			// the candidate is already invalid -- but it OVERWRITES the more
			// specific reject_reason tag the upstream guard set, mislabeling
			// rotation_no_deltas / rotation_planar / translation_planar rows as
			// below_floor_or_worse in the debug log. Triage clarity is much better
			// when the tag reflects what actually fired.
			constexpr double kRejectionFloor = 0.005; // 5 mm
			if (std::isfinite(newError)) {
				const double effectivePrior = std::max(priorCalibrationError, kRejectionFloor);
				// Surface the value the gate actually used so a triage reader
				// can see why a small-looking prior didn't admit the candidate
				// (the floor clamps below 5 mm).
				Metrics::effectivePriorMm.Push(effectivePrior * 1000.0);
				if (effectivePrior < newError * threshold) {
					newCalibrationValid = false;
					shouldRapidCorrect = false;
					m_rejectReasonTag = "below_floor_or_worse";
				}
			}
		}

		Metrics::error_rawComputed.Push(newError * 1000);

		ComputeInstantOffset();
	}


	

#if 0
		char tmp[256];
		snprintf(tmp, sizeof tmp, "Prior calibration error: %.3f (valid: %s) sct %d; new error %.3f; new better? %s\n",
			priorCalibrationError, m_isValid ? "yes" : "no", stableCt, newError, !oldCalibrationBetter ? "yes" : "no");
		CalCtx.Log(tmp);
#endif
		
	
	// Now, can we use the relative pose to perform a rapid correction?
	if (!newCalibrationValid && shouldRapidCorrect) {
		
		double existingPoseErrorUsingRelPosition = RetargetingErrorRMS(m_refToTargetPose.translation(), m_estimatedTransformation);
		Metrics::error_currentCalRelPose.Push(existingPoseErrorUsingRelPosition * 1000);
		// Item #8: the second OR-arm of this condition was unreachable. The
		// surrounding block is gated on `if (!newCalibrationValid && shouldRapidCorrect)`,
		// so `newCalibrationValid` is guaranteed false here — the
		// `newCalibrationValid && relPoseError < newError` arm could never
		// fire. Dropped.
		if (relPoseError * threshold < existingPoseErrorUsingRelPosition) {
			newCalibrationValid = true;
			usingRelPose = true;
			newError = relPoseError;
			calibration = byRelPose;
		}
	}

	if (newCalibrationValid) {
		lerp = m_isValid;
		m_relativePosCalibrated = m_relativePosCalibrated || newError < 0.005;
		if (!m_isValid) {
			CalCtx.Log("Applying initial transformation...");
		}
		else if (m_relativePosCalibrated) {
			CalCtx.Log("Applying updated transformation...");
		} else {
			CalCtx.Log("Applying temporary transformation...");
		}

		// Publish-time blend. Two paths:
		//   - Default: single-step EMA at alpha = 0.3 between prior and candidate.
		//     Slow enough to suppress per-tick wobble, fast enough to track drift.
		//   - Opt-in (useBlendFilter): Kalman filter on (yaw, tx, ty, tz) with
		//     proper process and measurement covariances. State carried across
		//     ticks via m_blendFilter; resets on Clear() and on detected
		//     divergence (large per-component innovation -> graceful fallback
		//     to the EMA path for that tick).
		//
		// Skip both for rapid-correct (usingRelPose) since that path is supposed
		// to snap to a known-better solution.
		if (m_isValid && !usingRelPose) {
			bool emaThisCycle = !useBlendFilter;
			if (useBlendFilter) {
				// Extract scalar yaw of the candidate via swing-twist quaternion
				// decomposition (same form CalibrateRotation uses); the filter
				// runs on yaw-only because the calibration rotation is yaw-only.
				const Eigen::Quaterniond qCand(calibration.rotation());
				const Eigen::Quaterniond twistY(qCand.w(), 0.0, qCand.y(), 0.0);
				const double twistNorm = std::sqrt(twistY.w() * twistY.w() + twistY.y() * twistY.y());
				const double measYaw = (twistNorm > 1e-12)
					? 2.0 * std::atan2(twistY.y() / twistNorm, twistY.w() / twistNorm)
					: 0.0;
				const Eigen::Vector3d measT = calibration.translation();

				const double now = m_lastSampleTime;
				const double dt = std::max(0.0, now - m_blendFilterLastUpdateTime);
				m_blendFilterLastUpdateTime = now;

				double yawInnov = 0.0;
				double posInnov = 0.0;
				spacecal::blendfilter::Update(m_blendFilter,
					measYaw, measT.x(), measT.y(), measT.z(),
					dt, yawInnov, posInnov);

				if (spacecal::blendfilter::IsDivergent(yawInnov, posInnov)) {
					// Large innovation: filter does not expect this jump (post-
					// relocalize snap, geometry-shift recovery, etc.). Reset the
					// filter and fall through to the EMA path so the publish is
					// still smooth-ish for this one tick. The next accept will
					// reseed the filter from the new measurement.
					Metrics::WriteLogAnnotation(
						"blend_filter_divergence: resetting state and falling back to EMA");
					spacecal::blendfilter::Reset(m_blendFilter);
					emaThisCycle = true;
				} else {
					// Filter update produced a smoothed estimate; rebuild the
					// publish transform from the filter state.
					const Eigen::AngleAxisd yawAA(m_blendFilter.yaw, Eigen::Vector3d::UnitY());
					Eigen::AffineCompact3d blended;
					blended.linear() = yawAA.toRotationMatrix();
					blended.translation() = Eigen::Vector3d(
						m_blendFilter.tx, m_blendFilter.ty, m_blendFilter.tz);
					m_estimatedTransformation = blended;
				}
			}
			if (emaThisCycle) {
				const double alpha = 0.3;
				Eigen::Quaterniond priorQ(m_estimatedTransformation.rotation());
				Eigen::Quaterniond newQ(calibration.rotation());
				Eigen::Quaterniond blendedQ = priorQ.slerp(alpha, newQ);
				Eigen::Vector3d blendedT = m_estimatedTransformation.translation() * (1.0 - alpha) +
					calibration.translation() * alpha;
				Eigen::AffineCompact3d blended;
				blended.linear() = blendedQ.toRotationMatrix();
				blended.translation() = blendedT;
				m_estimatedTransformation = blended;
			}
		}
		else {
			m_estimatedTransformation = calibration; // first calibration or rapid-correct snap
			// First-accept: seed the Kalman filter from this measurement so the
			// next tick has a meaningful prior. Skipped on rapid-correct since
			// the snap may not be representative of steady-state.
			if (useBlendFilter && !usingRelPose) {
				const Eigen::Quaterniond qCand(calibration.rotation());
				const Eigen::Quaterniond twistY(qCand.w(), 0.0, qCand.y(), 0.0);
				const double twistNorm = std::sqrt(twistY.w() * twistY.w() + twistY.y() * twistY.y());
				const double measYaw = (twistNorm > 1e-12)
					? 2.0 * std::atan2(twistY.y() / twistNorm, twistY.w() / twistNorm)
					: 0.0;
				const Eigen::Vector3d measT = calibration.translation();
				double y = 0.0, p = 0.0;
				spacecal::blendfilter::Reset(m_blendFilter);
				spacecal::blendfilter::Update(m_blendFilter,
					measYaw, measT.x(), measT.y(), measT.z(),
					0.0, y, p);
				m_blendFilterLastUpdateTime = m_lastSampleTime;
			}
		}
		m_isValid = true;
		m_axisVariance = newVariance;

		if (!usingRelPose) {
			m_refToTargetPose = EstimateRefToTargetPose(m_estimatedTransformation);
		}

		Metrics::calibrationApplied.Push(!usingRelPose);

		m_consecutiveRejections = 0;
		m_lastSuccessfulIncrementalTime = m_lastSampleTime;
		m_rejectReasonTag.clear();
		Metrics::lastRejectReason.clear();
		m_healthyHoldAnnotated = false;

		return true;
	}
	else {
		// Stuck-loop watchdog: if we've been valid for a while but every recent attempt
		// to improve the calibration has been rejected, the prior estimate may be a
		// bad fixpoint that the 1.5x threshold gate is preventing us from leaving.
		// After enough consecutive rejections, demote to invalid + clear samples so
		// the overlay drops to ContinuousStandby and re-acquires from scratch.
		//
		// Health gate: don't fire when the prior calibration is already excellent.
		// A sub-cm prior with low live residual means rejection is the *correct*
		// behavior (new samples can't honestly improve it), not a stuck loop. Only
		// the combination of high prior error + sustained rejection signals a bad
		// fixpoint.
		//
		// Floor at kRejectionFloor (5 mm) — anything tighter than the per-tick
		// rejection floor IS the noise floor of the validate gate; demoting at
		// that point would just thrash. Anything *looser* (the previous 10 mm)
		// turned out to be a trap in real Quest Pro sessions: error_currentCal
		// plateaus at 5–12 mm with the 1.5× gate then refusing replacement
		// (need new < 3.3 mm to beat 5 mm prior — usually impossible at the
		// validate noise floor) and the healthy-skip preventing the watchdog
		// from clearing the bad fixpoint. Lowering this to match kRejectionFloor
		// lets the watchdog fire whenever the prior is provably above the noise
		// floor — i.e. when there *might* be room for an honest improvement
		// that the threshold gate can't admit.
		m_consecutiveRejections++;
		// Constants are now pinned in WatchdogDecisions.h so any future tuning
		// of the rejection cap or healthy-prior floor is forced through the
		// regression-test suite. See WatchdogDecisions.h doc comment for the
		// failure mode the boundary is calibrated against.
		const int MaxConsecutiveRejections = spacecal::watchdog::kMaxConsecutiveRejections;

		Metrics::consecutiveRejections.Push((double)m_consecutiveRejections);
		// Mirror the per-rejection tag into the metrics namespace so
		// WriteLogEntry's reject_reason column is filled this tick.
		Metrics::lastRejectReason = m_rejectReasonTag;

		const bool calibrationHealthy =
			spacecal::watchdog::IsCalibrationHealthy(m_isValid, priorCalibrationError);

		// Surface the wedged-at-noise-floor symptom as a per-tick metric.
		// Pushes 1.0 only when the watchdog WOULD have fired (we hit the
		// rejection cap on a valid prior) but skipped because the prior is
		// inside the healthy band. Otherwise 0.0. CSV grep `watchdogHealthySkip,1`
		// will reveal exactly the wedge-state ticks the user hit in the last
		// session — which the once-per-run annotation made invisible.
		const bool watchdogHealthySkipFiring =
			m_isValid && m_consecutiveRejections >= MaxConsecutiveRejections && calibrationHealthy;
		Metrics::watchdogHealthySkip.Push(watchdogHealthySkipFiring ? 1.0 : 0.0);

		if (m_isValid && m_consecutiveRejections >= MaxConsecutiveRejections) {
			if (calibrationHealthy) {
				// Don't blow up a perfectly good calibration; just stop incrementing
				// the rejection counter so we don't oscillate at the threshold.
				// Annotate ONCE per "healthy hold" run — we set a flag the first
				// time and clear it on accept. Without the flag, every tick at the
				// boundary would print this and the log would be noise.
				if (!m_healthyHoldAnnotated) {
					char buf[160];
					snprintf(buf, sizeof buf,
					         "watchdog_skipped: prior_error=%.2fmm — calibration healthy, rejecting noise",
					         priorCalibrationError * 1000.0);
					Metrics::WriteLogAnnotation(buf);
					m_healthyHoldAnnotated = true;
				}
				m_consecutiveRejections = MaxConsecutiveRejections - 5;
				m_rejectReasonTag = "healthy_below_floor";
			} else {
				CalCtx.Log("Continuous calibration appears stuck — recollecting samples\n");
				Metrics::WriteLogAnnotation("watchdog: continuous calibration stuck, clearing");
				m_isValid = false;
				m_watchdogResets++;
				Clear();
				// Clear() drops m_samples, resets m_consecutiveRejections, and zeroes
				// m_estimatedTransformation — see CalibrationCalc.cpp:118. Sample
				// buffer purge is essential: the stale samples that drove the
				// rejection are exactly what would re-feed a bad fixpoint on retry.
			}
		}

		return false;
	}
}
