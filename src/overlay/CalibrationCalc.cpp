#include "CalibrationCalc.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "Protocol.h"

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
		return acos((rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) / 2.0);
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
	m_axisVariance = 0.0;
	m_refToTargetPose = Eigen::AffineCompact3d::Identity();
	m_relativePosCalibrated = false;
	m_rotationConditionRatio = 0.0;
	m_consecutiveRejections = 0;
	// `m_lastSampleTime` and `m_lastSuccessfulIncrementalTime` deliberately retained
	// across Clear() so the watchdog can still see the gap if continuous calibration
	// is restarted faster than fresh samples can be collected.
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

		// Average via least-squares on the 4-vector. We follow the original solver
		// shape (identity coefficients on a Nx4 system) so the math stays equivalent
		// to the prior single-pass implementation when MaxIters == 1.
		Eigen::MatrixXd coefficients(validCount * 4, 4);
		Eigen::VectorXd constraints(validCount * 4);
		size_t row = 0;
		for (size_t i = 0; i < m_samples.size(); i++) {
			if (!valids[i]) continue;
			coefficients.block<4, 4>(4 * row, 0) = Eigen::Matrix4d::Identity();
			constraints.block<4, 1>(4 * row, 0) = Eigen::Vector4d(
				perSampleQuat[i].w(), perSampleQuat[i].x(), perSampleQuat[i].y(), perSampleQuat[i].z());
			row++;
		}
		Eigen::Vector4d result = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constraints);
		Eigen::Quaterniond quatExt(result(0), result(1), result(2), result(3));
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

	// Kabsch algorithm

	// Initialize 2D points and centroids
	Eigen::MatrixXd refPoints(deltas.size(), 2), targetPoints(deltas.size(), 2);
	Eigen::Vector2d refCentroid(0, 0), targetCentroid(0, 0);

	// Fill matrices and calculate centroids
	for (size_t i = 0; i < deltas.size(); i++) {
		refPoints.row(i) << deltas[i].ref[0], deltas[i].ref[2];  // Take only the x and z components
		refCentroid += refPoints.row(i);

		targetPoints.row(i) << deltas[i].target[0], deltas[i].target[2];  // Take only the x and z components
		targetCentroid += targetPoints.row(i);
	}

	refCentroid /= (double)deltas.size();
	targetCentroid /= (double)deltas.size();

	// Center the points
	for (size_t i = 0; i < deltas.size(); i++) {
		refPoints.row(i) -= refCentroid;
		targetPoints.row(i) -= targetCentroid;
	}

	// Calculate cross-covariance matrix
	auto crossCV = refPoints.transpose() * targetPoints;

	// Singular Value Decomposition (SVD)
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Record the condition ratio (min/max singular value) of the 2D covariance.
	// Near-zero means the rotation deltas all lie close to a single line in the
	// yaw plane — i.e., the user only rotated in one axis. The yaw solution is
	// then ill-conditioned and ComputeIncremental will reject it.
	{
		const auto& sv = svd.singularValues();
		double smax = sv.size() > 0 ? sv(0) : 0.0;
		double smin = sv.size() > 1 ? sv(sv.size() - 1) : 0.0;
		m_rotationConditionRatio = (smax > 0.0) ? (smin / smax) : 0.0;
	}

	// Calculate 2D rotation matrix
	Eigen::Matrix2d i = Eigen::Matrix2d::Identity();
	Eigen::Matrix2d rot = svd.matrixV() * i * svd.matrixU().transpose();

	// Calculate yaw angle in radians
	double yaw = std::atan2(rot(1, 0), rot(0, 0));

	// Convert to degrees
	Eigen::Vector3d euler(0.0, yaw * 180.0 / EIGEN_PI, 0.0);

	//snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
	//CalCtx.Log(buf);
	return euler;
}

Eigen::Vector3d CalibrationCalc::CalibrateTranslation(const Eigen::Matrix3d &rotation) const
{
	// Each delta-pair carries an associated weight derived from the smaller of the
	// two rotation magnitudes between samples i and j. Pairs with tiny rotation
	// contribute mostly noise to the translation solve (the LS rows are small),
	// so we down-weight them with sqrt(weight) row-scaling.
	struct DeltaRow {
		Eigen::Vector3d c;
		Eigen::Matrix3d q;
		double weight;
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
			double refA = AngleFromRotationMatrix3(s_i.ref.rot * s_j.ref.rot.transpose());
			double targetA = AngleFromRotationMatrix3(s_i.target.rot * s_j.target.rot.transpose());
			double weight = std::min(refA, targetA);
			if (!std::isfinite(weight) || weight < 0.01) weight = 0.01;

			auto QAi = s_i.ref.rot.transpose();
			auto QAj = s_j.ref.rot.transpose();
			auto dQA = QAj - QAi;
			auto CA = QAj * (s_j.ref.trans - s_j.target.trans) - QAi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back({ CA, dQA, weight });

			auto QBi = s_i.target.rot.transpose();
			auto QBj = s_j.target.rot.transpose();
			auto dQB = QBj - QBi;
			auto CB = QBj * (s_j.ref.trans - s_j.target.trans) - QBi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back({ CB, dQB, weight });
		}
	}

	Eigen::VectorXd constants(deltas.size() * 3);
	Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		double sw = std::sqrt(deltas[i].weight);
		for (int axis = 0; axis < 3; axis++)
		{
			constants(i * 3 + axis) = deltas[i].c(axis) * sw;
			coefficients.row(i * 3 + axis) = deltas[i].q.row(axis) * sw;
		}
	}

	Eigen::Vector3d trans = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
	auto transcm = trans * 100.0;

	//char buf[256];
	//snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
	//CalCtx.Log(buf);
	return trans;
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

	return sqrt(errorAccum / sampleCount);
}

double CalibrationCalc::ReferenceJitter() const {
	Eigen::Vector3d m_oldM, m_newM, m_oldS, m_newS;
	int sampleCount = 0;

	for (auto& sample : m_samples) {
		if (!sample.valid) continue;

		if (sampleCount == 0) {
			m_oldM = m_newM = sample.ref.trans;
			m_oldS = Eigen::Vector3d();
		} else {
			m_newM = m_oldM + (sample.ref.trans - m_oldM) / sampleCount;
			m_newS = m_oldS + (sample.ref.trans - m_oldM).cwiseProduct(sample.ref.trans - m_newM);

			// set up for next iteration
			m_oldM = m_newM;
			m_oldS = m_newS;
		}

		sampleCount++;
	}

	double var_x = sqrt(((sampleCount > 1) ? m_newS.x() / (sampleCount - 1) : 0.0));
	double var_y = sqrt(((sampleCount > 1) ? m_newS.y() / (sampleCount - 1) : 0.0));
	double var_z = sqrt(((sampleCount > 1) ? m_newS.z() / (sampleCount - 1) : 0.0));

	// Take magnitude of standard deviation vector
	return sqrt(var_x * var_x + var_y * var_y + var_z * var_z);
}

double CalibrationCalc::TargetJitter() const {
	Eigen::Vector3d m_oldM, m_newM, m_oldS, m_newS;
	int sampleCount = 0;

	for (auto& sample : m_samples) {
		if (!sample.valid) continue;

		if (sampleCount == 0) {
			m_oldM = m_newM = sample.target.trans;
			m_oldS = Eigen::Vector3d();
		}
		else {
			m_newM = m_oldM + (sample.target.trans - m_oldM) / sampleCount;
			m_newS = m_oldS + (sample.target.trans - m_oldM).cwiseProduct(sample.target.trans - m_newM);

			// set up for next iteration
			m_oldM = m_newM;
			m_oldS = m_newS;
		}

		sampleCount++;
	}

	double var_x = sqrt(((sampleCount > 1) ? std::abs(m_newS.x() / (sampleCount - 1)) : 0.0));
	double var_y = sqrt(((sampleCount > 1) ? std::abs(m_newS.y() / (sampleCount - 1)) : 0.0));
	double var_z = sqrt(((sampleCount > 1) ? std::abs(m_newS.z() / (sampleCount - 1)) : 0.0));

	// Take magnitude of standard deviation vector
	return sqrt(var_x * var_x + var_y * var_y + var_z * var_z);
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
	if (rmsError > 0.1) ok = false;

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



bool CalibrationCalc::ComputeOneshot(const bool ignoreOutliers) {
	auto calibration = ComputeCalibration(ignoreOutliers);

	bool valid = ValidateCalibration(calibration);

	if (valid) {
		m_estimatedTransformation = calibration; // @NOTE: Normal calibration
		m_isValid = true;
		return true;
	}
	else {
		CalCtx.Log("Not updating: Low-quality calibration result\n");
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
		} else if (m_rotationConditionRatio > 0.0 && m_rotationConditionRatio < RotationConditionMin) {
			// Degenerate motion: insufficient yaw-plane axis spread. Don't update
			// rotation; keep the prior calibration (if any). Allow rapid-correct via
			// relative pose since that path doesn't depend on this conditioning.
			CalCtx.Log("Insufficient motion variety (single-axis rotation) — holding rotation\n");
			newCalibrationValid = false;
		} else {
			newCalibrationValid = ValidateCalibration(calibration, &newError, &m_posOffset);
			Metrics::posOffset_rawComputed.Push(m_posOffset * 1000);
		}

		if (m_isValid) {
			if (priorCalibrationError < newError * threshold) {
				// If we have a more noisy calibration than before, avoid updating.
				newCalibrationValid = false;
				shouldRapidCorrect = false;
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
		if (relPoseError * threshold < existingPoseErrorUsingRelPosition || newCalibrationValid && relPoseError < newError) {
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

		// Apply a single-step EMA on the published transform when we already had a
		// valid prior estimate. The 1.5x threshold gate prevents most bad updates
		// from being accepted, but accepted updates still carry per-cycle noise
		// that's visible as small jitter on attached objects. Alpha = 0.3 weights
		// the new estimate at 30% — fast enough for legitimate drift correction,
		// slow enough to suppress per-tick wobble. Skip for rapid-correct (usingRelPose)
		// since that path is supposed to snap to a known-better solution.
		if (m_isValid && !usingRelPose) {
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
		else {
			m_estimatedTransformation = calibration; // first calibration or rapid-correct snap
		}
		m_isValid = true;
		m_axisVariance = newVariance;

		if (!usingRelPose) {
			m_refToTargetPose = EstimateRefToTargetPose(m_estimatedTransformation);
		}

		Metrics::calibrationApplied.Push(!usingRelPose);

		m_consecutiveRejections = 0;
		m_lastSuccessfulIncrementalTime = m_lastSampleTime;

		return true;
	}
	else {
		// Stuck-loop watchdog: if we've been valid for a while but every recent attempt
		// to improve the calibration has been rejected, the prior estimate may be a
		// bad fixpoint that the 1.5x threshold gate is preventing us from leaving.
		// After enough consecutive rejections, demote to invalid + clear samples so
		// the overlay drops to ContinuousStandby and re-acquires from scratch.
		m_consecutiveRejections++;
		const int MaxConsecutiveRejections = 50; // ~25s at the post-buffer-fill cadence

		Metrics::consecutiveRejections.Push((double)m_consecutiveRejections);

		if (m_isValid && m_consecutiveRejections >= MaxConsecutiveRejections) {
			CalCtx.Log("Continuous calibration appears stuck — recollecting samples\n");
			Metrics::WriteLogAnnotation("watchdog: continuous calibration stuck, clearing");
			m_isValid = false;
			m_watchdogResets++;
			Clear();
			// Clear() resets m_consecutiveRejections; ensure we don't immediately retrigger.
		}

		return false;
	}
}
