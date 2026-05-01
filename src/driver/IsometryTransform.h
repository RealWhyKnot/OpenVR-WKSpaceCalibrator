#pragma once 
#define EIGEN_MPL2_ONLY

#include <Eigen/Dense>

/**
 * Contains an isometric transformation, represented as the pair of a rotation quaternion and translation vector.
 * The translation is applied to the left of the quaternion.
 */
struct IsoTransform {
	Eigen::Quaterniond rotation;
	Eigen::Vector3d translation;

	IsoTransform() : rotation(Eigen::Quaterniond::Identity()), translation(Eigen::Vector3d::Zero()) {}
	// `explicit` is required: now that operator*(IsoTransform, Vector3d) uses
	// `rotation * p` directly (instead of materialising an Isometry3d), an
	// implicit Quaterniond -> IsoTransform conversion would make our overload
	// ambiguous with Eigen's RotationBase::operator*(EigenBase) at any call site
	// like `quat * vec3`. These single-arg constructors are not used implicitly
	// anywhere in the project.
	explicit IsoTransform(const Eigen::Quaterniond &rot) : rotation(rot), translation(Eigen::Vector3d::Zero()) {}
	explicit IsoTransform(const Eigen::Vector3d &trans) : rotation(Eigen::Quaterniond::Identity()), translation(trans) {}
	IsoTransform(const Eigen::Quaterniond& rot, const Eigen::Vector3d& trans) : rotation(rot), translation(trans) {}
	
	void pretranslate(const Eigen::Vector3d& t) {
		translation += t;
	}

	/**
	 * Interpolates between this transform and target. The position of localPoint after transformation will smoothly
	 * lerp between (this * localPoint) and (target * localPoint), despite rotation occurring around it.
	 */
	IsoTransform interpolateAround(double lerp, const IsoTransform& target, const Eigen::Vector3d& localPoint) const;
};

inline IsoTransform operator*(const IsoTransform& a, const IsoTransform& b) {
	// tA * rA * tB * rB = tA * (trans(rA * tB)) * rA * rB.
	// `Eigen::Quaterniond * Vector3d` rotates the vector directly without
	// materialising a 4x4 isometry; equivalent result, fewer allocations and
	// arithmetic ops on the pose-update hot path.
	//
	// Renormalise the composed rotation. Eigen's quaternion multiplication
	// does not auto-normalise. On the driver's pose-update hot path this
	// composes 1+ kHz across all tracked devices, with one of the operands
	// (device.transform.rotation, populated by interpolateAround's slerp)
	// itself the product of millions of prior compositions over a multi-hour
	// session. ULP-level scale drift compounds into a shear that downstream
	// SteamVR consumers — which don't always re-normalise either — render
	// as a subtle skew/wobble. Cost: one rsqrt + 4 multiplies per call, sub-
	// microsecond, lost in the noise of the hot path.
	auto rot = (a.rotation * b.rotation).normalized();
	Eigen::Vector3d trans = a.translation + a.rotation * b.translation;

	return IsoTransform(rot, trans);
}

inline Eigen::Vector3d operator*(const IsoTransform& a, const Eigen::Vector3d& p) {
	return a.translation + a.rotation * p;
}

inline IsoTransform IsoTransform::interpolateAround(double lerp, const IsoTransform& target, const Eigen::Vector3d& localPoint) const {
	auto initialPos = (*this) * localPoint;
	Eigen::Vector3d finalPos = initialPos * (1 - lerp) + (target * localPoint) * lerp;

	// slerp on unit quaternions returns a unit quaternion to within ULP, but
	// our inputs (this->rotation and target.rotation) accumulate error over
	// thousands of prior interpolateAround calls because we feed the result
	// back into the same field. Defensive normalise on the slerp output keeps
	// the running rotation pinned to the unit sphere indefinitely.
	auto newRotation = rotation.slerp(lerp, target.rotation).normalized();
	Eigen::Vector3d newTranslation = finalPos - newRotation * localPoint;

	return IsoTransform(newRotation, newTranslation);
}