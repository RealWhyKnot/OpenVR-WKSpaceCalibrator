// Starter unit tests for CalibrationCalc.
//
// These tests build synthetic sample sets that exactly satisfy the model
// CalibrationCalc assumes:
//
//     reference_pose * staticOffset = calibration * target_pose
//
// (See the long-form comment in CalibrationCalc.cpp above EstimateRefToTargetPose.)
//
// We pick a known calibration C, generate a random sequence of reference poses,
// and compute the corresponding target poses as `target = C^-1 * ref` (with
// staticOffset = identity for simplicity). The solver should then recover C
// from the sample stream.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <random>
#include <vector>

#include "CalibrationCalc.h"

namespace {

// Build an AffineCompact3d from a yaw-pitch-roll euler triple (in radians) and
// a translation. The order matches what CalibrationCalc effectively recovers
// (yaw-only on Y) so tests match the solver's coordinate convention.
Eigen::AffineCompact3d MakeTransform(double yawRad, double pitchRad, double rollRad,
                                     const Eigen::Vector3d& translation) {
    Eigen::Quaterniond q =
        Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(pitchRad, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(rollRad, Eigen::Vector3d::UnitZ());
    Eigen::AffineCompact3d t;
    t.linear() = q.toRotationMatrix();
    t.translation() = translation;
    return t;
}

// Convert an AffineCompact3d into a Pose (the type the solver consumes).
Pose AffineToPose(const Eigen::AffineCompact3d& a) {
    Pose p;
    p.rot = a.rotation();
    p.trans = a.translation();
    return p;
}

// Generate one sample pair such that calibration * targetPose == refPose.
// The reference pose is random; the target pose is the same physical event
// expressed in the un-calibrated target space. timestamp is monotonically
// increasing so PushSample's lastSampleTime stays well-formed.
Sample MakeSample(const Eigen::AffineCompact3d& refPose,
                  const Eigen::AffineCompact3d& calibration,
                  double timestamp) {
    Eigen::AffineCompact3d targetPose = calibration.inverse() * refPose;
    return Sample(AffineToPose(refPose), AffineToPose(targetPose), timestamp);
}

// Generate `numSamples` reference poses with random rotation on all three axes
// and small random translation, then return synthetic Sample pairs satisfying
// the given calibration. Uses a fixed seed for reproducibility.
std::vector<Sample> MakeSamplePairs(const Eigen::AffineCompact3d& calibration,
                                    int numSamples,
                                    uint32_t seed = 42) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> angleDist(-EIGEN_PI, EIGEN_PI);
    std::uniform_real_distribution<double> transDist(-1.0, 1.0);

    std::vector<Sample> samples;
    samples.reserve(numSamples);
    for (int i = 0; i < numSamples; i++) {
        Eigen::AffineCompact3d refPose = MakeTransform(
            angleDist(rng), angleDist(rng) * 0.5, angleDist(rng) * 0.5,
            Eigen::Vector3d(transDist(rng), transDist(rng), transDist(rng)));
        samples.push_back(MakeSample(refPose, calibration, /*timestamp=*/i * 0.01));
    }
    return samples;
}

// As MakeSamplePairs, but rotations are exclusively around the Y axis. This
// is the strict "user only spun around Y" degenerate case: every delta-axis
// projects to (0, 0) in the xz plane, the Kabsch cross-covariance is the
// zero matrix, and the recovered yaw is meaningless (typically 0 deg
// regardless of the true calibration).
std::vector<Sample> MakeYOnlySamples(const Eigen::AffineCompact3d& calibration,
                                     int numSamples,
                                     uint32_t seed = 42) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> angleDist(-EIGEN_PI, EIGEN_PI);
    std::uniform_real_distribution<double> transDist(-1.0, 1.0);

    std::vector<Sample> samples;
    samples.reserve(numSamples);
    for (int i = 0; i < numSamples; i++) {
        Eigen::AffineCompact3d refPose = MakeTransform(
            angleDist(rng), 0.0, 0.0,
            Eigen::Vector3d(transDist(rng), transDist(rng), transDist(rng)));
        samples.push_back(MakeSample(refPose, calibration, /*timestamp=*/i * 0.01));
    }
    return samples;
}

// Compute the rotation discrepancy in degrees between two affine transforms.
double RotationErrorDegrees(const Eigen::AffineCompact3d& a, const Eigen::AffineCompact3d& b) {
    Eigen::Matrix3d delta = a.rotation() * b.rotation().transpose();
    // angle = acos((trace - 1) / 2), clamped for numerical safety.
    double trace = delta.trace();
    double cosAngle = std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0));
    return std::acos(cosAngle) * 180.0 / EIGEN_PI;
}

// Number of synthetic samples used by every test. Large enough that the
// least-squares solves are well-determined; small enough that the suite stays
// fast (~100ms per test).
constexpr int kSampleCount = 60;

} // namespace

// ---------------------------------------------------------------------------
// Identity calibration: feed samples with target == reference. The recovered
// transform should be (numerically) the identity.
// ---------------------------------------------------------------------------
TEST(CalibrationCalcTest, RecoversIdentity) {
    Eigen::AffineCompact3d expected = Eigen::AffineCompact3d::Identity();

    CalibrationCalc calc;
    for (auto& s : MakeSamplePairs(expected, kSampleCount)) {
        calc.PushSample(s);
    }

    ASSERT_TRUE(calc.ComputeOneshot(/*ignoreOutliers=*/false));
    auto recovered = calc.Transformation();

    EXPECT_LT(recovered.translation().norm(), 1e-3)
        << "Recovered translation: " << recovered.translation().transpose();
    EXPECT_LT(RotationErrorDegrees(recovered, expected), 0.5);
}

// ---------------------------------------------------------------------------
// Pure 30 deg yaw: target frame is the reference frame rotated 30 deg around
// the world Y axis. Solver should recover the yaw, with near-zero translation.
// ---------------------------------------------------------------------------
TEST(CalibrationCalcTest, RecoversPureYaw) {
    const double yawRad = 30.0 * EIGEN_PI / 180.0;
    Eigen::AffineCompact3d expected = MakeTransform(yawRad, 0, 0, Eigen::Vector3d::Zero());

    CalibrationCalc calc;
    for (auto& s : MakeSamplePairs(expected, kSampleCount)) {
        calc.PushSample(s);
    }

    ASSERT_TRUE(calc.ComputeOneshot(/*ignoreOutliers=*/false));
    auto recovered = calc.Transformation();

    EXPECT_LT(recovered.translation().norm(), 5e-3)
        << "Recovered translation: " << recovered.translation().transpose();
    EXPECT_LT(RotationErrorDegrees(recovered, expected), 0.5);
}

// ---------------------------------------------------------------------------
// Pure translation, no rotation: target frame is offset by (1, 0, 2) m.
// Solver should recover that offset; rotation near identity.
// ---------------------------------------------------------------------------
TEST(CalibrationCalcTest, RecoversPureTranslation) {
    Eigen::Vector3d trans(1.0, 0.0, 2.0);
    Eigen::AffineCompact3d expected = MakeTransform(0, 0, 0, trans);

    CalibrationCalc calc;
    for (auto& s : MakeSamplePairs(expected, kSampleCount)) {
        calc.PushSample(s);
    }

    ASSERT_TRUE(calc.ComputeOneshot(/*ignoreOutliers=*/false));
    auto recovered = calc.Transformation();

    EXPECT_LT((recovered.translation() - trans).norm(), 5e-3)
        << "Recovered translation: " << recovered.translation().transpose()
        << ", expected: " << trans.transpose();
    EXPECT_LT(RotationErrorDegrees(recovered, expected), 0.5);
}

// ---------------------------------------------------------------------------
// Combined 10 deg yaw plus a (0.5, 0.1, -0.3) m offset. The solver should
// recover both within tolerance from the same sample stream.
// ---------------------------------------------------------------------------
TEST(CalibrationCalcTest, RecoversCombinedOffset) {
    const double yawRad = 10.0 * EIGEN_PI / 180.0;
    Eigen::Vector3d trans(0.5, 0.1, -0.3);
    Eigen::AffineCompact3d expected = MakeTransform(yawRad, 0, 0, trans);

    CalibrationCalc calc;
    for (auto& s : MakeSamplePairs(expected, kSampleCount)) {
        calc.PushSample(s);
    }

    ASSERT_TRUE(calc.ComputeOneshot(/*ignoreOutliers=*/false));
    auto recovered = calc.Transformation();

    EXPECT_LT((recovered.translation() - trans).norm(), 1e-2)
        << "Recovered translation: " << recovered.translation().transpose()
        << ", expected: " << trans.transpose();
    EXPECT_LT(RotationErrorDegrees(recovered, expected), 1.0);
}

// ---------------------------------------------------------------------------
// Degenerate single-axis motion: every sample's reference rotation is around
// the Y axis only. The 2D Kabsch sees no spread in xz and cannot identify the
// true yaw (the cross-covariance is the zero matrix). The solver itself
// happily returns *some* result, so the meaningful contract is that the
// recovered transform does NOT match the true calibration -- distinguishing
// "the solver got lucky" from "the solver had enough information to know".
//
// We also assert that the rotation-condition diagnostic was set to 0.0,
// which is what tells ComputeIncremental's guard that the input was fully
// degenerate. (The `>0.0` half of the guard is what catches *partially*
// degenerate motion in production; the >=0.05 half catches well-conditioned
// motion. Pure single-axis is the boundary case where the guard's `>0.0`
// short-circuits.)
// ---------------------------------------------------------------------------
TEST(CalibrationCalcTest, DegenerateMotionDoesNotRecoverYaw) {
    const double yawRad = 15.0 * EIGEN_PI / 180.0;
    Eigen::AffineCompact3d expected = MakeTransform(yawRad, 0, 0, Eigen::Vector3d::Zero());

    CalibrationCalc calc;
    for (auto& s : MakeYOnlySamples(expected, kSampleCount)) {
        calc.PushSample(s);
    }

    // ComputeOneshot may or may not pass ValidateCalibration depending on
    // numerical noise; what we really care about is that it does NOT recover
    // 15 deg of yaw, because the sample set has zero information about the
    // true cross-axis orientation.
    (void)calc.ComputeOneshot(/*ignoreOutliers=*/false);
    auto recovered = calc.Transformation();

    SCOPED_TRACE("rotationConditionRatio = " +
                 std::to_string(calc.m_rotationConditionRatio));

    // Recovered yaw must be far enough from 15 deg that it could not have
    // been a successful recovery. We allow a 5 deg slack to keep the test
    // numerically stable across compiler/arch variation.
    EXPECT_GT(RotationErrorDegrees(recovered, expected), 5.0)
        << "Solver should not have recovered the true yaw from single-axis "
           "samples";

    // Diagnostic: the 2D Kabsch covariance is exactly zero, so the
    // condition-ratio metric reports 0.0 (smax == 0 special-case in
    // CalibrateRotation). This is what makes the in-production
    // `m_rotationConditionRatio > 0.0` precondition gate further checks.
    EXPECT_DOUBLE_EQ(calc.m_rotationConditionRatio, 0.0)
        << "Pure single-axis motion should yield a zero-rank covariance";
}

// ---------------------------------------------------------------------------
// Outlier injection: well-behaved samples plus a handful of deliberately
// corrupted entries (random poses unrelated to the true calibration). With
// ignoreOutliers=true, the iterated DetectOutliers pass should keep the
// recovered rotation/translation close to ground truth — verified against
// the underlying Transformation directly so we don't conflate the math with
// the cross-sample RMS validator (which sees outliers regardless of the
// rotation/translation solver's filtering).
// ---------------------------------------------------------------------------
TEST(CalibrationCalcTest, IgnoresOutliers) {
    const double yawRad = 20.0 * EIGEN_PI / 180.0;
    Eigen::Vector3d trans(0.2, 0.0, 0.4);
    Eigen::AffineCompact3d expected = MakeTransform(yawRad, 0, 0, trans);

    // Use more samples than the clean tests so the RMS validator stays under
    // the 0.1 m gate even with the outlier translations contributing to the
    // sum-of-squares before DetectOutliers strips them from the rotation
    // solve.
    const int outlierTestSamples = 200;
    auto samples = MakeSamplePairs(expected, outlierTestSamples);

    // Corrupt ~5% of the samples with corrupted rotation axes. We perturb
    // only the rotation (not translation) because the validator's RMS metric
    // is purely positional — large translation outliers would push the test
    // past the validate gate even with perfect rotation/translation
    // recovery, conflating "solver doesn't handle outliers" with "validator
    // can't tolerate them". Rotation outliers exercise DetectOutliers'
    // axis-comparison logic, which is its real job.
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> angleDist(-EIGEN_PI, EIGEN_PI);
    int corruptedCount = 0;
    for (size_t i = 0; i < samples.size(); i += 20) {
        // Replace the target rotation with a random rotation. The translation
        // is preserved so validator RMS error stays bounded.
        Eigen::Quaterniond randomRot =
            Eigen::AngleAxisd(angleDist(rng), Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(angleDist(rng), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(angleDist(rng), Eigen::Vector3d::UnitZ());
        samples[i].target.rot = randomRot.toRotationMatrix();
        corruptedCount++;
    }
    ASSERT_GT(corruptedCount, 0);

    CalibrationCalc calc;
    for (auto& s : samples) {
        calc.PushSample(s);
    }

    ASSERT_TRUE(calc.ComputeOneshot(/*ignoreOutliers=*/true));
    auto recovered = calc.Transformation();

    // Looser tolerances than the clean cases — outlier rejection won't get us
    // back to numerical exact, but a few cm / 2 deg is achievable when the
    // outlier set is small enough for DetectOutliers to converge.
    EXPECT_LT((recovered.translation() - trans).norm(), 5e-2)
        << "Recovered translation: " << recovered.translation().transpose()
        << ", expected: " << trans.transpose();
    EXPECT_LT(RotationErrorDegrees(recovered, expected), 2.0);
}
