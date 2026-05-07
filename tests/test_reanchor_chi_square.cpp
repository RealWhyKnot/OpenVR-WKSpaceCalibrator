// Tests for the chi-square re-anchor sub-detector. Pins the rolling velocity
// estimate, the running variance update, and the Mahalanobis-distance gate.

#include <gtest/gtest.h>

#include "ReanchorChiSquareDetector.h"

#include <cmath>

using spacecal::reanchor_chi::DetectorState;
using spacecal::reanchor_chi::PushPose;
using spacecal::reanchor_chi::Reset;
using spacecal::reanchor_chi::TickAndCheckCandidate;
using spacecal::reanchor_chi::IsFrozen;
using spacecal::reanchor_chi::EstimateLinearVelocity;
using spacecal::reanchor_chi::PredictTranslation;
using spacecal::reanchor_chi::MahalanobisSquared;
using spacecal::reanchor_chi::kChiSquare6DoF_p1e4;
using spacecal::reanchor_chi::kFreezeWindowSec;

namespace {

Eigen::Quaterniond Identity() { return Eigen::Quaterniond::Identity(); }

void Warmup(DetectorState& s, double startNow, int ticks) {
    // Push a steady-velocity stream so warmup completes without firing.
    const double dt = 0.05;
    for (int i = 0; i < ticks; ++i) {
        const double t = startNow + i * dt;
        const Eigen::Vector3d pos(0.01 * i, 0.0, 0.0);
        TickAndCheckCandidate(s, pos, Identity(), t, dt);
    }
}

} // namespace

TEST(ReanchorChiSquareTest, EarlyTicksDoNotFire) {
    DetectorState s;
    const double dt = 0.05;
    for (int i = 0; i < 3; ++i) {
        const Eigen::Vector3d pos(0.01 * i, 0.0, 0.0);
        EXPECT_FALSE(TickAndCheckCandidate(s, pos, Identity(), i * dt, dt));
    }
}

TEST(ReanchorChiSquareTest, SteadyMotionDoesNotFire) {
    DetectorState s;
    Warmup(s, 0.0, 50);

    const double dt = 0.05;
    bool anyFired = false;
    for (int i = 50; i < 100; ++i) {
        const Eigen::Vector3d pos(0.01 * i, 0.0, 0.0);
        if (TickAndCheckCandidate(s, pos, Identity(), i * dt, dt)) anyFired = true;
    }
    EXPECT_FALSE(anyFired)
        << "Steady linear motion must not trip the chi-square gate";
}

TEST(ReanchorChiSquareTest, SuddenJumpFires) {
    DetectorState s;
    Warmup(s, 0.0, 50);

    // 50 cm jump on the next tick. Chi-square against the warmed-up
    // variance (which has been seeing tiny residuals from steady motion)
    // should be very high.
    const double dt = 0.05;
    const Eigen::Vector3d jumpedPos(0.01 * 50 + 0.50, 0.0, 0.0);
    EXPECT_TRUE(TickAndCheckCandidate(s, jumpedPos, Identity(), 50 * dt, dt));
}

TEST(ReanchorChiSquareTest, FreezeWindowAfterFire) {
    DetectorState s;
    Warmup(s, 0.0, 50);

    const double dt = 0.05;
    const Eigen::Vector3d jumped(0.01 * 50 + 0.50, 0.0, 0.0);
    const double tFire = 50 * dt;
    ASSERT_TRUE(TickAndCheckCandidate(s, jumped, Identity(), tFire, dt));
    EXPECT_TRUE(IsFrozen(s, tFire + 0.1));
    EXPECT_TRUE(IsFrozen(s, tFire + 0.4));
    EXPECT_FALSE(IsFrozen(s, tFire + kFreezeWindowSec + 0.01));
}

TEST(ReanchorChiSquareTest, MahalanobisGreaterThanThresholdFires) {
    Eigen::Matrix<double, 6, 1> resid;
    resid << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, 6, 1> var;
    var << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;
    const double m = MahalanobisSquared(resid, var);
    EXPECT_GT(m, kChiSquare6DoF_p1e4);
}

TEST(ReanchorChiSquareTest, MahalanobisLowVarianceProducesHighScore) {
    Eigen::Matrix<double, 6, 1> resid;
    resid << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, 6, 1> varSmall;
    varSmall << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    Eigen::Matrix<double, 6, 1> varLarge;
    varLarge << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    const double mSmall = MahalanobisSquared(resid, varSmall);
    const double mLarge = MahalanobisSquared(resid, varLarge);
    EXPECT_GT(mSmall, mLarge);
}

TEST(ReanchorChiSquareTest, ResetClearsState) {
    DetectorState s;
    Warmup(s, 0.0, 30);
    EXPECT_GT(s.historyCount, 0u);
    Reset(s);
    EXPECT_EQ(s.historyCount, 0u);
    EXPECT_FALSE(IsFrozen(s, 0.0));
}
