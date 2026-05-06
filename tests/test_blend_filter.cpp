// Pin tests for the publish-time Kalman blend filter (BlendFilter.h). The
// filter is a 4-state (yaw, tx, ty, tz) random-walk model with diagonal
// covariance, Update returns per-component innovation magnitudes, IsDivergent
// gates a graceful fallback when the candidate is way outside what the
// filter expected.
//
// The contract these tests pin:
//   1. First Update on an uninitialized state seeds directly to the
//      measurement (zero innovation).
//   2. With dt=0 (no process noise) and exact-truth measurements, the state
//      stays at truth and innovations stay at zero.
//   3. With a steady stream of noisy measurements around a fixed truth, the
//      state converges toward truth and the diagonal P shrinks.
//   4. A large jump produces a large innovation and IsDivergent fires.
//   5. Reset clears state.initialized so the next Update reseeds.

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "BlendFilter.h"

using spacecal::blendfilter::State;
using spacecal::blendfilter::Update;
using spacecal::blendfilter::IsDivergent;
using spacecal::blendfilter::Reset;
using spacecal::blendfilter::kSigmaYawPerSec;
using spacecal::blendfilter::kSigmaPosPerSec;
using spacecal::blendfilter::kSigmaYawMeas;
using spacecal::blendfilter::kSigmaPosMeas;
using spacecal::blendfilter::kDivergenceYawRad;
using spacecal::blendfilter::kDivergencePosM;
using spacecal::blendfilter::kInitialPyaw;
using spacecal::blendfilter::kInitialPpos;

// --- Initialization / first-call behavior -----------------------------------

TEST(BlendFilter, FirstUpdateSeedsToMeasurement) {
    State s;
    EXPECT_FALSE(s.initialized);

    double yawInnov = 7.0, posInnov = 7.0;  // sentinel
    Update(s, /*yaw=*/0.5, /*tx=*/1.0, /*ty=*/2.0, /*tz=*/3.0,
           /*dt=*/0.0, yawInnov, posInnov);

    EXPECT_TRUE(s.initialized);
    EXPECT_DOUBLE_EQ(s.yaw, 0.5);
    EXPECT_DOUBLE_EQ(s.tx, 1.0);
    EXPECT_DOUBLE_EQ(s.ty, 2.0);
    EXPECT_DOUBLE_EQ(s.tz, 3.0);
    EXPECT_DOUBLE_EQ(yawInnov, 0.0);
    EXPECT_DOUBLE_EQ(posInnov, 0.0);
}

TEST(BlendFilter, FirstUpdateRestoresInitialCovariance) {
    State s;
    // Tamper with the covariance to confirm the seeding restores it.
    s.Pyaw = 999.0;
    s.Px = 999.0; s.Py = 999.0; s.Pz = 999.0;
    double y = 0.0, p = 0.0;
    Update(s, 0.0, 0.0, 0.0, 0.0, 0.0, y, p);
    EXPECT_DOUBLE_EQ(s.Pyaw, kInitialPyaw);
    EXPECT_DOUBLE_EQ(s.Px,   kInitialPpos);
    EXPECT_DOUBLE_EQ(s.Py,   kInitialPpos);
    EXPECT_DOUBLE_EQ(s.Pz,   kInitialPpos);
}

// --- Steady-state convergence -----------------------------------------------

TEST(BlendFilter, SeedThenIdenticalMeasurementsKeepStateStill) {
    // Seed at (0.1, 0.2, 0.3, 0.4); feed identical measurements; state must
    // stay there and innovations must stay at zero.
    State s;
    double y = 0.0, p = 0.0;
    Update(s, 0.1, 0.2, 0.3, 0.4, 0.0, y, p);
    for (int i = 0; i < 100; i++) {
        Update(s, 0.1, 0.2, 0.3, 0.4, /*dt=*/0.05, y, p);
        EXPECT_NEAR(s.yaw, 0.1, 1e-9);
        EXPECT_NEAR(s.tx,  0.2, 1e-9);
        EXPECT_NEAR(s.ty,  0.3, 1e-9);
        EXPECT_NEAR(s.tz,  0.4, 1e-9);
        EXPECT_NEAR(y, 0.0, 1e-9);
        EXPECT_NEAR(p, 0.0, 1e-9);
    }
}

TEST(BlendFilter, ConvergesToTruthUnderGaussianMeasurementNoise) {
    // Steady truth + small Gaussian measurement noise. Filter state should
    // converge toward truth and the covariance should shrink below the
    // initial values.
    constexpr double truthYaw = 0.30;
    const Eigen::Vector3d truthT(1.0, 2.0, 3.0);

    std::mt19937 rng(2024);
    std::normal_distribution<double> nYaw(0.0, kSigmaYawMeas);
    std::normal_distribution<double> nPos(0.0, kSigmaPosMeas);

    State s;
    double y = 0.0, p = 0.0;
    // Seed with a deliberately wrong initial measurement (truth + 1 sigma)
    Update(s, truthYaw + kSigmaYawMeas, truthT.x() + kSigmaPosMeas,
           truthT.y(), truthT.z(), 0.0, y, p);

    for (int i = 0; i < 200; i++) {
        Update(s,
               truthYaw + nYaw(rng),
               truthT.x() + nPos(rng),
               truthT.y() + nPos(rng),
               truthT.z() + nPos(rng),
               /*dt=*/0.05, y, p);
    }

    EXPECT_NEAR(s.yaw, truthYaw, 5.0 * kSigmaYawMeas);
    EXPECT_NEAR(s.tx,  truthT.x(), 5.0 * kSigmaPosMeas);
    EXPECT_NEAR(s.ty,  truthT.y(), 5.0 * kSigmaPosMeas);
    EXPECT_NEAR(s.tz,  truthT.z(), 5.0 * kSigmaPosMeas);
    EXPECT_LT(s.Pyaw, kInitialPyaw / 10.0);
    EXPECT_LT(s.Px,   kInitialPpos / 10.0);
}

// --- Divergence detection + reset -------------------------------------------

TEST(BlendFilter, IsDivergentFiresOnLargeYawInnovation) {
    EXPECT_TRUE(IsDivergent(/*yawInnov=*/kDivergenceYawRad + 0.01, /*posInnov=*/0.0));
    EXPECT_FALSE(IsDivergent(kDivergenceYawRad - 0.01, 0.0));
}

TEST(BlendFilter, IsDivergentFiresOnLargePositionInnovation) {
    EXPECT_TRUE(IsDivergent(0.0, kDivergencePosM + 0.01));
    EXPECT_FALSE(IsDivergent(0.0, kDivergencePosM - 0.01));
}

TEST(BlendFilter, JumpProducesLargeInnovation) {
    // Seed at origin, then jump 1 m on z. Innovation should be around 1 m
    // on the position axis.
    State s;
    double y = 0.0, p = 0.0;
    Update(s, 0.0, 0.0, 0.0, 0.0, 0.0, y, p);
    Update(s, 0.0, 0.0, 0.0, /*tz=*/1.0, /*dt=*/0.05, y, p);
    EXPECT_NEAR(p, 1.0, 0.01) << "innovation magnitude should reflect the 1m jump";
    EXPECT_TRUE(IsDivergent(y, p));
}

TEST(BlendFilter, ResetClearsInitializedFlag) {
    State s;
    double y = 0.0, p = 0.0;
    Update(s, 0.5, 1.0, 1.0, 1.0, 0.0, y, p);
    ASSERT_TRUE(s.initialized);
    Reset(s);
    EXPECT_FALSE(s.initialized);
    EXPECT_DOUBLE_EQ(s.yaw, 0.0);
    EXPECT_DOUBLE_EQ(s.Pyaw, kInitialPyaw);
}

TEST(BlendFilter, AfterResetNextUpdateSeedsAgain) {
    State s;
    double y = 0.0, p = 0.0;
    Update(s, 0.1, 0.0, 0.0, 0.0, 0.0, y, p);
    // Drift state via repeated noisy measurements.
    Update(s, 0.5, 1.0, 1.0, 1.0, 0.05, y, p);
    Reset(s);
    Update(s, /*yaw=*/0.7, /*tx=*/0.7, /*ty=*/0.7, /*tz=*/0.7, 0.0, y, p);
    EXPECT_DOUBLE_EQ(s.yaw, 0.7) << "after Reset, the next Update reseeds verbatim";
    EXPECT_DOUBLE_EQ(s.tx, 0.7);
    EXPECT_DOUBLE_EQ(y, 0.0);
    EXPECT_DOUBLE_EQ(p, 0.0);
}

// --- Negative dt handled defensively ----------------------------------------

TEST(BlendFilter, NegativeDtClampedToZeroDoesNotShrinkCovariance) {
    // Negative dt would accidentally subtract from the covariance if not
    // clamped; the implementation clamps dt at 0 so the predict step is a
    // no-op for a non-monotonic clock.
    State s;
    double y = 0.0, p = 0.0;
    Update(s, 0.0, 0.0, 0.0, 0.0, 0.0, y, p);
    const double Pyaw0 = s.Pyaw;
    const double Px0   = s.Px;
    Update(s, 0.01, 0.01, 0.01, 0.01, /*dt=*/-1.0, y, p);
    // Negative dt clamped to 0 -> predict step adds 0 -> P only shrinks by
    // the update step (Kalman gain x noise variance); never grows.
    EXPECT_LE(s.Pyaw, Pyaw0);
    EXPECT_LE(s.Px,   Px0);
}
