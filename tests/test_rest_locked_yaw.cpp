// Tests for the rest-locked yaw drift correction. Pins the per-tracker phase
// machine (Moving / RecentlyAtRest / AtRest), the bounded-rate cap, and the
// weighted aggregation across multiple at-rest trackers.
//
// Phase 1+2 (DELETED 2026-04-29 at main 9fab09d) failed because its accept
// gate was tautological: ComputeOneshot is mathematically guaranteed to find
// a candidate with RMS <= the prior on the SAME stationary buffer, so
// "accept if better" never rejected. The new design's gate is per-tracker
// rotation-dwell against an absolute reference (the orientation 1 s ago),
// not buffer-RMS comparison; it cannot fool itself the way Phase 1+2 did.
// The test below pins the analogous failure mode by feeding 1000 zero-delta
// ticks and asserting the locked reference does not slide.

#include <gtest/gtest.h>

#include "RestLockedYaw.h"

#include <cmath>

using spacecal::rest_yaw::RestState;
using spacecal::rest_yaw::RestPhase;
using spacecal::rest_yaw::UpdatePhase;
using spacecal::rest_yaw::ApplyBoundedYawStep;
using spacecal::rest_yaw::SignedYawDeltaRad;
using spacecal::rest_yaw::FuseYawContributionsRad;
using spacecal::rest_yaw::YawContribution;
using spacecal::rest_yaw::TrackingSystemClass;
using spacecal::rest_yaw::CapForClass;
using spacecal::rest_yaw::RateCaps;
using spacecal::rest_yaw::ClassWeight;
using spacecal::rest_yaw::AgeWeight;
using spacecal::rest_yaw::QualityWeight;

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Quaterniond YawQuat(double yawDeg) {
    const double half = (yawDeg * kPi / 180.0) * 0.5;
    return Eigen::Quaterniond(std::cos(half), 0.0, std::sin(half), 0.0);
}

} // namespace

TEST(RestLockedYawTest, PhaseMachineEntersAtRestAfterDwell) {
    RestState s;
    Eigen::Quaterniond q = YawQuat(0.0);
    s = UpdatePhase(s, q, 0.0, 0.0);
    EXPECT_EQ(s.phase, RestPhase::Moving);
    EXPECT_FALSE(s.haveLock);

    // Hold still for 1.0 s -> RecentlyAtRest after one tick, AtRest after the
    // dwell elapses. Sample at 20 Hz: 21 ticks across 1.0 s.
    for (int i = 1; i <= 21; ++i) {
        s = UpdatePhase(s, q, i * 0.05, 0.05);
    }
    EXPECT_EQ(s.phase, RestPhase::AtRest);
    EXPECT_TRUE(s.haveLock);
}

TEST(RestLockedYawTest, PhaseMachineExitsOnMotion) {
    RestState s;
    Eigen::Quaterniond rest = YawQuat(0.0);
    for (int i = 0; i <= 21; ++i) {
        s = UpdatePhase(s, rest, i * 0.05, 0.05);
    }
    ASSERT_EQ(s.phase, RestPhase::AtRest);
    ASSERT_TRUE(s.haveLock);

    // Single tick at 10 deg yaw rotation (200 deg/s at 20 Hz, above the 120
    // deg/s threshold) -> phase resets to Moving, lock dropped.
    s = UpdatePhase(s, YawQuat(10.0), 1.10, 0.05);
    EXPECT_EQ(s.phase, RestPhase::Moving);
    EXPECT_FALSE(s.haveLock);
}

TEST(RestLockedYawTest, BoundedRateCapClampsExtremeError) {
    // 90 deg error in radians, dt = 0.05 s (one 20 Hz tick), cap = 0.15 deg/s.
    // Worst-case step = 0.15 * 0.05 = 0.0075 deg = 0.000131 rad. The clamped
    // step must equal the cap regardless of how large the input error is.
    const double err = kPi / 2.0;
    const double dt = 0.05;
    const double cap = 0.15;
    const double step = ApplyBoundedYawStep(err, dt, cap);
    const double expected = cap * (kPi / 180.0) * dt;
    EXPECT_NEAR(step, expected, 1e-12);

    // Negative error: clamps to negative cap.
    const double negStep = ApplyBoundedYawStep(-err, dt, cap);
    EXPECT_NEAR(negStep, -expected, 1e-12);

    // Small error within cap: passes through unchanged.
    const double small = expected * 0.5;
    EXPECT_NEAR(ApplyBoundedYawStep(small, dt, cap), small, 1e-12);
}

TEST(RestLockedYawTest, WeightedAverageWithMultipleTrackers) {
    std::vector<YawContribution> xs;
    YawContribution a;
    a.yawErrRad = 5.0 * kPi / 180.0;
    a.weight = 1.0;
    xs.push_back(a);

    YawContribution b;
    b.yawErrRad = -3.0 * kPi / 180.0;
    b.weight = 1.0;
    xs.push_back(b);

    const double mean = FuseYawContributionsRad(xs);
    // Circular mean of 5 deg and -3 deg: atan2(sin5+sin(-3), cos5+cos(-3)).
    // Numerically very close to the arithmetic mean (1 deg) for small angles
    // but not identical; use a tolerance that passes the circular formula.
    const double expected = std::atan2(
        std::sin(5.0 * kPi / 180.0) + std::sin(-3.0 * kPi / 180.0),
        std::cos(5.0 * kPi / 180.0) + std::cos(-3.0 * kPi / 180.0));
    EXPECT_NEAR(mean, expected, 1e-12);
}

TEST(RestLockedYawTest, EmptyContributionsReturnZero) {
    std::vector<YawContribution> xs;
    EXPECT_EQ(FuseYawContributionsRad(xs), 0.0);
}

TEST(RestLockedYawTest, ZeroWeightContributionsReturnZero) {
    std::vector<YawContribution> xs;
    YawContribution a;
    a.yawErrRad = 5.0 * kPi / 180.0;
    a.weight = 0.0;
    xs.push_back(a);
    EXPECT_EQ(FuseYawContributionsRad(xs), 0.0);
}

TEST(RestLockedYawTest, ClassWeightLighthouseTrustedHighest) {
    EXPECT_GT(ClassWeight(TrackingSystemClass::Lighthouse), ClassWeight(TrackingSystemClass::Quest));
    EXPECT_GT(ClassWeight(TrackingSystemClass::Quest), ClassWeight(TrackingSystemClass::SlimeVR));
}

TEST(RestLockedYawTest, AgeWeightDecaysWithTime) {
    EXPECT_NEAR(AgeWeight(0.0, 120.0), 1.0, 1e-12);
    EXPECT_NEAR(AgeWeight(120.0, 120.0), std::exp(-1.0), 1e-12);
    EXPECT_GT(AgeWeight(0.0, 120.0), AgeWeight(60.0, 120.0));
    EXPECT_GT(AgeWeight(60.0, 120.0), AgeWeight(120.0, 120.0));
}

TEST(RestLockedYawTest, QualityWeightFallsWithVariance) {
    EXPECT_NEAR(QualityWeight(0.0), 1.0, 1e-12);
    EXPECT_LT(QualityWeight(0.1), QualityWeight(0.0));
    EXPECT_LT(QualityWeight(0.5), QualityWeight(0.1));
}

TEST(RestLockedYawTest, CapForClassRespectsCeiling) {
    RateCaps caps;
    caps.lighthouse_deg_per_sec = 10.0;
    caps.global_ceiling_deg_per_sec = 0.5;
    EXPECT_NEAR(CapForClass(TrackingSystemClass::Lighthouse, caps), 0.5, 1e-12);

    caps.lighthouse_deg_per_sec = 0.05;
    EXPECT_NEAR(CapForClass(TrackingSystemClass::Lighthouse, caps), 0.05, 1e-12);
}

TEST(RestLockedYawTest, SignedYawDeltaIsCorrectAroundY) {
    Eigen::Quaterniond a = YawQuat(0.0);
    Eigen::Quaterniond b = YawQuat(10.0);
    EXPECT_NEAR(SignedYawDeltaRad(a, b), 10.0 * kPi / 180.0, 1e-9);

    Eigen::Quaterniond c = YawQuat(-10.0);
    EXPECT_NEAR(SignedYawDeltaRad(a, c), -10.0 * kPi / 180.0, 1e-9);
}

// ---------------------------------------------------------------------------
// REGRESSION GUARD analogous to the deleted Phase 1+2 silent-recal failure.
//
// Phase 1+2's accept gate compared candidate-RMS vs current-RMS on the SAME
// stationary buffer; the comparison is tautological. The new design's gate
// is per-tracker rotation-dwell against an absolute reference (the
// orientation at lock time). Test that 1000 zero-rotation-delta ticks
// preserve the AtRest phase AND do not silently slide the locked reference
// (which would be the analogous failure mode).
// ---------------------------------------------------------------------------
TEST(RestLockedYawTest, RestPhaseStaysAtRestAcrossManyZeroDeltaTicks) {
    RestState s;
    const Eigen::Quaterniond q = YawQuat(0.0);

    // Enter rest.
    for (int i = 0; i <= 21; ++i) {
        s = UpdatePhase(s, q, i * 0.05, 0.05);
    }
    ASSERT_EQ(s.phase, RestPhase::AtRest);
    ASSERT_TRUE(s.haveLock);

    const Eigen::Quaterniond lockedAtEntry = s.lockedRot;

    // 1000 more zero-delta ticks. The locked reference must not slide.
    for (int i = 22; i < 1022; ++i) {
        s = UpdatePhase(s, q, i * 0.05, 0.05);
        EXPECT_EQ(s.phase, RestPhase::AtRest);
        EXPECT_TRUE(s.haveLock);
        EXPECT_NEAR(s.lockedRot.dot(lockedAtEntry), 1.0, 1e-12);
    }
}

// Bounded-rate cap is provably safe: 1000 ticks at maximum error cannot
// inject more than rate * total_duration of yaw correction. This is the
// mathematical guarantee that the deleted Phase 1+2 lacked.
TEST(RestLockedYawTest, BoundedRateMathematicalSafetyAcrossLongRun) {
    const double dt = 0.05;
    const int    ticks = 1000;
    const double cap = 0.15; // deg/s
    const double absurdError = 1000.0 * kPi; // way past anything real

    double accumulated = 0.0;
    for (int i = 0; i < ticks; ++i) {
        accumulated += ApplyBoundedYawStep(absurdError, dt, cap);
    }
    const double maxAccumDeg = (cap * ticks * dt);
    EXPECT_LE(accumulated * 180.0 / kPi, maxAccumDeg + 1e-9);
}
