// Pure-function pin tests for the geometry-shift fast watchdog. Covers the
// per-tick spike check + the sustained-firings gate. The decision is split so
// each half is unit-testable in isolation; the production caller in
// CalibrationTick owns the running counter.
//
// This is audit row #3 from project_upstream_regression_audit_2026-05-04.md
// — the fork-only geometry-shift detector at Calibration.cpp:2098-2142,
// added in commit 9d0ba0b. No active regression observed; tests exist to
// pin the contract so any future tuning surfaces deliberately.

#include <gtest/gtest.h>

#include "GeometryShiftDetector.h"

using spacecal::geometry_shift::IsCurrentErrorSpike;
using spacecal::geometry_shift::ShouldFireGeometryShiftRecovery;
using spacecal::geometry_shift::kSpikeRatio;
using spacecal::geometry_shift::kMedianFloor;
using spacecal::geometry_shift::kMinSustainedSpikes;

// ---------------------------------------------------------------------------
// IsCurrentErrorSpike: median floor. If the median is below kMedianFloor
// (essentially zero noise on the time series), the spike check is meaningless
// — return false. Without this floor, any noise spike against a near-zero
// median would trip 5× ratio trivially and the detector would fire on bootstrap.
// ---------------------------------------------------------------------------
TEST(GeometryShiftDetectorTest, IsCurrentErrorSpike_NearZeroMedianIsNotASpike) {
    EXPECT_FALSE(IsCurrentErrorSpike(/*current=*/100.0, /*median=*/0.0));
    EXPECT_FALSE(IsCurrentErrorSpike(100.0, 1e-10));  // below floor
    EXPECT_FALSE(IsCurrentErrorSpike(0.001, 1e-12));
}

// ---------------------------------------------------------------------------
// IsCurrentErrorSpike: 5× ratio. Anything > 5× the median fires; anything <=
// does not. Boundary at exactly 5× is *not* a spike (strict >).
// ---------------------------------------------------------------------------
TEST(GeometryShiftDetectorTest, IsCurrentErrorSpike_RatioBoundary) {
    EXPECT_FALSE(IsCurrentErrorSpike(/*current=*/4.99, /*median=*/1.0))
        << "Just under 5× should not fire";
    EXPECT_FALSE(IsCurrentErrorSpike(5.0, 1.0))
        << "Exactly 5× must NOT fire (strict-greater-than)";
    EXPECT_TRUE(IsCurrentErrorSpike(5.01, 1.0))
        << "Just over 5× must fire";
    EXPECT_TRUE(IsCurrentErrorSpike(50.0, 1.0));
    EXPECT_TRUE(IsCurrentErrorSpike(0.6, 0.1)) << "Scale-invariant";
}

// ---------------------------------------------------------------------------
// IsCurrentErrorSpike: realistic continuous-cal numbers. error_currentCal at
// 1-3 mm during healthy operation; a real geometry shift jumps to 30+ mm.
// Pin that the detector catches the genuine case and stays quiet on the
// normal noise band.
// ---------------------------------------------------------------------------
TEST(GeometryShiftDetectorTest, IsCurrentErrorSpike_HealthyHuntingNotFlagged) {
    // Healthy continuous-cal: error 1.5-3.5 mm, median 2.0 mm. Noise should
    // not flag.
    for (double current : {1.5, 1.8, 2.0, 2.4, 3.5}) {
        EXPECT_FALSE(IsCurrentErrorSpike(current, /*median=*/2.0))
            << "Healthy hunting at current=" << current << " mm flagged spuriously";
    }
}

TEST(GeometryShiftDetectorTest, IsCurrentErrorSpike_RealGeometryShiftFlagged) {
    // Lighthouse bumped: error jumps from 2 mm to 30+ mm. Must flag.
    EXPECT_TRUE(IsCurrentErrorSpike(/*current=*/30.0, /*median=*/2.0));
    EXPECT_TRUE(IsCurrentErrorSpike(50.0, 5.0));
}

// ---------------------------------------------------------------------------
// ShouldFireGeometryShiftRecovery: sustain count. Fires at exactly
// kMinSustainedSpikes, NOT before. Pin the boundary so the 3-tick delay
// (~100-300 ms) is never accidentally tightened or loosened in code review.
// ---------------------------------------------------------------------------
TEST(GeometryShiftDetectorTest, ShouldFireGeometryShiftRecovery_AtBoundary) {
    EXPECT_FALSE(ShouldFireGeometryShiftRecovery(0));
    EXPECT_FALSE(ShouldFireGeometryShiftRecovery(1));
    EXPECT_FALSE(ShouldFireGeometryShiftRecovery(kMinSustainedSpikes - 1))
        << "Just under threshold (" << (kMinSustainedSpikes - 1)
        << ") must not fire";
    EXPECT_TRUE(ShouldFireGeometryShiftRecovery(kMinSustainedSpikes))
        << "At threshold (" << kMinSustainedSpikes << ") must fire";
    EXPECT_TRUE(ShouldFireGeometryShiftRecovery(kMinSustainedSpikes + 1));
    EXPECT_TRUE(ShouldFireGeometryShiftRecovery(100));
}

// ---------------------------------------------------------------------------
// constexpr pins. Both functions evaluate at compile time; static_assert
// fails the build (not just the test) if the contract is broken.
// ---------------------------------------------------------------------------
static_assert(!IsCurrentErrorSpike(100.0, 0.0),
    "near-zero median must short-circuit the spike check");
static_assert(IsCurrentErrorSpike(5.01, 1.0),
    "5.01× the median must register as a spike");
static_assert(!IsCurrentErrorSpike(5.0, 1.0),
    "exactly 5× must NOT fire — strict greater-than");
static_assert(!ShouldFireGeometryShiftRecovery(2),
    "2 sustained spikes must not yet fire");
static_assert(ShouldFireGeometryShiftRecovery(3),
    "3 sustained spikes is the documented trigger");

// ---------------------------------------------------------------------------
// CUSUM (Page 1954) opt-in path. Same recovery action as the legacy detector;
// different per-tick decision. The pure helper UpdateCusumGeometryShift
// owns the increment + threshold logic so we can pin it without spinning up
// the calibration tick.
// ---------------------------------------------------------------------------
TEST(CusumGeometryShiftTest, NoiseAtBaselineDoesNotAccumulate) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    using spacecal::geometry_shift::kCusumDriftMm;
    using spacecal::geometry_shift::kCusumThreshold;
    CusumState s{};
    // Feed 100 ticks of "current = baseline" -- per-sample increment is
    // (0 - drift) = -kCusumDriftMm < 0, so S stays clamped at 0.
    for (int i = 0; i < 100; i++) {
        const bool fire = UpdateCusumGeometryShift(s, /*current=*/2.0, /*baseline=*/2.0);
        EXPECT_FALSE(fire);
        EXPECT_DOUBLE_EQ(s.S, 0.0);
    }
}

TEST(CusumGeometryShiftTest, BelowDriftDoesNotFire) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    using spacecal::geometry_shift::kCusumDriftMm;
    using spacecal::geometry_shift::kCusumThreshold;
    CusumState s{};
    // Per-sample excursion at exactly +0.4 mm (below the 0.5 mm drift). Each
    // tick contributes (0.4 - 0.5) = -0.1 to S, clamped at 0. No fire ever.
    for (int i = 0; i < 1000; i++) {
        const bool fire = UpdateCusumGeometryShift(s, /*current=*/2.4, /*baseline=*/2.0);
        EXPECT_FALSE(fire);
    }
    EXPECT_DOUBLE_EQ(s.S, 0.0);
}

TEST(CusumGeometryShiftTest, SustainedShiftFiresWithinExpectedTicks) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    using spacecal::geometry_shift::kCusumDriftMm;
    using spacecal::geometry_shift::kCusumThreshold;
    CusumState s{};
    // 5 mm sustained shift: increment per tick = (5 - 0) - 0.5 = 4.5 mm.
    // Threshold 5.0 mm reached on the 2nd tick. (After tick 1: S = 4.5; not
    // yet > 5.0. After tick 2: S = 9.0; FIRE; S resets to 0.)
    EXPECT_FALSE(UpdateCusumGeometryShift(s, /*current=*/5.0, /*baseline=*/0.0));
    EXPECT_NEAR(s.S, 4.5, 1e-9);
    EXPECT_TRUE(UpdateCusumGeometryShift(s, 5.0, 0.0));
    EXPECT_DOUBLE_EQ(s.S, 0.0) << "CUSUM must reset to 0 after firing";
}

TEST(CusumGeometryShiftTest, RecoversFromSpike_ResumesQuiet) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    CusumState s{};
    // Sustained shift fires.
    UpdateCusumGeometryShift(s, 5.0, 0.0);
    UpdateCusumGeometryShift(s, 5.0, 0.0);
    // Now post-fire: error returns to baseline. State is at 0; no further fires.
    for (int i = 0; i < 100; i++) {
        EXPECT_FALSE(UpdateCusumGeometryShift(s, 0.0, 0.0));
    }
    EXPECT_DOUBLE_EQ(s.S, 0.0);
}

TEST(CusumGeometryShiftTest, ManualThresholdTuningWorks) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    CusumState s{};
    // Tighter threshold (1.0 mm) fires sooner on the same shift.
    EXPECT_TRUE(UpdateCusumGeometryShift(s, /*current=*/3.0, /*baseline=*/0.0,
                                          /*driftMm=*/0.5, /*threshold=*/1.0));
    EXPECT_DOUBLE_EQ(s.S, 0.0);
}

TEST(CusumGeometryShiftTest, ResetClampPreventsNegativeAccumulation) {
    using spacecal::geometry_shift::CusumState;
    using spacecal::geometry_shift::UpdateCusumGeometryShift;
    CusumState s{};
    // Long quiet period must NOT accumulate negative S that would delay a
    // later real shift's fire. The clamp at S = max(0, ...) is what prevents
    // this; without it, 1000 quiet ticks would push S to -500 mm and a real
    // shift would need to overcome that before firing.
    for (int i = 0; i < 1000; i++) UpdateCusumGeometryShift(s, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(s.S, 0.0);
    // Now a real shift fires within 2 ticks (same as the fresh-state case).
    EXPECT_FALSE(UpdateCusumGeometryShift(s, 5.0, 0.0));
    EXPECT_TRUE(UpdateCusumGeometryShift(s, 5.0, 0.0));
}
