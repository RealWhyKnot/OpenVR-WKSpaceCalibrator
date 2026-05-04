// Regression tests for the HMD-stall handling. Specifically pinning the revert
// of commit 9d0ba0b's buffer-purge + state-demotion behaviour, which the user's
// 2026-05-04 spacecal_log proved caused 7-9 cm cumulative drift on every HMD
// off/on cycle.
//
// Test purpose (from the audit's "transparent gaps" section): the old test
// suite did not exercise the runtime CalibrationTick state machine, which is
// where 9d0ba0b shipped its bug. We can't easily test CalibrationTick directly
// (huge function, OpenVR/glfw dependencies), so we extracted the decision
// — "should we demote on HMD stall?" — into a pure function and pinned its
// contract here. Anyone trying to re-introduce the demote/purge behaviour by
// flipping `ShouldDemoteOnHmdStall` to return true will fail this test.

#include <gtest/gtest.h>

#include "StallDecisions.h"

using spacecal::stall::ShouldDemoteOnHmdStall;

// ---------------------------------------------------------------------------
// REGRESSION GUARD for commit 9d0ba0b (HMD-stall buffer-purge).
//
// This test must NEVER pass with a `true` return. If you're modifying
// ShouldDemoteOnHmdStall: read its docstring first. The drift mechanism is:
//   1. Stall count crosses MaxHmdStalls (30 ticks ~= 1.5 s)
//   2. calibration.Clear() purges the sample buffer
//   3. State demotes Continuous → ContinuousStandby
//   4. On HMD recovery, StartContinuousCalibration() re-applies the saved
//      refToTargetPose warm-start (relativePosCalibrated NOT reset — the
//      asymmetry vs the geometry-shift detector is the active footgun)
//   5. Continuous-cal converges from new post-stall samples against the
//      stale constraint, lands on a different local minimum
//   6. SaveProfile persists the shifted fit; cumulative drift wedges the
//      saved profile
//
// The user's spacecal_log.2026-05-04T17-14-50.txt has empirical evidence:
// two stalls (56 ticks, 95 ticks) each caused 7-9 cm Z-shift in
// posOffset_currentCal immediately post-recovery.
// ---------------------------------------------------------------------------
TEST(StallDecisionsTest, Regression_9d0ba0b_NeverDemoteOnHmdStall_ZeroStalls) {
    EXPECT_FALSE(ShouldDemoteOnHmdStall(0, 30));
}

TEST(StallDecisionsTest, Regression_9d0ba0b_NeverDemoteOnHmdStall_AtThreshold) {
    // The original buggy code triggered exactly at consecutiveStalls == MaxHmdStalls.
    // Pin: even at the original trigger point, we don't demote.
    EXPECT_FALSE(ShouldDemoteOnHmdStall(30, 30));
}

TEST(StallDecisionsTest, Regression_9d0ba0b_NeverDemoteOnHmdStall_FarPastThreshold) {
    // The user's actual session had a 95-tick stall. Verify well past that.
    EXPECT_FALSE(ShouldDemoteOnHmdStall(95, 30));
    EXPECT_FALSE(ShouldDemoteOnHmdStall(1000, 30));
    EXPECT_FALSE(ShouldDemoteOnHmdStall(100000, 30));
}

TEST(StallDecisionsTest, Regression_9d0ba0b_NeverDemoteOnHmdStall_DegenerateInputs) {
    // Negative or zero `maxStalls` — defensive: shouldn't crash, shouldn't
    // suddenly start demoting.
    EXPECT_FALSE(ShouldDemoteOnHmdStall(0, 0));
    EXPECT_FALSE(ShouldDemoteOnHmdStall(50, 0));
    EXPECT_FALSE(ShouldDemoteOnHmdStall(-1, 30));
    EXPECT_FALSE(ShouldDemoteOnHmdStall(0, -1));
}

// ---------------------------------------------------------------------------
// constexpr-evaluation pin. Because ShouldDemoteOnHmdStall is constexpr, the
// compiler can fold the comparison at compile time. A static_assert is a
// stronger check than a runtime EXPECT_FALSE — if anyone flips the function
// to a runtime form (or worse, returns true), the build itself breaks.
// ---------------------------------------------------------------------------
static_assert(!ShouldDemoteOnHmdStall(0, 30),
    "ShouldDemoteOnHmdStall must always be false — see commit 9d0ba0b regression");
static_assert(!ShouldDemoteOnHmdStall(30, 30),
    "ShouldDemoteOnHmdStall must always be false at the legacy trigger point");
static_assert(!ShouldDemoteOnHmdStall(1000, 30),
    "ShouldDemoteOnHmdStall must always be false even far past the legacy threshold");
