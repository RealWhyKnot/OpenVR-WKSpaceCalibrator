// Tests for the runtime wedge detector. Exercises the pure-function form
// extracted in src/overlay/WedgeDetector.h so the firing logic can be pinned
// without dragging in CalibrationTick / OpenVR / glfw / the full overlay.
//
// The detector's job: fire exactly once when ctx.calibratedTranslation's
// magnitude has been above the plausibility bound (500 cm post-2026-05-05) for at least
// kRuntimeWedgeDebounceSec (30 s) of uninterrupted ticks. A drop below the
// bound resets the timer; a fire consumes the firing condition (a second
// call won't refire until magnitude drops below + climbs back above).
//
// State lives in caller-owned `wedgeSince`, sentinel <0 = "no active wedge".

#include <gtest/gtest.h>

#include "WedgeDetector.h"

using spacecal::wedge::ShouldFireRuntimeWedgeRecovery;
using spacecal::wedge::kMaxPlausibleCalibrationMagnitudeCm;
using spacecal::wedge::kRuntimeWedgeDebounceSec;

// ---------------------------------------------------------------------------
// Magnitude under the bound returns false and resets the wedge timer to its
// sentinel. The healthy case — most ticks of a normal session.
// ---------------------------------------------------------------------------
TEST(WedgeDetectorTest, BelowBoundReturnsFalseAndResetsTimer) {
    double wedgeSince = 100.0;  // pretend a prior tick set this
    EXPECT_FALSE(ShouldFireRuntimeWedgeRecovery(/*mag=*/127.0,
                                                /*now=*/200.0, wedgeSince));
    EXPECT_LT(wedgeSince, 0.0)
        << "A tick under the bound must reset wedgeSince to the <0 sentinel";

    // Right at the bound is still treated as healthy (boundary inclusive).
    wedgeSince = 100.0;
    EXPECT_FALSE(ShouldFireRuntimeWedgeRecovery(
        kMaxPlausibleCalibrationMagnitudeCm, 200.0, wedgeSince));
    EXPECT_LT(wedgeSince, 0.0);
}

// ---------------------------------------------------------------------------
// First tick above the bound stamps the wedgeSince timestamp but does NOT
// fire — the debounce window hasn't elapsed yet.
// ---------------------------------------------------------------------------
TEST(WedgeDetectorTest, FirstTickAboveBoundStampsButDoesNotFire) {
    double wedgeSince = -1.0;
    EXPECT_FALSE(ShouldFireRuntimeWedgeRecovery(/*mag=*/600.0,
                                                /*now=*/100.0, wedgeSince));
    EXPECT_DOUBLE_EQ(wedgeSince, 100.0)
        << "First over-bound tick must record the timestamp for future debounce";
}

// ---------------------------------------------------------------------------
// Sustained over-bound ticks within the debounce window do not fire. Pinned
// at the user's chosen 30 s window — if the constant moves, both the constant
// and this test should be updated together.
// ---------------------------------------------------------------------------
TEST(WedgeDetectorTest, SustainedAboveBoundWithinDebounceDoesNotFire) {
    double wedgeSince = -1.0;
    // First tick at t=100 stamps.
    ShouldFireRuntimeWedgeRecovery(600.0, 100.0, wedgeSince);
    ASSERT_DOUBLE_EQ(wedgeSince, 100.0);

    // 5 s later, still above bound: no fire, timer unchanged.
    EXPECT_FALSE(ShouldFireRuntimeWedgeRecovery(610.0, 105.0, wedgeSince));
    EXPECT_DOUBLE_EQ(wedgeSince, 100.0);

    // 29.9 s later: still under the debounce ceiling, no fire.
    EXPECT_FALSE(ShouldFireRuntimeWedgeRecovery(610.0, 129.9, wedgeSince));
    EXPECT_DOUBLE_EQ(wedgeSince, 100.0);
}

// ---------------------------------------------------------------------------
// Over-bound for the full debounce window fires exactly once. After the
// fire, wedgeSince resets to the sentinel so a follow-up call without any
// state change does NOT refire (re-entry requires drop below + climb above).
// ---------------------------------------------------------------------------
TEST(WedgeDetectorTest, OverBoundForDebounceWindowFiresOnceThenSilent) {
    double wedgeSince = -1.0;
    ShouldFireRuntimeWedgeRecovery(600.0, 100.0, wedgeSince);

    // Exactly at the debounce boundary: should fire (>= comparison).
    const double fireTime = 100.0 + kRuntimeWedgeDebounceSec;
    EXPECT_TRUE(ShouldFireRuntimeWedgeRecovery(600.0, fireTime, wedgeSince));
    EXPECT_LT(wedgeSince, 0.0)
        << "A fire must reset wedgeSince to the sentinel so next tick "
           "starts a fresh 30 s window if magnitude is still high";

    // Immediately after firing, magnitude still over bound. The detector
    // begins a NEW debounce window (re-stamp), does NOT fire again.
    EXPECT_FALSE(ShouldFireRuntimeWedgeRecovery(600.0, fireTime + 0.1, wedgeSince));
    EXPECT_DOUBLE_EQ(wedgeSince, fireTime + 0.1);
}

// ---------------------------------------------------------------------------
// A transient single-tick spike (above bound for one tick, then back below)
// must not fire. This is the practical justification for the debounce — even
// during convergence, a single noisy tick shouldn't clobber a healthy cal.
// ---------------------------------------------------------------------------
TEST(WedgeDetectorTest, TransientSpikeBelowDebounceDoesNotFire) {
    double wedgeSince = -1.0;
    // Spike above bound at t=100.
    ShouldFireRuntimeWedgeRecovery(600.0, 100.0, wedgeSince);
    ASSERT_GE(wedgeSince, 0.0);

    // 0.1 s later, magnitude drops back under bound. Timer resets.
    EXPECT_FALSE(ShouldFireRuntimeWedgeRecovery(127.0, 100.1, wedgeSince));
    EXPECT_LT(wedgeSince, 0.0)
        << "A drop below bound must reset the timer so a later spike starts "
           "its own fresh debounce window";

    // Another over-bound tick a long time later: timer starts fresh, no
    // fire (because wedgeSince was reset, this tick is the new "first").
    EXPECT_FALSE(ShouldFireRuntimeWedgeRecovery(600.0, 200.0, wedgeSince));
    EXPECT_DOUBLE_EQ(wedgeSince, 200.0);
}

// ---------------------------------------------------------------------------
// Healthy fresh-start hunting (8-12 cm transients during the first ~10 min
// after a cold-start) is nowhere near the 500 cm threshold, so no firing.
// Pin this so a future tightening of the bound is forced to think about
// the fresh-start hunting case explicitly.
// ---------------------------------------------------------------------------
TEST(WedgeDetectorTest, HealthyFreshStartHuntingDoesNotFire) {
    double wedgeSince = -1.0;
    // Simulate 60 s of healthy hunting around 127 cm with 8-12 cm spikes.
    for (int i = 0; i < 60; ++i) {
        const double mag = (i % 2 == 0) ? 119.0 : 135.0;
        EXPECT_FALSE(ShouldFireRuntimeWedgeRecovery(mag, 100.0 + i, wedgeSince))
            << "Healthy hunting must never fire; iteration " << i;
    }
    EXPECT_LT(wedgeSince, 0.0);
}
