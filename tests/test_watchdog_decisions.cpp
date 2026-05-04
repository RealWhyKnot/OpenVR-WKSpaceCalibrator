// Pure-function pin tests for the stuck-loop watchdog decisions.
// Audit row #5 from project_upstream_regression_audit_2026-05-04.md.
//
// The watchdog itself is fork-only (commit 9d0ba0b). It cannot catch
// wedged-self-consistent calibrations — the wedge guard from 8e5e111
// backstops that case — but it does catch genuine stuck loops where
// continuous-cal has been rejecting every new sample for ~25 s on a
// calibration whose prior error is above the noise floor.
//
// These tests pin the boundary constants (kMaxConsecutiveRejections=50,
// kHealthyPriorErrorMaxMeters=5e-3) so any future tuning is forced
// through the regression-test suite.

#include <gtest/gtest.h>

#include "WatchdogDecisions.h"

using spacecal::watchdog::IsCalibrationHealthy;
using spacecal::watchdog::ShouldClearViaWatchdog;
using spacecal::watchdog::kMaxConsecutiveRejections;
using spacecal::watchdog::kHealthyPriorErrorMaxMeters;

// ---------------------------------------------------------------------------
// IsCalibrationHealthy: false when not valid (regardless of error).
// ---------------------------------------------------------------------------
TEST(WatchdogDecisionsTest, IsCalibrationHealthy_NotValidIsAlwaysUnhealthy) {
    EXPECT_FALSE(IsCalibrationHealthy(/*isValid=*/false, /*priorMeters=*/0.0));
    EXPECT_FALSE(IsCalibrationHealthy(false, 0.001));
    EXPECT_FALSE(IsCalibrationHealthy(false, 1.0));
}

// ---------------------------------------------------------------------------
// IsCalibrationHealthy: 5 mm boundary. Strictly less than the floor is
// healthy; equal or greater is not. Pinned because the floor was raised
// from 10 mm to 5 mm in the original commit per a real-session diagnosis;
// any future widening must be deliberate.
// ---------------------------------------------------------------------------
TEST(WatchdogDecisionsTest, IsCalibrationHealthy_FiveMillimeterBoundary) {
    EXPECT_TRUE(IsCalibrationHealthy(true, 0.0));
    EXPECT_TRUE(IsCalibrationHealthy(true, 0.001));
    EXPECT_TRUE(IsCalibrationHealthy(true, 0.00499));
    EXPECT_FALSE(IsCalibrationHealthy(true, 0.005))
        << "Exactly 5 mm must NOT be healthy (strict less-than floor)";
    EXPECT_FALSE(IsCalibrationHealthy(true, 0.006));
    EXPECT_FALSE(IsCalibrationHealthy(true, 0.050));
}

// ---------------------------------------------------------------------------
// ShouldClearViaWatchdog: requires all three preconditions.
//   - isValid (so we have something to clear, and the rejections are
//     against an established fixpoint not bootstrap noise)
//   - rejectionCount >= kMaxConsecutiveRejections (50)
//   - prior is NOT in the healthy band (otherwise it's the skip-fire path)
// ---------------------------------------------------------------------------
TEST(WatchdogDecisionsTest, ShouldClearViaWatchdog_RequiresAllConditions) {
    // All three present: clear.
    EXPECT_TRUE(ShouldClearViaWatchdog(true, 50, 0.010));
    EXPECT_TRUE(ShouldClearViaWatchdog(true, 100, 0.020));

    // Not valid: never clear.
    EXPECT_FALSE(ShouldClearViaWatchdog(false, 50, 0.010));
    EXPECT_FALSE(ShouldClearViaWatchdog(false, 1000, 1.0));

    // Below cap: never clear.
    EXPECT_FALSE(ShouldClearViaWatchdog(true, 0, 0.010));
    EXPECT_FALSE(ShouldClearViaWatchdog(true, 49, 0.010))
        << "49 rejections is below the 50 cap; must wait one more.";

    // Healthy prior: never clear (skip-fire path).
    EXPECT_FALSE(ShouldClearViaWatchdog(true, 50, 0.001))
        << "Healthy prior must not be cleared — that's the wedge-guard's "
           "job (with magnitude check) or the user's job, not this watchdog.";
    EXPECT_FALSE(ShouldClearViaWatchdog(true, 1000, 0.004));
}

// ---------------------------------------------------------------------------
// ShouldClearViaWatchdog: rejection cap boundary. Pinned so a future tuning
// of the 50-rejection trigger is forced through the test suite.
// ---------------------------------------------------------------------------
TEST(WatchdogDecisionsTest, ShouldClearViaWatchdog_RejectionCapBoundary) {
    EXPECT_FALSE(ShouldClearViaWatchdog(true, kMaxConsecutiveRejections - 1, 0.010));
    EXPECT_TRUE(ShouldClearViaWatchdog(true, kMaxConsecutiveRejections, 0.010));
    EXPECT_TRUE(ShouldClearViaWatchdog(true, kMaxConsecutiveRejections + 1, 0.010));
}

// ---------------------------------------------------------------------------
// constexpr pins. static_assert on the documented boundary points so a
// future change that breaks the contract fails the build, not just the
// tests.
// ---------------------------------------------------------------------------
static_assert(IsCalibrationHealthy(true, 0.001),
    "1 mm prior on a valid cal must be healthy");
static_assert(!IsCalibrationHealthy(true, 0.005),
    "5 mm prior is NOT healthy (strict less-than floor)");
static_assert(!IsCalibrationHealthy(false, 0.001),
    "invalid cal cannot be healthy regardless of prior error");
static_assert(ShouldClearViaWatchdog(true, 50, 0.010),
    "50 rejections + 10 mm prior + valid must fire clear");
static_assert(!ShouldClearViaWatchdog(true, 50, 0.001),
    "50 rejections + 1 mm prior must NOT clear (skip-fire path)");
static_assert(!ShouldClearViaWatchdog(true, 49, 0.010),
    "49 rejections is below cap — must not yet clear");
