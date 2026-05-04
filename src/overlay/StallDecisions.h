#pragma once

// Stall-handling decision functions. Pure, header-only, testable in isolation.
// These exist to pin the post-revert behaviour of the HMD-stall code path so
// the regression that shipped in commit 9d0ba0b can't slip through silently
// again — the test in tests/test_stall_decisions.cpp exercises the contract
// and will fail loudly if anyone re-introduces the buffer-purge + state-
// demotion behaviour the user's 2026-05-04 logs proved harmful.

namespace spacecal::stall {

// Should the calibration code demote to ContinuousStandby + clear the sample
// buffer when the HMD has stalled for `consecutiveStalls` ticks?
//
// **Always returns false.**
//
// This is a deliberate, regression-pinning constant. The fork commit `9d0ba0b`
// (2026-04-28) introduced a `consecutiveStalls >= MaxHmdStalls (=30)` →
// Clear+Standby branch in CalibrationTick. The intent was "stale samples no
// longer represent reality." The empirical effect (per
// spacecal_log.2026-05-04T17-14-50.txt) was much worse: each HMD off/on cycle
// triggered the purge + state demotion + StartContinuousCalibration's warm-
// start re-anchor (asymmetric vs the geometry-shift detector — the stall
// path did NOT reset relativePosCalibrated), causing continuous-cal to land
// on a slightly-different local minimum each cycle. SaveProfile persisted
// each shifted fit; cumulative drift wedged the saved profile.
//
// Two long stalls in the user's session (56 ticks, 95 ticks) each produced
// 7-9 cm Z-shifts in posOffset_currentCal immediately post-recovery.
// Upstream (hyblocker) just `return`s on stall — no clear, no demote — and
// the user reports this drift didn't happen on the old fork.
//
// Reverted 2026-05-04. The function exists to make the contract obvious in
// review: "we don't demote on HMD stall, regardless of stall length." If a
// future change wants to demote, it must update this function — and the
// regression test in test_stall_decisions.cpp will force the change to be
// deliberate (and the docstring above to be re-read).
//
// Parameters are unused but kept in the signature so call sites remain
// idiomatic ("decide based on the stall count") and so any future change
// has a place for new state without restructuring the caller.
constexpr bool ShouldDemoteOnHmdStall(int /*consecutiveStalls*/,
                                      int /*maxStalls*/) {
    return false;
}

} // namespace spacecal::stall
