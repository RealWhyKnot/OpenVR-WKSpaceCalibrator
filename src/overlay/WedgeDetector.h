#pragma once

// Wedge detection — single source of truth for the architectural-wedge case
// described in project_watchdog_wedged_cal_limitation.md (memory).
//
// A wedged calibration is one where the math has converged to a self-consistent
// fit (low error_currentCal, low error_byRelPose, both methods agree) that is
// nonetheless physically wrong — the user's body trackers appear in nonsense
// places, often 2-3 m offset from where they should be. Continuous-cal cannot
// escape it from inside the data because new samples either confirm the wedge
// (so the rejection counter never grows and the watchdog's wedge-recovery
// branch never fires) or are rejected as noise (counter grows but the prior-
// error gate keeps the watchdog asleep).
//
// The fork's previous behaviour was to ship a manual "Recalibrate from
// scratch" button so the user could escape it themselves. The user's
// 2026-05-04 ask: stop expecting users to know to do that — auto-detect, and
// silently auto-recover. This header centralises the bound + the runtime
// debounce-and-fire decision so both the load-time guard (Configuration.cpp,
// ParseProfile) and the runtime detector (Calibration.cpp, CalibrationTick)
// agree on the threshold and the test suite can pin the firing logic.

namespace spacecal::wedge {

// Plausibility bound on the calibration translation magnitude (cm).
// Anything above this on load OR on a sustained runtime tick is treated as
// the wedge case and silently cleared.
//
// Threshold rationale: 200 cm. The user's reported wedge measured 295 cm;
// their healthy converged fit was 127 cm. 200 cm catches the wedge with
// margin and clears any room-scale Quest+Lighthouse setup whose physical
// origin separation falls in the typical 1.0–1.5 m band. Setups whose
// origin separation legitimately exceeds 2 m would trip this on every load
// — if that pattern shows up in the wild, raise this constant or move it
// behind a setting. Don't tighten without checking
// `feedback_dont_break_existing_calibrations.md`.
constexpr double kMaxPlausibleCalibrationMagnitudeCm = 200.0;

// Runtime debounce window (s). The runtime detector only fires after the
// magnitude has stayed above the bound for this long uninterrupted —
// prevents single-tick numerical glitches (rare-but-possible during normal
// continuous-cal operation) from spuriously clobbering a healthy calibration.
// 30 s is well above any healthy-fresh-start hunting window (which spikes at
// 8–12 cm, nowhere near 200 cm), so there is no false-positive concern; the
// only purpose is single-tick filtering.
constexpr double kRuntimeWedgeDebounceSec = 30.0;

// Pure runtime wedge-detector tick. State lives in the caller-owned
// `wedgeSince`, initialised to a sentinel < 0 ("no active wedge") and
// updated each call.
//
// Returns true exactly once per wedge episode, when the magnitude has been
// above the bound for at least kRuntimeWedgeDebounceSec consecutive seconds.
// On a fire, resets `wedgeSince` to the sentinel so a subsequent call won't
// re-fire until magnitude drops back under the bound and re-climbs.
//
// Returns false when:
//   - magnitude is at or below the bound (and resets wedgeSince to sentinel)
//   - magnitude is above the bound but the debounce window hasn't elapsed
//
// `now` is glfwGetTime() seconds in production; tests pass synthetic times.
inline bool ShouldFireRuntimeWedgeRecovery(double magnitudeCm,
                                           double now,
                                           double& wedgeSince) {
    if (magnitudeCm <= kMaxPlausibleCalibrationMagnitudeCm) {
        wedgeSince = -1.0;
        return false;
    }
    // Magnitude is above the bound.
    if (wedgeSince < 0.0) {
        wedgeSince = now;
        return false;
    }
    if ((now - wedgeSince) >= kRuntimeWedgeDebounceSec) {
        wedgeSince = -1.0;  // consume the fire; require re-entry below->above to refire
        return true;
    }
    return false;
}

} // namespace spacecal::wedge
