#pragma once

// Stuck-loop watchdog — fork-only, lives at CalibrationCalc.cpp:~1455-1518.
// Fires `calibration.Clear()` when continuous-cal has been rejecting every
// new sample for ~25 s on a calibration whose prior-error is *above* the
// 5 mm noise floor. Doesn't fire on rejections against an already-healthy
// prior, because rejecting noise IS the correct behavior there.
//
// Decision split into two trivially-testable predicates so the constants
// (rejection cap, healthy floor) are pinned by the test suite. Production
// code in `CalibrationCalc::ComputeIncremental` calls both helpers and
// uses the difference to decide between "skip with healthy-hold annotation"
// and "actually clear."
//
// Per project_watchdog_wedged_cal_limitation.md: this watchdog cannot
// catch wedged-self-consistent calibrations (priorError stays low even
// when the cal is physically wrong). The wedge guard from 8e5e111
// backstops the case the watchdog can't see; this header pins the
// watchdog's healthy-floor boundary so any future tuning is deliberate.

namespace spacecal::watchdog {

// Number of consecutive `ComputeIncremental` rejections after which the
// watchdog evaluates whether to clear. ~25 s of solid rejection at the
// post-buffer-fill cadence; below this is treated as normal solver
// quiescence.
constexpr int kMaxConsecutiveRejections = 50;

// Above this prior error, the watchdog will fire (`isValid` provided).
// Below or equal, the calibration is considered healthy enough that
// rejecting new samples is the correct behavior, not a stuck loop.
// Matches kRejectionFloor inside the validate gate; deliberately tight
// (was 10 mm previously, lowered per CalibrationCalc.cpp:1462-1472).
constexpr double kHealthyPriorErrorMaxMeters = 0.005;

// Healthy-cal predicate. True when the cal is currently valid AND the
// prior error is below the noise floor.
constexpr bool IsCalibrationHealthy(bool isValid, double priorErrorMeters) {
    return isValid && priorErrorMeters < kHealthyPriorErrorMaxMeters;
}

// Should the watchdog clear (`calibration.Clear()`) right now?
// Fires when: cal is valid + rejection counter has reached the cap +
// the prior is NOT in the healthy band. Returning false is either
// "not at cap yet" or "at cap but healthy — skip-fire path."
constexpr bool ShouldClearViaWatchdog(bool isValid,
                                      int rejectionCount,
                                      double priorErrorMeters) {
    return isValid
        && rejectionCount >= kMaxConsecutiveRejections
        && !IsCalibrationHealthy(isValid, priorErrorMeters);
}

} // namespace spacecal::watchdog
