#pragma once

// Geometry-shift detector — fork-only fast watchdog at Calibration.cpp:2098-2142.
// Independent of the slower 50-rejection watchdog inside CalibrationCalc:
// catches catastrophic geometry shifts (lighthouse bumped, tracker through a
// portal) within ~3 ticks instead of ~25 s.
//
// Decision logic split into two trivially-testable parts:
//   - IsCurrentErrorSpike: per-tick boolean from the error time series
//   - ShouldFireGeometryShiftRecovery: gate on sustained spikes
//
// Both functions are pure + constexpr-friendly. Caller (CalibrationTick) owns
// the counter, increments on IsCurrentErrorSpike==true, resets on false.

namespace spacecal::geometry_shift {

// Threshold above which `currentError` is treated as anomalous relative to
// the rolling median. Empirically 5× the recent median catches genuine
// geometry shifts without firing on normal solver noise (per the original
// 9d0ba0b implementation comment).
constexpr double kSpikeRatio = 5.0;

// Floor on the rolling median: when the median itself is sub-nanometer-scale,
// the ratio test is meaningless (numerator might be tiny even at 5×). Skip
// the spike check entirely — there's no real signal to compare against.
constexpr double kMedianFloor = 1e-9;

// Required sustain count: don't fire on transient single-tick spikes that
// could be noise. 3 consecutive accepted samples gives ~100-300 ms of
// sustained anomaly before action.
constexpr int kMinSustainedSpikes = 3;

// Per-tick spike check. True when `currentError` exceeds the rolling median
// by at least kSpikeRatio×, with a floor on the median to avoid spurious
// firings on near-zero noise.
constexpr bool IsCurrentErrorSpike(double currentError, double rollingMedian) {
    return rollingMedian > kMedianFloor
        && currentError > kSpikeRatio * rollingMedian;
}

// Sustain gate. Caller passes the running count of consecutive ticks where
// IsCurrentErrorSpike returned true; this returns whether the count has
// reached the trigger.
constexpr bool ShouldFireGeometryShiftRecovery(int sustainedSpikes) {
    return sustainedSpikes >= kMinSustainedSpikes;
}

} // namespace spacecal::geometry_shift
