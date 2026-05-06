#pragma once

// Sustained gravity-axis disagreement diagnostic.
//
// Two tracking systems should agree on which way is "down". When they don't,
// the residual pitch+roll component of the Kabsch rotation fit is non-zero.
// CalibrateRotation already computes this per-solve as `m_residualPitchRollDeg`
// and prints a one-line warning when > 2.0 deg per tick.
//
// What was missing: a SUSTAINED indicator. A single noisy frame at 3 deg is
// just bad luck; 1.5 deg sustained over a minute means the user's two
// tracking systems genuinely disagree about gravity, which manifests as
// 1-3 cm of position error at arm's length and is worth surfacing to the
// user (probably a re-run of room setup is in order).
//
// Logging-only diagnostic: the helpers in this header compute a decision;
// the caller emits a log annotation on transition. Calibration math is
// unchanged. This is the safe first step toward Pacher-2021-style gravity
// reconciliation -- get a real-session signal that the disagreement IS
// happening before plumbing acceleration data through to CORRECT it.
//
// Pure helpers, header-only, testable in isolation. Same pattern as
// MotionGate.h, GeometryShiftDetector.h, WatchdogDecisions.h, etc.

#include <algorithm>
#include <deque>
#include <vector>

namespace spacecal::gravity {

// Threshold above which the median tilt-over-window is considered sustained
// disagreement. 1.0 deg is half the per-tick warning threshold (2.0 deg in
// CalibrateRotation) -- tighter on duration, looser on instant.
constexpr double kSustainedTiltThresholdDeg = 1.0;

// Hysteresis: once alarmed, stay alarmed until the median falls below
// (threshold * fraction). Without this the alarm flaps near the boundary on
// noisy data; with it the on/off transitions are decisive.
constexpr double kHysteresisFraction = 0.5;

// Window length for the rolling median, in seconds.
constexpr double kSustainedWindowSeconds = 60.0;

// Minimum number of samples required before a decision is emitted.
// 30 samples (~ 1 sample/2s over 60s) is the floor below which the median
// is too noisy to interpret.
constexpr int kMinSamplesForDecision = 30;

struct TiltSample {
    double timestamp_s = 0.0;  // glfwGetTime() or equivalent monotonic clock
    double tiltDeg = 0.0;      // residual pitch+roll magnitude in degrees
};

// Decision returned by EvaluateTilt below. medianDeg is -1 when the window
// is too short for a meaningful decision (fewer than kMinSamplesForDecision
// samples in [now-windowSec, now]); in that case, sustainedDisagreement
// equals the previous state (wasAlarmed) -- we don't toggle on missing data.
struct TiltDecision {
    bool sustainedDisagreement = false;
    double medianDeg = -1.0;
};

// Median of tiltDeg over the time window [now-windowSec, now]. Returns -1
// if the window contains fewer than `minSamples` entries.
//
// Implementation: copy in-window tilts to a temp vector and run nth_element
// for the median. O(K) where K is in-window count. K is bounded by the
// caller's deque size (call site trims to ~60 entries for a 60s window at
// 1 Hz), so this is cheap.
inline double MedianOverWindow(const std::deque<TiltSample>& window,
                               double now_s,
                               double windowSec = kSustainedWindowSeconds,
                               int minSamples = kMinSamplesForDecision) {
    const double cutoff = now_s - windowSec;
    std::vector<double> inWindow;
    inWindow.reserve(window.size());
    for (const auto& s : window) {
        if (s.timestamp_s >= cutoff) inWindow.push_back(s.tiltDeg);
    }
    if (static_cast<int>(inWindow.size()) < minSamples) return -1.0;
    auto mid = inWindow.begin() + static_cast<std::ptrdiff_t>(inWindow.size() / 2);
    std::nth_element(inWindow.begin(), mid, inWindow.end());
    return *mid;
}

// Compute the sustained-disagreement decision given the current window and
// the prior alarm state. Pure: same inputs always produce same outputs.
// The caller is responsible for storing wasAlarmed across ticks and for
// emitting any log annotation on transitions.
//
// Hysteresis:
//   - From not-alarmed: flip to alarmed when median > thresholdDeg.
//   - From alarmed: stay alarmed until median < thresholdDeg * hysteresisFraction.
//   - When the window is too short (median < 0): hold the prior state.
inline TiltDecision EvaluateTilt(const std::deque<TiltSample>& window,
                                 double now_s,
                                 bool wasAlarmed,
                                 double thresholdDeg = kSustainedTiltThresholdDeg,
                                 double hysteresisFraction = kHysteresisFraction,
                                 double windowSec = kSustainedWindowSeconds,
                                 int minSamples = kMinSamplesForDecision) {
    TiltDecision out;
    out.medianDeg = MedianOverWindow(window, now_s, windowSec, minSamples);
    if (out.medianDeg < 0.0) {
        out.sustainedDisagreement = wasAlarmed;
        return out;
    }
    if (wasAlarmed) {
        out.sustainedDisagreement = out.medianDeg >= thresholdDeg * hysteresisFraction;
    } else {
        out.sustainedDisagreement = out.medianDeg > thresholdDeg;
    }
    return out;
}

} // namespace spacecal::gravity
