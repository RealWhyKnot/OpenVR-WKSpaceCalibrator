// Pin tests for the gravity-axis disagreement diagnostic. Pure helpers in
// src/overlay/TiltDiagnostic.h compute a rolling-median tilt over a fixed
// time window and decide whether the disagreement is "sustained" with
// hysteresis. The contract these tests pin:
//
//   1. Median is correctly computed from in-window samples only; out-of-
//      window samples don't influence the median.
//   2. Decision returns -1 median when fewer than minSamples samples are
//      in the window (we don't alarm on too-little data).
//   3. From not-alarmed: decision flips ON when median > threshold.
//   4. From alarmed: decision stays ON until median < threshold *
//      hysteresisFraction (no flapping near the boundary).
//   5. When the window is too short, the prior alarm state is preserved.
//
// All tests construct synthetic TiltSample sequences and verify the pure-
// function behavior directly. No state, no dependencies on the overlay.

#include <gtest/gtest.h>

#include <deque>

#include "TiltDiagnostic.h"

using spacecal::gravity::TiltSample;
using spacecal::gravity::TiltDecision;
using spacecal::gravity::EvaluateTilt;
using spacecal::gravity::MedianOverWindow;
using spacecal::gravity::kSustainedTiltThresholdDeg;
using spacecal::gravity::kHysteresisFraction;
using spacecal::gravity::kSustainedWindowSeconds;
using spacecal::gravity::kMinSamplesForDecision;

namespace {

// Build a window of `n` evenly-spaced samples ending at `now_s`, all with
// the same tilt value. Useful for the "sustained at X deg" case.
std::deque<TiltSample> EvenWindow(int n, double tiltDeg, double now_s,
                                  double spacing_s = 1.0) {
    std::deque<TiltSample> w;
    for (int i = 0; i < n; i++) {
        TiltSample s;
        s.timestamp_s = now_s - static_cast<double>(n - 1 - i) * spacing_s;
        s.tiltDeg = tiltDeg;
        w.push_back(s);
    }
    return w;
}

} // namespace

// --- MedianOverWindow ---------------------------------------------------------

TEST(TiltMedian, ReturnsMinusOneWhenWindowTooShort) {
    auto w = EvenWindow(/*n=*/10, /*tiltDeg=*/2.0, /*now=*/100.0);
    EXPECT_LT(MedianOverWindow(w, 100.0), 0.0);  // 10 < kMinSamplesForDecision (30)
}

TEST(TiltMedian, ComputesMedianOfInWindowSamplesOnly) {
    // 30 samples at 1 sample/s, ending at t=100. Half at 0.5 deg, half at 2.0.
    std::deque<TiltSample> w;
    for (int i = 0; i < 15; i++) w.push_back({100.0 - (29 - i), 0.5});
    for (int i = 0; i < 15; i++) w.push_back({100.0 - (14 - i), 2.0});
    // Median of [0.5 x15, 2.0 x15] is 2.0 (16th element after sort).
    const double m = MedianOverWindow(w, 100.0);
    EXPECT_NEAR(m, 2.0, 1e-9);
}

TEST(TiltMedian, IgnoresSamplesOutsideWindow) {
    // 30 in-window at 0.3 deg + 100 out-of-window at 5 deg.
    std::deque<TiltSample> w;
    // Out-of-window: timestamps far before the window cutoff (now - 60s).
    for (int i = 0; i < 100; i++) w.push_back({0.0, 5.0});  // t=0, way before now=200
    for (int i = 0; i < 30; i++) w.push_back({200.0 - (29 - i), 0.3});
    const double m = MedianOverWindow(w, 200.0);
    EXPECT_NEAR(m, 0.3, 1e-9);
}

TEST(TiltMedian, RespectsCustomMinSamplesArg) {
    auto w = EvenWindow(20, 1.5, 100.0);
    // Default minSamples (30): window of 20 should return -1.
    EXPECT_LT(MedianOverWindow(w, 100.0), 0.0);
    // Custom minSamples=10: should compute median.
    EXPECT_NEAR(MedianOverWindow(w, 100.0, kSustainedWindowSeconds, 10), 1.5, 1e-9);
}

// --- EvaluateTilt: not-alarmed → alarmed transition --------------------------

TEST(TiltEvaluate, NotAlarmedBelowThresholdStaysQuiet) {
    auto w = EvenWindow(40, /*tiltDeg=*/0.5, 100.0);
    auto d = EvaluateTilt(w, 100.0, /*wasAlarmed=*/false);
    EXPECT_FALSE(d.sustainedDisagreement);
    EXPECT_NEAR(d.medianDeg, 0.5, 1e-9);
}

TEST(TiltEvaluate, NotAlarmedAtThresholdStaysQuiet) {
    // Strict > threshold to flip. At-threshold should NOT alarm.
    auto w = EvenWindow(40, kSustainedTiltThresholdDeg, 100.0);
    auto d = EvaluateTilt(w, 100.0, false);
    EXPECT_FALSE(d.sustainedDisagreement);
}

TEST(TiltEvaluate, NotAlarmedAboveThresholdFlipsOn) {
    auto w = EvenWindow(40, kSustainedTiltThresholdDeg + 0.1, 100.0);
    auto d = EvaluateTilt(w, 100.0, false);
    EXPECT_TRUE(d.sustainedDisagreement);
}

// --- EvaluateTilt: alarmed → not-alarmed transition with hysteresis ---------

TEST(TiltEvaluate, AlarmedAboveHysteresisStaysOn) {
    // 0.6 deg is above 1.0*0.5=0.5 hysteresis floor; once alarmed, stay alarmed.
    auto w = EvenWindow(40, 0.6, 100.0);
    auto d = EvaluateTilt(w, 100.0, /*wasAlarmed=*/true);
    EXPECT_TRUE(d.sustainedDisagreement);
}

TEST(TiltEvaluate, AlarmedBelowHysteresisFlipsOff) {
    // 0.4 deg is below 0.5 hysteresis floor → flip off.
    auto w = EvenWindow(40, 0.4, 100.0);
    auto d = EvaluateTilt(w, 100.0, /*wasAlarmed=*/true);
    EXPECT_FALSE(d.sustainedDisagreement);
}

TEST(TiltEvaluate, AlarmedAtHysteresisStaysOn) {
    // 0.5 deg is exactly at the hysteresis floor; >= hysteresis should hold.
    auto w = EvenWindow(40, 0.5, 100.0);
    auto d = EvaluateTilt(w, 100.0, /*wasAlarmed=*/true);
    EXPECT_TRUE(d.sustainedDisagreement);
}

// --- Hysteresis prevents flapping at the threshold ---------------------------

TEST(TiltEvaluate, HysteresisPreventsFlappingAtBoundary) {
    // Median oscillates between 0.95 and 1.05 deg around the 1.0 threshold.
    // Hysteresis should keep the alarm in whatever state it started in.
    auto wHigh = EvenWindow(40, 1.05, 100.0);
    auto wLow  = EvenWindow(40, 0.95, 100.0);

    // Starting from not-alarmed, only crossing strictly above threshold flips on.
    auto d1 = EvaluateTilt(wHigh, 100.0, false);
    EXPECT_TRUE(d1.sustainedDisagreement);

    // Now alarmed. Drop to 0.95: above hysteresis (0.5), stay alarmed.
    auto d2 = EvaluateTilt(wLow, 100.0, true);
    EXPECT_TRUE(d2.sustainedDisagreement);

    // Bounce back to 1.05 with the on state: still alarmed.
    auto d3 = EvaluateTilt(wHigh, 100.0, true);
    EXPECT_TRUE(d3.sustainedDisagreement);
}

// --- Insufficient data preserves prior state --------------------------------

TEST(TiltEvaluate, ShortWindowPreservesPriorState_WasAlarmed) {
    auto w = EvenWindow(5, 5.0, 100.0);  // only 5 samples; below minSamples=30
    auto d = EvaluateTilt(w, 100.0, /*wasAlarmed=*/true);
    EXPECT_TRUE(d.sustainedDisagreement);  // hold prior
    EXPECT_LT(d.medianDeg, 0.0);
}

TEST(TiltEvaluate, ShortWindowPreservesPriorState_WasNotAlarmed) {
    auto w = EvenWindow(5, 5.0, 100.0);
    auto d = EvaluateTilt(w, 100.0, /*wasAlarmed=*/false);
    EXPECT_FALSE(d.sustainedDisagreement);
    EXPECT_LT(d.medianDeg, 0.0);
}

// --- Real-world scenario: warm-up period + sustained disagreement ------------

TEST(TiltEvaluate, WarmUpThenSustainedDisagreement) {
    // Three phases at 1 sample/sec on a 60-sample window:
    //   ticks  1..29: warm-up (fewer than 30 in window), alarm stays prior
    //   tick   30..40: 30+ samples in window, all at 1.5 deg, alarm flips on
    //   ticks 41..101: 0.3 deg samples accumulate; once the in-window 0.3
    //                  count exceeds 30 the median drops below the
    //                  hysteresis floor (0.5 deg) and alarm flips off
    std::deque<TiltSample> w;
    bool alarmed = false;

    auto runTick = [&](double t, double tiltDeg) {
        w.push_back({t, tiltDeg});
        const double cutoff = t - kSustainedWindowSeconds;
        while (!w.empty() && w.front().timestamp_s < cutoff) w.pop_front();
        auto d = EvaluateTilt(w, t, alarmed);
        alarmed = d.sustainedDisagreement;
        return d;
    };

    // Warm-up: 20 ticks at 1.5 deg, only 20 in window, below minSamples=30.
    for (int i = 1; i <= 20; i++) runTick(static_cast<double>(i), 1.5);
    EXPECT_FALSE(alarmed);

    // Sustained 1.5 deg for 20 more ticks. By tick 30 we have 30 in-window
    // samples and the median (1.5) crosses the threshold, flipping alarm on.
    for (int i = 21; i <= 40; i++) runTick(static_cast<double>(i), 1.5);
    EXPECT_TRUE(alarmed);

    // Drop to 0.3 deg. 0.3 < hysteresis (0.5) so the alarm WILL flip, but
    // only once the median is below 0.5. With 1Hz sampling and a 60s
    // window, the 0.3-deg samples need to outnumber the 1.5-deg samples in
    // the window, which happens around tick 71 (cutoff=11, 31 samples at
    // 0.3 vs 29 at 1.5 -> median = 0.3). We run to 101 to be well clear.
    for (int i = 41; i <= 101; i++) runTick(static_cast<double>(i), 0.3);
    EXPECT_FALSE(alarmed);
}
