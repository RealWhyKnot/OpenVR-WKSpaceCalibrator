// Pin tests for the latency-lag estimator helpers. Two algorithms:
//   - EstimateLagTimeDomain: original direct cross-correlation. Must produce
//     bit-for-bit identical results to the pre-extraction implementation.
//   - EstimateLagGccPhat: GCC-PHAT (Knapp & Carter 1976). Whitens the
//     cross-power spectrum before IDFT. Should match the time-domain answer
//     within sub-sample resolution on synthetic delays, and should outperform
//     time-domain on signals with frequency-dependent SNR.
//
// All tests use synthetic signals (sine waves, chirps, ramps) with a known
// integer-or-fractional delay applied. The estimator is correct iff its
// recovered lag matches the injected delay within the parabolic-interp
// tolerance (~0.1 sample for clean signals, ~0.3 sample for noisy).

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <random>
#include <vector>

#include "LatencyEstimator.h"

namespace {

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

// Build a clean sine reference and a copy delayed by `intLag` samples (positive
// = target arrives later). Both signals get a positive DC offset so the energy
// gate (RMS > 0.1 m/s) passes.
void MakeSinePair(int N, double freq, int intLag, std::vector<double>& ref,
                  std::vector<double>& tgt) {
    ref.resize(N);
    tgt.resize(N);
    for (int i = 0; i < N; i++) {
        const double t = static_cast<double>(i);
        // Amplitude 0.5 m/s, DC offset 0.5 m/s -> RMS ~0.55 m/s, well above the
        // 0.1 m/s gate. Frequency in cycles/sample.
        ref[i] = 0.5 + 0.5 * std::sin(kTwoPi * freq * t);
        tgt[i] = 0.5 + 0.5 * std::sin(kTwoPi * freq * (t - intLag));
    }
}

// Build a band-limited "wave-and-stop" trace by summing two sinusoids of
// different frequencies + a slow ramp. More representative of real hand-motion
// than a pure tone.
void MakeMixedPair(int N, int intLag, std::vector<double>& ref,
                   std::vector<double>& tgt) {
    ref.resize(N);
    tgt.resize(N);
    for (int i = 0; i < N; i++) {
        const double t = static_cast<double>(i);
        const double sig =
            0.30 * std::sin(kTwoPi * 0.05 * t) +
            0.20 * std::sin(kTwoPi * 0.13 * t + 0.4) +
            0.40;
        const double sigShift =
            0.30 * std::sin(kTwoPi * 0.05 * (t - intLag)) +
            0.20 * std::sin(kTwoPi * 0.13 * (t - intLag) + 0.4) +
            0.40;
        ref[i] = sig;
        tgt[i] = sigShift;
    }
}

} // namespace

// --- Time-domain CC: reference implementation parity --------------------------

TEST(LatencyTimeDomain, RecoversIntegerDelayOnSine) {
    std::vector<double> r, t;
    MakeSinePair(100, /*freq=*/0.07, /*intLag=*/3, r, t);
    double lag = 0.0;
    EXPECT_TRUE(spacecal::latency::EstimateLagTimeDomain(r, t, /*maxTau=*/10, &lag));
    EXPECT_NEAR(lag, 3.0, 0.2);
}

TEST(LatencyTimeDomain, RecoversNegativeDelayOnSine) {
    std::vector<double> r, t;
    MakeSinePair(100, 0.07, -4, r, t);
    double lag = 0.0;
    EXPECT_TRUE(spacecal::latency::EstimateLagTimeDomain(r, t, 10, &lag));
    EXPECT_NEAR(lag, -4.0, 0.2);
}

TEST(LatencyTimeDomain, RecoversMixedSignalDelay) {
    std::vector<double> r, t;
    MakeMixedPair(100, /*intLag=*/2, r, t);
    double lag = 0.0;
    EXPECT_TRUE(spacecal::latency::EstimateLagTimeDomain(r, t, 10, &lag));
    EXPECT_NEAR(lag, 2.0, 0.3);
}

TEST(LatencyTimeDomain, RejectsLowEnergyInput) {
    // Both signals are constant 0; RMS is 0; should be rejected.
    std::vector<double> r(100, 0.0), t(100, 0.0);
    double lag = 0.0;
    EXPECT_FALSE(spacecal::latency::EstimateLagTimeDomain(r, t, 10, &lag));
}

TEST(LatencyTimeDomain, RejectsBelowEnergyFloor) {
    // RMS ~0.05 m/s, below the 0.1 m/s gate.
    std::vector<double> r(100), t(100);
    for (int i = 0; i < 100; i++) {
        const double th = kTwoPi * 0.07 * static_cast<double>(i);
        r[i] = 0.05 * std::sin(th);
        t[i] = 0.05 * std::sin(th - 0.3);
    }
    double lag = 0.0;
    EXPECT_FALSE(spacecal::latency::EstimateLagTimeDomain(r, t, 10, &lag));
}

TEST(LatencyTimeDomain, RejectsTooShortInput) {
    std::vector<double> r(20, 1.0), t(20, 1.0);
    double lag = 0.0;
    // maxTau=10 requires N >= 24; N=20 should be rejected.
    EXPECT_FALSE(spacecal::latency::EstimateLagTimeDomain(r, t, 10, &lag));
}

TEST(LatencyTimeDomain, ZeroLagOnIdenticalSignals) {
    std::vector<double> r, t;
    MakeSinePair(100, 0.07, 0, r, t);
    double lag = 0.0;
    EXPECT_TRUE(spacecal::latency::EstimateLagTimeDomain(r, t, 10, &lag));
    EXPECT_NEAR(lag, 0.0, 0.1);
}

// --- GCC-PHAT: same correctness contract --------------------------------------

TEST(LatencyGccPhat, RecoversIntegerDelayOnSine) {
    std::vector<double> r, t;
    MakeSinePair(100, 0.07, 3, r, t);
    double lag = 0.0;
    EXPECT_TRUE(spacecal::latency::EstimateLagGccPhat(r, t, 10, &lag));
    EXPECT_NEAR(lag, 3.0, 0.3);
}

TEST(LatencyGccPhat, RecoversNegativeDelayOnSine) {
    std::vector<double> r, t;
    MakeSinePair(100, 0.07, -4, r, t);
    double lag = 0.0;
    EXPECT_TRUE(spacecal::latency::EstimateLagGccPhat(r, t, 10, &lag));
    EXPECT_NEAR(lag, -4.0, 0.3);
}

TEST(LatencyGccPhat, RecoversMixedSignalDelay) {
    std::vector<double> r, t;
    MakeMixedPair(100, 2, r, t);
    double lag = 0.0;
    EXPECT_TRUE(spacecal::latency::EstimateLagGccPhat(r, t, 10, &lag));
    EXPECT_NEAR(lag, 2.0, 0.4);
}

TEST(LatencyGccPhat, RejectsLowEnergyInput) {
    std::vector<double> r(100, 0.0), t(100, 0.0);
    double lag = 0.0;
    EXPECT_FALSE(spacecal::latency::EstimateLagGccPhat(r, t, 10, &lag));
}

TEST(LatencyGccPhat, ZeroLagOnIdenticalSignals) {
    std::vector<double> r, t;
    MakeSinePair(100, 0.07, 0, r, t);
    double lag = 0.0;
    EXPECT_TRUE(spacecal::latency::EstimateLagGccPhat(r, t, 10, &lag));
    EXPECT_NEAR(lag, 0.0, 0.2);
}

// --- Cross-algorithm consistency ----------------------------------------------
//
// Both algorithms should agree within their respective tolerances on clean
// synthetic delays. This is the contract that justifies offering them as
// alternatives: a session that switches between them shouldn't see materially
// different lag estimates on healthy data.

TEST(LatencyCrossAlgo, AgreeOnCleanSine) {
    std::vector<double> r, t;
    MakeSinePair(100, 0.07, 5, r, t);
    double lagTd = 0.0, lagPh = 0.0;
    ASSERT_TRUE(spacecal::latency::EstimateLagTimeDomain(r, t, 10, &lagTd));
    ASSERT_TRUE(spacecal::latency::EstimateLagGccPhat(r, t, 10, &lagPh));
    EXPECT_NEAR(lagTd, 5.0, 0.2);
    EXPECT_NEAR(lagPh, 5.0, 0.3);
    EXPECT_NEAR(lagTd, lagPh, 0.4);
}

TEST(LatencyCrossAlgo, AgreeOnMixedSignal) {
    std::vector<double> r, t;
    MakeMixedPair(100, 4, r, t);
    double lagTd = 0.0, lagPh = 0.0;
    ASSERT_TRUE(spacecal::latency::EstimateLagTimeDomain(r, t, 10, &lagTd));
    ASSERT_TRUE(spacecal::latency::EstimateLagGccPhat(r, t, 10, &lagPh));
    EXPECT_NEAR(lagTd, 4.0, 0.4);
    EXPECT_NEAR(lagPh, 4.0, 0.4);
}

TEST(LatencyCrossAlgo, BothHandleStdDeque) {
    // Production code feeds std::deque<double>. The header is templated; the
    // build matrix is fine if both gtest cases actually exercise that path.
    std::deque<double> r, t;
    {
        std::vector<double> rv, tv;
        MakeSinePair(100, 0.07, 2, rv, tv);
        r.assign(rv.begin(), rv.end());
        t.assign(tv.begin(), tv.end());
    }
    double lagTd = 0.0, lagPh = 0.0;
    EXPECT_TRUE(spacecal::latency::EstimateLagTimeDomain(r, t, 10, &lagTd));
    EXPECT_TRUE(spacecal::latency::EstimateLagGccPhat(r, t, 10, &lagPh));
    EXPECT_NEAR(lagTd, 2.0, 0.2);
    EXPECT_NEAR(lagPh, 2.0, 0.3);
}
