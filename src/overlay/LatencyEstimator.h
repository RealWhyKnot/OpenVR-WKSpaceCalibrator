#pragma once

// Latency-lag estimation between two speed-magnitude time series.
//
// Provides two algorithms:
//   - EstimateLagTimeDomain: the original direct cross-correlation in the time
//     domain with quadratic peak interpolation. O(N * maxTau) per call. The
//     2026-05-04 math review marked this "empirically validated, not a sore
//     point"; keep as the default.
//   - EstimateLagGccPhat: GCC-PHAT (Knapp & Carter 1976, IEEE TASSP 24(4)).
//     Whitens the cross-power spectrum before inverse-DFT so the peak is
//     sharper across frequency bands of unequal SNR (relevant when one
//     tracking system's IMU fusion low-passes angular velocity at a
//     different cutoff than the other). O(N^2) for the in-place DFTs at
//     N=100; trivial at our once-per-second cadence. Opt-in.
//
// Both functions:
//   - Take container-agnostic random-access ranges (std::vector / std::deque
//     work transparently via iterator pairs).
//   - Apply the same RMS-speed energy gate (0.1 m/s) so we don't chase noise
//     when the user is standing still.
//   - Mean-subtract so a constant-offset signal doesn't dominate.
//   - Search lag in [-maxTau, +maxTau] with parabolic sub-sample
//     interpolation around the integer peak.
//   - Return false if the energy gate fails or the input is too short.
//   - On success, *lagSamplesOut is the lag in samples; positive means the
//     target signal lags the reference (target arrives later).
//
// Both functions are pure: no global state, no allocations beyond the local
// scratch buffers. Suitable for direct unit testing without the overlay
// runtime.
//
// Header-only deliberately: keeps the math co-located with the contract and
// makes it trivial to unit-test (pure-function extraction pattern, same as
// MotionGate.h, GeometryShiftDetector.h, WatchdogDecisions.h, etc).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace spacecal::latency {

constexpr double kMinRmsSpeed = 0.1; // m/s

// Internal: parabolic interpolation around the integer peak of a 1D array.
// Returns the fractional offset from the peak index, clamped to [-1, 1].
// Mirrors the original time-domain implementation byte-for-byte.
inline double ParabolicSubsamplePeak(const std::vector<double>& C, int bestIdx) {
    if (bestIdx <= 0 || bestIdx >= (int)C.size() - 1) return 0.0;
    const double y0 = C[bestIdx - 1];
    const double y1 = C[bestIdx];
    const double y2 = C[bestIdx + 1];
    const double denom = (y0 - 2.0 * y1 + y2);
    if (std::fabs(denom) <= 1e-12) return 0.0;
    double frac = 0.5 * (y0 - y2) / denom;
    if (!std::isfinite(frac) || std::fabs(frac) > 1.0) frac = 0.0;
    return frac;
}

// Internal: shared energy gate + mean-subtract. Returns true on pass; on
// success, populates `r` and `t` with mean-subtracted copies of the inputs.
template <class Container>
bool PrepareSeries(const Container& ref, const Container& tgt,
                   std::vector<double>& r, std::vector<double>& t) {
    const size_t N = ref.size();
    if (N < 4 || tgt.size() != N) return false;

    double refE = 0.0, tgtE = 0.0;
    for (size_t i = 0; i < N; i++) { refE += ref[i] * ref[i]; tgtE += tgt[i] * tgt[i]; }
    const double refRms = std::sqrt(refE / static_cast<double>(N));
    const double tgtRms = std::sqrt(tgtE / static_cast<double>(N));
    if (refRms < kMinRmsSpeed || tgtRms < kMinRmsSpeed) return false;

    double refMean = 0.0, tgtMean = 0.0;
    for (size_t i = 0; i < N; i++) { refMean += ref[i]; tgtMean += tgt[i]; }
    refMean /= static_cast<double>(N);
    tgtMean /= static_cast<double>(N);

    r.resize(N);
    t.resize(N);
    for (size_t i = 0; i < N; i++) { r[i] = ref[i] - refMean; t[i] = tgt[i] - tgtMean; }
    return true;
}

// Time-domain cross-correlation. Original algorithm; preserved bit-for-bit
// from the pre-extraction implementation.
template <class Container>
bool EstimateLagTimeDomain(const Container& ref, const Container& tgt,
                           int maxTau, double* lagSamplesOut) {
    const size_t N = ref.size();
    if (N < static_cast<size_t>(2 * maxTau + 4)) return false;

    std::vector<double> r, t;
    if (!PrepareSeries(ref, tgt, r, t)) return false;

    // C(tau) = sum_k r[k] * t[k + tau], for tau in [-maxTau, +maxTau].
    // Truncated biased estimator; fine since N >> maxTau.
    std::vector<double> C(2 * maxTau + 1, 0.0);
    int bestIdx = -1;
    double bestVal = -std::numeric_limits<double>::infinity();
    for (int tau = -maxTau; tau <= maxTau; tau++) {
        const int idx = tau + maxTau;
        double sum = 0.0;
        const int kStart = std::max(0, -tau);
        const int kEnd = std::min(static_cast<int>(N), static_cast<int>(N) - tau);
        for (int k = kStart; k < kEnd; k++) sum += r[k] * t[k + tau];
        C[idx] = sum;
        if (sum > bestVal) { bestVal = sum; bestIdx = idx; }
    }

    if (bestIdx <= 0 || bestIdx >= static_cast<int>(C.size()) - 1) {
        *lagSamplesOut = static_cast<double>(bestIdx - maxTau);
        return true;
    }
    *lagSamplesOut = static_cast<double>(bestIdx - maxTau) + ParabolicSubsamplePeak(C, bestIdx);
    return true;
}

// GCC-PHAT (Knapp & Carter 1976). Returns the same lag sign convention as
// EstimateLagTimeDomain: positive means the target signal lags the reference.
//
// Complex DFT/IDFT implemented inline as direct O(N^2) sums. At N=100 that
// is ~40k real multiplies per call -- well under the per-tick budget at our
// 1 Hz invocation rate. We avoid pulling an FFT dependency for what is a
// once-per-second computation.
template <class Container>
bool EstimateLagGccPhat(const Container& ref, const Container& tgt,
                        int maxTau, double* lagSamplesOut) {
    const size_t N = ref.size();
    if (N < static_cast<size_t>(2 * maxTau + 4)) return false;

    std::vector<double> r, t;
    if (!PrepareSeries(ref, tgt, r, t)) return false;

    // Forward DFT of each signal: X[k] = sum_n x[n] * exp(-i*2*pi*k*n/N).
    // Stored as paired arrays for real and imaginary parts (no <complex>
    // dependency keeps the header lean and the test build smaller).
    const int Ni = static_cast<int>(N);
    std::vector<double> xRe(N, 0.0), xIm(N, 0.0);
    std::vector<double> yRe(N, 0.0), yIm(N, 0.0);
    const double twoPiOverN = -2.0 * 3.14159265358979323846 / static_cast<double>(N);
    for (int k = 0; k < Ni; k++) {
        double xr = 0.0, xi = 0.0, yr = 0.0, yi = 0.0;
        for (int n = 0; n < Ni; n++) {
            const double th = twoPiOverN * static_cast<double>(k) * static_cast<double>(n);
            const double c = std::cos(th);
            const double s = std::sin(th);
            xr += r[n] * c;
            xi += r[n] * s;
            yr += t[n] * c;
            yi += t[n] * s;
        }
        xRe[k] = xr; xIm[k] = xi;
        yRe[k] = yr; yIm[k] = yi;
    }

    // Cross-power spectrum R_xy[k] = conj(X[k]) * Y[k]; then PHAT-normalize:
    // P_phat[k] = R_xy[k] / max(|R_xy[k]|, eps). This is the spectral
    // whitening step that distinguishes GCC-PHAT from plain cross-correlation.
    //
    // Convention check: for the discrete cross-correlation
    //   r_xy(tau) = sum_n x[n] * y[n + tau]
    // the DFT relation is R_xy[k] = conj(X[k]) * Y[k] (NOT X * conj(Y) -- that
    // gives conj(R_xy), whose IDFT is the time-reversed CC, peaking at -D
    // instead of +D for a target delayed by D samples). Matches the time-
    // domain implementation above and the standard Knapp-Carter formulation.
    std::vector<double> pRe(N, 0.0), pIm(N, 0.0);
    constexpr double kPhatFloor = 1e-12;
    for (int k = 0; k < Ni; k++) {
        const double pr = xRe[k] * yRe[k] + xIm[k] * yIm[k];   // Re(conj(X) * Y)
        const double pi = xRe[k] * yIm[k] - xIm[k] * yRe[k];   // Im(conj(X) * Y)
        const double mag = std::sqrt(pr * pr + pi * pi);
        const double inv = 1.0 / std::max(mag, kPhatFloor);
        pRe[k] = pr * inv;
        pIm[k] = pi * inv;
    }

    // Inverse DFT to time domain: c[n] = (1/N) * sum_k P_phat[k] * exp(+i*2*pi*k*n/N).
    // We only need the real part (cross-correlation is real for real inputs);
    // and we only evaluate it at lags in [-maxTau, +maxTau], which we map to
    // n = (lag % N + N) % N (negative lags wrap to the upper half of the
    // periodic IDFT output, by the standard cross-correlation/DFT identity).
    std::vector<double> C(2 * maxTau + 1, 0.0);
    int bestIdx = -1;
    double bestVal = -std::numeric_limits<double>::infinity();
    const double twoPiOverNpos = 2.0 * 3.14159265358979323846 / static_cast<double>(N);
    const double invN = 1.0 / static_cast<double>(N);
    for (int tau = -maxTau; tau <= maxTau; tau++) {
        // Wrap tau into [0, N).
        int n = tau % Ni;
        if (n < 0) n += Ni;
        double cr = 0.0;
        for (int k = 0; k < Ni; k++) {
            const double th = twoPiOverNpos * static_cast<double>(k) * static_cast<double>(n);
            const double c = std::cos(th);
            const double s = std::sin(th);
            // Re((pr + i*pi) * (c + i*s)) = pr*c - pi*s.
            cr += pRe[k] * c - pIm[k] * s;
        }
        cr *= invN;
        const int idx = tau + maxTau;
        C[idx] = cr;
        if (cr > bestVal) { bestVal = cr; bestIdx = idx; }
    }

    if (bestIdx <= 0 || bestIdx >= static_cast<int>(C.size()) - 1) {
        *lagSamplesOut = static_cast<double>(bestIdx - maxTau);
        return true;
    }
    *lagSamplesOut = static_cast<double>(bestIdx - maxTau) + ParabolicSubsamplePeak(C, bestIdx);
    return true;
}

} // namespace spacecal::latency
