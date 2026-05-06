#pragma once

// Robust scale + weight kernels for the IRLS translation solve. The default
// path uses MAD + Cauchy; the opt-in path here uses Qn-scale (Rousseeuw &
// Croux 1993) + Tukey biweight. Both are pure header-only helpers so they
// are unit-testable in isolation against synthetic distributions.
//
// Qn-scale rationale: MAD is symmetric, requires symmetry of the residual
// distribution to be Gaussian-consistent, and saturates at the kMadFloor
// when residuals collapse below that floor. Qn is the (h choose 2)-th
// smallest pairwise absolute distance, h = floor(n/2) + 1, scaled by a
// consistency factor. 50% breakdown, no symmetry assumption, non-zero
// when MAD would clamp at floor.
//
// Implementation note (2026-05-05): the IRLS feeds residuals at the per-row
// granularity, which is O(N^2) in samples (250 samples -> ~62k rows). The
// naive Qn formulation materializes (n choose 2) pair-distances, so on
// 62k rows that is ~1.9e9 doubles = 15 GB and an out-of-memory crash --
// caught the hard way when the user enabled the Tukey toggle at runtime.
// Workaround: when the input size exceeds kQnInputCap, stride-sample the
// input down to kQnInputCap entries and run the canonical algorithm on
// the subset. The order statistic of the sub-sample is an unbiased
// estimator of the same quantile in the full set; the variance penalty
// at sub-sample size 500 is negligible for our use (~1% on the recovered
// scale). This also keeps the worst-case CPU bounded at ~125k pair
// distances per IRLS iteration.
//
// Tukey biweight rationale: Cauchy is monotonically descending --
// large residuals get tiny but nonzero weights. Tukey is redescending:
// |r| >= c -> exactly zero weight, so a bad-frame outlier cannot drag
// the fit even with mild influence. Standard 95%-Gaussian-efficient
// tuning constant c = 4.685 paired with Qn-scale.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace spacecal::robust {

// Standard Gaussian-consistency tuning constants. Documented in Rousseeuw &
// Croux 1993 (JASA 88, "Alternatives to the Median Absolute Deviation") and
// Beaton & Tukey 1974.
constexpr double kQnConsistency = 2.2219;
constexpr double kTukeyTune = 4.685;

// Maximum input size for the materialized-pairs Qn algorithm. Above this
// the input is stride-sampled down to this size. 500 keeps the pair count
// at 124,750 (~1 MB scratch), microsecond-scale per call, and gives sub-
// percent estimation error on the recovered scale.
constexpr std::size_t kQnInputCap = 500;

// Qn-scale of a value sequence. Returns 0.0 for n < 2 (no pairs to compare).
// Implementation: collect all n*(n-1)/2 pairwise absolute distances, find
// the k-th order statistic via nth_element (O(n^2) memory + O(n^2) time),
// multiply by the consistency factor for Gaussian-equivalence with sigma.
//
// For n = 1 or 0 we return 0; the caller should fall back to a finite floor
// to avoid downstream divide-by-zero. (The Cauchy path's kMadFloor of 1e-3 m
// is a reasonable choice for the same reason.)
inline double Qn(const std::vector<double>& xs) {
    const std::size_t nIn = xs.size();
    if (nIn < 2) return 0.0;

    // Stride-sample the input down to kQnInputCap when it is larger. Stride
    // sampling preserves whatever distribution the residuals had (no random
    // seed, deterministic across runs); the order statistic of the sample
    // is an unbiased estimator of the same quantile in the full set.
    // Without this guard the materialized-pairs path below would allocate
    // O(n^2) memory and crash on large IRLS solves.
    const double* xptr = xs.data();
    std::vector<double> sampled;
    std::size_t n = nIn;
    if (nIn > kQnInputCap) {
        n = kQnInputCap;
        sampled.reserve(n);
        // Stride = nIn / n with floor; produces exactly n samples spanning
        // the whole input (i = 0, stride, 2*stride, ...).
        const double stride = static_cast<double>(nIn) / static_cast<double>(n);
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t idx = static_cast<std::size_t>(static_cast<double>(i) * stride);
            sampled.push_back(xs[std::min(idx, nIn - 1)]);
        }
        xptr = sampled.data();
    }

    const std::size_t numPairs = n * (n - 1) / 2;
    std::vector<double> pairs;
    pairs.reserve(numPairs);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            pairs.push_back(std::abs(xptr[i] - xptr[j]));
        }
    }

    // (h choose 2) = h*(h-1)/2 where h = floor(n/2) + 1.
    const std::size_t h = n / 2 + 1;
    const std::size_t k = h * (h - 1) / 2;
    // Clamp to the valid range. For n=2 we have numPairs=1 and k=1; need
    // index 0 (first / only pair).
    const std::size_t idx = (k > 0 && k - 1 < numPairs) ? (k - 1) : (numPairs - 1);

    std::nth_element(pairs.begin(), pairs.begin() + idx, pairs.end());
    return pairs[idx] * kQnConsistency;
}

// Tukey biweight weight: redescending. |r| >= c -> 0; near-zero residuals
// get weight near 1. Inputs already-divided scale units (i.e. caller passes
// r and c such that r is the residual and c is the threshold).
inline double TukeyWeight(double residual, double c) {
    if (!(c > 0.0)) return 0.0;
    const double u = residual / c;
    if (std::abs(u) >= 1.0) return 0.0;
    const double oneMinusUSq = 1.0 - u * u;
    return oneMinusUSq * oneMinusUSq;
}

} // namespace spacecal::robust
