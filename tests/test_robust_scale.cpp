// Pin tests for the Qn-scale + Tukey biweight helpers used by the opt-in
// IRLS path. Pure helpers, header-only, testable in isolation against
// synthetic distributions with known properties.

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "RobustScale.h"

using spacecal::robust::Qn;
using spacecal::robust::TukeyWeight;
using spacecal::robust::kQnConsistency;
using spacecal::robust::kTukeyTune;

// --- Qn-scale ---------------------------------------------------------------

TEST(RobustScaleQn, ReturnsZeroOnEmpty) {
    EXPECT_DOUBLE_EQ(Qn({}), 0.0);
}

TEST(RobustScaleQn, ReturnsZeroOnSingleton) {
    EXPECT_DOUBLE_EQ(Qn({3.14}), 0.0);
}

TEST(RobustScaleQn, TwoPointDistanceTimesConsistency) {
    // n=2: only one pair, distance |a-b|. Qn = |a-b| * c = 2 * 2.2219.
    EXPECT_NEAR(Qn({0.0, 2.0}), 2.0 * kQnConsistency, 1e-12);
    EXPECT_NEAR(Qn({-1.0, 1.0}), 2.0 * kQnConsistency, 1e-12);
}

TEST(RobustScaleQn, ConstantSeriesIsZero) {
    // All pairwise distances are 0; Qn = 0 regardless of n.
    std::vector<double> ones(50, 1.0);
    EXPECT_DOUBLE_EQ(Qn(ones), 0.0);
}

TEST(RobustScaleQn, RecoversGaussianSigmaApproximately) {
    // Qn is asymptotically consistent with Gaussian sigma. With a 5000-sample
    // N(0, 1) draw the recovered Qn should sit within ~5% of 1.0.
    std::mt19937 rng(2024);
    std::normal_distribution<double> g(0.0, 1.0);
    std::vector<double> xs;
    xs.reserve(5000);
    for (int i = 0; i < 5000; i++) xs.push_back(g(rng));
    const double q = Qn(xs);
    EXPECT_GT(q, 0.95);
    EXPECT_LT(q, 1.05);
}

TEST(RobustScaleQn, ResistsFiftyPercentOutliers) {
    // 50% breakdown: half the data can be arbitrarily corrupted and Qn
    // should still report close to the clean sigma. Mix 50 samples from
    // N(0, 1) with 49 huge outliers at +1000.
    std::mt19937 rng(2024);
    std::normal_distribution<double> g(0.0, 1.0);
    std::vector<double> xs;
    for (int i = 0; i < 50; i++) xs.push_back(g(rng));
    for (int i = 0; i < 49; i++) xs.push_back(1000.0);
    const double q = Qn(xs);
    // Should still be order-of-magnitude 1, not 1000. (A non-robust
    // estimator like std-dev would explode here.)
    EXPECT_LT(q, 5.0)
        << "Qn lost robustness under 49% outlier contamination: " << q;
}

TEST(RobustScaleQn, LargeInputDoesNotOOM) {
    // Regression: the production IRLS feeds the Qn helper with O(N^2) per-row
    // residuals -- 250 samples produces ~62k rows. The original Qn allocated
    // a (rows choose 2) pair-distance vector, which is ~1.9e9 doubles =
    // 15 GB on 62k input -- bad_alloc -> overlay crash. The user hit this
    // by enabling the Tukey/Qn toggle at runtime on 2026-05-05; build
    // 2026.5.5.16-15D8 crashed within 0.5 sec of the toggle flip.
    //
    // The fix stride-samples the input down to kQnInputCap when it is
    // larger. This test validates: (a) no crash on a 100k-element input,
    // (b) the recovered scale is still close to the true sigma.
    std::mt19937 rng(2024);
    std::normal_distribution<double> g(0.0, 1.0);
    std::vector<double> xs;
    xs.reserve(100000);
    for (int i = 0; i < 100000; i++) xs.push_back(g(rng));
    const double q = Qn(xs);
    EXPECT_GT(q, 0.85);
    EXPECT_LT(q, 1.15);
}

TEST(RobustScaleQn, HandlesAsymmetricDistribution) {
    // Qn does not assume symmetry of the residual distribution. Mix samples
    // from a left-skewed distribution: 60 samples in [-0.1, 0.0] and 40 in
    // [0.0, 1.0]. MAD on this is biased by the symmetry assumption; Qn
    // returns a value that reflects the actual spread.
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> left(-0.1, 0.0);
    std::uniform_real_distribution<double> right(0.0, 1.0);
    std::vector<double> xs;
    for (int i = 0; i < 60; i++) xs.push_back(left(rng));
    for (int i = 0; i < 40; i++) xs.push_back(right(rng));
    const double q = Qn(xs);
    // Spread is dominated by the right tail; Qn should report a positive
    // value comparable to the right-tail width (not zero like MAD-floor would).
    EXPECT_GT(q, 0.05);
    EXPECT_LT(q, 5.0);
}

// --- Tukey biweight ---------------------------------------------------------

TEST(RobustScaleTukey, MaxWeightAtZeroResidual) {
    EXPECT_DOUBLE_EQ(TukeyWeight(0.0, /*c=*/1.0), 1.0);
    EXPECT_DOUBLE_EQ(TukeyWeight(0.0, /*c=*/4.685), 1.0);
}

TEST(RobustScaleTukey, ZeroAtAndBeyondCutoff) {
    // Redescending: residuals at or beyond c get exactly zero weight.
    EXPECT_DOUBLE_EQ(TukeyWeight(/*r=*/1.0, /*c=*/1.0), 0.0);
    EXPECT_DOUBLE_EQ(TukeyWeight(/*r=*/1.5, /*c=*/1.0), 0.0);
    EXPECT_DOUBLE_EQ(TukeyWeight(/*r=*/-1.0, /*c=*/1.0), 0.0);
    EXPECT_DOUBLE_EQ(TukeyWeight(/*r=*/-2.0, /*c=*/1.0), 0.0);
    EXPECT_DOUBLE_EQ(TukeyWeight(/*r=*/100.0, /*c=*/4.685), 0.0);
}

TEST(RobustScaleTukey, MonotonicallyDecreasingInsideCutoff) {
    // Within [-c, c], weight strictly decreases as |r| grows.
    constexpr double c = 4.685;
    double prev = 1.0 + 1e-9;
    for (double r = 0.0; r < c; r += 0.1) {
        const double w = TukeyWeight(r, c);
        EXPECT_LT(w, prev) << "non-monotonic at r=" << r;
        prev = w;
    }
}

TEST(RobustScaleTukey, SymmetricAroundZero) {
    constexpr double c = 4.685;
    for (double r = 0.1; r < c; r += 0.5) {
        EXPECT_NEAR(TukeyWeight(r, c), TukeyWeight(-r, c), 1e-12);
    }
}

TEST(RobustScaleTukey, GuardsAgainstNonPositiveCutoff) {
    EXPECT_DOUBLE_EQ(TukeyWeight(0.5, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(TukeyWeight(0.5, -1.0), 0.0);
}

TEST(RobustScaleTukey, ContrastsWithCauchyAtLargeResiduals) {
    // Cauchy w(r=10c) = 1 / (1 + 100) ~ 0.0099 -- small but nonzero.
    // Tukey w(r=10c) = 0 exactly. This is the redescending property.
    constexpr double c = 1.0;
    const double rLarge = 10.0 * c;
    const double cauchyW = 1.0 / (1.0 + (rLarge / c) * (rLarge / c));
    EXPECT_GT(cauchyW, 0.0);
    EXPECT_DOUBLE_EQ(TukeyWeight(rLarge, c), 0.0);
}
