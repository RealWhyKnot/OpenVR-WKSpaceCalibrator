// Tests for the predictive recovery-delta buffer. Pins the ring math, the
// recency-weighted mean direction, the consistency gate, and the per-tick
// rate cap.
//
// Why these tests matter: the deleted Phase 1+2 silent recal accepted a
// candidate without bounded-rate protection. Rec C records HMD jumps from
// the high-SNR 30 cm relocalization detector and predictively pre-corrects
// between events; the bounded rate cap and consistency gate together
// prevent rec C from re-introducing the same failure mode.

#include <gtest/gtest.h>

#include "RecoveryDeltaBuffer.h"

#include <cmath>

using spacecal::recovery_delta::Buffer;
using spacecal::recovery_delta::Push;
using spacecal::recovery_delta::Clear;
using spacecal::recovery_delta::LiveCount;
using spacecal::recovery_delta::PredictNext;
using spacecal::recovery_delta::ComputePerTickNudge;
using spacecal::recovery_delta::kBufferSize;
using spacecal::recovery_delta::kMinEvents;

namespace {

Eigen::Vector3d Vec(double x, double y, double z) { return {x, y, z}; }

} // namespace

TEST(RecoveryDeltaBufferTest, RingPushOverwritesOldest) {
    Buffer b;
    for (size_t i = 0; i < kBufferSize + 1; ++i) {
        Push(b, Vec(0.30, 0.0, 0.0), (double)i);
    }
    EXPECT_EQ(LiveCount(b), kBufferSize);
    EXPECT_EQ(b.count, kBufferSize + 1);
}

TEST(RecoveryDeltaBufferTest, BelowMinEventsRejectsPrediction) {
    Buffer b;
    for (int i = 0; i < kMinEvents - 1; ++i) {
        Push(b, Vec(0.30, 0.0, 0.0), (double)i);
    }
    Eigen::Vector3d dir;
    double mag;
    EXPECT_FALSE(PredictNext(b, (double)(kMinEvents - 1), dir, mag));
}

TEST(RecoveryDeltaBufferTest, ConsistentDirectionPasses) {
    Buffer b;
    for (int i = 0; i < kMinEvents; ++i) {
        Push(b, Vec(0.30, 0.0, 0.0), (double)i);
    }
    Eigen::Vector3d dir;
    double mag;
    ASSERT_TRUE(PredictNext(b, (double)kMinEvents, dir, mag));
    EXPECT_NEAR(dir.x(), 1.0, 1e-9);
    EXPECT_NEAR(mag, 0.30, 1e-9);
}

TEST(RecoveryDeltaBufferTest, ScatteredDirectionsFailGate) {
    Buffer b;
    Push(b, Vec(0.30, 0.0, 0.0), 0.0);
    Push(b, Vec(0.0, 0.30, 0.0), 1.0);
    Push(b, Vec(0.0, 0.0, 0.30), 2.0);
    Eigen::Vector3d dir;
    double mag;
    EXPECT_FALSE(PredictNext(b, 2.0, dir, mag))
        << "Three orthogonal events must fail the consistency gate";
}

TEST(RecoveryDeltaBufferTest, RecencyWeightFavorsRecent) {
    Buffer b;
    Push(b, Vec(-1.0, 0.0, 0.0) * 0.30, 0.0);     // old, opposite direction
    Push(b, Vec(1.0, 0.0, 0.0) * 0.30, 1000.0);   // newer, +X
    Push(b, Vec(1.0, 0.0, 0.0) * 0.30, 1001.0);   // newer, +X
    Push(b, Vec(1.0, 0.0, 0.0) * 0.30, 1002.0);   // newer, +X

    Eigen::Vector3d dir;
    double mag;
    ASSERT_TRUE(PredictNext(b, 1002.0, dir, mag));
    EXPECT_GT(dir.x(), 0.5)
        << "Recent +X events must dominate the old -X event under recency weighting";
}

TEST(RecoveryDeltaBufferTest, BoundedRateCapsTheNudge) {
    Buffer b;
    for (int i = 0; i < kMinEvents; ++i) {
        Push(b, Vec(1.0, 0.0, 0.0) * 0.30, (double)i);
    }
    const double dt = 0.05;       // 20 Hz tick
    const double cap = 0.01;      // 1 cm/s
    const Eigen::Vector3d step = ComputePerTickNudge(b, (double)kMinEvents, dt, cap);
    const double maxStepM = cap * dt;
    EXPECT_LE(step.norm(), maxStepM + 1e-12);
}

TEST(RecoveryDeltaBufferTest, ZeroDtProducesNoNudge) {
    Buffer b;
    for (int i = 0; i < kMinEvents; ++i) {
        Push(b, Vec(1.0, 0.0, 0.0) * 0.30, (double)i);
    }
    const Eigen::Vector3d step = ComputePerTickNudge(b, (double)kMinEvents, 0.0, 0.01);
    EXPECT_EQ(step.norm(), 0.0);
}

TEST(RecoveryDeltaBufferTest, ClearResetsBuffer) {
    Buffer b;
    for (int i = 0; i < kMinEvents; ++i) {
        Push(b, Vec(1.0, 0.0, 0.0) * 0.30, (double)i);
    }
    ASSERT_EQ(LiveCount(b), (size_t)kMinEvents);
    Clear(b);
    EXPECT_EQ(LiveCount(b), 0u);
    EXPECT_EQ(b.count, 0u);
}

// ---------------------------------------------------------------------------
// REGRESSION GUARD: rec C's bounded-rate cap is a mathematical safety
// guarantee. 1000 ticks at maximum predicted nudge cannot sum to more than
// rate * total_duration (the same shape of guarantee that rec A's bounded
// step provides). Pin the worst case so any future change that bypasses the
// cap must explicitly remove or relax this assertion.
// ---------------------------------------------------------------------------
TEST(RecoveryDeltaBufferTest, BoundedRateMathematicalSafetyAcrossLongRun) {
    Buffer b;
    // Spam the buffer with kBufferSize events all in +X; this is the worst
    // case for sustained nudging.
    for (size_t i = 0; i < kBufferSize; ++i) {
        Push(b, Vec(1.0, 0.0, 0.0) * 0.30, (double)i);
    }
    const double dt = 0.05;
    const double cap = 0.01;
    Eigen::Vector3d accum = Eigen::Vector3d::Zero();
    const int ticks = 1000;
    for (int i = 0; i < ticks; ++i) {
        accum += ComputePerTickNudge(b, (double)kBufferSize + i * dt, dt, cap);
    }
    const double maxAccum = cap * ticks * dt;
    EXPECT_LE(accum.norm(), maxAccum + 1e-9);
}
