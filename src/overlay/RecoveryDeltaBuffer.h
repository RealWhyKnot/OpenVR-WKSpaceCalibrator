#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cmath>

// Predictive pre-correction from a rolling buffer of recovery deltas (rec C).
// When RecoverFromWedgedCalibration fires, push the (HMD-jump direction,
// magnitude, time-of-fire) into the ring. If the last N events trend in a
// consistent direction, apply a small fraction of the predicted next-jump
// to the active calibration BETWEEN events so the next fire has a smaller
// magnitude.
//
// Why this is safe: bounded twice. The fraction of the predicted next
// magnitude applied per event is small (kAmount = 0.10 vs SlimeVR's 0.8 for
// rotation-only), and the per-tick rate cap further limits the size of a
// single tick's nudge. The deleted Phase 1+2 silent-recal was unbounded;
// rec C is bounded by construction.
//
// Why this avoids the silent-recal failure mode: rec C consumes the
// 30 cm relocalization detector's output (a high-SNR signal -- the HMD
// physically jumped >30 cm), not a buffer-RMS comparison. The "is there a
// real signal here" question has already been answered before any event
// reaches this ring; rec C only chooses how to extrapolate.
//
// Pure-function header following the StallDecisions pattern. Tested in
// isolation in tests/test_recovery_delta_buffer.cpp.

namespace spacecal::recovery_delta {

constexpr size_t kBufferSize          = 6;       // matches SlimeVR maxResets
constexpr int    kMinEvents           = 3;       // gate predictions until N seen
constexpr double kAmount              = 0.10;    // fraction of predicted next mag
constexpr double kConsistencyCosine   = 0.7;     // mean cos(theta) vs mean direction
constexpr double kHalfLifeSec         = 300.0;   // 5 minutes recency weight

struct RecoveryEvent {
    Eigen::Vector3d direction = Eigen::Vector3d::Zero(); // unit vector
    double magnitude = 0.0;                              // meters
    double timestamp = 0.0;                              // session-local seconds
};

struct Buffer {
    std::array<RecoveryEvent, kBufferSize> events{};
    size_t count = 0;        // events ever pushed; ring index = count % kBufferSize
};

// Push a new event into the ring. Older entries roll off when count exceeds
// kBufferSize.
inline void Push(Buffer& b, const Eigen::Vector3d& deltaVec, double now) {
    const double mag = deltaVec.norm();
    RecoveryEvent& slot = b.events[b.count % kBufferSize];
    slot.magnitude = mag;
    if (mag > 1e-9) {
        slot.direction = deltaVec / mag;
    } else {
        slot.direction = Eigen::Vector3d::Zero();
    }
    slot.timestamp = now;
    b.count++;
}

inline void Clear(Buffer& b) {
    b = Buffer();
}

inline size_t LiveCount(const Buffer& b) {
    return std::min(b.count, kBufferSize);
}

// Compute the recency-weighted mean direction and magnitude across the live
// events. Returns false if not enough events have been collected to predict,
// or if the events lack directional consistency. On success, outDir is a
// unit vector and outMag is the recency-weighted mean magnitude.
inline bool PredictNext(const Buffer& b, double now,
                        Eigen::Vector3d& outDir, double& outMag) {
    const size_t n = LiveCount(b);
    if (n < kMinEvents) return false;

    Eigen::Vector3d weightedSum = Eigen::Vector3d::Zero();
    double weightedMag = 0.0;
    double weightTotal = 0.0;

    // First pass: weighted mean direction + magnitude.
    for (size_t i = 0; i < n; ++i) {
        const auto& ev = b.events[i];
        const double age = std::max(0.0, now - ev.timestamp);
        const double w = std::exp(-age / kHalfLifeSec);
        weightedSum += w * ev.direction * ev.magnitude;
        weightedMag += w * ev.magnitude;
        weightTotal += w;
    }
    if (weightTotal <= 0.0 || weightedMag <= 0.0) return false;

    const double meanMag = weightedMag / weightTotal;
    Eigen::Vector3d centroid = weightedSum / weightTotal;
    const double centroidNorm = centroid.norm();
    if (centroidNorm < 1e-9) return false;
    const Eigen::Vector3d meanDir = centroid / centroidNorm;

    // Second pass: consistency check. The recency-weighted mean cosine
    // between event directions and the centroid must exceed the threshold;
    // scattered events fail this gate and cannot drive a prediction.
    double cosWeighted = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const auto& ev = b.events[i];
        const double age = std::max(0.0, now - ev.timestamp);
        const double w = std::exp(-age / kHalfLifeSec);
        cosWeighted += w * ev.direction.dot(meanDir);
    }
    const double meanCos = cosWeighted / weightTotal;
    if (meanCos < kConsistencyCosine) return false;

    outDir = meanDir;
    outMag = meanMag;
    return true;
}

// Compute the per-tick nudge to apply, bounded by capMps * dt. Predicts the
// next event from the buffer; returns Vector3d::Zero if the gate rejects.
// dt is in seconds; cap is in meters per second.
inline Eigen::Vector3d ComputePerTickNudge(const Buffer& b, double now,
                                           double dt, double capMps) {
    Eigen::Vector3d dir;
    double mag;
    if (!PredictNext(b, now, dir, mag)) return Eigen::Vector3d::Zero();
    if (dt <= 0.0) return Eigen::Vector3d::Zero();

    // Apply a fraction of the predicted magnitude as the tick step. Then
    // clamp to capMps * dt so a long-period prediction cannot inject more
    // than the rate cap allows.
    Eigen::Vector3d step = kAmount * mag * dir;
    const double maxStep = capMps * dt;
    if (step.norm() > maxStep) {
        step = step.normalized() * maxStep;
    }
    return step;
}

} // namespace spacecal::recovery_delta
