#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cmath>

// Sub-30 cm re-anchor sub-detector (rec F). Runs a chi-square test on the
// residual between (i) HMD pose predicted from a 200 ms rolling velocity
// estimate and (ii) the actual next HMD pose. When the residual's
// Mahalanobis distance exceeds the chi-square threshold for 6 DoF at
// p < 1e-4 (about 27.86), the detector raises a candidate flag. SC's
// continuous corrections (rec A's yaw nudge, rec C's translation nudge)
// freeze for kFreezeWindowSec after a candidate so the existing 30 cm
// detector has a clean window to confirm a real re-anchor without our
// corrections diluting the signal.
//
// Why this is structurally different from a naive position-jump threshold:
// the chi-square gate is scale-aware (Or 2025 arXiv 2512.18508). A 5 cm
// jump during normal head motion has a different significance than a 5 cm
// jump while stationary; the Mahalanobis distance against the running
// residual variance encodes exactly that distinction.
//
// Why "freeze, don't correct" is the right policy: if we predictively
// applied a correction, a false-positive would inject bias into the
// calibration. Freezing only stalls our drift corrections briefly, which
// at worst loses a few seconds of refinement -- and by construction
// avoids the deleted Phase 1+2 silent-recal failure mode.
//
// Pure-function header; tested in tests/test_reanchor_chi_square.cpp.

namespace spacecal::reanchor_chi {

constexpr double kChiSquare6DoF_p1e4 = 27.8563;  // chi-sq inverse CDF, 6 DoF, p=1e-4
constexpr double kVelocityWindowSec  = 0.20;
constexpr double kVarianceWindowSec  = 5.0;
constexpr double kFreezeWindowSec    = 0.50;
constexpr size_t kHistoryCapacity    = 32;       // 200 ms at 20 Hz = 4 entries; cap is 32 for headroom
constexpr double kVarianceFloor      = 1e-8;     // prevent div-by-zero on perfect predictions

struct PoseSample {
    Eigen::Vector3d trans = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
    double timestamp = 0.0;
};

struct DetectorState {
    std::array<PoseSample, kHistoryCapacity> history{};
    size_t historyCount = 0;

    // Online residual variance per DoF, updated each tick. Initialized at
    // a small floor so the first few ticks do not divide by zero.
    Eigen::Matrix<double, 6, 1> residualVariance =
        (Eigen::Matrix<double, 6, 1>() << kVarianceFloor, kVarianceFloor, kVarianceFloor,
                                          kVarianceFloor, kVarianceFloor, kVarianceFloor).finished();
    int    varianceCount = 0;

    double freezeUntil   = -1e9;
    double lastFireTime  = -1e9;
    double lastChiSquared = 0.0;
};

inline void Reset(DetectorState& s) {
    s = DetectorState();
}

// Push a new HMD pose into the history ring.
inline void PushPose(DetectorState& s,
                     const Eigen::Vector3d& trans,
                     const Eigen::Quaterniond& rot,
                     double now) {
    PoseSample& slot = s.history[s.historyCount % kHistoryCapacity];
    slot.trans = trans;
    slot.rot = rot;
    slot.timestamp = now;
    s.historyCount++;
}

// Estimate per-axis linear velocity from the rolling 200 ms window.
// Returns Vector3::Zero on insufficient history.
inline Eigen::Vector3d EstimateLinearVelocity(const DetectorState& s, double now) {
    const size_t live = std::min(s.historyCount, kHistoryCapacity);
    if (live < 2) return Eigen::Vector3d::Zero();

    // Find the oldest sample within the window.
    Eigen::Vector3d earliestTrans = Eigen::Vector3d::Zero();
    double earliestTime = 0.0;
    bool haveEarliest = false;
    for (size_t i = 0; i < live; ++i) {
        const auto& p = s.history[i];
        if ((now - p.timestamp) <= kVelocityWindowSec
            && (!haveEarliest || p.timestamp < earliestTime)) {
            earliestTrans = p.trans;
            earliestTime = p.timestamp;
            haveEarliest = true;
        }
    }
    if (!haveEarliest) return Eigen::Vector3d::Zero();

    // Find the latest sample.
    Eigen::Vector3d latestTrans = Eigen::Vector3d::Zero();
    double latestTime = -1e9;
    for (size_t i = 0; i < live; ++i) {
        const auto& p = s.history[i];
        if (p.timestamp > latestTime) {
            latestTrans = p.trans;
            latestTime = p.timestamp;
        }
    }
    const double dt = latestTime - earliestTime;
    if (dt <= 1e-6) return Eigen::Vector3d::Zero();
    return (latestTrans - earliestTrans) / dt;
}

// Predicted pose at `now` from the most recent sample plus rolling velocity.
inline Eigen::Vector3d PredictTranslation(const DetectorState& s, double now) {
    const size_t live = std::min(s.historyCount, kHistoryCapacity);
    if (live == 0) return Eigen::Vector3d::Zero();

    Eigen::Vector3d latestTrans = Eigen::Vector3d::Zero();
    double latestTime = -1e9;
    for (size_t i = 0; i < live; ++i) {
        const auto& p = s.history[i];
        if (p.timestamp > latestTime) {
            latestTrans = p.trans;
            latestTime = p.timestamp;
        }
    }
    const Eigen::Vector3d v = EstimateLinearVelocity(s, latestTime);
    const double dt = std::max(0.0, now - latestTime);
    return latestTrans + v * dt;
}

// Update the online residual-variance estimate with a new 6-DoF residual.
// Uses an EWMA so old residuals decay; covariance window is implicit in
// the EWMA tau.
inline void UpdateResidualVariance(DetectorState& s,
                                   const Eigen::Matrix<double, 6, 1>& resid,
                                   double dt) {
    const double tau = kVarianceWindowSec;
    const double alpha = (dt > 0.0 && dt < tau) ? (dt / tau) : 0.05;
    for (int i = 0; i < 6; ++i) {
        const double r2 = resid(i) * resid(i);
        s.residualVariance(i) = (1.0 - alpha) * s.residualVariance(i) + alpha * r2;
        if (s.residualVariance(i) < kVarianceFloor) s.residualVariance(i) = kVarianceFloor;
    }
    s.varianceCount++;
}

// Mahalanobis squared distance between residual and its variance estimate.
inline double MahalanobisSquared(const Eigen::Matrix<double, 6, 1>& resid,
                                 const Eigen::Matrix<double, 6, 1>& variance) {
    double sum = 0.0;
    for (int i = 0; i < 6; ++i) {
        sum += (resid(i) * resid(i)) / std::max(variance(i), kVarianceFloor);
    }
    return sum;
}

// Process the next observed HMD pose. Returns true if a candidate
// re-anchor was detected on this tick. Caller is responsible for using
// IsFrozen() to check whether SC corrections should be temporarily
// suspended after a fire.
//
// Sequence:
//   1. Compute predicted translation from rolling-velocity history.
//   2. Compare to observed translation; combined with a small rotation
//      residual to make the test 6-DoF.
//   3. Mahalanobis squared against running per-axis variance.
//   4. If above kChiSquare6DoF_p1e4 AND we have enough warmup
//      samples, fire candidate and start the freeze window.
//   5. Update history (always) and variance (only when not firing,
//      so the candidate event itself does not get folded in).
inline bool TickAndCheckCandidate(DetectorState& s,
                                  const Eigen::Vector3d& observedTrans,
                                  const Eigen::Quaterniond& observedRot,
                                  double now,
                                  double dt) {
    // Need at least a few samples before the velocity estimate is meaningful.
    const size_t liveBefore = std::min(s.historyCount, kHistoryCapacity);
    if (liveBefore < 4) {
        PushPose(s, observedTrans, observedRot, now);
        return false;
    }

    const Eigen::Vector3d predictedTrans = PredictTranslation(s, now);
    const Eigen::Vector3d transResid = observedTrans - predictedTrans;

    // Predicted rotation: assume zero rotational velocity for v1; a more
    // complete impl would track angular velocity from a quaternion log.
    // The latest observed rotation suffices here; the residual is then
    // pure measurement noise during normal motion and a large quaternion
    // delta during a re-anchor.
    Eigen::Quaterniond latestRot = Eigen::Quaterniond::Identity();
    {
        double latestTime = -1e9;
        for (size_t i = 0; i < liveBefore; ++i) {
            const auto& p = s.history[i];
            if (p.timestamp > latestTime) {
                latestRot = p.rot;
                latestTime = p.timestamp;
            }
        }
    }
    const Eigen::Quaterniond rotDelta = observedRot * latestRot.conjugate();
    Eigen::Vector3d rotResid(rotDelta.x(), rotDelta.y(), rotDelta.z());

    Eigen::Matrix<double, 6, 1> resid;
    resid << transResid.x(), transResid.y(), transResid.z(),
             rotResid.x(),  rotResid.y(),  rotResid.z();

    const double mahalanobisSq = MahalanobisSquared(resid, s.residualVariance);
    s.lastChiSquared = mahalanobisSq;

    // Need enough variance samples before the threshold is meaningful;
    // otherwise the detector trips on its own warmup transients.
    const bool warm = s.varianceCount >= 20;
    bool fired = false;
    if (warm && mahalanobisSq > kChiSquare6DoF_p1e4) {
        s.freezeUntil = now + kFreezeWindowSec;
        s.lastFireTime = now;
        fired = true;
    }

    PushPose(s, observedTrans, observedRot, now);
    if (!fired) {
        UpdateResidualVariance(s, resid, dt);
    }
    return fired;
}

// True if the detector is inside its post-fire freeze window.
inline bool IsFrozen(const DetectorState& s, double now) {
    return now < s.freezeUntil;
}

} // namespace spacecal::reanchor_chi
