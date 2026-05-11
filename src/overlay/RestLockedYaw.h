#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <unordered_map>
#include <vector>

// Rest-locked yaw drift correction. Pure-function math + per-tracker phase
// machine, isolated from CalibrationContext so the contract can be pinned by
// gtest in tests/test_rest_locked_yaw.cpp.
//
// Modeled on SlimeVR's Stay Aligned (server/core/.../stayaligned/). When a
// tracker stays at rest for kRestEnterSec, lock its current orientation as an
// absolute reference. On every subsequent tick where it is still at rest, the
// yaw delta between the locked reference and the orientation predicted from
// the active SE(3) is evidence of HMD-frame-vs-target-frame drift. Apply a
// bounded-rate yaw correction (capped per tracking-system class) to the
// active calibration.
//
// Why this is structurally different from the deleted Phase 1+2 silent recal
// (DELETED 2026-04-29 at main 9fab09d): the deleted gate compared candidate-
// RMS against current-RMS on the same sample buffer, which is tautological
// for stationary buffers. The locked reference here is absolute -- the
// orientation 1 s ago, NOT a re-fit of any cross-system transform -- so a
// stationary buffer cannot fool it. The bounded-rate cap further degrades
// gracefully: worst-case bad yaw introduced over a 30 s wedge is at most
// rate * duration (0.15 deg/s * 30 s = 4.5 deg), well below the 30 cm
// auto-recovery floor.

namespace spacecal::rest_yaw {

// Per-class correction-rate caps in deg/s. Per-class rather than per-device
// because the dominant axis of variation is sensor class, not individual unit
// (Borenstein & Ojeda 2009/2010 iHDE; SlimeVR v0.16.0 release notes).
struct RateCaps {
    double lighthouse_deg_per_sec = 0.05;
    double quest_deg_per_sec      = 0.15;
    double slimevr_deg_per_sec    = 0.30;
    double unknown_deg_per_sec    = 0.15;
    double global_ceiling_deg_per_sec = 0.50;
};

enum class TrackingSystemClass {
    Unknown   = 0,
    Lighthouse = 1,
    Quest     = 2,
    SlimeVR   = 3,
    // Add new classes here. The classification table lives in Calibration.cpp
    // because it depends on driver-name strings exposed via OpenVR property
    // queries.
};

constexpr double kRestEnterDegPerSec = 120.0; // 2 deg/tick at 60 Hz; rate-invariant
constexpr double kRestEnterSec       = 1.0;
constexpr double kRestExitDegPerSec  = 120.0; // symmetric for v1
constexpr double kRestExitSec        = 3.0;
constexpr double kFallbackYawCap  = 0.15;  // single-cap fallback if classification fails

enum class RestPhase : uint8_t {
    Moving             = 0,
    RecentlyAtRest     = 1,
    AtRest             = 2,
};

struct RestState {
    RestPhase phase = RestPhase::Moving;
    Eigen::Quaterniond lastRot = Eigen::Quaterniond::Identity();
    double phaseEnteredAt = 0.0;
    Eigen::Quaterniond lockedRot = Eigen::Quaterniond::Identity();
    bool haveLock = false;
    bool initialized = false;
};

// Phase-machine update for one tracker. Pure function: takes the existing
// state, the new orientation, the timestamp, and the tick duration (dt_sec).
// dt_sec is used to convert angular distance to deg/s so the rest threshold
// is sample-rate independent. Caller is responsible for storing the result.
inline RestState UpdatePhase(const RestState& prior,
                             const Eigen::Quaterniond& newRot,
                             double now,
                             double dt_sec) {
    RestState s = prior;

    if (!s.initialized) {
        s.phase = RestPhase::Moving;
        s.lastRot = newRot;
        s.phaseEnteredAt = now;
        s.haveLock = false;
        s.initialized = true;
        return s;
    }

    // Angular velocity between the prior tick's rotation and this one.
    // Divide by dt_sec to get deg/s so the threshold is rate-independent.
    // Guard against near-zero dt (first tick after enable) by clamping.
    const double dotProd = std::clamp(std::abs(s.lastRot.dot(newRot)), 0.0, 1.0);
    const double angleRad = 2.0 * std::acos(dotProd);
    const double angleDeg = angleRad * (180.0 / 3.14159265358979323846);
    const double safeDt = std::max(dt_sec, 1e-4);
    const double angularVelDegPerSec = angleDeg / safeDt;
    s.lastRot = newRot;

    if (angularVelDegPerSec > kRestEnterDegPerSec) {
        s.phase = RestPhase::Moving;
        s.phaseEnteredAt = now;
        s.haveLock = false;
        return s;
    }

    if (s.phase == RestPhase::Moving) {
        s.phase = RestPhase::RecentlyAtRest;
        s.phaseEnteredAt = now;
    } else if (s.phase == RestPhase::RecentlyAtRest
            && (now - s.phaseEnteredAt) >= kRestEnterSec) {
        s.phase = RestPhase::AtRest;
        s.lockedRot = newRot;
        s.haveLock = true;
    }

    return s;
}

// Reset a rest state to the uninitialized phase. Call when a tracker drops
// pose validity, when AssignTargets() reseats device IDs, or when the
// session ends.
inline void ResetState(RestState& s) {
    s = RestState();
}

// Signed yaw delta between two quaternions, in radians, returned in
// (-pi, +pi]. Yaw is the rotation about world-Y. The convention matches the
// existing cal_rotation_yaw log line and EulerRotation()'s second component.
inline double SignedYawDeltaRad(const Eigen::Quaterniond& fromQ,
                                const Eigen::Quaterniond& toQ) {
    // delta * fromQ = toQ  ->  delta = toQ * fromQ.conjugate(). Project onto
    // the world-Y axis by extracting yaw via atan2.
    const Eigen::Quaterniond delta = toQ * fromQ.conjugate();
    const double siny_cosp = 2.0 * (delta.w() * delta.y() + delta.z() * delta.x());
    const double cosy_cosp = 1.0 - 2.0 * (delta.x() * delta.x() + delta.y() * delta.y());
    return std::atan2(siny_cosp, cosy_cosp);
}

// Bounded-rate clamp on a yaw correction. Worst-case bad yaw over a duration
// is rate * duration; choose the cap conservatively. Returns the clamped
// step in radians.
inline double ApplyBoundedYawStep(double meanYawErrRad,
                                  double dtSec,
                                  double capDegPerSec) {
    const double capRad = capDegPerSec * (3.14159265358979323846 / 180.0) * dtSec;
    if (meanYawErrRad >  capRad) return  capRad;
    if (meanYawErrRad < -capRad) return -capRad;
    return meanYawErrRad;
}

// Per-class cap selector. Caller supplies the class and the cap table.
inline double CapForClass(TrackingSystemClass c, const RateCaps& caps) {
    double v = caps.unknown_deg_per_sec;
    switch (c) {
        case TrackingSystemClass::Lighthouse: v = caps.lighthouse_deg_per_sec; break;
        case TrackingSystemClass::Quest:      v = caps.quest_deg_per_sec; break;
        case TrackingSystemClass::SlimeVR:    v = caps.slimevr_deg_per_sec; break;
        default: break;
    }
    return std::min(v, caps.global_ceiling_deg_per_sec);
}

// Per-tracker contribution to the fused yaw correction. Used by the Markley
// matrix-weighted average in rec I; tested independently.
struct YawContribution {
    double yawErrRad = 0.0;     // signed yaw delta (locked vs predicted)
    double weight    = 0.0;     // w_class * w_age * w_quality
    TrackingSystemClass cls = TrackingSystemClass::Unknown;
};

// Per-class trust weight. Lighthouse is the cleanest absolute reference; IMU-
// only trackers contribute less because their own yaw drifts during the lock.
inline double ClassWeight(TrackingSystemClass c) {
    switch (c) {
        case TrackingSystemClass::Lighthouse: return 1.0;
        case TrackingSystemClass::Quest:      return 0.6;
        case TrackingSystemClass::SlimeVR:    return 0.3;
        default:                              return 0.5;
    }
}

inline double AgeWeight(double secondsSinceLock, double tauSec = 120.0) {
    if (secondsSinceLock <= 0.0) return 1.0;
    return std::exp(-secondsSinceLock / tauSec);
}

inline double QualityWeight(double sigmaYawRad) {
    return 1.0 / (1.0 + sigmaYawRad * sigmaYawRad);
}

// Markley matrix-weighted quaternion average over yaw-only contributions.
// Reduces to a 1-D weighted mean because all contributions are pure yaw
// rotations; the eigenproblem collapses to sum(w_i * yaw_i) / sum(w_i).
//
// Kept as a separate function so a future extension to full-quaternion
// fusion (full-SE(3) corrections from rec J) can replace this with the real
// Markley Section III eigensolver without rewriting callers.
inline double FuseYawContributionsRad(const std::vector<YawContribution>& xs) {
    double sumSin = 0.0, sumCos = 0.0;
    double weightTotal = 0.0;
    for (const auto& x : xs) {
        sumSin += x.weight * std::sin(x.yawErrRad);
        sumCos += x.weight * std::cos(x.yawErrRad);
        weightTotal += x.weight;
    }
    if (weightTotal <= 0.0) return 0.0;
    return std::atan2(sumSin, sumCos);
}

// Sentinel for "no correction this tick." Callers compare against this to
// know whether to apply the SE(3) yaw nudge or skip. Distinct from "applied
// a zero correction" because callers may want to log differently.
struct TickResult {
    double appliedStepRad = 0.0;
    double meanErrRad     = 0.0;
    int    lockedTrackers = 0;
    bool   applied        = false;
};

} // namespace spacecal::rest_yaw

// Compile-time pinning for the bounded-rate cap. Mirrors the StallDecisions
// pattern: the math is small enough that a static_assert plus a runtime test
// pins both the formula and a few known inputs.
namespace spacecal::rest_yaw::detail {
constexpr double kPi = 3.14159265358979323846;
constexpr double ConstexprBoundedYawStep(double err, double dtSec, double capDegPerSec) {
    return (err >  capDegPerSec * (kPi / 180.0) * dtSec)
        ?  capDegPerSec * (kPi / 180.0) * dtSec
        : (err < -capDegPerSec * (kPi / 180.0) * dtSec)
        ? -capDegPerSec * (kPi / 180.0) * dtSec
        : err;
}
static_assert(ConstexprBoundedYawStep(0.0, 0.05, 0.15) == 0.0,
              "zero error stays zero");
static_assert(ConstexprBoundedYawStep(1.0, 0.05, 0.15) > 0.0
           && ConstexprBoundedYawStep(1.0, 0.05, 0.15) < 1.0,
              "large positive error gets clamped to a positive value below it");
static_assert(ConstexprBoundedYawStep(-1.0, 0.05, 0.15) < 0.0
           && ConstexprBoundedYawStep(-1.0, 0.05, 0.15) > -1.0,
              "large negative error gets clamped to a negative value above it");
} // namespace spacecal::rest_yaw::detail
