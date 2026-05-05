#pragma once

// Motion-gate floor — "smart by correction size" (option 3 from
// feedback_calibration_blending_request.md, picked by user 2026-05-04).
//
// Problem this solves: when `recalibrateOnMovement` is on (default true), the
// driver's BlendTransform multiplies the time-based lerp by a 0..1 motion
// gate so calibration shifts hide in the user's natural movement. The bug:
// when the user is still, motionGate≈0 and the lerp freezes — the user has
// to wave a controller before convergence resumes. The user's report:
// "I have to wave a limb to get it to calibrate."
//
// The fix: per-regime still-floor. Compute the correction size (the magnitude
// of |targetTransform - transform|), classify it, look up the floor for that
// regime. Final gate is max(motionGate, regimeFloor) — when moving, motionGate
// dominates; when still, regimeFloor kicks in. Three regimes:
//
//   Tiny (noise-level correction)    → 10% floor — won't fight jitter.
//   Normal (steady mm-scale)         → 50% floor — moderate convergence.
//   Large (cm-scale, e.g. recovery)  → 90% floor — effectively snap.
//
// Thresholds picked from the user's 2026-05-04 logs:
//   - Healthy continuous-cal currentCal hovers in 1-3 mm error band → "tiny"
//     should be ≤ 1 mm so noise corrections don't trigger steady drift.
//   - Real per-tick corrections during convergence are mm-scale → "normal"
//     covers 1-5 mm.
//   - HMD-on/off events (the 56-tick + 95-tick stalls) caused 7-9 cm shifts
//     → "large" needs to fire at >5 mm so post-stall corrections snap fast.
//
// Pure helpers — no state. Caller (BlendTransform) computes the correction
// magnitudes and asks for the regime. Tests in test_motion_gate.cpp pin the
// classification at every boundary.
//
// Header lives in src/common/ because both the driver (BlendTransform) and
// the overlay (regression tests, future direct callers) include it.

namespace spacecal::motiongate {

// Three classes of correction magnitude. Naming reflects user-feel intent,
// not the math: "tiny" = below the noise floor, "large" = catastrophic
// (something real happened — Quest re-localized, lighthouse bumped).
enum class Regime {
    Tiny,    // both pos and rot in the noise band — drift slowly
    Normal,  // ordinary mm-scale convergence
    Large,   // catastrophic correction — snap fast even when still
};

// Boundary thresholds. Position in millimetres, rotation in degrees.
// Documented above; do not change without revisiting the rationale.
constexpr double kTinyMaxPosMm = 1.0;
constexpr double kTinyMaxRotDeg = 0.05;
constexpr double kLargeMinPosMm = 5.0;
constexpr double kLargeMinRotDeg = 0.5;

// Still-floor for each regime. The effective gate is max(motionGate,
// StillFloor(regime)) — these are the lower bounds when the user is
// motionless, NOT a multiplier when moving.
constexpr double kTinyStillFloor   = 0.10;
constexpr double kNormalStillFloor = 0.50;
constexpr double kLargeStillFloor  = 0.90;

// Classify a pending correction by its magnitude.
//
// Per the 2026-05-04 user spec, classification uses the OR of pos/rot:
//   - Large takes precedence: catastrophic correction in EITHER axis means
//     we need to snap regardless of the other.
//   - Tiny: noise in EITHER axis — even a 2 mm pos correction is treated as
//     noise-region if its rot delta is sub-arc-minute (matches user's
//     "1 mm OR 0.05°" wording).
constexpr Regime ClassifyCorrection(double posDeltaMm, double rotDeltaDeg) {
    if (posDeltaMm > kLargeMinPosMm || rotDeltaDeg > kLargeMinRotDeg) {
        return Regime::Large;
    }
    if (posDeltaMm <= kTinyMaxPosMm || rotDeltaDeg <= kTinyMaxRotDeg) {
        return Regime::Tiny;
    }
    return Regime::Normal;
}

constexpr double StillFloor(Regime regime) {
    switch (regime) {
        case Regime::Tiny:   return kTinyStillFloor;
        case Regime::Normal: return kNormalStillFloor;
        case Regime::Large:  return kLargeStillFloor;
    }
    return kNormalStillFloor;  // unreachable
}

// Cycle-level "should this profile-apply use smooth blending or snap?"
// decision. Used by the overlay's ScanAndApplyProfile when constructing
// the SetDeviceTransform payload.lerp field, and pinned here as a pure
// helper so the auto-recovery snap contract (option-3 bundle, 2026-05-04)
// is testable without instantiating the full overlay state machine.
//
// Returns false (i.e. SNAP — driver assigns transform := target without
// blending) in three cases:
//   1. We're not in continuous state — only continuous mode lerps; one-
//      shot finalisations always snap to truth.
//   2. The device is freshly adopted — its `transform` is identity-or-
//      stale; blending in from there would look like a slow drift.
//   3. The current cycle is a post-recovery snap (snapThisCycle=true) —
//      RecoverFromWedgedCalibration set the flag so the brand-new cal
//      lands discontinuously.
//
// Otherwise returns true (smooth blend via BlendTransform).
constexpr bool ShouldBlendCycle(bool inContinuousState,
                                bool isFreshlyAdopted,
                                bool snapThisCycle) {
    return inContinuousState && !isFreshlyAdopted && !snapThisCycle;
}

} // namespace spacecal::motiongate
