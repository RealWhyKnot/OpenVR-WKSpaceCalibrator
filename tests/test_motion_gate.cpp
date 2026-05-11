// Pure-function pin tests for the option-3 motion gate "smart by correction
// size" floor. The contract these tests pin is set by feedback_calibration_
// blending_request.md — user-feel intent quoted at the top of each test
// group. If the underlying header in src/common/MotionGate.h ever changes
// thresholds or floors, these tests fail and the change has to be deliberate.
//
// User said (verbatim, 2026-05-04):
//   "Add a new part where the tracking only applies as the user moves that
//    limb. Instead, can we do that plus a very slow, moving over to where it
//    needs to be? That way it doesn't look like, you know, a sudden shift,
//    but we still slowly move if the user is being really still. ... I've
//    had a couple instances where I kinda have to, like, awkward move a limb
//    around to get it to calibrate, which can be a little annoying."
//
// Option 3 (smart by correction size) translates the user-feel intent into:
//   - Tiny correction (noise) → 10% floor when still. Won't fight jitter.
//   - Normal correction (mm-scale) → 50% floor. Steady convergence.
//   - Large correction (cm-scale, e.g. recovery) → 90% floor. Effectively snap.

#include <gtest/gtest.h>

#include "MotionGate.h"

using spacecal::motiongate::Regime;
using spacecal::motiongate::ClassifyCorrection;
using spacecal::motiongate::StillFloor;
using spacecal::motiongate::kTinyMaxPosMm;
using spacecal::motiongate::kTinyMaxRotDeg;
using spacecal::motiongate::kLargeMinPosMm;
using spacecal::motiongate::kLargeMinRotDeg;
using spacecal::motiongate::kTinyStillFloor;
using spacecal::motiongate::kNormalStillFloor;
using spacecal::motiongate::kLargeStillFloor;

// ---------------------------------------------------------------------------
// "won't fight jitter" — the user's complaint was lerp freezing while still,
// forcing them to wave a controller. Tiny corrections are noise-level; the
// user can't tell whether the cal is converging or not, so 10% floor is
// invisible-but-non-zero progress.
// ---------------------------------------------------------------------------
TEST(MotionGateTest, Tiny_NoiseLevelCorrectionsClassifiedAsTiny) {
    // Sub-mm position, sub-arc-minute rotation = pure noise.
    EXPECT_EQ(ClassifyCorrection(/*posMm=*/0.5, /*rotDeg=*/0.01), Regime::Tiny);
    EXPECT_EQ(ClassifyCorrection(0.0, 0.0), Regime::Tiny);
    EXPECT_EQ(ClassifyCorrection(1.0, 0.05), Regime::Tiny);
}

TEST(MotionGateTest, Tiny_AndSemantics_BothAxesMustBeNoise) {
    // Tiny now requires BOTH axes to be in the noise region. A 4 mm pure
    // translation with no rotation is a real correction, not noise, and
    // should ride the Normal floor (50 percent) rather than the Tiny
    // floor (10 percent). This supersedes the earlier OR rule.
    EXPECT_EQ(ClassifyCorrection(/*posMm=*/2.0, /*rotDeg=*/0.04), Regime::Normal)
        << "2 mm pos with tiny rot: pos is above noise, rot is in noise; "
           "Tiny needs BOTH, so this is Normal";
    EXPECT_EQ(ClassifyCorrection(0.5, 0.4), Regime::Normal)
        << "Sub-mm pos with 0.4 deg rot: rot is above noise, pos is in noise; "
           "Tiny needs BOTH, so this is Normal";
}

// ---------------------------------------------------------------------------
// "steady convergence" — corrections in the 1-5mm and 0.05-0.5° band are
// real but small. 50% floor moves the cal at half-rate when still — enough
// to settle within seconds without being visible as a drift.
// ---------------------------------------------------------------------------
TEST(MotionGateTest, Normal_MmScaleCorrectionsClassifiedAsNormal) {
    // Both axes need to be above tiny but neither above large.
    EXPECT_EQ(ClassifyCorrection(/*posMm=*/3.0, /*rotDeg=*/0.3), Regime::Normal);
    EXPECT_EQ(ClassifyCorrection(2.0, 0.2), Regime::Normal);
    EXPECT_EQ(ClassifyCorrection(5.0, 0.5), Regime::Normal)
        << "Boundary inclusive on the upper end too — exactly 5mm + 0.5° "
           "is the highest 'Normal'";
}

// ---------------------------------------------------------------------------
// "effectively snap" — catastrophic corrections (post-stall recovery, Quest
// re-localization, lighthouse bumped) need to converge fast. 90% floor
// means user perceives near-instant correction even when standing still.
//
// User's session log evidence: HMD on/off triggered 7-9 cm shifts. The
// 5mm threshold here puts those firmly in the Large band; pos>5 alone is
// enough regardless of rot.
// ---------------------------------------------------------------------------
TEST(MotionGateTest, Large_CatastrophicCorrectionClassifiedAsLarge) {
    // Either axis > threshold = Large. Mirrors the user's "OR" wording.
    EXPECT_EQ(ClassifyCorrection(/*posMm=*/10.0, /*rotDeg=*/0.0), Regime::Large);
    EXPECT_EQ(ClassifyCorrection(0.0, 1.0), Regime::Large);
    EXPECT_EQ(ClassifyCorrection(50.0, 5.0), Regime::Large);
}

TEST(MotionGateTest, Large_TakesPrecedenceOverTiny) {
    // pos=10mm (Large) + rot=0.01° (also tiny by OR) — Large wins.
    // Without the precedence rule, the user's HMD-on/off shift would
    // classify as Tiny if rot happened to be quiet, drifting at 10%
    // floor instead of snapping.
    EXPECT_EQ(ClassifyCorrection(10.0, 0.01), Regime::Large);
    EXPECT_EQ(ClassifyCorrection(0.5, 1.0), Regime::Large);
}

// ---------------------------------------------------------------------------
// Boundary tests. Each threshold is checked at exactly the boundary value
// + epsilon on each side so a future tweak (e.g. swapping > for >=) trips
// the test rather than silently shifting behaviour.
// ---------------------------------------------------------------------------
TEST(MotionGateTest, Boundaries_LargePosThreshold) {
    EXPECT_EQ(ClassifyCorrection(kLargeMinPosMm, 0.0), Regime::Normal)
        << "Exactly 5mm pos with 0 deg rot: not Large (strict > 5 mm does "
           "not fire at exactly 5), pos above tiny so Tiny needs both; "
           "result is Normal";
    EXPECT_EQ(ClassifyCorrection(kLargeMinPosMm + 0.01, 0.0), Regime::Large)
        << "Just above 5mm fires Large";
}

TEST(MotionGateTest, Boundaries_LargeRotThreshold) {
    EXPECT_EQ(ClassifyCorrection(0.0, kLargeMinRotDeg), Regime::Normal)
        << "Pos in noise, rot exactly at 0.5 deg (Normal upper edge); "
           "AND rule means Normal";
    EXPECT_EQ(ClassifyCorrection(0.0, kLargeMinRotDeg + 0.001), Regime::Large);
}

TEST(MotionGateTest, Boundaries_TinyPosThreshold) {
    // 1.0mm pos + 0.5 deg rot: pos at the tiny upper bound, rot at the
    // upper Normal edge. AND rule: rot is not in noise, so Normal.
    EXPECT_EQ(ClassifyCorrection(kTinyMaxPosMm, 0.5), Regime::Normal);
    EXPECT_EQ(ClassifyCorrection(kTinyMaxPosMm + 0.01, 0.5), Regime::Normal);
    // Both in noise -> Tiny.
    EXPECT_EQ(ClassifyCorrection(kTinyMaxPosMm, kTinyMaxRotDeg), Regime::Tiny);
}

// ---------------------------------------------------------------------------
// Floor lookup. Pinned per-regime so a future regime addition or floor
// retune surfaces deliberately.
// ---------------------------------------------------------------------------
TEST(MotionGateTest, StillFloor_PerRegime) {
    EXPECT_DOUBLE_EQ(StillFloor(Regime::Tiny),   kTinyStillFloor);
    EXPECT_DOUBLE_EQ(StillFloor(Regime::Tiny),   0.10);
    EXPECT_DOUBLE_EQ(StillFloor(Regime::Normal), kNormalStillFloor);
    EXPECT_DOUBLE_EQ(StillFloor(Regime::Normal), 0.50);
    EXPECT_DOUBLE_EQ(StillFloor(Regime::Large),  kLargeStillFloor);
    EXPECT_DOUBLE_EQ(StillFloor(Regime::Large),  0.90);
}

TEST(MotionGateTest, StillFloor_OrderingMonotonic) {
    // Pin the relative ordering: a larger correction should never get a
    // LOWER still-floor than a smaller one. If someone retunes the
    // constants, this catches an ordering inversion.
    EXPECT_LT(StillFloor(Regime::Tiny),   StillFloor(Regime::Normal));
    EXPECT_LT(StillFloor(Regime::Normal), StillFloor(Regime::Large));
}

// ---------------------------------------------------------------------------
// constexpr pins. Catches the breakage at compile time as well as runtime.
// ---------------------------------------------------------------------------
static_assert(ClassifyCorrection(0.5, 0.01) == Regime::Tiny,
    "noise-level correction must classify as Tiny");
static_assert(ClassifyCorrection(3.0, 0.3) == Regime::Normal,
    "mm-scale correction with no axis in noise must classify as Normal");
static_assert(ClassifyCorrection(10.0, 0.01) == Regime::Large,
    "Large takes precedence over Tiny — catastrophic pos with quiet rot is still Large");
static_assert(StillFloor(Regime::Tiny)   == 0.10, "Tiny floor pinned at 10%");
static_assert(StillFloor(Regime::Normal) == 0.50, "Normal floor pinned at 50%");
static_assert(StillFloor(Regime::Large)  == 0.90, "Large floor pinned at 90%");

// ---------------------------------------------------------------------------
// Integration-shaped test (still pure): simulate the BlendTransform decision
// as `lerp *= max(motionGate, StillFloor(regime))`. Pin the two extreme
// user-feel scenarios:
//   1. User still, tiny correction — lerp drifts at 10% rate (won't freeze)
//   2. User moving, any correction — motionGate dominates, full speed
// ---------------------------------------------------------------------------
TEST(MotionGateTest, EffectiveGate_StillUserPlusTinyCorrection_Floor10Percent) {
    const double motionGate = 0.0;  // user perfectly still
    const auto regime = ClassifyCorrection(0.5, 0.01);
    const double effective = std::max(motionGate, StillFloor(regime));
    EXPECT_DOUBLE_EQ(effective, 0.10)
        << "Standing still with sub-mm correction: gate must be the 10% "
           "tiny floor. Without this, the lerp freezes at 0 and the user "
           "has to wave a controller — exactly the bug the user reported.";
}

TEST(MotionGateTest, EffectiveGate_MovingUser_MotionGateDominates) {
    const double motionGate = 1.0;  // full natural motion
    const auto regime = ClassifyCorrection(0.5, 0.01);
    const double effective = std::max(motionGate, StillFloor(regime));
    EXPECT_DOUBLE_EQ(effective, 1.0)
        << "Moving user: motionGate=1 wins over any floor. Floor only "
           "kicks in when the user is still.";
}

TEST(MotionGateTest, EffectiveGate_StillUserPlusLargeCorrection_Floor90Percent) {
    const double motionGate = 0.0;
    const auto regime = ClassifyCorrection(50.0, 5.0);
    const double effective = std::max(motionGate, StillFloor(regime));
    EXPECT_DOUBLE_EQ(effective, 0.90)
        << "Standing still + post-stall recovery shift: gate snaps fast "
           "via the 90% large floor. The user doesn't have to move to "
           "see the recovery converge.";
}

// ---------------------------------------------------------------------------
// Auto-recovery snap (option-3 bundle, 2026-05-04). The cycle-level
// ShouldBlendCycle helper decides whether the SetDeviceTransform payload's
// `lerp` field is true (smooth blend via BlendTransform) or false (driver
// snaps transform := target). After RecoverFromWedgedCalibration fires, the
// next ScanAndApplyProfile cycle MUST snap so the recovery doesn't get
// smoothed through whatever stale steady-state the driver had cached.
// ---------------------------------------------------------------------------
using spacecal::motiongate::ShouldBlendCycle;

TEST(AutoRecoverySnapTest, RecoveryCycleSnaps_OverridesEverything) {
    // snapThisCycle=true must override both the continuous-state and the
    // freshly-adopted gates. This is the post-RecoverFromWedgedCalibration
    // tick; we want a discontinuity, not a smooth ride.
    EXPECT_FALSE(ShouldBlendCycle(/*continuous=*/true,
                                  /*freshlyAdopted=*/false,
                                  /*snapThisCycle=*/true))
        << "Recovery snap must override the normal continuous-blend path "
           "— otherwise the user's brand-new post-recovery cal gets slowly "
           "interpolated from the wedged steady-state, defeating the "
           "recovery.";
    EXPECT_FALSE(ShouldBlendCycle(true,  true,  true));
    EXPECT_FALSE(ShouldBlendCycle(false, false, true));
    EXPECT_FALSE(ShouldBlendCycle(false, true,  true));
}

TEST(AutoRecoverySnapTest, FreshAdoptionSnaps) {
    // Independent of recovery: a tracker that just connected has no
    // meaningful `transform` to blend FROM. Snap so it doesn't ramp in
    // from identity.
    EXPECT_FALSE(ShouldBlendCycle(/*continuous=*/true,
                                  /*freshlyAdopted=*/true,
                                  /*snapThisCycle=*/false));
}

TEST(AutoRecoverySnapTest, OneShotStateSnaps) {
    // Only continuous mode lerps; the one-shot finalisation should snap
    // because there's no continuous-update stream to smooth toward.
    EXPECT_FALSE(ShouldBlendCycle(/*continuous=*/false,
                                  /*freshlyAdopted=*/false,
                                  /*snapThisCycle=*/false));
}

TEST(AutoRecoverySnapTest, NormalCycleBlends) {
    // The healthy steady-state: continuous mode, established device, no
    // recovery in flight. This is the only combination that should blend.
    EXPECT_TRUE(ShouldBlendCycle(/*continuous=*/true,
                                 /*freshlyAdopted=*/false,
                                 /*snapThisCycle=*/false));
}

TEST(AutoRecoverySnapTest, FlagIsOneShot_NextCycleResumesBlend) {
    // Simulates the pattern in production: cycle N fires recovery and snaps
    // (snapThisCycle=true), cycle N+1 the flag is cleared and blending
    // resumes. The flag-management is in ScanAndApplyProfile (consume at
    // end); this test pins the per-cycle pure decision both halves use.
    bool snapThisCycle = true;  // set by RecoverFromWedgedCalibration
    EXPECT_FALSE(ShouldBlendCycle(true, false, snapThisCycle))
        << "Cycle N (recovery): must snap";
    snapThisCycle = false;  // cleared at the end of cycle N
    EXPECT_TRUE(ShouldBlendCycle(true, false, snapThisCycle))
        << "Cycle N+1: blending resumes";
}

static_assert(!ShouldBlendCycle(true, false, true),
    "snap flag must override continuous-mode blend");
static_assert(ShouldBlendCycle(true, false, false),
    "the only blend-true case is continuous + established + no snap");
static_assert(!ShouldBlendCycle(false, false, false),
    "non-continuous state must snap");
