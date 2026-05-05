// State-machine legal-transition pin tests. The table in StateTransitions.h
// is the single source of truth for which CalibrationState transitions are
// allowed. These tests verify the table covers every transition the production
// code actually performs (per a grep of `state = CalibrationState::` sites)
// and rejects every plausibly-illegal transition.
//
// Per the audit's "transparent gaps" section: the runtime CalibrationTick
// state machine wasn't covered by tests, which is how 9d0ba0b's HMD-stall
// state demote shipped silently. This file closes that gap for the state-
// machine-shape part of the problem; the wedge guard + StallDecisions cover
// the value-of-state-fields part.

#include <gtest/gtest.h>

#include "StateTransitions.h"

using spacecal::state::IsLegalTransition;
using S = CalibrationState;

// ---------------------------------------------------------------------------
// Self-loops: always legal — represent "no transition this tick."
// ---------------------------------------------------------------------------
TEST(StateTransitionsTest, SelfLoopsAlwaysLegal) {
    for (auto s : {S::None, S::Begin, S::Rotation, S::Translation,
                   S::Editing, S::Continuous, S::ContinuousStandby}) {
        EXPECT_TRUE(IsLegalTransition(s, s))
            << "state -> state must always be legal (no-op)";
    }
}

// ---------------------------------------------------------------------------
// All entry points from None. The idle state can transition into any
// "active" mode through the documented entry points (StartCalibration,
// StartContinuousCalibration, ParseProfile autostart, Edit Calibration UI).
// ---------------------------------------------------------------------------
TEST(StateTransitionsTest, FromNone_LegalEntries) {
    EXPECT_TRUE(IsLegalTransition(S::None, S::Begin));
    EXPECT_TRUE(IsLegalTransition(S::None, S::Continuous));
    EXPECT_TRUE(IsLegalTransition(S::None, S::ContinuousStandby));
    EXPECT_TRUE(IsLegalTransition(S::None, S::Editing));
}

TEST(StateTransitionsTest, FromNone_IllegalDirectJumpToOneShotPhases) {
    // Phases of one-shot must go through Begin first. Jumping None->Rotation
    // or None->Translation would skip CollectSample initialisation.
    EXPECT_FALSE(IsLegalTransition(S::None, S::Rotation));
    EXPECT_FALSE(IsLegalTransition(S::None, S::Translation));
}

// ---------------------------------------------------------------------------
// One-shot flow: Begin -> Rotation -> Translation -> None. Each phase only
// transitions to its successor or bails to None. Going backwards or skipping
// is illegal.
// ---------------------------------------------------------------------------
TEST(StateTransitionsTest, OneShotPhases_ForwardOnly) {
    EXPECT_TRUE(IsLegalTransition(S::Begin, S::Rotation));
    EXPECT_TRUE(IsLegalTransition(S::Rotation, S::Translation));
    EXPECT_TRUE(IsLegalTransition(S::Translation, S::None));

    // No backward jumps.
    EXPECT_FALSE(IsLegalTransition(S::Translation, S::Rotation));
    EXPECT_FALSE(IsLegalTransition(S::Translation, S::Begin));
    EXPECT_FALSE(IsLegalTransition(S::Rotation, S::Begin));

    // No skipping a phase.
    EXPECT_FALSE(IsLegalTransition(S::Begin, S::Translation));
}

// ---------------------------------------------------------------------------
// Bail-to-None is universal from active states. CollectSample failures and
// CalibrationTick early-returns set state=None from any active phase.
// ---------------------------------------------------------------------------
TEST(StateTransitionsTest, BailToNone_LegalFromAllActiveStates) {
    EXPECT_TRUE(IsLegalTransition(S::Begin, S::None));
    EXPECT_TRUE(IsLegalTransition(S::Rotation, S::None));
    EXPECT_TRUE(IsLegalTransition(S::Translation, S::None));
    EXPECT_TRUE(IsLegalTransition(S::Editing, S::None));
    EXPECT_TRUE(IsLegalTransition(S::Continuous, S::None));
    EXPECT_TRUE(IsLegalTransition(S::ContinuousStandby, S::None));
}

// ---------------------------------------------------------------------------
// StartContinuousCalibration's intra-function transitions. The function
// internally calls StartCalibration (which sets state=Begin) then immediately
// sets state=Continuous. Both legs must be legal regardless of starting state.
// ---------------------------------------------------------------------------
TEST(StateTransitionsTest, StartContinuousCalibration_FromAnyEntryPoint) {
    // Begin -> Continuous (the second leg of StartContinuousCalibration).
    EXPECT_TRUE(IsLegalTransition(S::Begin, S::Continuous));

    // Continuous -> Begin (when StartContinuousCalibration is called from
    // RecoverFromWedgedCalibration's StartContinuousCalibration; first leg
    // sets state to Begin via StartCalibration).
    EXPECT_TRUE(IsLegalTransition(S::Continuous, S::Begin));

    // ContinuousStandby -> Begin (same — StartContinuousCalibration also
    // fires from the ContinuousStandby state-machine branch).
    EXPECT_TRUE(IsLegalTransition(S::ContinuousStandby, S::Begin));
}

// ---------------------------------------------------------------------------
// ContinuousStandby -> Continuous. AssignTargets succeeded; the state-machine
// branch at Calibration.cpp:2353 calls StartContinuousCalibration.
// ---------------------------------------------------------------------------
TEST(StateTransitionsTest, ContinuousStandby_ToContinuousLegal) {
    EXPECT_TRUE(IsLegalTransition(S::ContinuousStandby, S::Continuous));
}

// ---------------------------------------------------------------------------
// Continuous -> ContinuousStandby. The geometry-shift detector at
// Calibration.cpp:2152 demotes on sustained anomaly. THIS IS THE ONLY
// fork-only "demote" path remaining post-revert of 9d0ba0b's HMD-stall demote.
// Pinned so a future re-introduction of the HMD-stall demote forces an
// explicit table update + test change.
// ---------------------------------------------------------------------------
TEST(StateTransitionsTest, Regression_GeometryShiftDemoteOnly) {
    // Demote IS legal — the geometry-shift detector uses it.
    EXPECT_TRUE(IsLegalTransition(S::Continuous, S::ContinuousStandby));
    // No analogous "demote" path from any other active state. Calibration.cpp's
    // one-shot states are designed to bail to None on failure, not demote.
    EXPECT_FALSE(IsLegalTransition(S::Begin, S::ContinuousStandby));
    EXPECT_FALSE(IsLegalTransition(S::Rotation, S::ContinuousStandby));
    EXPECT_FALSE(IsLegalTransition(S::Translation, S::ContinuousStandby));
    EXPECT_FALSE(IsLegalTransition(S::Editing, S::ContinuousStandby));
}

// ---------------------------------------------------------------------------
// Editing is a dead-end UI-only state — only leaves to None. No path into
// any active calibration mode without first returning to None.
// ---------------------------------------------------------------------------
TEST(StateTransitionsTest, Editing_DeadEndToNoneOnly) {
    EXPECT_TRUE(IsLegalTransition(S::Editing, S::None));
    EXPECT_FALSE(IsLegalTransition(S::Editing, S::Begin));
    EXPECT_FALSE(IsLegalTransition(S::Editing, S::Continuous));
    EXPECT_FALSE(IsLegalTransition(S::Editing, S::ContinuousStandby));
    EXPECT_FALSE(IsLegalTransition(S::Editing, S::Rotation));
    EXPECT_FALSE(IsLegalTransition(S::Editing, S::Translation));
}

// ---------------------------------------------------------------------------
// Editing entry: only from None (UI button is disabled outside None per
// UserInterface.cpp's gating). No active state can go directly to Editing.
// ---------------------------------------------------------------------------
TEST(StateTransitionsTest, Editing_OnlyEnteredFromNone) {
    EXPECT_TRUE(IsLegalTransition(S::None, S::Editing));
    EXPECT_FALSE(IsLegalTransition(S::Begin, S::Editing));
    EXPECT_FALSE(IsLegalTransition(S::Rotation, S::Editing));
    EXPECT_FALSE(IsLegalTransition(S::Translation, S::Editing));
    EXPECT_FALSE(IsLegalTransition(S::Continuous, S::Editing));
    EXPECT_FALSE(IsLegalTransition(S::ContinuousStandby, S::Editing));
}

// ---------------------------------------------------------------------------
// constexpr pins for the most load-bearing transitions. static_assert fails
// the build (not just the test) if any of these contracts is broken.
// ---------------------------------------------------------------------------
static_assert(IsLegalTransition(S::None, S::None),
    "self-loops must be legal");
static_assert(IsLegalTransition(S::None, S::Begin),
    "StartCalibration entry must be legal");
static_assert(IsLegalTransition(S::Begin, S::Rotation),
    "one-shot phase 1 must transition forward");
static_assert(IsLegalTransition(S::Rotation, S::Translation),
    "one-shot phase 2 must transition forward");
static_assert(IsLegalTransition(S::Continuous, S::ContinuousStandby),
    "geometry-shift detector demote must be legal");
static_assert(IsLegalTransition(S::ContinuousStandby, S::Continuous),
    "AssignTargets resume must be legal");
static_assert(!IsLegalTransition(S::None, S::Rotation),
    "skipping Begin to enter Rotation must be illegal");
static_assert(!IsLegalTransition(S::Editing, S::Continuous),
    "Editing must not promote to Continuous without going through None");
