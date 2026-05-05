#pragma once

#include "Calibration.h"

// Calibration state-machine transition table. Every site that mutates
// `CalCtx.state` should produce a transition listed here as legal — anything
// else is a state-machine bug.
//
// Production code does NOT call IsLegalTransition() before assigning state
// today (that would add ~12 call-sites for marginal benefit). The function
// exists primarily for the test suite: tests/test_state_transitions.cpp
// pins the table so any change to the state machine is visible in code
// review and forced through deliberate test-suite update.
//
// Legal transitions (mapped from grep of "state = CalibrationState::" sites):
//   None -> Begin              (StartCalibration)
//   None -> Continuous         (very brief; StartContinuousCalibration goes Begin->Continuous in same call)
//   None -> ContinuousStandby  (ParseProfile autostart_continuous_calibration)
//   None -> Editing            (UI: Edit Calibration button)
//   Begin -> None              (CollectSample bail, CalibrationTick bail)
//   Begin -> Rotation          (one-shot phase 1 starts)
//   Begin -> Continuous        (StartContinuousCalibration: Begin set by StartCalibration, then immediately Continuous)
//   Rotation -> None           (CollectSample bail)
//   Rotation -> Translation    (one-shot phase 2 starts)
//   Translation -> None        (one-shot solve completes, CalibrationTick bail)
//   Editing -> None            (UI: leave edit mode)
//   Continuous -> None         (EndContinuousCalibration)
//   Continuous -> Begin        (StartContinuousCalibration via auto-recovery / wedge recovery)
//   Continuous -> ContinuousStandby (geometry-shift detector at Calibration.cpp:2152)
//   ContinuousStandby -> None  (EndContinuousCalibration, CollectSample bail)
//   ContinuousStandby -> Begin (StartContinuousCalibration's StartCalibration call)
//   ContinuousStandby -> Continuous (AssignTargets succeeded, line 2353)
//
// Self-loops (state -> state) always legal — represent "no transition this tick."

namespace spacecal::state {

constexpr bool IsLegalTransition(CalibrationState from, CalibrationState to) {
    if (from == to) return true;

    switch (from) {
    case CalibrationState::None:
        return to == CalibrationState::Begin
            || to == CalibrationState::Continuous
            || to == CalibrationState::ContinuousStandby
            || to == CalibrationState::Editing;

    case CalibrationState::Begin:
        return to == CalibrationState::None
            || to == CalibrationState::Rotation
            || to == CalibrationState::Continuous;

    case CalibrationState::Rotation:
        return to == CalibrationState::None
            || to == CalibrationState::Translation;

    case CalibrationState::Translation:
        return to == CalibrationState::None;

    case CalibrationState::Editing:
        return to == CalibrationState::None;

    case CalibrationState::Continuous:
        return to == CalibrationState::None
            || to == CalibrationState::Begin
            || to == CalibrationState::ContinuousStandby;

    case CalibrationState::ContinuousStandby:
        return to == CalibrationState::None
            || to == CalibrationState::Begin
            || to == CalibrationState::Continuous;
    }
    return false;
}

} // namespace spacecal::state
