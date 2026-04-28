#pragma once

// First-run setup wizard.
//
// On first launch (when CalibrationContext::wizardCompleted is false) the
// wizard walks the user through:
//   1. Welcome / what-this-does intro.
//   2. Auto-detection of tracking systems via VRState.
//   3. Branching:
//        - 1 system  -> "you don't need this app, all your devices share a
//                        single tracking space"
//        - 2 systems -> calibrate the non-HMD system against the HMD
//        - 3+ systems -> calibrate each non-HMD system one by one, looping
//                        until all are done or the user opts out
//   4. Per-system: pick a target device, run continuous calibration, confirm.
//
// The wizard doesn't replace the existing calibration flow -- it steers the
// user through it. Each "calibrate this system" step ultimately calls into
// StartContinuousCalibration with the chosen ref+target devices. After the
// first system is calibrated, additional systems are appended to
// CalibrationContext::additionalCalibrations.
//
// The wizard is drawn each frame from BuildMainWindow when active. State
// (current step, pending systems, current target system index) is owned by
// a single static instance inside the .cpp.

#include <string>
#include <vector>

namespace spacecal::wizard {

// Returns true while the wizard is running (modal active). Caller can suppress
// other top-level UI / popups while this is true so they don't compete for
// focus with the wizard modal.
bool IsActive();

// Activate the wizard. Resets internal state to step zero. Called on first
// run by BuildMainWindow, and from a button in Advanced settings.
void Open();

// Draw the wizard modal if active. No-op when inactive. Call once per frame
// from BuildMainWindow.
void Draw();

} // namespace spacecal::wizard
