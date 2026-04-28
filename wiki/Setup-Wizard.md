# First-run Setup Wizard

The first time you launch the WhyKnot fork of Space Calibrator, a wizard opens and walks you through getting calibrated. The goal: a brand-new user with two tracking systems should be running in under a minute, without ever clicking into the Settings tabs.

This page documents what the wizard does, when it runs, and how to re-run it.

## When does it appear?

- **First launch on a new install**, when no profile has been calibrated yet (`wizardCompleted` is false in the saved profile).
- **Manually**, via the **Run setup wizard** button on the Advanced tab. Use this after changing your hardware (added a new tracking system, removed one, swapped your HMD).

Once you finish the wizard or click any "Skip" button, `wizardCompleted` flips to true in your profile and the wizard never auto-opens again. It's a first-run experience, not a nag.

## The flow

The wizard is one modal dialog with several screens. Each transition is a button click.

### Step 1 — Welcome

Brief intro, two buttons:

- **Continue** — proceeds to detection.
- **Skip wizard** — sets `wizardCompleted = true` and exits. Use this if you know exactly what you want to do and want to drive the existing tabs yourself.

### Step 2 — Auto-detection

The wizard reads the SteamVR device list and groups everything by tracking system name (`oculus`, `lighthouse`, `slimevr`, `tundra`, etc.). It identifies your HMD's tracking system and treats every other detected system as a candidate for calibration.

Three branches based on what's found:

#### One tracking system only

> "Only one tracking system detected (X). All your tracked devices share a single coordinate space already, so calibration would not help and could actually add noise. You can close this app — it is not needed for this setup."

This is the lighthouse-only setup case. Vive/Index HMD with Vive lighthouse trackers — everything is already in the same coordinate space, and Space Calibrator would *increase* error by trying to align it to itself. The wizard tells you to close the app and walks away.

#### Two tracking systems (the common case)

> Quest HMD + SlimeVR body trackers, Quest HMD + lighthouse trackers, etc.

The wizard moves directly into the per-system flow with one entry to calibrate.

#### Three or more tracking systems

> Quest HMD + SlimeVR + lighthouse, etc.

Same flow as two systems, but after each one finishes you're asked if you want to calibrate the next one. The math runs on each pair (HMD ↔ system N) independently — see [[Multi-Ecosystem]] for the details.

### Step 3 — Pick a target

For the current pending system, the wizard shows you a list of devices from that system (excluding the HMD). You pick one to use as the calibration target. Buttons:

- **Start calibration** — uses your selected device + the HMD as the calibration pair, kicks off continuous calibration, and moves to the next screen.
- **Skip this system** — drops the current system from the queue without calibrating it. Useful if you have a system you don't actually wear (e.g. a stale Vive tracker on the shelf).
- **Skip wizard** — exits the wizard entirely; nothing for this or subsequent systems is calibrated.

Why does the wizard ask you to pick *one* tracker per system? Because the per-system fallback transform we calibrate against that one tracker applies to every device from that system. Calibrating a single SlimeVR tracker also aligns all your other SlimeVR trackers, so picking the most-easily-moved one (e.g. one held in your hand, or the hip tracker) is enough.

### Step 4 — Calibrating

Continuous calibration is now running. The screen says:

> "Calibrating SlimeVR. Slowly move and rotate the chosen tracker (or walk around if it is on your body) for ~30 seconds. The math figures out the alignment from your motion."

It also shows a status:

- "Waiting for first valid calibration..." while the math hasn't accepted any solution yet.
- "Calibration accepted." (green) once it has.

Click **This system is done** when the alignment looks right. Click **Cancel** to bail out (it leaves the calibration running but exits the wizard).

The wizard does *not* auto-detect "calibration is good now" — it leaves that judgement to you. The reasoning: continuous calibration converges asymptotically; there's no clean "done" event. Erring on the side of "let the user decide" is safer than auto-advancing too early.

### Step 5 — System done, more pending?

If there are systems still in the queue:

> "Done! We still see Vive Lighthouse waiting to be calibrated. Want to do that next?"

- **Yes, calibrate next** — loops back to step 3 with the next system.
- **No, finish wizard** — closes the wizard with the queue partially processed.

### Step 6 — All done

Closing screen. `wizardCompleted` is set to true and the profile is saved.

## What does the wizard set?

Sane defaults — same as a fresh install:

- `calibrationSpeed = AUTO` (picks Fast/Slow/Very Slow from observed jitter)
- `recalibrateOnMovement = true` (motion-gated blend, hides drift while stationary)
- `lockRelativePositionMode = AUTO` (detects rigid attachment automatically)
- `quashTargetInContinuous = false` (the target tracker is visible to other apps as itself)
- `enableStaticRecalibration = false`
- `requireTriggerPressToApply = false`
- `ignoreOutliers = false` (the in-code default; can be enabled in Advanced)

If you want different settings (e.g. you taped a tracker to your headset and want to set Lock to ON immediately rather than letting AUTO detect it), drop into the Advanced tab after the wizard finishes.

## Re-running

Hit **Run setup wizard** on the Advanced tab. The wizard restarts from step 1 with fresh device detection. Any existing calibrations stay applied — re-running doesn't auto-clear them; the wizard just walks you through reconfiguring whichever system you point it at.

If you want to start completely fresh, hit **Reset settings** on the Advanced tab first (clears the whole profile), then re-run the wizard.

## Source

- [src/overlay/Wizard.h](../src/overlay/Wizard.h)
- [src/overlay/Wizard.cpp](../src/overlay/Wizard.cpp)
- Auto-open hook: `BuildMainWindow()` in [src/overlay/UserInterface.cpp](../src/overlay/UserInterface.cpp)
- Persistence flag: `wizardCompleted` in `CalibrationContext`, written in `WriteProfile()` ([src/overlay/Configuration.cpp](../src/overlay/Configuration.cpp))
